#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from simulation_pkg.pybullet_interface import PybulletInterface
from simulation_interfaces.msg import PandaJointsInfo, PandaDynamics
from simulation_pkg.kinematics_utils import calc_forward_kinematics, calc_full_jacobian_modified_dh, damped_pseudoinv
from simulation_pkg.panda_parameters import dh_parameters
import numpy as np
from enum import Enum

class ControlType(Enum):
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3

position_ctrl_topic = "/panda/robot/control/reference/position"
velocity_ctrl_topic = "/panda/robot/control/reference/velocity"
torque_ctrl_topic = "/panda/robot/control/reference/torque"

joints_info_topic = "/panda/robot/state/joints"
dynamics_topic = "/panda/robot/state/dynamics"

external_force_topic = "/panda/robot/external_force"

default_panda_urdf_path = "/home/nikolas/UNIPV/ros2_ws_prova/src/simulation_pkg/pybullet_data/robots/franka_panda/panda_no_hand.urdf"

q_init = np.deg2rad([0, -45, 0, -135, 0, 90, 0]).tolist()

"""
This class represents the ROS2 node which manages the PyBullet environment.
"""
class PandaPyBulletNode(Node):
    def __init__(self):
        super().__init__('panda_pybullet_node')
        
        # Definition of the ROS parameters
        self.sim_duration = self.declare_parameter("sim_duration", 100).value
        self.sim_timestep = self.declare_parameter("sim_timestep", 1/60).value
        self.enable_gravity = self.declare_parameter("enable_gravity", True).value
        self.status_publish_period = self.declare_parameter("status_publish_period", 0.05).value
        self.use_gui = self.declare_parameter("use_gui", True).value
        self.ctrl_type = self.declare_parameter("control_type", 1).value
        self.joint_friction_coeff = self.declare_parameter("joint_friction_coeff", 0.0).value
        self.robot_urdf_path = self.declare_parameter("robot_urdf_path", default_panda_urdf_path).value

        # Depending on the control type, setup the control topic to which this node will subscribe
        if self.ctrl_type == ControlType.POSITION.value:
            self.log_info("Robot will be controlled in POSITION.")
            self.control_topic = position_ctrl_topic
            self.ctrl_signal = np.array(q_init)
        elif self.ctrl_type == ControlType.VELOCITY.value:
            self.log_info("Robot will be controlled in VELOCITY.")
            self.control_topic = velocity_ctrl_topic
            self.ctrl_signal = np.zeros(7)
        elif self.ctrl_type == ControlType.TORQUE.value:
            self.log_info("Robot will be controlled in TORQUE.")
            self.control_topic = torque_ctrl_topic
            self.ctrl_signal = np.zeros(7)
        else:
            self.log_error("Invalid control type.")
            raise Exception()

        # Declare elements for PyBullet
        self.pb_int = PybulletInterface()
        self.robot_id = None
        self.controlled_joints_idx = None
        self.sim_time = 0
        
        # Define the timers 
        self.init_pybullet_timer = self.create_timer(timer_period_sec=0.001, callback=self.init_pybullet_callback)
        self.step_simulation_timer = self.create_timer(timer_period_sec=self.sim_timestep, callback=self.step_callback)
        self.publish_status_timer = self.create_timer(timer_period_sec=self.status_publish_period, callback=self.publish_robot_status)
        
        # The following timers will be resumed as soon as PyBullet is initialised
        self.step_simulation_timer.cancel() 
        self.publish_status_timer.cancel()

        # Define the publishers
        self.joints_info_pub = self.create_publisher(PandaJointsInfo, joints_info_topic, 1)
        self.dynamics_pub = self.create_publisher(PandaDynamics, dynamics_topic, 1)

        # Define the subscribers
        self.control_sub = self.create_subscription(Float32MultiArray, self.control_topic, callback=self.update_control, qos_profile=1)
        self.external_force_sub = self.create_subscription(Float32MultiArray, external_force_topic, callback=self.external_force_callback, qos_profile=1)
        
        # define the ROS messages that will be sent
        self.joints_info_msg = PandaJointsInfo()
        self.dynamics_msg = PandaDynamics()

        
        # Define the array containing the torque caused by the external force acting on the end-effector
        self.external_torque = np.zeros(7)

        self.log_info("Node initialized successfully.")

    """
    Callback which periodically applies the last control signal received and performs a simulation step
    until the maximum time is reached.
    """
    def step_callback(self):
        
        if self.ctrl_type == ControlType.POSITION.value:
            self.log_info(f"Reference:  {self.ctrl_signal.tolist()}.")
            self.pb_int.send_position_control(self.robot_id, self.ctrl_signal.tolist(), self.controlled_joints_idx)
        elif self.ctrl_type == ControlType.VELOCITY.value:
            self.pb_int.send_velocity_control(self.robot_id, self.ctrl_signal.tolist(), self.controlled_joints_idx)
        elif self.ctrl_type == ControlType.TORQUE.value:
            self.pb_int.send_torque_control(self.robot_id, (self.ctrl_signal + self.external_torque).tolist(), self.controlled_joints_idx)
        
        self.pb_int.step()
        self.sim_time += self.sim_timestep
        if self.sim_time >= self.sim_duration:
            self.step_simulation_timer.cancel()
            self.log_info(f"Maximum time ({self.sim_duration} s) reached.")
            self.pb_int.disconnect()

    """
    Callback which is called only when the node is launched and initialize the simulation environment.
    """
    def init_pybullet_callback(self):
        # Stop the timer so that this function is not called again
        self.init_pybullet_timer.cancel()
        # Initilize the PyBullet scene  
        self.log_info("Simulation init...")
        self.pb_int.connect(use_gui=self.use_gui)
        self.pb_int.init_scene(real_time=False, timestep=self.sim_timestep, gravity=self.enable_gravity)
        
        # Add the tobot to the scene and retrieve joints info
        self.robot_id = self.pb_int.add_object_to_scene(urdf_path=self.robot_urdf_path)
        self.controlled_joints_idx, _, _, _, _, = self.pb_int.get_joint_infos(robotID=self.robot_id)
        self.pb_int.reset_joints_state(robot_id=self.robot_id, q_init=q_init, joint_indices=self.controlled_joints_idx)

        # If the robot is controlled in torque, disable the default velocity motors
        if self.ctrl_type == ControlType.TORQUE.value:
            self.log_info("Setting up the simulation for torque control...")
            self.q, self.dq, _ = self.pb_int.get_robot_state(self.robot_id)
            _, _, gravity = self.pb_int.get_robot_model(self.robot_id, self.q.tolist(), self.dq.tolist())
            self.ctrl_signal = gravity  # Automatically handle gravity compensation in the first step
            self.pb_int.disable_default_motors(self.robot_id, self.controlled_joints_idx, self.joint_friction_coeff)
        self.log_info("Simulation init complete.")
        
        # Restart the timer that controls the simulation
        self.step_simulation_timer.reset()  
        self.publish_status_timer.reset()
        self.log_info("Simulation started.")

    """
    Store the reference signal published on the topic. 
    """
    def update_control(self, msg:Float32MultiArray):
        self.ctrl_signal = msg.data
    
    """
    Callback that converts a linear force applied on the end-effector into a torque for the robot joints
    relying on the jacobian pseudo-inverse. 
    """
    def external_force_callback(self, msg:Float32MultiArray):
        try:
            kinematic_chain = calc_forward_kinematics(dh_parameters, self.q)
            jacobian = calc_full_jacobian_modified_dh(kinematic_chain)
            jacobian_pinv = damped_pseudoinv(jacobian[0:3,:], damping=0.5)
            self.external_torque = jacobian_pinv @ msg.data

        except:
            pass
    
    """
    Callback which reads the robot information and publish them on the defined topics.
    """
    def publish_robot_status(self):
        # Collect joints and model information
        self.q, self.dq, _ = self.pb_int.get_robot_state(self.robot_id)
        mass_matrix, coriolis, gravity = self.pb_int.get_robot_model(self.robot_id, self.q.tolist(), self.dq.tolist())
        # Prepare the messages
        self.dynamics_msg.time = self.sim_time
        self.dynamics_msg.mass_matrix = np.ravel(mass_matrix.T)
        self.dynamics_msg.coriolis = coriolis
        self.dynamics_msg.gravity = gravity
        
        self.joints_info_msg.time = self.sim_time
        self.joints_info_msg.position = self.q
        self.joints_info_msg.velocity = self.dq
        # Publish the messages
        self.dynamics_pub.publish(self.dynamics_msg)
        self.joints_info_pub.publish(self.joints_info_msg)

    def log_info(self, text:str):
        self.get_logger().info(text)
    
    def log_error(self, text:str):
        self.get_logger().error(text)
    
    def log_warning(self, text:str):
        self.get_logger().warning(text)
    

def main(args=None):
    rclpy.init(args=args)
    node = PandaPyBulletNode()

    rclpy.spin(node)
    rclpy.shutdown()




if __name__=="__main__":
    main()
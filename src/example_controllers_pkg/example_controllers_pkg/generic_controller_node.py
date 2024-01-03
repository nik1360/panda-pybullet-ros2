#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from simulation_interfaces.msg import PandaJointsInfo, PandaDynamics
import numpy as np

import abc

"""
This is the class used to store the last information retrieved from the robot.
"""
class LastRobotData:
    def __init__(self):
        self.joints_info_check = False
        self.dynamics_check = False

        self.q = np.zeros(7)
        self.dq = np.zeros(7)
        self.inertia_matrix = np.zeros((7,7))
        self.coriolis = np.zeros(7)
        self.gravity = np.zeros(7)

"""
This is the class that must be extended when creating a new controller.
"""
class GenericControllerNode(Node):
    """
    Intialization method.
    @param node_name: name that will be assigned to the node
    @param ctrl_topic: topic on which the control is published 
    @param joints_info_topic: topic on which the robot joints data are published 
    @param dynamics_topic: topic on which the robot dynamics data are published
    """
    def __init__(self, node_name:str, ctrl_topic:str, joints_info_topic:str, dynamics_topic:str) -> None:
        
        super().__init__(node_name)
        compute_reference_period = self.declare_parameter("reference_pub_period", 0.02).value

        self.control_pub = self.create_publisher(Float32MultiArray, ctrl_topic, 1)

        self.last_robot_data = LastRobotData()
        
        self.joints_info_sub = self.create_subscription(PandaJointsInfo, joints_info_topic, 
                                                        callback=self.update_joints_info, qos_profile=1)
        self.dynamics_sub = self.create_subscription(PandaDynamics, dynamics_topic, 
                                                        callback=self.update_dynamics_info, qos_profile=1)
        self.compute_reference_timer = self.create_timer(compute_reference_period, callback=self.compute_reference)
        self.compute_reference_timer.cancel()

        self.ctrl_msg = Float32MultiArray() # The control signal is an array of 7 elements, 
                                            # independently from the type (position, velocity, torque)

    """
    This is the callback in which the control signal is computed. It must be 
    implemented in the class which extends GenericControllerNode.
    """
    @abc.abstractmethod
    def compute_reference(self):
        raise NotImplementedError
    
    """
    This is the callback that stores the last joints information received from the robot.
    """
    def update_joints_info(self, msg:PandaJointsInfo):
        self.last_robot_data.joints_info_check = True
        self.last_robot_data.q = msg.position  
        self.last_robot_data.dq = msg.velocity 
        
        if self.last_robot_data.dynamics_check and self.compute_reference_timer.is_canceled():
            self.log_info("Starting control.")
            self.compute_reference_timer.reset()


    """
    This is the callback that stores the last information received about the robot dynamics.
    """  
    def update_dynamics_info(self, msg:PandaDynamics):
        self.last_robot_data.dynamics_check = True
        self.last_robot_data.inertia_matrix = msg.mass_matrix.reshape(7,7).T
        self.last_robot_data.coriolis = msg.coriolis
        self.last_robot_data.gravity = msg.gravity

        if self.last_robot_data.joints_info_check and self.compute_reference_timer.is_canceled():
            self.log_info("Starting control.")
            self.compute_reference_timer.reset()

    def log_info(self, text:str):
        self.get_logger().info(text)
    
    def log_error(self, text:str):
        self.get_logger().error(text)
    
    def log_warning(self, text:str):
        self.get_logger().warning(text)
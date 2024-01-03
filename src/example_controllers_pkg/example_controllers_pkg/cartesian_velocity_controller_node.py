#!/usr/bin/env python3

import rclpy

from example_controllers_pkg.generic_controller_node import GenericControllerNode
from example_controllers_pkg.robotics_utils import (calc_forward_kinematics, calc_full_jacobian_modified_dh,
                                                          rotmat_to_angle_axis, damped_pseudoinv)
from simulation_pkg.panda_parameters import dh_parameters
import numpy as np

"""
This simple node controls the robot so that its end-effector reaches a desired pose in the cartesian space, providing a 
reference for the joint velocities. The orientation error is first expressed as a rotation matrix and then using the 
Angle-Axis representation.

In inherits the methods and the attributes from the GenericControllerNode class. Only the compute_reference
method is overloaded.
"""
class VelocityControllerNode(GenericControllerNode):
    def __init__(self):
        super().__init__(node_name="velocity_control_node", ctrl_topic="/panda/robot/control/reference/velocity", 
                         joints_info_topic="/panda/robot/state/joints", dynamics_topic="/panda/robot/state/dynamics")

        self.b_p_target = self.declare_parameter("b_p_target", [0.3, 0.3, 0.7]).value
        self.b_R_target = np.array(self.declare_parameter("b_R_target", [1, 0, 0, 0, 0, 1, 0, -1, 0]).value).reshape(3,3)
        
        self.position_gain = self.declare_parameter("position_gain", 1).value
        self.orientation_gain = self.declare_parameter("orientation_gain", 1).value

        self.jacobian = np.zeros((6,7))
        self.kinematic_chain = []
        self.log_info("Node successfully initialised!")
        self.log_info("Waiting for the robot to publish its status...")

    """
    Overloading of compute_reference of GenericControllerNode. 
    
    It computes the reference velocity relying on the jacobian pseudoinverse strategy.
    """
    def compute_reference(self):
        # Compute the forward kinematics and the jacobian matrix
        self.kinematic_chain = calc_forward_kinematics(dh_parameters, self.last_robot_data.q)  
        self.jacobian = calc_full_jacobian_modified_dh(self.kinematic_chain)
        
        b_R_ee = self.kinematic_chain[-1][0:3, 0:3]
        b_p_ee = self.kinematic_chain[-1][0:3, 3]

        # Compute the ee position error
        pos_error = self.b_p_target - b_p_ee
        # Compute the orientation erro in the angle-axis representation
        angle, ee_axis = rotmat_to_angle_axis(b_R_ee.T@self.b_R_target) 
        b_axis = b_R_ee @ ee_axis

        # Compute desired linear and angular velocities for the end-effector
        v_des_p = pos_error * self.position_gain
        v_des_o = angle * b_axis * self.orientation_gain
        v_des = np.array([v_des_p[0], v_des_p[1], v_des_p[2], v_des_o[0], v_des_o[1], v_des_o[2]])

        # Express such velocity in the joint space using the pseudoinverse of the jacobian
        q_dot_des = damped_pseudoinv(self.jacobian) @ v_des

        # Publish the control on the topic
        self.ctrl_msg.data = np.float32(q_dot_des).tolist()
        self.control_pub.publish(self.ctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
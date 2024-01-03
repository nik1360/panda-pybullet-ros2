#!/usr/bin/env python3
import rclpy
import numpy as np
from example_controllers_pkg.generic_controller_node import GenericControllerNode

"""
This simple node controls the robot so that it reaches a configuration in the joint space, modulating
the joint torques according to the Computed Torque strategy.
In inherits the methods and the attributes from the GenericControllerNode class. Only the compute_reference
method is overloaded.
"""
class TorqueControlNode(GenericControllerNode):
    def __init__(self):
        super().__init__(node_name="torque_control_node", ctrl_topic="/panda/robot/control/reference/torque", 
                         joints_info_topic="/panda/robot/state/joints", dynamics_topic="/panda/robot/state/dynamics")
        
        self.compute_reference_period = self.declare_parameter("compute_reference_period", 0.02).value
        self.proportional_gain = self.declare_parameter("proportional_gain", 10).value
        self.damping_gain = self.declare_parameter("damping_gain", 5).value
        self.q_target = np.deg2rad(self.declare_parameter("q_target", [0, -90, 0, -135, 0, 90, 0]).value)
    
        self.log_info("Node successfully initialised!")
        self.log_info("Waiting for the robot to publish its status...")

    
    """
    Overloading of compute_reference of GenericControllerNode. 
    
    It computes the control torque relying on the Computed Torque strategy.
    """
    def compute_reference(self):
        q = self. last_robot_data.q
        dq = self.last_robot_data.dq
        inertia_matrix = self.last_robot_data.inertia_matrix
        coriolis = self.last_robot_data.coriolis
        gravity = self.last_robot_data.gravity

        q_error = q - self.q_target
        tau = - inertia_matrix@(self.proportional_gain*q_error + self.damping_gain*dq) + coriolis + gravity 
        
        # Publish the control on the topic
        self.ctrl_msg.data = np.float32(tau).tolist()
        self.control_pub.publish(self.ctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueControlNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
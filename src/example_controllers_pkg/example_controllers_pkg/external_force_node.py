#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


joints_info_topic = "/panda/robot/state/joints"
external_force_topic = "/panda/robot/external_force"


class ExternalForceNode(Node):
    def __init__(self):
        super().__init__('external_force_node')

        force_generation_period = self.declare_parameter("force_generation_period", 0.1).value
        self.linear_force = np.array([self.declare_parameter("force_x", 0).value,
                               self.declare_parameter("force_y", 0).value,
                               self.declare_parameter("force_z", 0).value,], dtype=np.float32)
        

        self.force_pub = self.create_publisher(Float32MultiArray, external_force_topic, 1)
        self.generate_force_timer = self.create_timer(force_generation_period, callback=self.generate_force)

        self.force_msg = Float32MultiArray()

        self.log_info("Node successfully initialised!")
        

    def generate_force(self):
        self.force_msg.data = self.linear_force.tolist()
        self.force_pub.publish(self.force_msg)


    def log_info(self, text:str):
        self.get_logger().info(text)
    
    def log_error(self, text:str):
        self.get_logger().error(text)
    
    def log_warning(self, text:str):
        self.get_logger().warning(text)
    

def main(args=None):
    rclpy.init(args=args)
    node = ExternalForceNode()

    rclpy.spin(node)
    rclpy.shutdown()



if __name__=="__main__":
    main()
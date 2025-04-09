#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking_node')

        self.get_logger().info("Tracking Node Ready")
            
def main(args = None):
    rclpy.init(args = args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
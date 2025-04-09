#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.get_logger().info("Navigation Node Ready")
            
def main(args = None):
    rclpy.init(args = args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
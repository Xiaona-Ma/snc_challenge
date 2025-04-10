#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.get_logger().info("Detection Node Ready")
            
def main(args = None):
    rclpy.init(args = args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
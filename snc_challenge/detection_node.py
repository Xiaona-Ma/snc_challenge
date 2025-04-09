#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class DetectionNode(Node):

    def __init__(self):
        self._node_name = 'detection_node'
        super().__init__(self._node_name)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ros_time_stamp = self.get_clock().now
        # Display the message on the console
        self.get_logger().info(self._node_name + 'is alive... ' + str(ros_time_stamp))
            
def main(args = None):
    rclpy.init(args = args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
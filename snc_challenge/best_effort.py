#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

'''
This Node repeats a sensor_data (best_effort) QoS topic to
    a reliable QoS topic
'''
class BestEffortRepeater(Node):
    def __init__(self):
        super().__init__('best_effort_repeater')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sub_topic_name', '/be_topic'),
                # ('sub_topic_name', '/camera/color/image_raw'),
                # ('sub_topic_name', '/camera/color/image_raw/compressed'),
                ('repeat_topic_name', '/repeat_topic'),
                ('use_compressed', False),
            ]
        )
        self.sub_topic_name = self.get_parameter('sub_topic_name').value
        self.repeat_topic_name = self.get_parameter('repeat_topic_name').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        self.get_logger().info('Subscribing to: "%s"' % self.sub_topic_name)
        self.get_logger().info('Publishing to: "%s"' % self.repeat_topic_name)
        self.get_logger().info(f'Using Compressed: {self.use_compressed}')
        
        # Create subscriber - For an UNCOMPRESSED image
        if not self.use_compressed:
            self.sub_be = self.create_subscription(
                                Image,
                                self.sub_topic_name,
                                self.be_listener,
                                rclpy.qos.qos_profile_sensor_data
                        )
        else:
            # Create subscriber - For an COMPRESSED image
            self.sub_be = self.create_subscription(
                                CompressedImage,
                                self.sub_topic_name,
                                self.be_listener_compressed,
                                rclpy.qos.qos_profile_sensor_data
                        )

        # Repeater - Always outputs a non-compressed image
        self.pub_reliable = self.create_publisher(
                              Image,
                              self.repeat_topic_name,
                              1
                          )
        
        # Bridge for Compression conversion
        self.bridge = CvBridge()

    def be_listener(self, msg):
        # self.get_logger().info('Received: "%s"' % msg.data)
        
        # Repeat the data
        self.pub_reliable.publish(msg)
        # self.get_logger().info('Published: image')

    def be_listener_compressed(self, msg):
        # self.get_logger().info('Received: "%s"' % msg.data)
        
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # Convert OpenCV image to ROS Image message
        uncompressed = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        uncompressed.header = msg.header
        
        # Repeat the data
        self.pub_reliable.publish(uncompressed)
        # self.get_logger().info('Published (uncompressed): image')

def main(args=None):
    rclpy.init()
    node = BestEffortRepeater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    exit(0)
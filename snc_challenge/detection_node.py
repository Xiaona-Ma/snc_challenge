#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf2_ros

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('DetectionNode is Ready!')

        # Subscribe to Camera
        self.img_sub = self.subscriptions(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Subscribe to Laser
        self.laser_sub = self.subscriptions(
            LaserScan, '/scan', self.laser_callback, 10)

        # Issuance of hazard symbols
        self.marker_pub = self.create_publisher(
            Marker, '/hazards', 10)
        

    def image_callback(self, msg):
        # Image processing logic 
        # TO DO...

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Call find_object_2d to detect markers...
            # TO DO...
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def laser_callback(self, msg):
        # Laser Data Processing
        # TO DO...
        pass
            
def main(args = None):
    rclpy.init(args = args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
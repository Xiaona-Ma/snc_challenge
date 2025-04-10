#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf2_ros import TransformException, Buffer, TransformListener

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # Subscribe to Camera
        self.img_sub = self.subscriptions(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Subscribe to Laser
        self.laser_sub = self.subscriptions(
            LaserScan, '/scan', self.laser_callback, 10)

        # publish the hazard symbols
        self.marker_pub = self.create_publisher(
            Marker, '/hazards', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Storing detected markers
        self.marker_ids = {}
        self.bridge = CvBridge()
        self.get_logger().info("Detection Node Ready")
        

    def image_callback(self, msg):
        # Image processing logic 
        # TO DO...

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Call find_object_2d to detect markers...
            detected_markers = [
                {"id": 1, "center_x": 320, "center_y": 240},
                {"id": 5, "center_x": 400, "center_y": 200}
            ]
            for marker in detected_markers:
                if marker["id"] not in self.marker_ids:
                    self.publish_marker(marker)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
    
    def publish_marker(self, marker):
        self.get_logger().info(f"Publishing marker: {marker}")
        marker_msg = Marker()

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
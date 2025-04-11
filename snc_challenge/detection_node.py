#!/usr/bin/env python

import rclpy
import tf2_geometry_msgs
import tf2_ros

from rclpy.node import Node
from find_object_2d.msg import ObjectsStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # Create a CvBridge object
        self.bridge = CvBridge()
        self.camera_model = None
        self.current_depth_image = None

        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # Storage of detected flag IDs (to avoid double posting)
        self.detected_ids = set()

        # 1. Subscribe to find_object_2d output (/objects thread)
        self.objects_sub = self.create_subscription(
           ObjectsStamped,
            '/objects',
            self.objects_callback,
            rclpy.qos.qos_profile_sensor_data  # Use sensor data QoS profile
        )

        # 2.1 Subscribe to depth images
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        # 2.2 Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # 3. Post a hazard symbol to /hazards
        self.hazard_pub = self.create_publisher( Marker, '/hazards', 10 )

        self.get_logger().info("Detection with subscription Node Ready")

    # Handle the objects detected by find_object_2d
    def objects_callback(self, msg):
        # msg.objectIds contains the IDs of the detected objects
        self.get_logger().info(f"Objects detected: {msg.objectIds}")
        # Here you can process the detected objects

       
    # Handle the depth image
    def depth_callback(self, msg):
        # msg contains the depth image in a ROS Image format
        self.get_logger().info("Depth image received")
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Here you can process the depth image
        
    
    # Handle the camera info
    def camera_info_callback(self, msg):
        # msg contains the camera info in a ROS CameraInfo format
        self.get_logger().info("Camera info received")
        self.camera_model = msg
        # Here you can process the camera info
            

def main(args = None):
    rclpy.init(args = args)
    node = DetectionNode()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
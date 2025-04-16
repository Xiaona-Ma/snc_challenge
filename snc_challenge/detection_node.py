#!/usr/bin/env python

import rclpy
import tf2_geometry_msgs
import tf2_ros

from rclpy.node import Node
from find_object_2d.msg import ObjectsStamped 
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from image_geometry import PinholeCameraModel

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # 1. Subscribe to find_object_2d output (/objects thread)
        self.objects_sub = self.create_subscription(
           ObjectsStamped,
            '/objectsStamped',
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

        # Create a CvBridge object
        self.bridge = CvBridge()
        self.camera_model = None
        self.current_depth_image = None

        # 3. Post a hazard symbol to /hazards
        self.hazard_pub = self.create_publisher( Marker, '/hazards', 10 )

        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # Storage of detected flag IDs (to avoid double posting)
        self.detected_ids = set()

        self.get_logger().info("########## Detection with subscription Node Ready ##########")

    # Handle the objects detected by find_object_2d
    def objects_callback(self, msg):

        self.get_logger().warn("########## objects_callback ##########")

        if self.camera_model is None or self.current_depth_image is None:
            self.get_logger().warn("等待深度图像或相机参数...")
            return
        
        # Iterate over all objects detected
        for i in range(len(msg.objectIds)):
            marker_id = msg.objectIds[i]    # Object ID
            
            # Extraction of translation components from homography matrix (h31, h32)
            h31 = msg.h31[i]  # dx
            h32 = msg.h32[i]  # dy
            center_x = int(h31)
            center_y = int(h32)
            
            # Note: Depth images are in millimetres so need to be divided by 1000
            depth = self.current_depth_image[center_y, center_x] / 1000.0
            
            # Pixel coordinates to 3D coordinates (camera coordinate system)
            ray = self.camera_model.projectPixelTo3dRay((center_x, center_y))
            point_camera = [ray[0] * depth, ray[1] * depth, depth]
            
            # Transform the point from camera frame to map frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Creating a PointStamped Message
                point_camera_stamped = PointStamped()
                point_camera_stamped.header = msg.header
                point_camera_stamped.point = Point(
                    x=point_camera[0],
                    y=point_camera[1],
                    z=point_camera[2]
                )
                
                # transform the 
                point_map = tf2_geometry_msgs.do_transform_point(
                    point_camera_stamped,
                    transform
                ).point
                
                # publish hazard marker
                self.publish_hazard_marker(marker_id, point_map.x, point_map.y)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                   tf2_ros.ExtrapolationException) as ex:
                self.get_logger().error(f"坐标转换失败: {ex}")
        

    # Handle the depth image
    def depth_callback(self, msg):
        # msg contains the depth image in a ROS Image format
        self.get_logger().info("########## Depth image received ##########")
        
        # Convert the ROS Image message to a CV image
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"深度图像转换失败: {e}")
    
    
    # Handle the camera info
    def camera_info_callback(self, msg):
        # msg contains the camera info in a ROS CameraInfo format
        self.get_logger().info("########## Camera info received ##########")

        # Convert the ROS CameraInfo message to a PinholeCameraModel
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)


    def publish_hazard_marker(self, marker_id, x, y):

        self.get_logger().info("########## publish_hazard_marker ##########")
        
        if marker_id in self.detected_ids:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = marker_id

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 1.0 
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Infinite lifetime
        marker.lifetime.sec = 0
        
        # publish Marker
        self.hazard_pub.publish(marker)
        self.detected_ids.add(marker_id)
        self.get_logger().info(f"发布危险标志ID {marker_id} 在 ({x:.2f}, {y:.2f})")        

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
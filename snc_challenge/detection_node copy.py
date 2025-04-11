#!/usr/bin/env python

import rclpy
import tf2_geometry_msgs
import tf2_ros

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import numpy as np

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
            Float32MultiArray,
            '/detections',
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
        if self.camera_model is None or self.current_depth_image is None:
            self.get_logger().warn("等待深度图像或相机参数...")
            return
        
        for detection in msg.detections:
            if self.camera_model is None or self.current_depth_image is None:
                self.get_logger().warn("等待深度图像或相机参数...")
                return
        
        # 解析数据格式：每个对象12个浮点数 [id, width, height, h11, h12, h13, h21, h22, h23, h31, h32, h33]
        data = msg.data
        num_objects = len(data) // 12
        
        for i in range(num_objects):
            start_idx = i * 12
            marker_id = int(data[start_idx])          # 提取ID（根据文件名映射）
            center_x = int(data[start_idx + 9])       # h31 = x坐标（像素）
            center_y = int(data[start_idx + 10])      # h32 = y坐标（像素）
            
            # 从深度图像获取距离（单位：米，假设深度单位为毫米）
            depth = self.current_depth_image[center_y, center_x] / 1000.0
            
            # 将像素坐标转换为3D坐标（摄像头坐标系）
            ray = self.camera_model.projectPixelTo3dRay((center_x, center_y))
            point_camera = [ray[0] * depth, ray[1] * depth, depth]
            
            # 转换到map坐标系（参考transform.py的坐标变换逻辑）
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'camera_link',  # 摄像头坐标系名称
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # 构建PointStamped消息（参考transform.py）
                point_camera_stamped = PointStamped()
                point_camera_stamped.header.frame_id = 'camera_link'
                point_camera_stamped.header.stamp = self.get_clock().now().to_msg()
                point_camera_stamped.point = Point(
                    x=point_camera[0],
                    y=point_camera[1],
                    z=point_camera[2]
                )
                
                # 执行坐标变换
                point_map = tf2_geometry_msgs.do_transform_point(
                    point_camera_stamped,
                    transform
                ).point
                
                # 发布危险标志（参考publish_hazard.py）
                self.publish_hazard_marker(marker_id, point_map.x, point_map.y)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                self.get_logger().error(f"坐标转换失败: {ex}")
        

    # Handle the depth image
    def depth_callback(self, msg):
        # msg contains the depth image in a ROS Image format
        self.get_logger().info("Depth image received")
        
        # Convert the ROS Image message to a CV image
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"深度图像转换失败: {e}")
    
    
    # Handle the camera info
    def camera_info_callback(self, msg):
        # msg contains the camera info in a ROS CameraInfo format
        self.get_logger().info("Camera info received")

        # 解析相机参数，用于深度图像坐标转换
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)


    def publish_hazard_marker(self, marker_id, x, y):
        if marker_id in self.detected_ids:
            return  # 避免重复发布
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = marker_id  # 根据PDF中的ID表设置
        
        # 位置和尺寸
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        
        # 根据ID设置颜色
        if marker_id == 1:   # Explosive
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif marker_id == 2: # Flammable Gas
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        # ...其他ID的颜色设置
        
        marker.color.a = 1.0  # 不透明度
        
        # 发布Marker
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
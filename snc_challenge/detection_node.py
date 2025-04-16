#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Float32MultiArray
import tf2_geometry_msgs 
import cv2
import numpy as np

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # 初始化参数和工具
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 新增订阅find_object_2d的输出
        self.create_subscription(Float32MultiArray, '/objects', self.object_detection_callback, 10)
        self.last_detections = []  # 存储最新检测结果

        
        # 订阅RGB图像（来自best_effort_repeater的可靠话题）
        self.create_subscription(Image, '/image_topic_repeat', self.image_callback, 10)
        
        # 订阅深度图像和相机内参（来自示例代码配置）
        self.create_subscription(Image, '/depth_topic_repeat', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info_topic_repeat', self.camera_info_callback, 10)
        
        # 发布检测到的危险标记（参考publish_hazard.py）
        self.marker_pub = self.create_publisher(MarkerArray, '/hazards', 10)
        
        # 存储当前数据
        self.current_depth = None
        self.camera_info = None
        self.detection_history = {}  # 用于多帧验证
        
        # 标记ID映射（根据PDF定义）
        self.marker_ids = {
            "unknown": 0,
            "explosive": 1,
            "flammable_gas": 2,
            "non_flammable_gas": 3,
            "dangerous_when_wet": 4,
            "flammable_solid": 5,
            "spontaneously_combustible": 6,
            "oxidizer": 7,
            "organic_peroxide": 8,
            "Inhalation_toxic": 9,
            "poison": 10,
            "radioactive": 11,
            "corrosive": 12,
        }
    
    def object_detection_callback(self, msg):
        self.get_logger().info("===== 解析find_object_2d输出 =====")
        self.get_logger().info(f"=====find_object_2d输出: {msg} =====")

        self.last_detections = []
        data = msg.data
        for i in range(0, len(data), 7):
            obj = {
                "id": int(data[i]),      # 对象ID
                "width": data[i+1],      # 图像中宽度
                "height": data[i+2],     # 图像中高度
                "bbox": [
                    data[i+3],           # x1
                    data[i+4],           # y1
                    data[i+5],           # x2
                    data[i+6]            # y2
                ]
            }
            self.last_detections.append(obj)

    def camera_info_callback(self, msg):
        """存储相机内参（参考transform.py的TF处理）"""
        self.get_logger().info("===== 存储相机内参 =====")
        self.get_logger().info(f"=====相机信息: {msg} =====")
        # 仅在第一次接收时存储相机内参
        if not self.camera_info:
            self.camera_info = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info("相机内参已获取")

    def depth_callback(self, msg):
        """存储深度图像（添加滤波处理）"""
        self.get_logger().info("===== 存储深度图像 =====")
        self.get_logger().info(f"=====深度图像: {msg} =====")

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # 中值滤波去噪
            self.current_depth = cv2.medianBlur(depth_image, 5)
        except Exception as e:
            self.get_logger().error(f"深度图像处理失败: {e}")

    def image_callback(self, msg):
        """核心检测逻辑（整合示例代码的Marker发布）"""
        self.get_logger().info("===== 处理图像 =====")
        self.get_logger().info(f"=====图像: {msg} =====")
        
        # 检查深度图像和相机内参是否可用
        if self.current_depth is None or self.camera_info is None:
            self.get_logger().warn("等待深度数据或相机内参...")
            return
            
        try:
            # 转换RGB图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.last_detections:
                self.get_logger().warn("没有检测到任何对象")
                return
            
            markers = MarkerArray()
            for obj in self.last_detections:

                # 将find_object_2d的ID映射到PDF定义的ID
                marker_name = self._get_marker_name_by_id(obj["id"])
                if marker_name not in self.marker_ids:
                    continue

                # 计算边界框中心点
                cx = int((obj["bbox"][0] + obj["bbox"][2]) / 2)
                cy = int((obj["bbox"][1] + obj["bbox"][3]) / 2)
                
                # 获取深度值（取中心区域平均值）
                depth_roi = self.current_depth[cy-10:cy+10, cx-10:cx+10]
                depth_val = np.nanmedian(depth_roi)
                if depth_val <= 0 or depth_val > 5.0:  # 有效范围检查
                    continue
                
                # 坐标系转换（参考transform.py的TF变换）
                try:
                    # 从像素坐标系转换到相机坐标系
                    point_camera = PointStamped()
                    point_camera.header = msg.header
                    point_camera.point.x = (cx - self.camera_info['cx']) * depth_val / self.camera_info['fx']
                    point_camera.point.y = (cy - self.camera_info['cy']) * depth_val / self.camera_info['fy']
                    point_camera.point.z = depth_val
                    
                    # 转换到map坐标系（添加超时和坐标系检查）
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        msg.header.frame_id,  # 通常是camera_link
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
                    
                    # 多帧验证（防止瞬态误检）
                    marker_id = self.marker_ids[obj["id"]]
                    if marker_id not in self.detection_history:
                        self.detection_history[marker_id] = []
                    self.detection_history[marker_id].append(point_map.point)
                    
                    # 仅当连续3帧检测到同一位置时发布
                    if len(self.detection_history[marker_id]) >= 3:
                        avg_point = self._calculate_average_point(self.detection_history[marker_id])
                        
                        # 创建Marker（参考publish_hazard.py）
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.id = marker_id
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = avg_point.x
                        marker.pose.position.y = avg_point.y
                        marker.pose.position.z = 0.0  # 假设标记在地面
                        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
                        marker.color.r = 1.0
                        marker.color.a = 0.8
                        markers.markers.append(marker)
                        
                        # 清空历史记录
                        self.detection_history[marker_id] = []
                        
                except TransformException as e:
                    self.get_logger().warn(f"坐标变换失败: {e}")
                    continue
            
            # 发布所有有效标记
            if len(markers.markers) > 0:
                self.marker_pub.publish(markers)
                self.get_logger().info(f"已发布{len(markers.markers)}个标记")
                
        except Exception as e:
            self.get_logger().error(f"检测流程异常: {e}")

    def _calculate_average_point(self, points):
        """计算多个检测点的平均值"""
        x = sum([p.x for p in points]) / len(points)
        y = sum([p.y for p in points]) / len(points)
        return PointStamped().point(x=x, y=y, z=0.0)
    
    def _get_marker_name_by_id(self, find_object_id):
        """将find_object_2d的内部ID映射到PDF定义的标记名"""
        # 示例映射表（需根据实际训练模型调整）
        id_to_name = {
            0: "unknown",
            1: "explosive",
            2: "flammable_gas",
            3: "non_flammable_gas",
            4: "dangerous_when_wet",
            5: "flammable_solid",
            6: "spontaneously_combustible",
            7: "oxidizer",
            8: "organic_peroxide",
            9: "Inhalation_toxic",
            10: "poison",
            11: "radioactive",
            12: "corrosive",
        }
        return id_to_name.get(find_object_id, "unknown")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
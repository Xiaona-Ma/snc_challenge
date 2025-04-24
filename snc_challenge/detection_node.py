#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import String, Empty
import tf2_geometry_msgs
import cv2
import numpy as np

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.create_subscription(
            Empty,
            '/trigger_start',
            self._trigger_start_callback,
            10)
        
        # initialize parameters and tools
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.last_detections = []  # Store the latest test results
        self.current_depth = None
        self.camera_info = None
        self.detection_history = {}  # For multi-frame verification

        # Tag ID mapping (as defined in the PDF)
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
            "start": 13,
        }

        # Subscribe to the output of find_object_2d
        self.create_subscription(Float32MultiArray, 'objects', self.object_detection_callback, 10)

        # Subscribe to RGB images
        self.create_subscription(Image, 'image', self.image_callback, 10)
        
        # Subscribe to Depth Images and Camera Insights
        self.create_subscription(Image, 'depth', self.depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)
        
        # Issuance of detected hazard markers
        self.marker_pub = self.create_publisher(Marker, 'hazards', 10)
        self.start_marker_pub = self.create_publisher(PointStamped, 'start_marker', 10)
        self.status_pub = self.create_publisher(String, 'snc_status', 10)
        
        
        self.get_logger().info("========== Detection Node Ready ===========")
        self.status_pub.publish(String(data="========== DetectionNode 已初始化，等待输入数据 =========="))

    def _trigger_start_callback(self, msg):
        # 当收到 /trigger_start 时，查询机器人当前位置（base_link → map），并把这个点发布到 /start_marker。
        try:
            # 查最新的 base_link → map 变换
            tf = self.tf_buffer.lookup_transform(
                'map',              # 目标坐标系
                'base_link',        # 源坐标系
                Time(),  # 最新时刻
                timeout = Duration(seconds=0.1)
            )
            # 提取位置
            t = tf.transform.translation
            start_msg = PointStamped()
            start_msg.header.frame_id = 'map'
            start_msg.header.stamp = self.get_clock().now().to_msg()
            start_msg.point.x = t.x
            start_msg.point.y = t.y
            start_msg.point.z = t.z

            self.status_pub.publish(String(data="======== 收到 trigger_start，发布 Start Marker ========"))
            # 发布给 RViz
            self.start_marker_pub.publish(start_msg)
            self.get_logger().info("======== 触发 Start Marker 发布 at ({:.2f}, {:.2f}, {:.2f})======== "
                                .format(t.x, t.y, t.z))
        except Exception as e:
            self.get_logger().warn(f"======== trigger_start TF 失败: {e}======== ")
    

    def object_detection_callback(self, msg):

        self.last_detections.clear()
        data = msg.data

        for i in range(0, len(data), 6):
            obj = {
                "id": int(data[i]),      # Object ID
                "bbox": [
                    data[i+2],           # x1
                    data[i+3],           # y1
                    data[i+4],           # x2
                    data[i+5]            # y2
                ]
            }
            self.last_detections.append(obj)


    def camera_info_callback(self, msg):
        # Camera internal parameters stored only on first reception
        if not self.camera_info:
            self.camera_info = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info("========== Camera internal parameters have been acquired ==========")
            self.status_pub.publish(String(data="========== 已获取相机内参 =========="))


    def depth_callback(self, msg):
        # Store depth images (add filter processing)
        cvd = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.current_depth = cv2.medianBlur(cvd, 5)
        self.get_logger().info("======== Depth images have been acquired ========")
        self.status_pub.publish(String(data="========== 已获取深度图 =========="))


    # Main Process: Multi-frame Verification → Coordinate Transformation → MarkerArray Publishing
    def image_callback(self, msg):
        # 1. Waiting for necessary data
        if not self.last_detections or self.current_depth is None or self.camera_info is None:
            self.get_logger().warn("Waiting for depth data or internal camera parameters...")
            self.status_pub.publish(String(data="========== 等待深度或内参… ========== "))
            return
        
        self.status_pub.publish(String(data="========== 开始处理检测结果 =========="))

        markers = MarkerArray()
        for obj in self.last_detections:
            name = self._get_marker_name(obj["id"])

            if name == "start":
                self._process_start(obj, msg.header)
                continue

            # 2. 计算中心像素
            x1,y1,x2,y2 = map(int, obj["bbox"])
            cx, cy = (x1+x2)//2, (y1+y2)//2
            
            # 3. 深度过滤
            h, w = self.current_depth.shape
            x1, x2 = max(0, cx - 10), min(w, cx + 10)
            y1, y2 = max(0, cy - 10), min(h, cy + 10)
            roi = self.current_depth[y1:y2, x1:x2]
            # roi = self.current_depth[cy-10: cy + 10, cx-10: cx + 10]
            d = float(np.nanmedian(roi)) / 1000.0  # mm → m
            if d <= 0 or d > 5.0:  # 只取 0~5m
                continue
            # 4. 像素 → 相机坐标
            pc = PointStamped()
            pc.header = msg.header
            pc.point.x = (cx - self.camera_info['cx']) * d / self.camera_info['fx']
            pc.point.y = (cy - self.camera_info['cy']) * d / self.camera_info['fy']
            pc.point.z = d
            # 5. 相机坐标 → 地图坐标（使用拍摄时刻的时间戳）
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 
                    msg.header.frame_id, 
                    msg.header.stamp,
                    timeout = Duration(seconds=0.1)
                )
                pm = tf2_geometry_msgs.do_transform_point(pc, tf).point
            except TransformException as e:
                self.get_logger().warn(f"TF 失败: {e}")
                continue
            # 6. 多帧验证
            mid = self.marker_ids[name]
            self.detection_history.setdefault(mid, []).append(pm)
            if len(self.detection_history[mid]) < 3:
                continue
            avg = self._avg_point(self.detection_history[mid])
            self.detection_history[mid].clear()
            # 7. 生成并收集 Marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = avg
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r = 1.0; m.color.a = 0.8
            markers.markers.append(m)

        self.status_pub.publish(String(data=f"========== 多帧验证完成，准备发布 {len(markers.markers)} 个标志 =========="))
        for m in markers.markers:
            self.marker_pub.publish(m)
            self.status_pub.publish(String(data=f"========== Publishing hazard id={m.id} =========="))

    # Process the start marker
    def _process_start(self, obj, header):
        # 1. 计算中心像素
        x1, y1, x2, y2 = obj["bbox"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 2. 从 current_depth 中取 20×20 区域的中值深度
        roi = self.current_depth[cy-10:cy+10, cx-10:cx+10]
        depth_val = float(np.nanmedian(roi)) / 1000.0  # mm → m
        # 深度有效性检查
        if depth_val <= 0.0 or depth_val > 5.0:
            self.get_logger().warn(f"======== Start 标志深度异常：{depth_val} ========")
            return

        # 3. 像素 → 相机坐标
        point_cam = PointStamped()
        point_cam.header = header
        point_cam.point.x = (cx - self.camera_info['cx']) * depth_val / self.camera_info['fx']
        point_cam.point.y = (cy - self.camera_info['cy']) * depth_val / self.camera_info['fy']
        point_cam.point.z = depth_val

        # 4. 相机坐标 → 地图坐标（使用图像的时间戳保持同步）
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',               # 目标坐标系
                header.frame_id,     # 源坐标系，通常是 camera_link
                header.stamp,        # 使用图像的时间戳
                timeout = Duration(seconds=0.1)
            )
            point_map = tf2_geometry_msgs.do_transform_point(point_cam, tf)
        except TransformException as e:
            self.get_logger().warn(f"======== Start 标志 TF 转换失败：{e} ========")
            return

        # 5. 发布到 /start_marker
        start_msg = PointStamped()
        start_msg.header.frame_id = 'map'
        start_msg.header.stamp = self.get_clock().now().to_msg()
        start_msg.point = point_map.point
        self.start_marker_pub.publish(start_msg)
        self.get_logger().info("========  Start Marker 已检测并发布！ ========")


    def _avg_point(self, points):
        # Calculate the average point of multiple points
        x = sum(p.x for p in points) / len(points)
        y = sum(p.y for p in points) / len(points)
        z = sum(p.z for p in points) / len(points)
        return Point(x = x, y = y, z = z)
    
   
    def _get_marker_name(self, find_object_id):
        # Map find_object_2d's internal ID to the marker name defined in the PDF
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
            13: "start",
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
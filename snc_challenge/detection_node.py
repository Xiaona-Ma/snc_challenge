#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from find_object_2d.msg import ObjectsStamped  # find_object_2d 的输出消息类型

class HazardDetector(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # 参数声明
        self.declare_parameter('marker_frame', 'camera_link')  # 检测结果的原始坐标系
        self.declare_parameter('map_frame', 'map')            # 目标坐标系

        # TF2 相关初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅 find_object_2d 的检测结果
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/objectsStamped',  # find_object_2d 的默认输出话题
            self.detection_callback,
            10
        )

        # 发布危险标记的 Publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/hazards',
            10
        )

        self.get_logger().info("========== Hazard detection node initialized ===========")

    def detection_callback(self, msg):
        self.get_logger().info("========== Received detection message ==========")
        # 遍历所有检测到的对象
        for i in range(len(msg.objects.data)):
            if msg.objects.data[i] == 0:  # 跳过ID为0的未知对象
                continue

            try:
                # 获取坐标系变换（从摄像头坐标系到地图坐标系）
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('map_frame').value,
                    self.get_parameter('marker_frame').value,
                    rclpy.time.Time()
                )

                # 创建标记消息
                marker = Marker()
                marker.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 0.3  # 标记大小（单位：米）
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color.r = 1.0  # 红色
                marker.color.a = 1.0  # 不透明度

                # 使用检测到的位置和TF变换计算地图坐标系下的坐标
                marker.pose.position.x = msg.objects.data[i]  # 示例字段，需根据实际消息结构修改
                marker.pose.position.y = 0.0  # 示例字段
                marker.pose.position.z = 0.0  # 示例字段
                marker.pose.orientation.w = 1.0  # 无旋转

                # 发布标记
                self.marker_pub.publish(marker)
                self.get_logger().info(f"Published hazard {msg.objects.data[i]}")

            except tf2_ros.LookupException as e:
                self.get_logger().error(f"TF lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
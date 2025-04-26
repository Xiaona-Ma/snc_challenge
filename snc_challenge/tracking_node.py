import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import math
import time


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        self.get_logger().info("Tracking Node Ready")


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.explore_path = []
        self.return_path = []

        self.home_pose = None

        self.explore_path_pub = self.create_publisher(Path, '/path_explore', 10)
        self.return_path_pub = self.create_publisher(Path, '/path_return_home', 10)

        self.create_timer(0.5, self.record_pose)

        self.create_timer(60.0, self.return_home)  

        
    def record_pose(self):
        now = self.get_clock().now().to_msg()
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            ps = PoseStamped()
            ps.header.stamp = t.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = t.transform.rotation

            self.explore_path.append(ps)

            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = t.header.stamp
            path_msg.poses = self.explore_path
            self.explore_path_pub.publish(path_msg)

            if self.home_pose is None:
                self.home_pose = ps  
        except Exception as e:
            self.get_logger().warn(f"TF error while recording pose: {e}")

    def return_home(self):
        self.get_logger().info("Starting return-to-home sequence...")

        reversed_path = list(reversed(self.explore_path))

        for waypoint in reversed_path:
            self.send_goal_pose(waypoint)
            self.return_path.append(waypoint)

            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = self.return_path
            self.return_path_pub.publish(path_msg)

            time.sleep(5) 

    def send_goal_pose(self, pose: PoseStamped):
        goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info(f"Sending goal to x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        goal_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

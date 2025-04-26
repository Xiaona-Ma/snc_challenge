import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from tf2_ros import TransformListener, Buffer
import math
 
 
class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
 
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # record path
        self.explore_path = []
        self.return_path = []
 
        # publish path
        self.explore_path_pub = self.create_publisher(Path, '/explore_path', 10)
        self.return_path_pub = self.create_publisher(Path, '/return_path', 10)
 
        # 
        self.create_subscription(Empty, '/start_return_home', self.start_return_home_cb, 10)
 
        # record postion timer
        self.create_timer(1.0, self.track_position)
 
        # Action client
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
 
    def track_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
 
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
 
            if not self.explore_path or self._is_far_enough(self.explore_path[-1], pose):
                self.explore_path.append(pose)
 
            # publish explored path
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = self.explore_path
            self.explore_path_pub.publish(path_msg)
 
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
 
    def _is_far_enough(self, last_pose, new_pose, threshold=0.15):
        dx = last_pose.pose.position.x - new_pose.pose.position.x
        dy = last_pose.pose.position.y - new_pose.pose.position.y
        return math.hypot(dx, dy) > threshold
 
    def start_return_home_cb(self, msg):
        self.get_logger().info("start return home!way points publishing ready...")
        reversed_path = list(reversed(self.explore_path))
        self.return_path = reversed_path
 
        # publish return home path
        return_path_msg = Path()
        return_path_msg.header.frame_id = 'map'
        return_path_msg.header.stamp = self.get_clock().now().to_msg()
        return_path_msg.poses = self.return_path
        self.return_path_pub.publish(return_path_msg)
 
        self.send_waypoints_goal(self.return_path)
 
    def send_waypoints_goal(self, waypoints):
        # wait Action Server 
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available!')
            return
 
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
 
        self.get_logger().info(f"send {len(waypoints)} way points...")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)
 
    def feedback_cb(self, feedback_msg):
        current_idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'reached points {current_idx}')
 
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal was rejected')
            return
 
        self.get_logger().info('goal acccept,under progress...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)
 
    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'path finished!way points skipped: {result.missed_waypoints}')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
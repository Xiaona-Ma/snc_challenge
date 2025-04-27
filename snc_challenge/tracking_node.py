#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
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
 
        # record the path
        self.explore_path = []
 
        # publish the path
        self.explore_path_pub = self.create_publisher(Path, '/explore_path', 10)
 
        # Trigger
        self.create_subscription(Empty, '/trigger_home', self.start_return_home_cb, 10)
 
        # timer
        self.create_timer(1.0, self.track_position)
 
        # Action client (NavigateToPose)
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
 
        # save the start position
        self.start_pose = None
 
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
 
                # save the first postion as start position
                if self.start_pose is None:
                    self.start_pose = pose
 
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
        self.get_logger().info("start returning home!start goal is ready to send...")
 
        if self.start_pose is None:
            self.get_logger().error("unknown start position, can not returning!")
            return
 
        self.send_home_goal(self.start_pose)
 
    def send_home_goal(self, pose):
        # wait Action Server
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return
 
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
 
        self.get_logger().info(f"return home goal send: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
 
        self._send_goal_future = self.navigate_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)
 
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"moving: distance to goal {feedback.distance_remaining:.2f} meters")
 
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('return home goal was rejected!')
            return
 
        self.get_logger().info('returning home goal is accepting, under progress...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)
 
    def result_cb(self, future):
        result = future.result().result
        status = future.result().status
 
        if status == 4:  # STATUS_ABORTED
            self.get_logger().error('return home failed,being stopped!')
        elif status == 5:  # STATUS_REJECTED
            self.get_logger().error('return home failed,being rejected!')
        else:
            self.get_logger().info('return home success!')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()

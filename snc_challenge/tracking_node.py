#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import quaternion_from_euler
import time

class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking_node')

        self.get_logger().info("Tracking Node Ready!")
            
        self.tf_buffer = Buffer() #TF buffer
        self.tf_listener = TransformListener(self.tf_buffer, self) #TF listener

        self.path_exploring = [] #store exploring path
        self.path_returning = [] #store returning path
        
        #track current position every 0.5s
        self.timer = self.create_timer(0.5, self.track_position)

        self.state='exploring'
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.current_index = 0
        self.get_logger().info('RobotPathTracker node started.')

    def track_position(self):
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform('map', 'base_link', now)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = now.to_msg()
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = trans.transform.rotation

        if self.state == 'exploring':
            self.path_exploring.append(pose)
            self.publish_path(self.path_exploring, self.explore_path_pub)

        elif self.state == 'returning':
            self.path_returning.append(pose)
            self.publish_path(self.path_returning, self.return_path_pub)
    
    def publish_path(self, path_list, publisher):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.poses = path_list
        publisher.publish(msg)
    
    def start_return_to_home(self):
        self.get_logger().info('Starting return-to-home...')
        self.state = 'returning'
        self.path_returning = []
        self.return_waypoints = list(reversed(self.path_exploring))[::5]  
        self.current_index = 0
        self.send_next_goal()
    
    def send_next_goal(self):
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose = self.return_waypoints[self.current_index]
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Sending goal {self.current_index + 1}/{len(self.return_waypoints)}')

        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result
        self.get_logger().info('Goal reached.')
        self.current_index += 1
        time.sleep(1.0)  
        self.send_next_goal()



def main(args = None):
    rclpy.init(args = args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
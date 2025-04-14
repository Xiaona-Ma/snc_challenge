#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf2_ros
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node Ready')

        # --- PARAMETERS ---
        self.side = 'right'               # which wall to follow
        self.d_threshold = 1.5            # obstacle threshold (m)
        self.scan_speed = 0.5             # rad/s for 360° scan
        self.scan_duration = 2 * math.pi / self.scan_speed

        # --- STATE MACHINE ---
        self.state_ = 0                   # 0=find wall,1=turn left,2=follow wall
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
        self.scanning = False

        # --- LASER REGIONS ---
        self.regions_ = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0,
        }

        # --- PUBLISHERS & SUBSCRIBERS ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.path_pub = self.create_publisher(Path, '/path_explore', 10)

        # --- TF2 for path recording ---
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # prepare Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def scan_callback(self, msg: LaserScan):
        # dynamically split scan into 5 regions
        n = len(msg.ranges)
        seg = n // 5
        self.regions_ = {
            'right':  min(min(msg.ranges[0:seg]), 10.0),
            'fright': min(min(msg.ranges[seg:2*seg]), 10.0),
            'front':  min(min(msg.ranges[2*seg:3*seg]), 10.0),
            'fleft':  min(min(msg.ranges[3*seg:4*seg]), 10.0),
            'left':   min(min(msg.ranges[4*seg:5*seg]), 10.0),
        }
        self.take_action()

    def change_state(self, new_state: int):
        if new_state != self.state_:
            self.get_logger().info(
                f'Wall follower → [{new_state}] {self.state_dict_[new_state]}')
            self.state_ = new_state

    def take_action(self):
        r = self.regions_
        msg = Twist()

        # decide state
        if r['front'] > self.d_threshold and r['fleft'] > self.d_threshold and r['fright'] > self.d_threshold:
            self.change_state(0)       # find wall
        elif r['front'] < self.d_threshold:
            self.change_state(1)       # turn left (including any front+side combos)
        elif r['fright'] < self.d_threshold:
            self.change_state(2)       # follow wall
        else:
            self.change_state(0)

        # corner scan when turning left
        if self.state_ == 1 and not self.scanning:
            self.status_pub.publish(String(data='Scanning'))
            self.do_full_rotation()
            self.scanning = False

        # set velocity based on state
        if self.state_ == 0:
            # find wall: move forward + slight turn toward wall
            msg.linear.x = 0.2
            msg.angular.z = -0.3 if self.side == 'right' else 0.3

        elif self.state_ == 1:
            # turn left to clear front obstacle
            msg.angular.z = 0.3

        elif self.state_ == 2:
            # follow wall: straight ahead
            msg.linear.x = 0.5

        # publish cmd_vel
        self.cmd_pub.publish(msg)

        # publish status
        self.status_pub.publish(String(data=self.state_dict_[self.state_].capitalize()))

        # record pose for path
        self.record_pose()

    def do_full_rotation(self):
        """Blocking 360° rotation at scan_speed."""
        self.scanning = True
        twist = Twist()
        twist.angular.z = self.scan_speed if self.side == 'right' else -self.scan_speed

        t_start = self.get_clock().now()
        t_end = t_start + rclpy.duration.Duration(seconds=self.scan_duration)

        while rclpy.ok() and self.get_clock().now() < t_end:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # stop
        self.cmd_pub.publish(Twist())

    def record_pose(self):
        """Lookup map→base_link and append to Path."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
            ps = PoseStamped()
            ps.header.stamp = t.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.position.z = t.transform.translation.z
            ps.pose.orientation = t.transform.rotation

            self.path_msg.header.stamp = ps.header.stamp
            self.path_msg.poses.append(ps)
            self.path_pub.publish(self.path_msg)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import time
from collections import deque
 
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
 
        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
        # parameter setting
        self.linear_speed = 0.25
        self.angular_speed = 0.3
        self.safety_dist = 0.6
        self.side_angle = 60
        self.front_angle = 60
        self.clear_threshold = 1.2
        self.turn_check_angle = 45
 
        # status
        self.front_clear = True
        self.left_clearance = 0.0
        self.right_clearance = 0.0
        self.is_turning = False
        self.turn_direction = 0
 
        # path recording and timer
        self.motion_log = deque()
        self.start_time = time.time()
        self.max_explore_time = 30.0
        self.is_returning = False
 
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("=== robot started ===")
 
    def laser_callback(self, msg):
        def get_sector_min_distance(center_angle, angle_width):
            compensated_angle = center_angle + 180
            start_angle = math.radians(compensated_angle - angle_width/2)
            end_angle = math.radians(compensated_angle + angle_width/2)
            start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
            end_idx = int((end_angle - msg.angle_min) / msg.angle_increment)
            step = 1 if start_idx <= end_idx else -1
            min_dist = float('inf')
            for idx in range(start_idx, end_idx + step, step):
                if 0 <= idx < len(msg.ranges):
                    dist = msg.ranges[idx]
                    if msg.range_min < dist < msg.range_max:
                        min_dist = min(min_dist, dist)
            return min_dist
 
        self.front_clearance = get_sector_min_distance(0, self.front_angle)
        self.left_clearance = get_sector_min_distance(90, self.side_angle)
        self.right_clearance = get_sector_min_distance(-90, self.side_angle)
 
        check_angle = self.turn_check_angle if self.is_turning else self.front_angle
        self.front_clear = get_sector_min_distance(0, check_angle) > self.safety_dist
 
        self.get_logger().info(
            f"status: {'turning' if self.is_turning else 'moving forward'} | "
            f"distance to front: {self.front_clearance:.2f}m | "
            f"turning side: {'left' if self.turn_direction==1 else 'right' if self.turn_direction==-1 else 'none'}",
            throttle_duration_sec=0.5
        )
 
    def control_loop(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        cmd = Twist()
 
        # ========== return home logic ==========
        if elapsed >= self.max_explore_time and not self.is_returning:
            self.get_logger().info("explore ended，start returning home")
            self.is_returning = True
            self.motion_log = deque(reversed(self.motion_log))
            return  
 
        if self.is_returning:
            if self.motion_log:
                past_cmd = self.motion_log.popleft()
                cmd.linear.x = -past_cmd.linear.x
                cmd.angular.z = -past_cmd.angular.z
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info("returned home")
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.destroy_node()
                rclpy.shutdown()
            return
 
        # ========== explore/avoid obstcle==========
        if not self.front_clear and not self.is_turning:
            self.is_turning = True
            self.turn_direction = 1 if self.left_clearance > self.right_clearance else -1
            self.get_logger().warning(f"start {'left' if self.turn_direction==1 else 'right'} avoid obstcle")
        
        if self.is_turning:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * self.turn_direction
            if self.front_clear and self.front_clearance > self.clear_threshold:
                self.is_turning = False
                self.turn_direction = 0
                self.get_logger().info("turning is complete, moving forward")
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
 
        # Record the movement trajectory (only during the exploration phase).
        if elapsed < self.max_explore_time:
            saved_cmd = Twist()
            saved_cmd.linear.x = cmd.linear.x
            saved_cmd.angular.z = cmd.angular.z
            self.motion_log.append(saved_cmd)
 
        self.cmd_pub.publish(cmd)
 
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.get_logger().info("robot has been stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
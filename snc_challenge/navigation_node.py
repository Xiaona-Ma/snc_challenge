import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import String, Empty
from nav_msgs.msg import Path
import tf2_ros
import math
import subprocess

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node Ready')

        # publish detecting the start marker
        self.navigation_started = False
        self.start_sub = self.create_subscription(String, '/start_marker', self.start_callback, 10)
        self.manual_start_sub = self.create_subscription(Empty, '/trigger_start', self.manual_start_callback, 10)

        self.returning_home = False
        self.return_home_sub = self.create_subscription(Empty, 'start_return_home', self.return_home_callback, 10)

        # --- TIMER ---
        self.create_timer(0.3, self.take_action) 

        # --- PARAMETERS ---
        self.wall_min_distance = 0.35     # desired minimum distance to wall (m)
        self.d_threshold = 0.8            # obstacle threshold in front (m)
        self.forward_speed = 0.2          # linear speed
        self.turning_speed = 0.8          # angular speed
        self.lidar_angle_offset = math.pi # set to π if LiDAR is mounted backwards

        # --- STATE ---
        self.state_ = 0
        self.state_dict_ = {
            0: 'move forward',
            1: 'turn right',
            2: 'turn left'
        }

        # --- LASER REGIONS ---
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }

        # Publisher and Subscriber
        self.scan_sub   = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.path_pub   = self.create_publisher(Path, '/path_explore', 10)

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Paths message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'


    def start_callback(self, msg: String):
        if msg.data.lower() == 'start':
            self.get_logger().info('Identified start signal')
            self.navigation_started = True

    def manual_start_callback(self, msg: Empty):
        self.get_logger().info('Identified manual start trigger.')
        self.navigation_started = True


    # Find minimum distance within a sector centered at 'center' with 'width' radians
    def _sector_min(self, msg, center, width):
        amin = msg.angle_min + self.lidar_angle_offset
        ainc = msg.angle_increment
        n = len(msg.ranges)
        idx_c = int((center - amin) / ainc)
        hw = int((width / 2) / abs(ainc))
        lo, hi = max(0, idx_c - hw), min(n, idx_c + hw + 1)
        seg = [r for r in msg.ranges[lo:hi] if msg.range_min < r < msg.range_max]
        return min(seg) if seg else msg.range_max

    # Define the laser scans
    def scan_callback(self, msg: LaserScan):
        self.regions_ = {
            'right':  self._sector_min(msg, -math.pi/2, math.radians(10)),
            'fright': self._sector_min(msg, -math.pi/4, math.radians(10)),
            'front':  self._sector_min(msg,  0.0,       math.radians(10)),
            'fleft':  self._sector_min(msg,  math.pi/4, math.radians(10)),
            'left':   self._sector_min(msg,  math.pi/2, math.radians(10)),
        }


    # Change state (mainly for logs in terminal)
    def change_state(self, new_state: int):
        if new_state != self.state_:
            self.get_logger().info(f"Wall follower → [{new_state}] {self.state_dict_[new_state]}")
            self.state_ = new_state


    # Decide the behaviour from the laser scans
    def take_action(self):
        if self.returning_home:
            # Robot is returning home; disable navigation behavior
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        if not self.navigation_started:
            # Waiting for start signal
            self.get_logger().info('Waiting for start signal...')
            return

        r = self.regions_

        # Shows log of laser scan
        self.get_logger().info(
            f"Regions: right={r['right']:.2f}, fright={r['fright']:.2f}, front={r['front']:.2f}, "
            f"fleft={r['fleft']:.2f}, left={r['left']:.2f}"
    )
        cmd = Twist()

        right      = r['right']
        fright     = r['fright']
        front      = r['front']
        fleft      = r['fleft']
        left       = r['left']

        wall_min = self.wall_min_distance
        

        right_clear       = right > wall_min 
        fright_clear      = fright > wall_min 
        front_clear       = front > self.d_threshold
        fleft_clear       = fleft > wall_min
        front_blocked     = front < self.d_threshold
        right_wall_close  = right <= wall_min
        fright_close      = fright <= wall_min 

        # Right wall following logic

        # STATE 1
        # If the front has space and right side is not near wall, turn right
        if right_clear and fright_clear and (front > 0.3):
            cmd.linear.x = 0.15
            cmd.angular.z = -0.3
            self.change_state(1)
            self.cmd_pub.publish(cmd)
            msg = "Turn Right (path clear)"
            self.status_pub.publish(String(data=msg))
            self.get_logger().info(msg)
            return

        # STATE 2
        # If there is obstacle in the way, turn left
        if front_blocked and fright_close:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turning_speed
            self.change_state(2)
            self.cmd_pub.publish(cmd)
            msg = "Turn Left (obstacle ahead!)"
            self.status_pub.publish(String(data=msg))
            self.get_logger().info(msg)
            return

        # STATE 0
        # If it is within the safe right wall range and there is no obstacle in the way, move forward
        if right_wall_close and fright_close and front_clear:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.change_state(0)
            self.cmd_pub.publish(cmd)
            msg = "Follow Wall Forward"
            self.status_pub.publish(String(data=msg))
            self.get_logger().info(msg)
            self.record_pose()
            return

        # STATE 2
        # If it is too close to the wall it's following, turn left
        if fright_close and front_clear:
            cmd.linear.x = self.forward_speed * 0.8
            cmd.angular.z = self.turning_speed * 0.6
            self.change_state(2)
            self.cmd_pub.publish(cmd)
            msg = "Adjust Left (too close to wall!)"
            self.status_pub.publish(String(data=msg))
            self.get_logger().info(msg)
            return

        # STATE 0
        # Default behaviour 
        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0
        self.change_state(0)
        self.cmd_pub.publish(cmd)
        msg = "Default Forward"
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(msg)
        self.record_pose()


    # Record the current robot pose using tf2 and publish the explored path.
    def record_pose(self):
        now = self.get_clock().now()
        if not self.tf_buffer.can_transform('map', 'base_link', now, timeout=Duration(seconds=1.0)):
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', now)
            ps = PoseStamped()
            ps.header.stamp = t.header.stamp
            ps.header.frame_id = 'map'
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation
            ps.pose = pose
            self.path_msg.header.stamp = ps.header.stamp
            self.path_msg.poses.append(ps)
            self.path_pub.publish(self.path_msg)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            pass

    def return_home_callback(self, msg: Empty):
        self.get_logger().info('Return home signal received. Disabling navigation.')
        self.returning_home = True
        self.navigation_started = False
        # Stop the robot immediately
        self.cmd_pub.publish(Twist())
        self.cmd_pub.publish(stop_cmd)
        time.sleep(1.0)
        self.status_pub.publish(String(data="Exploration stopped: returning home"))

        
    def destroy_node(self):
        self.get_logger().info("Navigation node shutting down... Saving map.")
        try:
            subprocess.run([
                "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "auto_saved_map"
            ], check=True)
            self.get_logger().info("Map saved successfully as 'auto_saved_map'.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to save map: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
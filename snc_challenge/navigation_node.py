import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf2_ros
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node Ready')

        # Timer
        self.create_timer(0.3, self.take_action) 

        # Parameter
        # Minimal distance from right wall
        self.wall_min_distance = 0.35
        # Minimal obstacle threshold
        self.d_threshold = 0.8
        self.forward_speed = 0.2
        self.turning_speed = 0.8
        # Correction for lidar angle (180 degrees)
        self.lidar_angle_offset = math.pi

        # States
        self.state_dict_ = {
            0: 'move forward',
            1: 'turn',
            2: 'adjust left'
        }

        # Laser regions
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }

        # Publisher and Subscribers
        self.scan_sub   = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.path_pub   = self.create_publisher(Path, '/path_explore', 10)

        # TF2 buffer and listener for robot pose
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Path messages
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    # Find minimum distance within a sector centered at 'center' with 'width' radians.
    def _sector_min(self, msg, center, width):
        amin = msg.angle_min + self.lidar_angle_offset
        ainc = msg.angle_increment
        n = len(msg.ranges)
        idx_c = int((center - amin) / ainc)
        hw = int((width / 2) / abs(ainc))
        lo, hi = max(0, idx_c - hw), min(n, idx_c + hw + 1)
        seg = [r for r in msg.ranges[lo:hi] if msg.range_min < r < msg.range_max]
        return min(seg) if seg else msg.range_max

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
        front_blocked     = front < self.d_threshold
        right_wall_close  = right <= wall_min
        fright_close      = fright <= wall_min 


        # Right Wall Following Logic
        
        # STATE 1
        # If the front has space and right side is not near wall, turn right
        if right_clear and fright_clear and (front > 0.3):
            cmd.linear.x = 0.15
            cmd.angular.z = -0.3
            self.change_state(1)
            self.cmd_pub.publish(cmd)
            msg = "Turn Right"
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
        # log activity
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


# Main
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

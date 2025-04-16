#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Float32MultiArray
import tf2_geometry_msgs 
import cv2
import numpy as np

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # initialize parameters and tools
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the output of find_object_2d
        self.create_subscription(Float32MultiArray, '/objects', self.object_detection_callback, 10)
        self.last_detections = []  # Store the latest test results

        # Subscribe to RGB images
        self.create_subscription(Image, '/image_topic_repeat', self.image_callback, 10)
        
        # Subscribe to Depth Images and Camera Insights
        self.create_subscription(Image, '/depth_topic_repeat', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info_topic_repeat', self.camera_info_callback, 10)
        
        # Issuance of detected hazard markers
        self.marker_pub = self.create_publisher(MarkerArray, '/hazards', 10)

        self.start_marker_pub = self.create_publisher(PointStamped, '/start_marker', 10)
        
        # Storing current data
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
        self.get_logger().info("Detection Node Ready")
    

    def object_detection_callback(self, msg):

        self.last_detections = []
        data = msg.data
        for i in range(0, len(data), 7):
            obj = {
                "id": int(data[i]),      # Object ID
                "width": data[i+1],      # Width in image
                "height": data[i+2],     # Height in image
                "bbox": [
                    data[i+3],           # x1
                    data[i+4],           # y1
                    data[i+5],           # x2
                    data[i+6]            # y2
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
            self.get_logger().info("Camera internal parameters have been acquired")


    def depth_callback(self, msg):
        # Store depth images (add filter processing)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_depth = cv2.medianBlur(depth_image, 5)
            self.get_logger().info("Depth images have been acquired")
        except Exception as e:
            self.get_logger().error(f"Deep Image Processing Failure: {e}")


    def image_callback(self, msg):
        
        # Check that the depth image and internal camera parameters are available
        if self.current_depth is None or self.camera_info is None:
            self.get_logger().warn("Waiting for depth data or internal camera parameters...")
            return
            
        try:
            # Converting RGB images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.last_detections:
                self.get_logger().warn("No objects detected")
                return
            
            markers = MarkerArray()
            for obj in self.last_detections:

                # Mapping find_object_2d's IDs to hazard IDs
                marker_name = self._get_marker_name_by_id(obj["id"])

                # If the marker is not in the defined list, skip it
                if marker_name not in self.marker_ids:
                    continue
                
                # If the marker is "start", process it separately
                if marker_name == "start":
                    self._process_start_marker(obj, msg.header)
                    continue
                
                # If the marker is not "start", process it normally
                else:
                    # Calculate the centre point of the bounding box
                    cx = int((obj["bbox"][0] + obj["bbox"][2]) / 2)
                    cy = int((obj["bbox"][1] + obj["bbox"][3]) / 2)
                    
                    # Obtaining depth values
                    depth_roi = self.current_depth[cy-10:cy+10, cx-10:cx+10]
                    depth_val = np.nanmedian(depth_roi)
                    if depth_val <= 0 or depth_val > 5.0:  # Effective range check
                        continue
                
                    # Coordinate system conversion
                    try:
                        # Conversion from pixel to camera coordinate system
                        point_camera = PointStamped()
                        point_camera.header = msg.header
                        point_camera.point.x = (cx - self.camera_info['cx']) * depth_val / self.camera_info['fx']
                        point_camera.point.y = (cy - self.camera_info['cy']) * depth_val / self.camera_info['fy']
                        point_camera.point.z = depth_val
                        
                        # Convert to map coordinate system
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            msg.header.frame_id,  # camera_link
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
                        
                        # Multi-frame verification (to prevent transient misdetection)
                        marker_id = self.marker_ids[obj["id"]]
                        if marker_id not in self.detection_history:
                            self.detection_history[marker_id] = []
                        self.detection_history[marker_id].append(point_map.point)
                        
                        # Posted only when the same position is detected in 3 consecutive frames
                        if len(self.detection_history[marker_id]) >= 3:
                            avg_point = self._calculate_average_point(self.detection_history[marker_id])
                            
                            # Creare Marker
                            marker = Marker()
                            marker.header.frame_id = "map"
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.id = marker_id
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            marker.pose.position.x = avg_point.x
                            marker.pose.position.y = avg_point.y
                            marker.pose.position.z = 0.0  # Assuming the marker is on the ground
                            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
                            marker.color.r = 1.0
                            marker.color.a = 0.8
                            markers.markers.append(marker)
                            
                            # Clear History
                            self.detection_history[marker_id] = []
                            
                    except TransformException as e:
                        self.get_logger().warn(f"Coordinate transformation failure: {e}")
                        continue
            
            # Post all valid tags
            if len(markers.markers) > 0:
                self.marker_pub.publish(markers)
                self.get_logger().info(f"{len(markers.markers)} tags have been issued.")
                
        except Exception as e:
            self.get_logger().error(f"Detecting process anomalies: {e}")


    def _process_start_marker(self, obj, header):
        # Process the start marker separately

        try:
            # computational centre point of the bounding box
            cx = int((obj["bbox"][0] + obj["bbox"][2]) / 2)
            cy = int((obj["bbox"][1] + obj["bbox"][3]) / 2)
            
            # Get depth value
            depth_roi = self.current_depth[cy-10:cy+10, cx-10:cx+10]
            depth_val = np.nanmedian(depth_roi)
            
            # Coordinate system conversion
            point_camera = PointStamped()
            point_camera.header = header
            point_camera.point.x = (cx - self.camera_info['cx']) * depth_val / self.camera_info['fx']
            point_camera.point.y = (cy - self.camera_info['cy']) * depth_val / self.camera_info['fy']
            point_camera.point.z = depth_val
            
            # Convert to map coordinate system
            transform = self.tf_buffer.lookup_transform(
                'map',
                header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            # Post Start Marker Location
            start_msg = PointStamped()
            start_msg.header.frame_id = "map"
            start_msg.header.stamp = self.get_clock().now().to_msg()
            start_msg.point = point_map.point
            self.start_marker_pub.publish(start_msg)
            self.get_logger().info("Start Marker detected and published!")
            
        except Exception as e:
            self.get_logger().error(f"Start Marker process failure: {e}")


    def _calculate_average_point(self, points):
        # Calculate the average point of multiple points
        x = sum([p.x for p in points]) / len(points)
        y = sum([p.y for p in points]) / len(points)
        return PointStamped().point(x=x, y=y, z=0.0)
    
   
    def _get_marker_name_by_id(self, find_object_id):
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
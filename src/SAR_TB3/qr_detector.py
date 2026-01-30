#!/usr/bin/env python3
"""
QR Code Detection Node for Autonomous Exploration
Detects QR codes using YOLOv8, calculates actual QR code position in world using
camera geometry and LiDAR data, saves images and logs positions in map frame.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np
from pathlib import Path

# TF2 for transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector')
        
        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('save_dir', '')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('lidar_topic', '/scan')
        
        model_path = self.get_parameter('model_path').value
        save_dir = self.get_parameter('save_dir').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.save_interval = self.get_parameter('save_interval').value
        camera_topic = self.get_parameter('camera_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        
        # Default model path if not provided
        if not model_path:
            pkg_dir = Path(__file__).parent.parent
            model_path = str(pkg_dir / 'resource' / 'qr_best.pt')
        
        # Default save directory
        if not save_dir:
            pkg_dir = Path(__file__).parent.parent
            save_dir = str(pkg_dir / 'qr_detections')
        
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize YOLO model
        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed! Run: pip install ultralytics')
            raise ImportError('ultralytics package required')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(f'QR detection model not found: {model_path}')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded successfully')
        
        # CV Bridge
        self.br = CvBridge()
        
        # TF2 for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera parameters for TurtleBot3 Waffle Pi (Raspberry Pi Camera v2)
        # Approximate values - adjust based on actual calibration
        self.camera_fx = 320.0  # focal length x (pixels)
        self.camera_fy = 320.0  # focal length y (pixels)
        self.camera_cx = 320.0  # principal point x (center of 640x480 image)
        self.camera_cy = 240.0  # principal point y
        self.image_width = 640
        self.image_height = 480
        
        # LiDAR data for distance measurement
        self.latest_scan = None
        
        # Timing for saving
        self.last_save_time = 0.0
        self.detection_count = 0
        
        # Publisher for detection events
        self.detection_pub = self.create_publisher(String, '/qr_detections', 10)
        
        # Subscribe to LiDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.scan_callback,
            10
        )
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Create mission report file
        self.report_path = self.save_dir / 'qr_detection_log.txt'
        with open(self.report_path, 'w') as f:
            f.write(f"QR Code Detection Mission Log\n")
            f.write(f"Started: {time.ctime()}\n")
            f.write(f"Camera: fx={self.camera_fx}, fy={self.camera_fy}\n")
            f.write(f"=" * 60 + "\n\n")
        
        self.get_logger().info(f'QR Detector initialized')
        self.get_logger().info(f'Saving detections to: {self.save_dir}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Save interval: {self.save_interval}s')
    
    def scan_callback(self, msg):
        """Store latest LiDAR scan"""
        self.latest_scan = msg
    
    def get_distance_to_qr(self, center_x, center_y):
        """
        Estimate distance to QR code using LiDAR scan data.
        Maps the image center position to approximate LiDAR angle.
        """
        if self.latest_scan is None:
            return None
        
        # Convert image x position to angular direction
        # Camera FOV is approximately 62 degrees horizontal
        camera_fov = 62.0 * np.pi / 180.0  # radians
        
        # Calculate angle offset from center
        normalized_x = (center_x - self.camera_cx) / self.image_width
        angle_offset = normalized_x * camera_fov
        
        # LiDAR angle - camera points forward (0 degrees)
        # LiDAR typically starts from back (180 degrees) and goes CCW
        # Adjust based on your robot's camera-lidar alignment
        lidar_angle = -angle_offset  # Negative because image x increases to right
        
        # Convert to LiDAR index
        angle_min = self.latest_scan.angle_min
        angle_max = self.latest_scan.angle_max
        angle_increment = self.latest_scan.angle_increment
        
        # Normalize angle to lidar range
        if lidar_angle < angle_min:
            lidar_angle = angle_min
        if lidar_angle > angle_max:
            lidar_angle = angle_max
        
        index = int((lidar_angle - angle_min) / angle_increment)
        
        # Clamp index
        if index < 0:
            index = 0
        if index >= len(self.latest_scan.ranges):
            index = len(self.latest_scan.ranges) - 1
        
        # Get distance from LiDAR (average nearby readings for robustness)
        scan_window = 5
        start_idx = max(0, index - scan_window)
        end_idx = min(len(self.latest_scan.ranges), index + scan_window)
        
        valid_ranges = []
        for i in range(start_idx, end_idx):
            r = self.latest_scan.ranges[i]
            if r > self.latest_scan.range_min and r < self.latest_scan.range_max:
                valid_ranges.append(r)
        
        if valid_ranges:
            distance = np.median(valid_ranges)
            return distance
        
        return None
    
    def calculate_qr_position_in_camera_frame(self, box, distance):
        """
        Calculate 3D position of QR code in camera frame using bounding box and distance.
        """
        # Get bounding box center
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # Convert pixel coordinates to normalized camera coordinates
        # Using pinhole camera model
        x_norm = (center_x - self.camera_cx) / self.camera_fx
        y_norm = (center_y - self.camera_cy) / self.camera_fy
        
        # Calculate 3D position in camera frame
        # Camera frame: x=right, y=down, z=forward
        z_cam = distance
        x_cam = x_norm * z_cam
        y_cam = y_norm * z_cam
        
        return x_cam, y_cam, z_cam
    
    def transform_to_map_frame(self, x_cam, y_cam, z_cam, image_timestamp):
        """
        Transform QR code position from camera frame to map frame.
        Uses the timestamp from the image message for proper TF lookup.
        """
        try:
            # Create point in camera frame with image timestamp
            point_camera = PointStamped()
            point_camera.header.frame_id = 'camera_rgb_optical_frame'
            point_camera.header.stamp = image_timestamp
            point_camera.point.x = x_cam
            point_camera.point.y = y_cam
            point_camera.point.z = z_cam
            
            # Transform to map frame
            point_map = self.tf_buffer.transform(
                point_camera,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            return (point_map.point.x, 
                   point_map.point.y, 
                   point_map.point.z, 
                   True)
                   
        except (TransformException, Exception) as e:
            self.get_logger().warn(f'Transform failed: {e}', throttle_duration_sec=2.0)
            return 0.0, 0.0, 0.0, False
    
    def get_qr_world_position(self, box, image_timestamp):
        """
        Calculate actual QR code position in world coordinates.
        Uses bounding box for direction and LiDAR for distance.
        """
        # Get bounding box center for direction
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # Get distance from LiDAR
        distance = self.get_distance_to_qr(center_x, center_y)
        
        if distance is None or distance < 0.1 or distance > 10.0:
            # Fallback: estimate from bounding box size
            # Assume QR code is ~0.2m across, estimate distance from apparent size
            box_width = x2 - x1
            if box_width > 10:
                # Rough approximation: distance = (real_size * focal_length) / pixel_size
                estimated_distance = (0.2 * self.camera_fx) / box_width
                distance = estimated_distance
                self.get_logger().debug(f'Using estimated distance: {distance:.2f}m')
            else:
                return 0.0, 0.0, 0.0, False
        
        # Calculate 3D position in camera frame
        x_cam, y_cam, z_cam = self.calculate_qr_position_in_camera_frame(box, distance)
        
        # Transform to map frame
        x_map, y_map, z_map, success = self.transform_to_map_frame(x_cam, y_cam, z_cam, image_timestamp)
        
        return x_map, y_map, z_map, success
    
    def image_callback(self, msg):
        """Process camera images for QR detection"""
        current_time = time.time()
        
        # Store image timestamp for TF lookups
        image_timestamp = msg.header.stamp
        
        # Convert ROS image to OpenCV
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Run YOLO detection
        results = self.model(frame, verbose=False)
        
        # Process detections
        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0])
                
                # Check confidence and time interval
                if confidence > self.confidence_threshold and \
                   (current_time - self.last_save_time) > self.save_interval:
                    
                    # Calculate actual QR code position in world
                    x, y, z, success = self.get_qr_world_position(box, image_timestamp)
                    
                    # Only save if position is successfully determined
                    if not success:
                        self.get_logger().debug(f'QR detected but position unavailable, skipping save')
                        continue
                    
                    location_text = f"QR at X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}"
                    
                    # Increment detection counter
                    self.detection_count += 1
                    
                    # Log detection
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    log_msg = (
                        f"Detection #{self.detection_count}\n"
                        f"  Time: {time.ctime()}\n"
                        f"  QR Code Position: {location_text}\n"
                        f"  Confidence: {confidence:.3f}\n"
                    )
                    
                    self.get_logger().info(f"📍 {location_text} (conf: {confidence:.3f})")
                    
                    # Save to log file
                    with open(self.report_path, 'a') as f:
                        f.write(log_msg + "\n")
                    
                    # Save image with bounding box
                    annotated_frame = frame.copy()
                    
                    # Draw bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Add text overlay
                    text = f"QR Code (conf: {confidence:.2f})"
                    cv2.putText(annotated_frame, text, (x1, y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(annotated_frame, location_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    
                    # Save image with QR code position in filename
                    filename = f"qr_det_{self.detection_count:03d}_QRx{x:.2f}_QRy{y:.2f}_{timestamp}.jpg"
                    
                    image_path = self.save_dir / filename
                    cv2.imwrite(str(image_path), annotated_frame)
                    
                    self.get_logger().info(f"💾 Saved: {filename}")
                    
                    # Publish detection event
                    msg = String()
                    msg.data = f"{self.detection_count},{x:.2f},{y:.2f},{z:.2f},{confidence:.3f},{timestamp}"
                    self.detection_pub.publish(msg)
                    
                    # Update last save time
                    self.last_save_time = current_time


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QRDetectorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Frontier Detection Node for Autonomous Exploration.
Detects unexplored regions (frontiers) in SLAM-generated occupancy grid.
Publishes frontier points as potential navigation goals.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import cv2
from scipy.ndimage import label


class FrontierDetector(Node):
    """
    Detects frontier cells (boundary between known free space and unknown regions).
    Clusters frontiers and publishes the best exploration targets.
    """
    
    def __init__(self):
        super().__init__('frontier_detector')
        
        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('min_frontier_size', 5)  # Minimum cells for valid frontier
        self.declare_parameter('frontier_search_radius', 10.0)  # meters
        self.declare_parameter('update_frequency', 2.0)  # Hz
        
        self.map_topic = self.get_parameter('map_topic').value
        self.robot_frame = self.get_parameter('robot_base_frame').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.search_radius = self.get_parameter('frontier_search_radius').value
        
        # QoS for map subscription (transient local for map persistence)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )
        
        # Publishers
        self.frontier_pub = self.create_publisher(
            PoseStamped,
            '/frontier_goal',
            10
        )
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )
        
        # State
        self.current_map = None
        self.map_data = None
        self.map_info = None
        self.frontiers = []
        
        # Timer for periodic frontier detection
        update_period = 1.0 / self.get_parameter('update_frequency').value
        self.timer = self.create_timer(update_period, self.detect_frontiers)
        
        self.get_logger().info('Frontier Detector initialized')
    
    def map_callback(self, msg):
        """Store the latest map."""
        self.current_map = msg
        self.map_info = msg.info
        
        # Convert map data to numpy array
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
    def is_frontier_cell(self, grid, x, y):
        """
        Check if a cell is a frontier cell.
        A frontier cell is:
        - Free space (0)
        - Adjacent to unknown space (-1)
        """
        height, width = grid.shape
        
        # Check if cell is free
        if grid[y, x] != 0:
            return False
        
        # Check 8-connectivity neighbors for unknown cells
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                # Check bounds
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == -1:  # Unknown
                        return True
        
        return False
    
    def detect_frontiers(self):
        """Main frontier detection algorithm."""
        if self.map_data is None:
            return
        
        height, width = self.map_data.shape
        
        # Create binary frontier map
        frontier_map = np.zeros((height, width), dtype=np.uint8)
        
        # Find all frontier cells
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.is_frontier_cell(self.map_data, x, y):
                    frontier_map[y, x] = 1
        
        if frontier_map.sum() == 0:
            self.get_logger().info('No frontiers detected')
            return
        
        # Cluster frontiers using connected components
        labeled_frontiers, num_clusters = label(frontier_map)
        
        # Extract frontier clusters
        clusters = []
        for cluster_id in range(1, num_clusters + 1):
            cluster_points = np.argwhere(labeled_frontiers == cluster_id)
            
            # Filter small clusters
            if len(cluster_points) < self.min_frontier_size:
                continue
            
            # Compute centroid in grid coordinates
            centroid_y, centroid_x = cluster_points.mean(axis=0)
            
            # Convert to world coordinates
            world_x = centroid_x * self.map_info.resolution + self.map_info.origin.position.x
            world_y = centroid_y * self.map_info.resolution + self.map_info.origin.position.y
            
            clusters.append({
                'centroid': (world_x, world_y),
                'size': len(cluster_points),
                'points': cluster_points
            })
        
        self.frontiers = clusters
        
        if clusters:
            # Select best frontier (largest cluster for now)
            best_frontier = max(clusters, key=lambda c: c['size'])
            
            # Publish as navigation goal
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.current_map.header.frame_id
            goal.pose.position.x = best_frontier['centroid'][0]
            goal.pose.position.y = best_frontier['centroid'][1]
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0  # No specific orientation
            
            self.frontier_pub.publish(goal)
            
            # Visualize frontiers
            self.publish_frontier_markers(clusters, best_frontier)
            
            self.get_logger().info(
                f'Detected {len(clusters)} frontier clusters. '
                f'Best frontier at ({best_frontier["centroid"][0]:.2f}, {best_frontier["centroid"][1]:.2f}), '
                f'size: {best_frontier["size"]}'
            )
    
    def publish_frontier_markers(self, clusters, best_frontier):
        """Publish visualization markers for frontiers."""
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = self.current_map.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = cluster['centroid'][0]
            marker.pose.position.y = cluster['centroid'][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Highlight best frontier in green, others in yellow
            if cluster == best_frontier:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            
            marker_array.markers.append(marker)
        
        self.frontier_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

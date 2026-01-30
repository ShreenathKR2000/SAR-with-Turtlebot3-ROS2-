#!/usr/bin/env python3
"""
Multi-Robot Map Fusion and Exploration Coordinator

This node runs on a central PC and:
1. Subscribes to maps from multiple robots (robot1/map, robot2/map)
2. Fuses them into a global occupancy grid in 'world' frame
3. Detects frontiers globally
4. Assigns frontiers to robots to avoid overlap
5. Publishes goals to each robot's Nav2 stack

Architecture:
- Each robot publishes: map, pose, status
- Central PC fuses maps and coordinates exploration
- Robots receive goals and execute locally with SLAM+Nav2+TD3
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import numpy as np
from scipy.ndimage import label
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class MapFusionCoordinator(Node):
    """Central coordinator for multi-robot exploration"""
    
    def __init__(self):
        super().__init__('map_fusion_coordinator')
        
        # Parameters
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('fusion_rate', 1.0)  # Hz
        self.declare_parameter('frontier_min_size', 10)
        self.declare_parameter('frontier_threshold', 0.3)
        self.declare_parameter('min_robot_distance', 2.0)  # Minimum distance between robot goals
        
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.world_frame = self.get_parameter('world_frame').value
        self.fusion_rate = self.get_parameter('fusion_rate').value
        self.frontier_min_size = self.get_parameter('frontier_min_size').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Storage for robot data
        self.robot_maps = {}  # {namespace: OccupancyGrid}
        self.robot_poses = {}  # {namespace: PoseStamped}
        self.robot_status = {}  # {namespace: str}
        self.assigned_goals = {}  # {namespace: Point}
        
        # Initialize robot-specific subscriptions and publishers
        for robot_ns in self.robot_namespaces:
            # Subscribe to robot's map
            self.create_subscription(
                OccupancyGrid,
                f'/{robot_ns}/map',
                lambda msg, ns=robot_ns: self.map_callback(msg, ns),
                qos_profile
            )
            
            # Subscribe to robot's odometry
            self.create_subscription(
                Odometry,
                f'/{robot_ns}/odom',
                lambda msg, ns=robot_ns: self.odom_callback(msg, ns),
                10
            )
            
            # Subscribe to robot's status
            self.create_subscription(
                String,
                f'/{robot_ns}/exploration_status',
                lambda msg, ns=robot_ns: self.status_callback(msg, ns),
                10
            )
            
            # Publisher for robot's goal
            setattr(self, f'goal_pub_{robot_ns}', 
                   self.create_publisher(PoseStamped, f'/{robot_ns}/frontier_goal', 10))
            
            # Initialize storage
            self.robot_maps[robot_ns] = None
            self.robot_poses[robot_ns] = None
            self.robot_status[robot_ns] = "IDLE"
            self.assigned_goals[robot_ns] = None
        
        # Global map publisher
        self.global_map_pub = self.create_publisher(
            OccupancyGrid, '/global_map', qos_profile
        )
        
        # Frontier visualization
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, '/global_frontiers', 10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/coordinator_status', 10
        )
        
        # Timer for map fusion and coordination
        self.timer = self.create_timer(1.0 / self.fusion_rate, self.coordination_cycle)
        
        self.global_map = None
        self.get_logger().info(f'Map Fusion Coordinator initialized for robots: {self.robot_namespaces}')
    
    def map_callback(self, msg, robot_ns):
        """Store robot's local map"""
        self.robot_maps[robot_ns] = msg
    
    def odom_callback(self, msg, robot_ns):
        """Store robot's pose"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.robot_poses[robot_ns] = pose
    
    def status_callback(self, msg, robot_ns):
        """Store robot's exploration status"""
        self.robot_status[robot_ns] = msg.data
    
    def coordination_cycle(self):
        """Main coordination loop"""
        try:
            # Step 1: Fuse maps
            self.fuse_maps()
            
            # Step 2: Detect global frontiers
            if self.global_map is not None:
                frontiers = self.detect_global_frontiers()
                
                # Step 3: Assign frontiers to robots
                if frontiers:
                    self.assign_frontiers(frontiers)
                    
                # Publish status
                status = f"Active robots: {len([s for s in self.robot_status.values() if s != 'IDLE'])}, " \
                        f"Frontiers: {len(frontiers)}"
                self.status_pub.publish(String(data=status))
        
        except Exception as e:
            self.get_logger().error(f'Coordination error: {str(e)}')
    
    def fuse_maps(self):
        """Fuse multiple robot maps into global map in world frame"""
        # For simplicity, we'll use a basic approach:
        # 1. Create a global grid that encompasses all robot maps
        # 2. Transform each robot's map to world frame
        # 3. Merge using "occupied dominates" rule
        
        valid_maps = {ns: m for ns, m in self.robot_maps.items() if m is not None}
        
        if not valid_maps:
            return
        
        # For initial implementation, use first robot's map as reference
        # In production, this should compute a proper bounding box
        reference_ns = list(valid_maps.keys())[0]
        reference_map = valid_maps[reference_ns]
        
        # Create global map with same properties
        global_map = OccupancyGrid()
        global_map.header.frame_id = self.world_frame
        global_map.header.stamp = self.get_clock().now().to_msg()
        global_map.info = reference_map.info
        
        # Initialize with unknown
        width = reference_map.info.width
        height = reference_map.info.height
        global_data = np.full(width * height, -1, dtype=np.int8)
        
        # Merge all maps
        for robot_ns, robot_map in valid_maps.items():
            try:
                # For now, assume all maps are in compatible frames
                # In production, transform using TF
                robot_data = np.array(robot_map.data, dtype=np.int8)
                
                # Merge rule: occupied (100) dominates, then free (0), then unknown (-1)
                occupied_mask = robot_data == 100
                free_mask = (robot_data == 0) & (global_data == -1)
                
                global_data[occupied_mask] = 100
                global_data[free_mask] = 0
                
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform map from {robot_ns}: {ex}')
                continue
        
        global_map.data = global_data.tolist()
        self.global_map = global_map
        self.global_map_pub.publish(global_map)
    
    def detect_global_frontiers(self):
        """Detect frontiers in global map"""
        if self.global_map is None:
            return []
        
        width = self.global_map.info.width
        height = self.global_map.info.height
        resolution = self.global_map.info.resolution
        origin = self.global_map.info.origin
        
        grid = np.array(self.global_map.data).reshape((height, width))
        
        # Find frontier cells (free cells adjacent to unknown)
        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        
        frontier_mask = np.zeros_like(grid, dtype=bool)
        
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if free_mask[i, j]:
                    # Check 8-connected neighbors for unknown
                    neighbors = grid[i-1:i+2, j-1:j+2]
                    if np.any(neighbors == -1):
                        frontier_mask[i, j] = True
        
        # Label connected components
        labeled, num_features = label(frontier_mask)
        
        # Extract frontier clusters
        frontiers = []
        for region_id in range(1, num_features + 1):
            region = (labeled == region_id)
            size = np.sum(region)
            
            if size < self.frontier_min_size:
                continue
            
            # Compute centroid
            coords = np.argwhere(region)
            centroid_grid = coords.mean(axis=0)
            
            # Convert to world coordinates
            centroid_x = origin.position.x + (centroid_grid[1] + 0.5) * resolution
            centroid_y = origin.position.y + (centroid_grid[0] + 0.5) * resolution
            
            frontiers.append({
                'id': region_id,
                'size': size,
                'centroid': Point(x=centroid_x, y=centroid_y, z=0.0),
                'cells': coords
            })
        
        # Publish frontier markers
        self.publish_frontier_markers(frontiers)
        
        return frontiers
    
    def publish_frontier_markers(self, frontiers):
        """Visualize detected frontiers"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "global_frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = frontier['centroid']
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        self.frontier_marker_pub.publish(marker_array)
    
    def assign_frontiers(self, frontiers):
        """Assign frontiers to robots to maximize coverage"""
        available_robots = []
        
        # Find robots that need new goals
        for robot_ns in self.robot_namespaces:
            status = self.robot_status.get(robot_ns, "IDLE")
            if status in ["IDLE", "Frontier reached - searching for next frontier"]:
                if self.robot_poses.get(robot_ns) is not None:
                    available_robots.append(robot_ns)
        
        if not available_robots:
            return
        
        # Compute costs for each robot-frontier pair
        assignments = self.compute_assignments(available_robots, frontiers)
        
        # Publish goals
        for robot_ns, frontier in assignments.items():
            goal = PoseStamped()
            goal.header.frame_id = self.world_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position = frontier['centroid']
            goal.pose.orientation.w = 1.0
            
            # Get robot-specific publisher
            pub = getattr(self, f'goal_pub_{robot_ns}')
            pub.publish(goal)
            
            self.assigned_goals[robot_ns] = frontier['centroid']
            
            self.get_logger().info(
                f'Assigned frontier {frontier["id"]} to {robot_ns} at '
                f'({frontier["centroid"].x:.2f}, {frontier["centroid"].y:.2f})'
            )
    
    def compute_assignments(self, robots, frontiers):
        """Compute optimal robot-frontier assignments"""
        assignments = {}
        remaining_frontiers = frontiers.copy()
        
        # Greedy assignment: for each robot, pick closest unassigned frontier
        for robot_ns in robots:
            robot_pose = self.robot_poses[robot_ns]
            
            if not remaining_frontiers:
                break
            
            # Find closest frontier
            min_dist = float('inf')
            best_frontier = None
            
            for frontier in remaining_frontiers:
                dist = self.compute_distance(
                    robot_pose.pose.position,
                    frontier['centroid']
                )
                
                # Check if too close to other robot's goal
                too_close = False
                for other_ns, other_goal in self.assigned_goals.items():
                    if other_ns != robot_ns and other_goal is not None:
                        goal_dist = self.compute_distance(
                            frontier['centroid'],
                            other_goal
                        )
                        if goal_dist < self.min_robot_distance:
                            too_close = True
                            break
                
                if not too_close and dist < min_dist:
                    min_dist = dist
                    best_frontier = frontier
            
            if best_frontier:
                assignments[robot_ns] = best_frontier
                remaining_frontiers.remove(best_frontier)
        
        return assignments
    
    def compute_distance(self, point1, point2):
        """Euclidean distance between two points"""
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return np.sqrt(dx**2 + dy**2)


def main(args=None):
    rclpy.init(args=args)
    node = MapFusionCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

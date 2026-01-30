#!/usr/bin/env python3
"""
Packaged Frontier Detector (copied from ai_rescuebot)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.ndimage import label


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('frontier_search_radius', 10.0)
        self.declare_parameter('update_frequency', 2.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.frontier_pub = self.create_publisher(PoseStamped, '/frontier_goal', 10)
        self.frontier_marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        self.current_map = None
        self.map_data = None
        self.map_info = None

        update_period = 1.0 / self.get_parameter('update_frequency').value
        self.timer = self.create_timer(update_period, self.detect_frontiers)

        self.get_logger().info('Packaged Frontier Detector initialized')

    def map_callback(self, msg):
        self.current_map = msg
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def is_frontier_cell(self, grid, x, y):
        if grid[y, x] != 0:
            return False
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                    if grid[ny, nx] == -1:
                        return True
        return False

    def detect_frontiers(self):
        if self.map_data is None:
            return
        height, width = self.map_data.shape
        frontier_map = np.zeros((height, width), dtype=np.uint8)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.is_frontier_cell(self.map_data, x, y):
                    frontier_map[y, x] = 1
        if frontier_map.sum() == 0:
            return
        labeled_frontiers, num_clusters = label(frontier_map)
        clusters = []
        for cluster_id in range(1, num_clusters + 1):
            cluster_points = np.argwhere(labeled_frontiers == cluster_id)
            if len(cluster_points) < self.min_frontier_size:
                continue
            centroid_y, centroid_x = cluster_points.mean(axis=0)
            world_x = centroid_x * self.map_info.resolution + self.map_info.origin.position.x
            world_y = centroid_y * self.map_info.resolution + self.map_info.origin.position.y
            clusters.append({'centroid': (world_x, world_y), 'size': len(cluster_points)})
        if not clusters:
            return
        best_frontier = max(clusters, key=lambda c: c['size'])
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.current_map.header.frame_id
        goal.pose.position.x = best_frontier['centroid'][0]
        goal.pose.position.y = best_frontier['centroid'][1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.frontier_pub.publish(goal)
        self.publish_frontier_markers(clusters, best_frontier)

    def publish_frontier_markers(self, clusters, best_frontier):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cluster['centroid'][0]
            marker.pose.position.y = cluster['centroid'][1]
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            if cluster == best_frontier:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
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

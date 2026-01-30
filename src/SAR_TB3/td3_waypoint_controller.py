#!/usr/bin/env python3
"""
Packaged TD3 waypoint controller adapted from ai_rescuebot.
Loads model from default session path if none provided via CLI or param.
"""
import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import torch

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan

# Try to import local td3_agent from package
try:
    from .td3_agent import TD3
except Exception:
    from td3_agent import TD3


DEFAULT_MODEL = '/root/turtlebot3_ws/checkpoints/td3_sar/session_20260122_174855/model_ep1700.pt'


class TD3WaypointController(Node):
    def __init__(self, model_path=None):
        super().__init__('td3_waypoint_controller')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('waypoint_spacing', 1.0)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.84)
        self.declare_parameter('scan_bins', 24)

        if model_path is None:
            model_path = self.get_parameter('model_path').value

        # Fallback to default model path if not provided
        if not model_path:
            model_path = DEFAULT_MODEL

        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.scan_bins = self.get_parameter('scan_bins').value

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.td3_agent = None

        if os.path.exists(model_path):
            self.load_model(model_path)
        else:
            self.get_logger().warn(f'Model not found at {model_path}')

        # State
        self.current_pose = None
        self.current_scan = None
        self.current_waypoint = None
        self.waypoints = []
        self.waypoint_index = 0

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_waypoint_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Packaged TD3 Waypoint Controller initialized')

    def load_model(self, model_path):
        try:
            self.td3_agent = TD3(device=self.device)
            # Delegate loading and architecture inference to TD3.load_model
            self.td3_agent.load_model(model_path)
            self.td3_agent.actor.eval()
            self.get_logger().info(f'Loaded TD3 model from {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')

    # The rest of the code is adapted verbatim from original implementation
    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.waypoints = []
        last_waypoint = None
        for pose in msg.poses:
            if last_waypoint is None:
                self.waypoints.append(pose.pose)
                last_waypoint = pose.pose
            else:
                dx = pose.pose.position.x - last_waypoint.position.x
                dy = pose.pose.position.y - last_waypoint.position.y
                dist = np.sqrt(dx**2 + dy**2)
                if dist >= self.waypoint_spacing:
                    self.waypoints.append(pose.pose)
                    last_waypoint = pose.pose
        if len(msg.poses) > 0:
            final_pose = msg.poses[-1].pose
            if not self.waypoints or self.waypoints[-1] != final_pose:
                self.waypoints.append(final_pose)
        self.waypoint_index = 0
        if self.waypoints:
            self.current_waypoint = self.waypoints[0]
            self.get_logger().info(f'Received path with {len(self.waypoints)} waypoints')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.current_scan = msg

    def bin_laser_scan(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = scan_msg.range_max
        bin_size = max(1, len(ranges) // self.scan_bins)
        binned = []
        for i in range(self.scan_bins):
            start = i * bin_size
            end = min(start + bin_size, len(ranges))
            if start >= end:
                val = scan_msg.range_max
            else:
                val = np.min(ranges[start:end])
            binned.append(min(val / scan_msg.range_max, 1.0))
        return np.array(binned, dtype=np.float32)

    def compute_goal_info(self, goal_pose):
        if self.current_pose is None:
            return np.zeros(4, dtype=np.float32)
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        quat = self.current_pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_yaw = np.arctan2(siny_cosp, cosy_cosp)
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)
        angle_to_goal = np.arctan2(dy, dx)
        relative_angle = angle_to_goal - robot_yaw
        while relative_angle > np.pi:
            relative_angle -= 2 * np.pi
        while relative_angle < -np.pi:
            relative_angle += 2 * np.pi
        return np.array([dx, dy, distance, relative_angle], dtype=np.float32)

    def check_waypoint_reached(self):
        if self.current_pose is None or self.current_waypoint is None:
            return False
        dx = self.current_waypoint.position.x - self.current_pose.position.x
        dy = self.current_waypoint.position.y - self.current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        return distance < self.waypoint_tolerance

    def control_loop(self):
        if self.td3_agent is None:
            return
        if self.current_pose is None or self.current_scan is None:
            return
        if not self.waypoints or self.current_waypoint is None:
            self.publish_velocity(0.0, 0.0)
            return
        if self.check_waypoint_reached():
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('Reached final waypoint!')
                self.publish_velocity(0.0, 0.0)
                self.waypoints = []
                self.current_waypoint = None
                return
            else:
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.get_logger().info(f'Moving to waypoint {self.waypoint_index + 1}/{len(self.waypoints)}')
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'map'
        waypoint_msg.pose = self.current_waypoint
        self.current_waypoint_pub.publish(waypoint_msg)
        laser_bins = self.bin_laser_scan(self.current_scan)
        goal_info = self.compute_goal_info(self.current_waypoint)
        observation = np.concatenate([laser_bins, goal_info])
        with torch.no_grad():
            obs_tensor = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
            try:
                action = self.td3_agent.get_action(obs_tensor, is_training=False, step=0)
            except Exception:
                # Fallback to select_action if agent provides that
                action = self.td3_agent.actor(obs_tensor).detach().cpu().numpy()[0]
        linear_vel = action[0] * self.max_linear
        angular_vel = action[1] * self.max_angular
        self.publish_velocity(linear_vel, angular_vel)

    def publish_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='', help='Path to trained TD3 model')
    parsed, _ = parser.parse_known_args()

    model_path = parsed.model if parsed.model else None

    node = TD3WaypointController(model_path=model_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
TD3 Waypoint Controller Node.
Uses trained TD3 policy to navigate between waypoints provided by Nav2.
Local control with obstacle avoidance while following global path.
"""
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import torch

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan

# Ensure we use venv Python with PyTorch
venv_python = '/opt/venv/bin/python3'
if sys.executable != venv_python and os.path.exists(venv_python):
    os.execv(venv_python, [venv_python] + sys.argv)


class TD3WaypointController(Node):
    """
    Follows Nav2 path using TD3 local control.
    Receives global path from Nav2, extracts waypoints, and uses TD3 for reactive control.
    """
    
    def __init__(self, model_path=None):
        super().__init__('td3_waypoint_controller')
        
        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('waypoint_spacing', 1.0)  # meters between waypoints
        self.declare_parameter('waypoint_tolerance', 0.3)  # meters
        self.declare_parameter('max_linear_vel', 0.22)  # m/s
        self.declare_parameter('max_angular_vel', 2.84)  # rad/s
        self.declare_parameter('scan_bins', 24)
        
        # Get parameters
        if model_path is None:
            model_path = self.get_parameter('model_path').value
        
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.scan_bins = self.get_parameter('scan_bins').value
        
        # Load TD3 model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.td3_agent = None
        
        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
        else:
            self.get_logger().warn(f'No valid model path provided: {model_path}')
        
        # State
        self.current_pose = None
        self.current_scan = None
        self.current_waypoint = None
        self.waypoints = []
        self.waypoint_index = 0
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_waypoint_pub = self.create_publisher(
            PoseStamped,
            '/current_waypoint',
            10
        )
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('TD3 Waypoint Controller initialized')
    
    def load_model(self, model_path):
        """Load trained TD3 model."""
        try:
            # Import TD3 agent
            from .td3_agent import TD3
            
            # Create agent with observation/action dimensions
            # Observation: [24 laser bins + 4 goal info] = 28
            # Action: [linear_vel, angular_vel] = 2
            obs_dim = self.scan_bins + 4  # laser + goal_x, goal_y, distance, angle
            action_dim = 2
            
            self.td3_agent = TD3(
                state_dim=obs_dim,
                action_dim=action_dim,
                max_action=1.0,
                device=self.device
            )
            
            # Load checkpoint
            checkpoint = torch.load(model_path, map_location=self.device)
            
            if 'actor_state_dict' in checkpoint:
                self.td3_agent.actor.load_state_dict(checkpoint['actor_state_dict'])
            elif 'model_state_dict' in checkpoint:
                self.td3_agent.actor.load_state_dict(checkpoint['model_state_dict'])
            else:
                self.get_logger().error('Invalid checkpoint format')
                return
            
            self.td3_agent.actor.eval()
            
            self.get_logger().info(f'Loaded TD3 model from {model_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            import traceback
            traceback.print_exc()
    
    def path_callback(self, msg):
        """Receive new path from Nav2."""
        if len(msg.poses) == 0:
            return
        
        # Extract waypoints from path (every waypoint_spacing meters)
        self.waypoints = []
        last_waypoint = None
        
        for pose in msg.poses:
            if last_waypoint is None:
                self.waypoints.append(pose.pose)
                last_waypoint = pose.pose
            else:
                # Check distance from last waypoint
                dx = pose.pose.position.x - last_waypoint.position.x
                dy = pose.pose.position.y - last_waypoint.position.y
                dist = np.sqrt(dx**2 + dy**2)
                
                if dist >= self.waypoint_spacing:
                    self.waypoints.append(pose.pose)
                    last_waypoint = pose.pose
        
        # Always add final pose
        if len(msg.poses) > 0:
            final_pose = msg.poses[-1].pose
            if self.waypoints[-1] != final_pose:
                self.waypoints.append(final_pose)
        
        # Reset to first waypoint
        self.waypoint_index = 0
        if self.waypoints:
            self.current_waypoint = self.waypoints[0]
            self.get_logger().info(f'Received path with {len(self.waypoints)} waypoints')
    
    def odom_callback(self, msg):
        """Store current robot pose."""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        """Store current laser scan."""
        self.current_scan = msg
    
    def bin_laser_scan(self, scan_msg):
        """Convert 360-degree scan to fixed number of bins."""
        ranges = np.array(scan_msg.ranges)
        
        # Replace inf with max range
        ranges[np.isinf(ranges)] = scan_msg.range_max
        
        # Bin the scan
        bin_size = len(ranges) // self.scan_bins
        binned = []
        
        for i in range(self.scan_bins):
            start = i * bin_size
            end = start + bin_size
            bin_value = np.min(ranges[start:end])
            
            # Normalize to [0, 1]
            binned.append(min(bin_value / scan_msg.range_max, 1.0))
        
        return np.array(binned, dtype=np.float32)
    
    def compute_goal_info(self, goal_pose):
        """Compute goal information relative to robot."""
        if self.current_pose is None:
            return np.zeros(4, dtype=np.float32)
        
        # Robot position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Robot orientation (yaw)
        quat = self.current_pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        robot_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Goal position
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        
        # Relative goal
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)
        angle_to_goal = np.arctan2(dy, dx)
        
        # Relative angle (in robot frame)
        relative_angle = angle_to_goal - robot_yaw
        
        # Normalize angle to [-pi, pi]
        while relative_angle > np.pi:
            relative_angle -= 2 * np.pi
        while relative_angle < -np.pi:
            relative_angle += 2 * np.pi
        
        # Return [dx, dy, distance, angle]
        return np.array([
            dx,
            dy,
            distance,
            relative_angle
        ], dtype=np.float32)
    
    def check_waypoint_reached(self):
        """Check if current waypoint is reached."""
        if self.current_pose is None or self.current_waypoint is None:
            return False
        
        dx = self.current_waypoint.position.x - self.current_pose.position.x
        dy = self.current_waypoint.position.y - self.current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        
        return distance < self.waypoint_tolerance
    
    def control_loop(self):
        """Main control loop using TD3."""
        # Check prerequisites
        if self.td3_agent is None:
            return
        
        if self.current_pose is None or self.current_scan is None:
            return
        
        if not self.waypoints or self.current_waypoint is None:
            # No path to follow, stop
            self.publish_velocity(0.0, 0.0)
            return
        
        # Check if waypoint reached
        if self.check_waypoint_reached():
            self.waypoint_index += 1
            
            if self.waypoint_index >= len(self.waypoints):
                # Reached final waypoint
                self.get_logger().info('Reached final waypoint!')
                self.publish_velocity(0.0, 0.0)
                self.waypoints = []
                self.current_waypoint = None
                return
            else:
                # Move to next waypoint
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.get_logger().info(
                    f'Moving to waypoint {self.waypoint_index + 1}/{len(self.waypoints)}'
                )
        
        # Publish current waypoint for visualization
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'map'
        waypoint_msg.pose = self.current_waypoint
        self.current_waypoint_pub.publish(waypoint_msg)
        
        # Prepare observation for TD3
        laser_bins = self.bin_laser_scan(self.current_scan)
        goal_info = self.compute_goal_info(self.current_waypoint)
        
        observation = np.concatenate([laser_bins, goal_info])
        
        # Get action from TD3
        with torch.no_grad():
            obs_tensor = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
            action = self.td3_agent.select_action(obs_tensor)
        
        # Scale action to velocity limits
        linear_vel = action[0] * self.max_linear
        angular_vel = action[1] * self.max_angular
        
        # Publish velocity command
        self.publish_velocity(linear_vel, angular_vel)
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command."""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, required=True,
                       help='Path to trained TD3 model')
    
    parsed_args, unknown = parser.parse_known_args()
    
    node = TD3WaypointController(model_path=parsed_args.model)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

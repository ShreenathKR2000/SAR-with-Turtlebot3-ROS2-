"""
SAR Room Environment for TD3 Training.
Manages robot state, goal generation, collision detection, and episode termination.
"""
import math
import random
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Empty

from .settings import (
    NUM_SCAN_SAMPLES, LIDAR_DISTANCE_CAP, THRESHOLD_COLLISION,
    THRESHOLD_GOAL, EPISODE_TIMEOUT_SECONDS, SPEED_LINEAR_MAX,
    SPEED_ANGULAR_MAX, ENABLE_BACKWARD, ENABLE_MOTOR_NOISE,
    UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE,
    OBSTACLE_POSITIONS, ROOM1_BOUNDS, ROOM2_BOUNDS,
    DEFAULT_SPAWN_X, DEFAULT_SPAWN_Y, DEFAULT_SPAWN_Z, DEFAULT_SPAWN_YAW
)
from . import reward as rw


def euler_from_quaternion(quat):
    """Convert quaternion to euler angles."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

# Validated list of easy goals for curriculum learning


EASY_GOALS = [
    (-4.0, 0.0),
    (-3.5, 0.0),
    (-4.0, -2.0),
    (-2.0, 0.0),
    (-2.5, 0.0),
    (0.0, -1.0),
    (0.0, 1.0),
    (1.0, 3.0),
    (-4.0, 0.5),
    (0.0, 2.0),
    (-1.0, 0.0),
    (-4.0, 1.0)
]

class SAREnvironment(Node):
    """ROS2 node for managing TD3 training environment."""
    
    def __init__(self):
        super().__init__('sar_td3_environment')
        
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS
        
        # Curriculum learning
        # Add easy goals list to the class
        self.easy_goals = EASY_GOALS
        self.last_easy_goal_idx = -1

        # Robot state
        self.robot_x = DEFAULT_SPAWN_X
        self.robot_y = DEFAULT_SPAWN_Y
        self.robot_x_prev = DEFAULT_SPAWN_X
        self.robot_y_prev = DEFAULT_SPAWN_Y
        self.robot_heading = DEFAULT_SPAWN_YAW
        self.robot_tilt = 0.0
        self.total_distance = 0.0
        self.last_action_linear = 0.0
        self.last_action_angular = 0.0
        
        # Goal state
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.initial_distance_to_goal = 0.0

        # Sensor data
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP
        
        # Episode management
        self.done = False
        self.outcome = UNKNOWN
        self.local_step = 0
        self.time_sec = 0
        self.episode_start_time = 0
        
        # ROS2 setup
        qos = QoSProfile(depth=10)
        # Correct QoS for clock subscription to match Gazebo's publisher
        qos_clock = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
            
        # Service clients
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        self.pause_physics_client = self.create_client(Empty, '/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/unpause_physics')
        self.set_entity_client = self.create_client(SetEntityState, '/set_entity_state')
        
        self.get_logger().info('SAR TD3 Environment initialized')
        
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, self.robot_tilt, self.robot_heading = euler_from_quaternion(msg.pose.pose.orientation)
        
        # Calculate traveled distance
        if self.local_step > 0 and self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2
            )
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y
            
        # Update goal distance and angle
        diff_x = self.goal_x - self.robot_x
        diff_y = self.goal_y - self.robot_y
        
        self.goal_distance = math.sqrt(diff_x**2 + diff_y**2)
        heading_to_goal = math.atan2(diff_y, diff_x)
        goal_angle = heading_to_goal - self.robot_heading
        
        # Normalize angle to [-pi, pi]
        while goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        while goal_angle < -math.pi:
            goal_angle += 2 * math.pi
            
        self.goal_angle = goal_angle
        
    def scan_callback(self, msg):
        """Update lidar scan data."""
        self.obstacle_distance = LIDAR_DISTANCE_CAP
        for i in range(NUM_SCAN_SAMPLES):
            scan_dist = msg.ranges[i]
            if not math.isinf(scan_dist):
                self.scan_ranges[i] = np.clip(scan_dist / LIDAR_DISTANCE_CAP, 0, 1)
                if self.scan_ranges[i] < self.obstacle_distance:
                    self.obstacle_distance = self.scan_ranges[i]
        self.obstacle_distance *= LIDAR_DISTANCE_CAP
        
    def clock_callback(self, msg):
        """Update simulation time."""
        self.time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        
    def is_position_valid(self, x, y, min_clearance=0.5):
        """Check if position is valid (not on obstacle or too close to walls)."""
        # Check room boundaries
        in_room1 = (ROOM1_BOUNDS['x_min'] + min_clearance <= x <= ROOM1_BOUNDS['x_max'] - min_clearance and
                   ROOM1_BOUNDS['y_min'] + min_clearance <= y <= ROOM1_BOUNDS['y_max'] - min_clearance)
        in_room2 = (ROOM2_BOUNDS['x_min'] + min_clearance <= x <= ROOM2_BOUNDS['x_max'] - min_clearance and
                   ROOM2_BOUNDS['y_min'] + min_clearance <= y <= ROOM2_BOUNDS['y_max'] - min_clearance)
        if not (in_room1 or in_room2):
            return False
            
        # Check distance from all obstacles
        for obs_x, obs_y, obs_radius in OBSTACLE_POSITIONS:
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < (obs_radius + min_clearance):
                return False
        return True

    def generate_easy_goal(self):
        """Select a unique, easy goal from a predefined list."""
        available_indices = list(range(len(self.easy_goals)))
        if self.last_easy_goal_idx in available_indices and len(available_indices) > 1:
             available_indices.pop(self.last_easy_goal_idx)
        
        chosen_idx = random.choice(available_indices)
        self.last_easy_goal_idx = chosen_idx
        
        self.get_logger().info(f"Selected easy goal {chosen_idx+1}/{len(self.easy_goals)}")
        return self.easy_goals[chosen_idx]



    def generate_random_goal(self):
        """Generate a random valid goal position."""
        max_attempts = 100
        for _ in range(max_attempts):
            # Randomly choose room 1 or room 2
            if np.random.random() < 0.5:
                # Room 1
                x = np.random.uniform(ROOM1_BOUNDS['x_min'], ROOM1_BOUNDS['x_max'])
                y = np.random.uniform(ROOM1_BOUNDS['y_min'], ROOM1_BOUNDS['y_max'])
            else:
                # Room 2
                x = np.random.uniform(ROOM2_BOUNDS['x_min'], ROOM2_BOUNDS['x_max'])
                y = np.random.uniform(ROOM2_BOUNDS['y_min'], ROOM2_BOUNDS['y_max'])
                
            if self.is_position_valid(x, y, min_clearance=0.6):
                return x, y
                
        # Fallback to safe position in room 1
        self.get_logger().warn('Could not generate valid goal, using fallback')
        return -2.0, 0.0
        
    def reset_robot_to_spawn(self, random_yaw=None):
        """Reset robot to default spawn position with optional random orientation."""
        self.cmd_vel_pub.publish(Twist()) # Stop robot
        time.sleep(0.05)
        
        # Use reset_world to reset position
        if self.reset_world_client.wait_for_service(timeout_sec=2.0):
            reset_future = self.reset_world_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, reset_future, timeout_sec=2.0)
            # Wait longer for physics to settle after reset_world
            time.sleep(0.3)
        
        # If random yaw provided, set it using entity state
        if random_yaw is not None:
            # Pause physics before setting orientation
            if self.pause_physics_client.wait_for_service(timeout_sec=1.0):
                rclpy.spin_until_future_complete(self, self.pause_physics_client.call_async(Empty.Request()), timeout_sec=1.0)
                pause_future = self.pause_physics_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, pause_future, timeout_sec=1.0)
                time.sleep(0.05)
            
            state_msg = EntityState(name='turtlebot3', reference_frame='world')
            state_msg = EntityState()
            state_msg.name = 'turtlebot3'
            state_msg.pose.position.x = DEFAULT_SPAWN_X
            state_msg.pose.position.y = DEFAULT_SPAWN_Y
            state_msg.pose.position.z = DEFAULT_SPAWN_Z
            # Set quaternion for yaw rotation (rotation around z-axis)
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = math.sin(random_yaw / 2.0)
            state_msg.pose.orientation.w = math.cos(random_yaw / 2.0)
            
            # Zero all velocities
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.twist.angular.x = 0.0
            state_msg.twist.angular.y = 0.0
            state_msg.twist.angular.z = 0.0
            
            # Set reference frame to world
            state_msg.reference_frame = 'world'
            
            if self.set_entity_client.wait_for_service(timeout_sec=2.0):
                req = SetEntityState.Request(state=state_msg)
                req = SetEntityState.Request()
                req.state = state_msg
                set_future = self.set_entity_client.call_async(req)
                rclpy.spin_until_future_complete(self, set_future, timeout_sec=2.0)
                if set_future.result() is not None:
                    if not set_future.result().success:
                        self.get_logger().warn(f'Failed to set entity orientation to {random_yaw:.2f}')
                    else:
                        self.get_logger().info(f'Set orientation to {random_yaw:.2f} rad')
                time.sleep(0.1)
            
            # Unpause physics
            if self.unpause_physics_client.wait_for_service(timeout_sec=1.0):
                rclpy.spin_until_future_complete(self, self.unpause_physics_client.call_async(Empty.Request()), timeout_sec=1.0)
                unpause_future = self.unpause_physics_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, unpause_future, timeout_sec=1.0)
                time.sleep(0.1)
        
        # Stop robot
        self.cmd_vel_pub.publish(Twist())
        
        return True
        
    def _reset_entity_fallback(self):
        """Fallback method using entity state (may not work reliably)."""
        import time
        
        # Try to set entity state directly without pause/unpause
        state_msg = EntityState()
        state_msg.name = 'turtlebot3'
        state_msg.pose.position.x = DEFAULT_SPAWN_X
        state_msg.pose.position.y = DEFAULT_SPAWN_Y
        state_msg.pose.position.z = DEFAULT_SPAWN_Z
        state_msg.pose.orientation.w = math.cos(DEFAULT_SPAWN_YAW / 2)
        state_msg.pose.orientation.z = math.sin(DEFAULT_SPAWN_YAW / 2)
        
        # Zero velocities
        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0
        state_msg.twist.angular.x = 0.0
        state_msg.twist.angular.y = 0.0
        state_msg.twist.angular.z = 0.0
        
        if not self.set_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set entity service STILL not available after 5s!')
            return False
            
        req = SetEntityState.Request()
        req.state = state_msg
        set_future = self.set_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, set_future, timeout_sec=3.0)
        time.sleep(0.2)
        
        # Stop robot
        self.cmd_vel_pub.publish(Twist())
        
        return True
        
    def get_state(self, action_linear_prev, action_angular_prev):
        """Get current state observation."""
        state = list(self.scan_ranges)
        state.append(float(np.clip(self.goal_distance / 10.0, 0, 1)))
        state.append(float(self.goal_angle) / math.pi)
        state.append(float(action_linear_prev))
        state.append(float(action_angular_prev))
        
        self.local_step += 1
        
        if self.local_step > 30:
            if self.goal_distance < THRESHOLD_GOAL:
                self.outcome = SUCCESS
                self.done = True
            elif self.obstacle_distance < THRESHOLD_COLLISION:
                self.outcome = COLLISION_WALL
                self.done = True
            elif (self.time_sec - self.episode_start_time) >= self.episode_timeout:
                self.outcome = TIMEOUT
                self.done = True
            elif abs(self.robot_tilt) > 0.06:
                self.outcome = TUMBLE
                self.done = True
        
        # Grace period after reset
        if self.local_step <= 30:
            return state
            
        # Check terminal conditions
        # Success
        if self.goal_distance < THRESHOLD_GOAL:
            self.outcome = SUCCESS
            self.done = True
            
        # Collision
        elif self.obstacle_distance < THRESHOLD_COLLISION:
            self.outcome = COLLISION_WALL  # Could enhance to detect dynamic obstacles
            self.done = True
            
        # Timeout
        elif (self.time_sec - self.episode_start_time) >= self.episode_timeout:
            self.outcome = TIMEOUT
            self.done = True
            
        # Tumble
        elif abs(self.robot_tilt) > 0.06:
            self.outcome = TUMBLE
            self.done = True
            
        # Stop robot if episode done
        if self.done:
            self.cmd_vel_pub.publish(Twist())
            
        return state
        
    def step(self, action):
        """Execute action and return next state, reward, done."""
        """
        Execute action and return next state, reward, done.
        
        Args:
            action: [linear_vel, angular_vel] normalized to [-1, 1]
            
        Returns:
            tuple: (next_state, reward, done, info)
        """
        # Denormalize actions
        if ENABLE_BACKWARD:
            action_linear = action[0] * SPEED_LINEAR_MAX
        else:
            action_linear = (action[0] + 1) / 2 * SPEED_LINEAR_MAX
            
        action_angular = action[1] * SPEED_ANGULAR_MAX
        
        # Add motor noise if enabled
        if ENABLE_MOTOR_NOISE:
            action_linear += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)
            action_angular += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)
            
        # Publish velocity command
        twist = Twist()
        twist.linear.x = float(action_linear)
        twist.angular.z = float(action_angular)
        self.cmd_vel_pub.publish(twist)
        
        # Get next state
        next_state = self.get_state(action[0], action[1])
        
        # Calculate reward
        reward = rw.get_reward(
            self.outcome, action_linear, action_angular,
            self.goal_distance, self.goal_angle, self.obstacle_distance,
            self.last_action_linear, self.last_action_angular
        )
        info = {'outcome': self.outcome, 'distance_traveled': self.total_distance, 'goal_distance': self.goal_distance}
        
        # Update last actions for next step
        self.last_action_linear = action_linear
        self.last_action_angular = action_angular
        
        return next_state, reward, self.done, info
        
    def reset(self, episode_num=0):
        """Reset environment for new episode."""
        random_yaw = random.uniform(-math.pi, math.pi)
        
        # Reset robot to spawn with random orientation
        self.reset_robot_to_spawn(random_yaw=random_yaw)
        self.robot_heading = random_yaw
        
        # Small delay to let orientation settle
        time.sleep(0.1)
        
        # Generate new goal based on curriculum
        if episode_num < 400:
            self.goal_x, self.goal_y = self.generate_easy_goal()
            self.get_logger().info(f'Curriculum Learning (Easy Goal for ep < 400)')
        else:
            self.goal_x, self.goal_y = self.generate_random_goal()
        
        # Reset episode variables
        self.done = False
        self.outcome = UNKNOWN
        self.local_step = 0
        self.total_distance = 0.0
        self.last_action_linear = 0.0
        self.last_action_angular = 0.0
        self.robot_x_prev = DEFAULT_SPAWN_X
        self.robot_y_prev = DEFAULT_SPAWN_Y
        self.episode_start_time = self.time_sec
        
        # Calculate initial distance to goal
        diff_x = self.goal_x - DEFAULT_SPAWN_X
        diff_y = self.goal_y - DEFAULT_SPAWN_Y
        self.initial_distance_to_goal = math.sqrt(diff_x**2 + diff_y**2)
        self.goal_distance = self.initial_distance_to_goal
        
        # Initialize reward function
        rw.reward_initialize(self.initial_distance_to_goal)
        
        self.get_logger().info(f'New episode - Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}), '
                              f'Distance: {self.initial_distance_to_goal:.2f}m, '
                              f'Start orientation: {random_yaw:.2f}rad')
        
        # Return initial state
        return self.get_state(0.0, 0.0)
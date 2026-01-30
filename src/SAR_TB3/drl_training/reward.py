"""
Reward function for TD3 training in SAR room world.
Based on turtlebot3_drlnav reward function G (best performing).
"""
import math
from .settings import (
    SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE,
    THRESHOLD_COLLISION, THRESHOLD_GOAL, REWARD_FUNCTION
)

# Global variables for reward calculation
initial_distance = 0.0
last_distance = 0.0


def reward_initialize(init_distance):
    """Initialize reward function with starting distance to goal."""
    global initial_distance, last_distance
    initial_distance = init_distance
    last_distance = init_distance


def get_reward(outcome, action_linear, action_angular, goal_distance, 
               goal_angle, obstacle_distance, 
               prev_action_linear=0.0, prev_action_angular=0.0):
    """
    Calculate reward based on current state and outcome.
    
    Reward function G from turtlebot3_drlnav (best performing):
    - Large positive reward for reaching goal
    - Large negative reward for collision
    - Negative reward for timeout
    - Distance-based reward (progress toward goal)
    - Penalty for proximity to obstacles
    - Small penalty for angular velocity (encourage smooth movement)
    
    Args:
        outcome: Episode outcome (SUCCESS, COLLISION_WALL, etc.)
        action_linear: Linear velocity command
        action_angular: Angular velocity command
        goal_distance: Current distance to goal
        goal_angle: Angle to goal
        obstacle_distance: Distance to nearest obstacle
        prev_action_linear: Previous linear velocity
        prev_action_angular: Previous angular velocity
        
    Returns:
        float: Reward value
    """
    global last_distance
    reward = 0.0
    
    # Terminal state rewards
    if outcome == SUCCESS:
        reward = 200.0
    elif outcome == COLLISION_WALL or outcome == COLLISION_OBSTACLE:
        reward = -200.0
    elif outcome == TIMEOUT:
        reward = -50.0
    elif outcome == TUMBLE:
        reward = -200.0
    else:
        # Non-terminal rewards
        
        # 1. Progress Reward (The main driver)
        # Reward for getting closer, penalty for moving away
        progress = last_distance - goal_distance
        reward += progress * 10.0
        last_distance = goal_distance
        
        # 2. Heading Reward (Compass guide)
        # Helps when robot is confused or facing wrong way
        reward += (1.0 - abs(goal_angle) / math.pi) * 0.05

        # 3. Forward Bias (Small constant to encourage movement)
        if action_linear > 0:
            reward += action_linear * 0.1
            
        # 4. Angular Penalty (Stability)
        reward -= abs(action_angular) * 0.05
        # 5. Smoothness Penalty (Critical for SLAM/Real Robot)
        # Penalize rapid changes in angular velocity (Jerk)
        # This forces the robot to turn smoothly despite motor noise
        reward -= abs(action_angular - prev_action_angular) * 0.5

        # 6. Obstacle Safety (Progressive)
        if obstacle_distance < 0.5:
            reward -= (0.5 - obstacle_distance) * 4.0
        
    return reward

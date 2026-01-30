#!/bin/bash

# Clean environment and launch SAR_TB3 TD3 exploration

# Kill any existing Gazebo
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
sleep 1

# Set up environment
export TURTLEBOT3_MODEL=waffle_pi
export DISPLAY=:1

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source /root/turtlebot3_ws/install/setup.bash

# Check if TD3 model exists
TD3_MODEL="/root/turtlebot3_ws/checkpoints/td3_sar/model_latest.pt"
if [ ! -f "$TD3_MODEL" ]; then
    echo "WARNING: TD3 model not found at $TD3_MODEL"
    echo "Will use default random policy"
    TD3_MODEL=""
fi

# Launch
echo "Launching SAR_TB3 TD3 Exploration..."
echo "Model: $TD3_MODEL"

if [ -z "$TD3_MODEL" ]; then
    ros2 launch SAR_TB3 single_robot/sar_td3_exploration.launch.py
else
    ros2 launch SAR_TB3 single_robot/sar_td3_exploration.launch.py td3_model:=$TD3_MODEL
fi

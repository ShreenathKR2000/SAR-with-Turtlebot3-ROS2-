# Multi-Robot Launch Files

This directory contains launch files for multi-robot SAR operations in both simulation and real-world deployments.

## Available Launch Files

### 1. multi_robot_exploration.launch.py

**Purpose:** Multi-robot autonomous exploration in Gazebo simulation with cooperative coordination.

**Features:**
- Decentralized SLAM (each robot maintains own map)
- Centralized map merging with multirobot_map_merge
- MPPI controller for better obstacle avoidance
- TD3 support for local control
- Cooperative frontier exploration

**Usage:**
```bash
# Basic multi-robot exploration
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 multi_robot/multi_robot_exploration.launch.py

# With TD3 control
ros2 launch SAR_TB3 multi_robot/multi_robot_exploration.launch.py use_td3:=true
```

---

### 2. multi_robot_real.launch.py

**Purpose:** Deploy multi-robot autonomous exploration on real TurtleBot3 hardware.

**On Central PC:**
```bash
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py mode:=central
```

**On Robot 1:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py \
    mode:=robot robot_name:=robot1 use_td3:=true td3_model:=/path/to/model.pt
```

**On Robot 2:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py \
    mode:=robot robot_name:=robot2 use_td3:=true td3_model:=/path/to/model.pt
```

---

## Prerequisites

**Simulation:**
```bash
sudo apt install ros-humble-multirobot-map-merge
```

**Real Hardware:**
- All devices on same network
- Matching ROS_DOMAIN_ID on all devices
- SAR_TB3 built on all devices

---

See main README.md for detailed documentation.

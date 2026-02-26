## Important!

Doing this project has helped me understand the actual full scope of it. So I will be breaking it down to smaller elements first and improving them. So as part of that, I will be having a new repo focusing on Multi Robot *(Swarm)* Exploration using Turtlebots. I will be focusing on that first and then try to integrate it here or change the whole thing and start from fresh. I will also be using some other algorithm other than frontier based exploration.


# SAR_TB3: Search and Rescue TurtleBot3 Package

Complete autonomous exploration and TD3 training package for TurtleBot3 robots in SAR (Search and Rescue) environments.

## Features

- **TD3 Deep Reinforcement Learning Training** - Train autonomous navigation policies
- **Autonomous Exploration** - TD3-based autonomous exploration with QR code detection
- **Multi-Robot Cooperative Exploration (in works)** - Coordinated exploration with map merging
- **Real Robot Deployment Code** - Single and multi-robot real hardware support
- **QR Code Detection** - YOLO-based detection with precise 3D positioning using LiDAR + camera
- **SAR World Environment** - Custom Gazebo world with obstacles and QR codes

## Quick Start

### 1. Installation

```bash
cd /path/to/your/ros2_ws/src
git clone https://github.com/ShreenathKR2000/SAR-with-Turtlebot3-ROS2- SAR_TB3
cd ..
colcon build --packages-select SAR_TB3
source install/setup.bash
```

**Install Python dependencies:**
```bash
pip install torch torchvision ultralytics opencv-python numpy matplotlib
```

### 2. TD3 Training

Train a TD3 agent to navigate autonomously:

```bash
# Terminal 1: Launch SAR world
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 single_robot/sar_world.launch.py

# Terminal 2: Start TD3 training
ros2 run SAR_TB3 train_td3_sar --episodes 10000
```

See [docs/TD3_TRAINING_GUIDE.md](docs/TD3_TRAINING_GUIDE.md) for detailed training instructions.

### 3. Autonomous Exploration with QR Detection

Run autonomous exploration using trained TD3 model:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 single_robot/sar_td3_exploration.launch.py \
    td3_model:=/path/to/your/trained_model.pt
```

QR code detections with 3D positions will be saved to `qr_detections/`.

### 4. Multi-Robot Exploration (Simulation)

Launch cooperative exploration with 2 robots:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 multi_robot/multi_robot_exploration.launch.py
```

Or with TD3 control:
```bash
ros2 launch SAR_TB3 multi_robot/multi_robot_exploration.launch.py use_td3:=true
```

**Features:**
- Decentralized SLAM (each robot maintains own map)
- Centralized map merging (combined global map)
- MPPI controller for better dynamic obstacle avoidance
- Cooperative frontier exploration (avoiding duplicate work)

### 5. Real Robot Deployment (Single Robot)

Deploy on physical TurtleBot3:

```bash
# On the robot
export TURTLEBOT3_MODEL=waffle  # or burger
ros2 launch SAR_TB3 single_robot/real_robot_exploration.launch.py
```

With TD3 control:
```bash
ros2 launch SAR_TB3 single_robot/real_robot_exploration.launch.py \
    use_td3:=true \
    td3_model:=/path/to/trained_model.pt
```

**Prerequisites:**
- TurtleBot3 powered on and connected via network
- Robot's ROS 2 workspace sourced
- Correct TURTLEBOT3_MODEL environment variable set

### 6. Real Robot Multi-Robot Deployment

**Architecture:**
- **Robot PCs:** Run locally on each robot (SLAM + Nav2 + TD3 + Exploration)
- **Central PC:** Runs map fusion and coordination + RViz dashboard

**On Central PC:**
```bash
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py mode:=central
```

**On Robot 1:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py \
    mode:=robot \
    robot_name:=robot1 \
    use_td3:=true \
    td3_model:=/path/to/model.pt
```

**On Robot 2:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch SAR_TB3 multi_robot/multi_robot_real.launch.py \
    mode:=robot \
    robot_name:=robot2 \
    use_td3:=true \
    td3_model:=/path/to/model.pt
```

## Package Structure

```
SAR_TB3/
├── README.md                          # Complete usage guide  
├── LICENSE                            # MIT License
├── .gitignore                         # Git ignore rules
├── package.xml                        # ROS2 package manifest
├── setup.py                           # Python package setup
├── setup.cfg                          # Setup configuration
│
├── docs/                              # Documentation
│   ├── TD3_TRAINING_GUIDE.md         # Detailed training instructions
│   ├── PACKAGE_SUMMARY.md            # Package overview
│   ├── QUICK_START.md                # Quick reference
│   └── FEATURES.md                   # Feature highlights
│
├── launch/                            # Launch files
│   ├── single_robot/                 # Single robot operations
│   │   ├── sar_world.launch.py       # SAR world only (training)
│   │   ├── sar_td3_exploration.launch.py  # TD3 exploration + SLAM + QR
│   │   ├── real_robot_exploration.launch.py  # Real robot autonomous exploration
│   │   ├── full_autonomous_exploration.launch.py  # Nav2-based
│   │   └── autonomous_exploration.launch.py       # Frontier-based
│   └── multi_robot/                  # Multi-robot operations
│       ├── multi_robot_exploration.launch.py  # Simulation with 2 robots
│       └── multi_robot_real.launch.py         # Real hardware deployment
│
├── src/SAR_TB3/                      # Python source code
│   ├── __init__.py
│   ├── frontier_detector.py          # Frontier-based exploration
│   ├── autonomous_explorer.py        # Exploration coordinator
│   ├── td3_waypoint_controller.py    # TD3 waypoint navigation
│   ├── td3_agent.py                  # TD3 model loading
│   ├── qr_detector.py                # QR detection with 3D positioning
│   └── drl_training/                 # TD3 training suite
│       ├── __init__.py
│       ├── train_td3.py              # Training script
│       ├── td3_agent.py              # TD3 algorithm
│       ├── sar_environment.py        # Environment wrapper
│       ├── reward.py                 # Reward function
│       ├── settings.py               # Hyperparameters
│       ├── map_fusion_coordinator.py # Multi-robot map fusion
│       ├── laser_scan_adapter.py     # Scan interpolation for real robots
│       └── ...                       # Supporting modules
│
├── config/                            # Configuration files
│   ├── nav2_params.yaml              # Nav2 parameters (single robot)
│   ├── nav2_params_real.yaml         # Nav2 parameters (real robot)
│   ├── nav2_params_robot1.yaml       # Nav2 parameters (robot 1)
│   ├── nav2_params_robot2.yaml       # Nav2 parameters (robot 2)
│   ├── slam_params_robot1.yaml       # SLAM parameters (robot 1)
│   ├── slam_params_robot2.yaml       # SLAM parameters (robot 2)
│   ├── slam_params_real.yaml         # SLAM parameters (real robot)
│   └── multirobot_rviz.rviz          # RViz config for multi-robot
│
├── worlds/                            # Gazebo worlds
│   └── sar_room_world.sdf            # Custom SAR world
│
├── models/                            # Gazebo models
│   └── qr_cube/                      # QR code cube model
│
├── resource/                          # Package resources
│   ├── SAR_TB3                       # Ament marker
│   └── qr_best.pt                    # Pre-trained YOLO (6MB)
│
└── resources/                         # Runtime resources
    └── training_log.csv              # Training log template
```

## Usage

### Training a TD3 Model

1. **Launch SAR World:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 sar_world.launch.py
```

2. **Start Training (new terminal):**
```bash
ros2 run SAR_TB3 train_td3_sar --episodes 10000
```

3. **Monitor Progress:**
- Watch terminal output for episode rewards and success rates
- Training plots update every 10 episodes
- Models auto-save every 100 episodes to `checkpoints/td3_sar/`

4. **Resume Training:**
```bash
ros2 run SAR_TB3 train_td3_sar \
    --load-model ./checkpoints/td3_sar/session_XXX/model_ep1000.pt \
    --start-episode 1000 \
    --episodes 20000
```

### Autonomous Exploration with TD3

Use your trained TD3 model for autonomous exploration:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 sar_td3_exploration.launch.py \
    td3_model:=/absolute/path/to/model_ep5000.pt
```

**What happens:**
1. Gazebo launches with SAR world
2. Robot spawns at starting position
3. SLAM starts mapping the environment
4. TD3 agent controls the robot for exploration
5. QR codes are detected and positions saved

### QR Code Detection

QR codes are automatically detected during exploration:

- **Saved Images:** `qr_detections/qr_det_XXX_QRxY.YY_QRyZ.ZZ_TIMESTAMP.jpg`
- **Detection Log:** `qr_detections/qr_detection_log.txt`
- **Format:** QR 3D position in map frame (X, Y, Z coordinates)

#### Detection Algorithm

1. **YOLO Detection:** YOLOv8 detects QR codes in camera image
2. **LiDAR Distance:** Maps camera pixel center to LiDAR angle, gets distance to QR
3. **Camera Geometry:** Converts pixel position + distance to 3D point in camera frame using pinhole camera model
4. **TF Transform:** Transforms camera frame → base_footprint → map frame
5. **Saves:** Only when 3D position is successfully calculated

**Note:** Images are only saved when the QR code's 3D position in the map is successfully determined.

### Testing Trained Models

Test a trained model for a fixed number of episodes:

```bash
ros2 run SAR_TB3 test_td3_sar \
    --model ./checkpoints/td3_sar/session_XXX/model_ep5000.pt \
    --episodes 10
```

## Configuration

### TD3 Hyperparameters

Edit `src/SAR_TB3/drl_training/settings.py`:

```python
# Network architecture
HIDDEN_SIZE = 512
BATCH_SIZE = 128
LEARNING_RATE = 0.0003

# Training parameters
OBSERVE_STEPS = 25000              # Random exploration before training
EPISODE_TIMEOUT_SECONDS = 60.0    # Max episode duration
THRESHOLD_GOAL = 0.30              # Goal reach distance (meters)
THRESHOLD_COLLISION = 0.15         # Collision detection distance
```

### QR Detection Parameters

Configure in launch file or as ROS parameters:

```python
qr_detector_node = Node(
    package='SAR_TB3',
    executable='qr_detector',
    parameters=[{
        'confidence_threshold': 0.6,    # Minimum YOLO confidence
        'save_interval': 5.0,          # Min seconds between saves
        'camera_topic': '/camera/image_raw',
        'lidar_topic': '/scan'
    }]
)
```

## Dependencies

### System Requirements
- ROS 2 Humble
- Gazebo 11
- Python 3.10+
- CUDA-capable GPU (optional, for faster training)

### ROS 2 Packages
- `rclpy`
- `gazebo_ros`
- `turtlebot3_gazebo`
- `turtlebot3_description`
- `turtlebot3_bringup` (for real robot)
- `slam_toolbox`
- `nav2_bringup` (optional, for frontier-based exploration)
- `multirobot_map_merge` (for multi-robot exploration)
- `tf2_ros`
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`

### Python Packages
```bash
pip install torch torchvision ultralytics opencv-python numpy matplotlib
```

## Training Results

After 5000-10000 episodes of training:

| Metric | Expected Value |
|--------|---------------|
| Success Rate | 70-90% |
| Average Reward | 100-150 |
| Episode Duration | 10-20 seconds |
| Goal Reach Time | 8-15 seconds |

See [TD3_TRAINING_GUIDE.md](../../../TD3_TRAINING_GUIDE.md) for detailed training progression.

## Troubleshooting

### Robot Doesn't Move
- Check Gazebo is not paused (spacebar to pause/unpause)
- Verify TD3 model path is correct and file exists
- Ensure robot spawned successfully: `ros2 topic list | grep cmd_vel`
- Check terminal for TD3 loading errors

### QR Detection Not Saving Images
- **Images only save when 3D position is known**
- SLAM must be running (provides map frame)
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify camera topic: `ros2 topic echo /camera/image_raw --once`
- Check LiDAR data: `ros2 topic echo /scan --once`

### Training Converges Slowly
- Train longer (10k-20k episodes)
- Adjust learning rate: try 0.001 or 0.0001
- Check GPU is being used: `nvidia-smi`
- Verify reward function in `reward.py`
- Ensure robot can reach goals (check spawn/goal positions)

### "Transform timeout" or "TF2 error"
- SLAM needs time to initialize
- Wait 10-15 seconds after launch before expecting detections
- Check `/map` topic exists: `ros2 topic list | grep map`

### Training Loss Not Decreasing
- Ensure observation phase completed (25k steps)
- Check replay buffer has diverse experiences
- Verify state normalization (LiDAR ranges, distances)
- Try reducing learning rate to 0.0001

## File Outputs

### Training Checkpoints
```
checkpoints/td3_sar/session_TIMESTAMP/
├── model_ep100.pt          # Model at episode 100
├── model_ep200.pt          # Model at episode 200
├── model_latest.pt         # Always latest model
├── replay_buffer.pkl       # Experience replay buffer
├── training_log.csv        # Episode-by-episode statistics
└── _training_figure.png    # Training progress plots
```

### QR Detection Outputs
```
qr_detections/
├── qr_det_001_QRx-2.34_QRy3.12_20260130_123045.jpg
├── qr_det_002_QRx1.56_QRy-0.89_20260130_123050.jpg
├── qr_det_003_QRx4.23_QRy2.67_20260130_123055.jpg
└── qr_detection_log.txt
```

**Detection Log Format:**
```
Detection #1
  Time: Fri Jan 30 12:30:45 2026
  QR Code Position: QR at X: -2.34, Y: 3.12, Z: 0.05
  Confidence: 0.685
```

## Citation

This package is based on the td3_agent from[turtlebot3_drlnav](https://github.com/tomasvr/turtlebot3_drlnav) and implements TD3 from:

```bibtex
@article{fujimoto2018addressing,
  title={Addressing function approximation error in actor-critic methods},
  author={Fujimoto, Scott and Hoof, Herke and Meger, David},
  journal={International Conference on Machine Learning},
  year={2018}
}
```

## License

MIT License - See LICENSE file for details

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch  
3. Test thoroughly (training + exploration)
4. Submit pull request with clear description

## Support

For issues or questions:
- Review [TD3_TRAINING_GUIDE.md](../../../TD3_TRAINING_GUIDE.md)
- Check training logs: `training_log.csv`
- View training plots: `_training_figure.png`
- Verify dependencies: `ros2 pkg list`, `pip list`
- Check ROS topics: `ros2 topic list`

---

## Changelog

v 1.0.1
updated some files, Needs checking if it runs good.

v 1.0.0 - Initial Release

This is a new Repo. Old repo was a total mess, so i ahd to clean up and make it a little more organized.
Expect lots of pushes and updates in coming couple of weeks.

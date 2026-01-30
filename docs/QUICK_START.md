# SAR_TB3 Quick Reference

## Installation (One-Time Setup)

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repo-url> SAR_TB3

# Build package
cd ~/ros2_ws
colcon build --packages-select SAR_TB3
source install/setup.bash

# Install Python dependencies
pip install torch torchvision ultralytics opencv-python numpy matplotlib
```

## Training TD3 Model

```bash
# Terminal 1
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch SAR_TB3 sar_world.launch.py

# Terminal 2
ros2 run SAR_TB3 train_td3_sar --episodes 10000

# Resume training
ros2 run SAR_TB3 train_td3_sar \
    --load-model ./checkpoints/td3_sar/session_XXX/model_ep1000.pt \
    --start-episode 1000 --episodes 20000
```

**Output:** `checkpoints/td3_sar/session_TIMESTAMP/`

## Running Autonomous Exploration

```bash
export TURTLEBOT3_MODEL=waffle_pi

# TD3-based (recommended)
ros2 launch SAR_TB3 sar_td3_exploration.launch.py \
    td3_model:=/path/to/model_ep5000.pt

# Frontier-based (Nav2)
ros2 launch SAR_TB3 full_autonomous_exploration.launch.py
```

**Output:** QR detections in `qr_detections/`

## Key Commands

```bash
# List executables
ros2 pkg executables SAR_TB3

# Check package
ros2 pkg list | grep SAR_TB3

# View TF tree
ros2 run tf2_tools view_frames

# Monitor topics
ros2 topic list
ros2 topic echo /camera/image_raw --once
ros2 topic echo /scan --once

# Check training progress
tail -f training_log.csv
```

## File Locations

```
checkpoints/td3_sar/session_TIMESTAMP/
├── model_ep100.pt
├── model_latest.pt
├── training_log.csv
└── _training_figure.png

qr_detections/
├── qr_det_001_QRx-2.34_QRy3.12_TIMESTAMP.jpg
└── qr_detection_log.txt
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot doesn't move | Check Gazebo not paused (spacebar) |
| QR images not saving | Wait 15s for SLAM, images only save with position |
| Training slow | Use GPU, adjust learning rate |
| Nav2 behavior_server fails | Use sar_td3_exploration.launch.py instead |
| Transform timeout | SLAM needs time to initialize (~10-15s) |

## Configuration

**TD3 Hyperparameters:** `src/SAR_TB3/drl_training/settings.py`
```python
HIDDEN_SIZE = 512
LEARNING_RATE = 0.0003
OBSERVE_STEPS = 25000
```

**QR Detection:** Launch file parameters
```python
confidence_threshold: 0.6
save_interval: 5.0
```

## Expected Results

| Metric | Value (after 5k-10k episodes) |
|--------|-------------------------------|
| Success Rate | 70-90% |
| Avg Reward | 100-150 |
| Episode Duration | 10-20s |
| Goal Reach Time | 8-15s |

## Git Commands

```bash
cd SAR_TB3
git init
git add .
git commit -m "Initial commit"
git remote add origin <url>
git push -u origin main
```

---
**Full documentation:** README.md, TD3_TRAINING_GUIDE.md, PACKAGE_SUMMARY.md

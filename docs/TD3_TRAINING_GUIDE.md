# TD3 Deep Reinforcement Learning Training Guide

## 🤖 Overview

Train a TD3 (Twin Delayed Deep Deterministic Policy Gradient) agent to navigate autonomously in the SAR room world. The agent learns to reach randomly generated goals while avoiding obstacles, with a 60-second timeout per episode.

**Based on**: [turtlebot3_drlnav](https://github.com/tomasvr/turtlebot3_drlnav) implementation

**Key Features**:
- ✅ Episode-based training with automatic respawning
- ✅ Random goal generation (avoiding obstacles)
- ✅ 60-second timeout per episode
- ✅ Automatic termination (success, collision, or timeout)
- ✅ Live visualization with matplotlib
- ✅ Automatic model checkpointing every 100 episodes
- ✅ Default spawn position as respawn point

## 📐 Training Architecture

```
┌─────────────────────────────────────────────┐
│         SAR Room World (Gazebo)             │
│  - Two rooms with obstacles                 │
│  - Robot spawns at (-3.58, 1.749)          │
│  - Random goals generated each episode      │
└────────────────┬────────────────────────────┘
                 │
        ┌────────▼────────┐
        │   Sensors       │
        │  - LiDAR (360)  │
        │  - Odometry     │
        └────────┬────────┘
                 │
    ┌────────────▼─────────────┐
    │   TD3 Environment        │
    │  - State observation     │
    │  - Reward calculation    │
    │  - Goal spawning         │
    │  - Collision detection   │
    │  - Episode management    │
    └────────────┬─────────────┘
                 │
    ┌────────────▼─────────────┐
    │      TD3 Agent           │
    │  - Actor network (512)   │
    │  - Critic network (512)  │
    │  - Replay buffer (1M)    │
    │  - Target networks       │
    └────────────┬─────────────┘
                 │
    ┌────────────▼─────────────┐
    │   Training Loop          │
    │  - Episode management    │
    │  - Model checkpointing   │
    │  - Live plotting         │
    │  - Logging               │
    └──────────────────────────┘
```

## 🚀 Quick Start

### Step 1: Launch SAR World

Open a terminal:

```bash
cd /root/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 launch ai_rescuebot sar_gazebo_bringup.launch.py
```

**Expected**: Gazebo opens with SAR room world and TurtleBot3 at spawn position

### Step 2: Start TD3 Training

Open a **new terminal**:

```bash
cd /root/turtlebot3_ws
source install/setup.bash

# Start training
ros2 run ai_rescuebot train_td3_sar

# Or with custom parameters
ros2 run ai_rescuebot train_td3_sar --episodes 20000 --checkpoint-dir ./my_training
```

### Step 3: Monitor Training

Watch the terminal output and matplotlib plot window:

```
Starting TD3 training for 10000 episodes
Observation phase: 25000 steps
======================================================================
Observe: 12450/25000 steps
...
Ep: 10    R: 145      Outcome: Success           Steps: 543   Total: 25543  Time: 12.34s
Ep: 11    R: -156     Outcome: Collision Wall    Steps: 234   Total: 25777  Time: 5.67s
...
Ep: 20    R: 178      Outcome: Success           Steps: 621   Total: 32145  Time: 14.23s
  --> Last 10 eps: Success: 6/10, Avg Reward: 45.3
```

## 📊 Training Process

### Phase 1: Observation (First 25,000 steps)

- Agent takes **random actions** for exploration
- Fills replay buffer with diverse experiences
- **No training** occurs yet
- Terminal shows: `Observe: X/25000 steps`

### Phase 2: Training

Each episode:

1. **Episode Start**:
   - Robot spawns at default position (-3.58, 1.749, yaw=0.0)
   - Random goal generated in valid position
   - Initial distance to goal calculated

2. **Episode Loop**:
   - Agent observes: LiDAR (360 scans) + goal info + previous actions
   - Agent selects action: [linear_vel, angular_vel]
   - Environment executes action
   - Reward calculated based on progress and proximity
   - Episode continues until termination

3. **Episode Termination**:
   - ✅ **Success**: Robot within 0.3m of goal → Reward: **+200**
   - ❌ **Collision**: Robot within 0.15m of obstacle → Reward: **-200**
   - ⏱️ **Timeout**: 60 seconds elapsed → Reward: **-50**
   - 🔄 **Tumble**: Robot tilted > 3.4° → Reward: **-200**

4. **Episode End**:
   - Robot stops
   - Episode statistics logged
   - Robot respawns at default position
   - New goal generated for next episode

### Training Updates

**Every 10 Episodes**:
- Live plots updated
- Success rate printed
- Average reward printed
- Plot saved as PNG

**Every 100 Episodes**:
- Model checkpoint saved
- Replay buffer saved
- Plot data saved

## 📈 Live Training Plots

A matplotlib window displays 4 real-time plots:

```
┌─────────────────────────────────┬─────────────────────────────────┐
│   Outcomes per Episode          │   Avg Critic Loss per Episode   │
│   (Stacked Area Chart)          │   (Line Chart)                  │
│                                 │                                 │
│   Legend:                       │   Shows convergence of          │
│   - Success (green)             │   value function learning       │
│   - Collision Wall (red)        │                                 │
│   - Collision Obstacle (cyan)   │                                 │
│   - Timeout (magenta)           │                                 │
│   - Unknown (blue)              │                                 │
│   - Tumble (yellow)             │                                 │
├─────────────────────────────────┼─────────────────────────────────┤
│   Avg Actor Loss per Episode    │   Avg Reward over 10 Episodes   │
│   (Line Chart)                  │   (Line Chart)                  │
│                                 │                                 │
│   Shows convergence of          │   Shows overall training        │
│   policy learning               │   progress and stability        │
└─────────────────────────────────┴─────────────────────────────────┘
```

## 🎯 Command Line Options

```bash
ros2 run ai_rescuebot train_td3_sar [OPTIONS]
```

### Options:

| Option | Description | Default |
|--------|-------------|---------|
| `--episodes` | Total episodes to train | 10000 |
| `--checkpoint-dir` | Directory for checkpoints | `./checkpoints/td3_sar` |
| `--load-model` | Path to model to continue training | None |
| `--start-episode` | Starting episode number | 0 |

### Examples:

**Train from scratch (10k episodes)**:
```bash
ros2 run ai_rescuebot train_td3_sar
```

**Train for 20k episodes with custom directory**:
```bash
ros2 run ai_rescuebot train_td3_sar --episodes 20000 --checkpoint-dir ./my_td3_training
```

**Resume training from checkpoint**:
```bash
ros2 run ai_rescuebot train_td3_sar \
    --load-model ./checkpoints/td3_sar/session_20250119_143022/model_ep500.pt \
    --start-episode 500 \
    --episodes 20000
```

## 📁 Training Files

All training files are organized in the checkpoint directory:

```
checkpoints/td3_sar/
└── session_20250119_143022/
    ├── model_ep100.pt              # Model checkpoint at episode 100
    ├── model_ep200.pt              # Model checkpoint at episode 200
    ├── model_ep500.pt              # Model checkpoint at episode 500
    ├── model_latest.pt             # Latest model (always updated)
    │
    ├── replay_buffer_ep100.pkl     # Replay buffer at episode 100
    ├── replay_buffer.pkl           # Latest replay buffer
    │
    ├── plot_data_ep100.npz         # Plot data at episode 100
    ├── plot_data.npz               # Latest plot data
    │
    ├── training_log.csv            # Detailed episode statistics
    └── _training_figure.png        # Latest training plot image
```

### training_log.csv Format

```csv
episode,reward,outcome,duration,steps,total_steps,buffer_size,avg_critic_loss,avg_actor_loss
1,145.3,1,12.34,543,25543,25543,0.523,0.124
2,-156.7,2,5.67,234,25777,25777,0.498,0.118
...
```

Outcome codes: 0=Unknown, 1=Success, 2=Collision Wall, 3=Collision Obstacle, 4=Timeout, 5=Tumble

## ⚙️ Configuration

Edit `ai_rescuebot/drl_training/settings.py` to customize training:

### Episode Parameters

```python
EPISODE_TIMEOUT_SECONDS = 60.0    # Max time per episode
THRESHOLD_GOAL = 0.30             # Distance to reach goal (meters)
THRESHOLD_COLLISION = 0.15        # Distance for collision (meters)
ENABLE_BACKWARD = True            # Allow backward movement
ENABLE_MOTOR_NOISE = False        # Add noise to motor commands
```

### TD3 Hyperparameters

```python
HIDDEN_SIZE = 512                 # Network hidden layer size
BATCH_SIZE = 128                  # Training batch size
BUFFER_SIZE = 1000000             # Replay buffer size
LEARNING_RATE = 0.0003            # Learning rate (both networks)
DISCOUNT_FACTOR = 0.99            # Gamma (reward discount)
TAU = 0.003                       # Soft update rate
POLICY_NOISE = 0.2                # Target policy noise
POLICY_NOISE_CLIP = 0.5           # Noise clipping range
POLICY_UPDATE_FREQUENCY = 2       # Delayed actor updates
```

### Training Control

```python
OBSERVE_STEPS = 25000             # Random action steps at start
MODEL_STORE_INTERVAL = 100        # Save model every N episodes
GRAPH_DRAW_INTERVAL = 10          # Update plot every N episodes
GRAPH_AVERAGE_REWARD = 10         # Avg reward window size
```

### Speed Limits

```python
SPEED_LINEAR_MAX = 0.3            # Max linear velocity (m/s)
SPEED_ANGULAR_MAX = 1.0           # Max angular velocity (rad/s)
```

## 🎁 Reward Function

Based on turtlebot3_drlnav **"Function G"** (best performing):

### Terminal Rewards

| Outcome | Reward |
|---------|--------|
| Success | +200 |
| Collision | -200 |
| Timeout | -50 |
| Tumble | -200 |

### Step Rewards

Component | Value | Description
----------|-------|------------
Distance progress | +10 max | Progress toward goal
Goal proximity | +20 max | Bonus when near goal
Obstacle penalty | -5 max | Penalty for being near obstacles
Heading alignment | +0.5 max | Reward for facing goal
Forward motion | +0.2 max | Encourage forward movement
Angular velocity | -0.1 max | Penalize excessive turning

Formula in `ai_rescuebot/drl_training/reward.py`

## 🎯 Goal Generation

Goals are randomly spawned in valid positions:

### Room 1
- **Bounds**: x ∈ [-4.8, 4.8], y ∈ [-3.8, 3.8]
- **Clearance**: 0.6m from all obstacles

### Room 2
- **Bounds**: x ∈ [5.2, 14.8], y ∈ [-3.8, 3.8]
- **Clearance**: 0.6m from all obstacles

### Validation
- Checks distance from 50+ obstacle positions
- Ensures minimum clearance for safe navigation
- Falls back to safe position (-2.0, 0.0) if generation fails after 100 attempts

### Obstacle Positions

All obstacle positions are precisely defined in `settings.py`:

- **Room 1**: bars, boxes, debris (32 obstacles)
- **Room 2**: cylinders, walls, boxes (24 obstacles)
- **Total**: 56 obstacle zones with accurate (x, y, radius)

## 📊 Expected Training Results

### Training Progress

| Episodes | Success Rate | Avg Reward | Episode Duration |
|----------|--------------|------------|------------------|
| 0-500 | 10-20% | -50 to 0 | 20-40s |
| 500-2000 | 30-50% | 0 to 50 | 15-30s |
| 2000-5000 | 50-70% | 50 to 100 | 12-20s |
| 5000+ | 70-90% | 100 to 150 | 10-15s |

### Converged Policy

After 5000-10000 episodes:
- ✅ **Success rate**: 70-90%
- 📈 **Average reward**: 100-150
- ⏱️ **Episode duration**: 10-20 seconds (successful)
- 🎯 **Navigation quality**: Smooth, obstacle-avoiding paths

## 🔧 Troubleshooting

### Robot Doesn't Move

**Symptoms**: Robot static in Gazebo

**Solutions**:
- Check Gazebo is running (not paused)
- Verify ROS2 topics: `ros2 topic list | grep cmd_vel`
- Check terminal for connection errors
- Restart Gazebo and training script

### Training Very Slow

**Symptoms**: < 1 episode per minute

**Solutions**:
- Check Gazebo real-time factor (should be near 1.0)
- Close unnecessary programs
- Use GPU if available: `nvidia-smi`
- Reduce `MODEL_STORE_INTERVAL` to save less frequently
- Disable matplotlib window (comment out plot updates)

### Low Success Rate

**Symptoms**: < 20% success after 2000 episodes

**Solutions**:
- Train longer (10k+ episodes)
- Check reward function is appropriate
- Verify goal positions are valid
- Tune hyperparameters:
  - Increase `LEARNING_RATE` to 0.001
  - Decrease `BATCH_SIZE` to 64
  - Adjust reward weights in `reward.py`

### Goals Spawn on Obstacles

**Symptoms**: Episodes immediately end in collision

**Solutions**:
- Verify obstacle positions in `settings.py`
- Increase `min_clearance` in `generate_random_goal()`
- Check `is_position_valid()` function
- Review Gazebo world file for obstacle positions

### Plot Window Doesn't Appear

**Symptoms**: No matplotlib window

**Solutions**:
```bash
# Check matplotlib backend
echo $MPLBACKEND  # Should be TkAgg

# Set correct backend
export MPLBACKEND=TkAgg

# Install tkinter if missing
apt-get install python3-tk

# Restart training
```

### High Critic/Actor Loss

**Symptoms**: Losses don't decrease, remain > 5.0

**Solutions**:
- Verify observation phase completed (25k steps)
- Check replay buffer has diverse experiences
- Reduce `LEARNING_RATE` to 0.0001
- Increase `BATCH_SIZE` to 256
- Verify state normalization (LiDAR, goal distance)

### Robot Gets Stuck

**Symptoms**: Robot oscillates or stops

**Solutions**:
- Adjust reward for forward motion
- Penalize low velocities in reward function
- Add reward for moving away from visited areas
- Increase exploration noise during training

## 🚀 Using Trained Models

### Loading a Model

```python
from ai_rescuebot.drl_training.td3_agent import TD3
import torch
import numpy as np

# Setup
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
agent = TD3(device=device)

# Load trained model
model_path = './checkpoints/td3_sar/session_XXX/model_ep5000.pt'
agent.load_model(model_path)

print('Model loaded successfully!')
```

### Inference (No Training)

```python
# Get state from environment
state = env.get_state(0.0, 0.0)

# Get action (no exploration noise)
action = agent.get_action(
    state,
    is_training=False,  # Disable exploration
    step=0,
    visualize=False
)

# Action is [linear_vel, angular_vel] in range [-1, 1]
print(f'Action: linear={action[0]:.3f}, angular={action[1]:.3f}')

# Execute in environment
next_state, reward, done, info = env.step(action)
```

### Deployment Script

Create `test_trained_model.py`:

```python
#!/usr/bin/env python3
import rclpy
from ai_rescuebot.drl_training.td3_agent import TD3
from ai_rescuebot.drl_training.sar_environment import SAREnvironment
import torch

def main():
    rclpy.init()
    
    # Load model
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    agent = TD3(device=device)
    agent.load_model('./checkpoints/td3_sar/session_XXX/model_latest.pt')
    
    # Create environment
    env = SAREnvironment()
    
    # Test for 10 episodes
    for episode in range(10):
        state = env.reset()
        done = False
        episode_reward = 0
        
        while not done:
            action = agent.get_action(state, is_training=False, step=0)
            state, reward, done, info = env.step(action)
            episode_reward += reward
            rclpy.spin_once(env, timeout_sec=0.001)
            
        print(f"Episode {episode + 1}: Reward={episode_reward:.1f}, "
              f"Outcome={info['outcome']}")
    
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🧪 Advanced Topics

### Curriculum Learning

Start with easier goals (closer), progressively increase difficulty:

```python
# In sar_environment.py, modify generate_random_goal()
def generate_random_goal(self, episode):
    max_distance = min(10.0, 3.0 + episode * 0.01)  # Increase with episodes
    # Generate goal within max_distance of spawn point
    ...
```

### Transfer Learning

Use trained model as starting point for new environment:

```bash
# Train on new world, starting from SAR-trained model
ros2 run ai_rescuebot train_td3_sar \
    --load-model ./checkpoints/td3_sar/model_latest.pt \
    --checkpoint-dir ./checkpoints/td3_new_world
```

### Reward Shaping

Experiment with different reward components:

```python
# In reward.py
def get_reward(outcome, action_linear, action_angular, 
               goal_distance, goal_angle, obstacle_distance):
    # Add custom rewards
    reward += smooth_velocity_bonus(action_linear, action_angular)
    reward += exploration_bonus(visited_positions)
    reward += time_penalty(episode_duration)
    ...
```

## 📚 References

- **Paper**: [TD3: Addressing Function Approximation Error in Actor-Critic Methods](https://arxiv.org/abs/1802.09477)
- **Implementation**: [turtlebot3_drlnav](https://github.com/tomasvr/turtlebot3_drlnav)
- **Robot**: [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- **Simulation**: [Gazebo](https://gazebosim.org/)

## 🤝 Contributing

Found a bug or have improvements? 

1. Test your changes thoroughly
2. Document modifications in code
3. Update this guide if adding features
4. Submit with clear description

## 📝 License

MIT License - See package LICENSE file

---

**Happy Training! 🤖🚀**

For questions or issues, check:
- Training logs: `training_log.csv`
- Plot visualization: `_training_figure.png`
- Terminal output for error messages
- ROS2 topics: `ros2 topic list`

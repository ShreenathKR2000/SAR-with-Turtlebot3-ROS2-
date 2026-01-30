"""
Training settings for TD3 agent in SAR room world.
Based on turtlebot3_drlnav implementation.
"""

# ===================================================================== #
#                       ENVIRONMENT SETTINGS                            #
# ===================================================================== #

# Episode parameters
EPISODE_TIMEOUT_SECONDS = 80.0  # 60s timeout per episode
ENABLE_BACKWARD = False         # Allow backward movement
ENABLE_MOTOR_NOISE = True      # Add noise to motor commands

# Reward function
REWARD_FUNCTION = "G"          # Reward function type

# Goal and collision thresholds  
THRESHOLD_COLLISION = 0.15     # meters, minimum distance that counts as collision
THRESHOLD_GOAL = 0.30          # meters, minimum distance that counts as reaching goal

# Speed limits
SPEED_LINEAR_MAX = 0.3         # m/s
SPEED_ANGULAR_MAX = 2.0        # rad/s

# Lidar
NUM_SCAN_SAMPLES = 360         # Number of lidar scan samples
LIDAR_DISTANCE_CAP = 3.5       # Maximum lidar distance (meters)

# NOTE: Real TurtleBot3 Scan Compatibility
# - Gazebo simulation: 360 samples
# - Real TurtleBot3 LDS-01: 223 samples  
# The TD3 controller uses BINNING (24 bins) which works with any scan size!
# Optionally use laser_scan_adapter to interpolate 223→360 for exact match.

# SAR Room World Bounds
ARENA_LENGTH = 15.0            # Room 1 + Room 2 total length (meters)
ARENA_WIDTH = 8.0              # Room width (meters)

# Default spawn position (from launch file)
DEFAULT_SPAWN_X = -4.0
DEFAULT_SPAWN_Y = -1.0
DEFAULT_SPAWN_Z = 0.0078
DEFAULT_SPAWN_YAW = 0.0

# ===================================================================== #
#                       DRL ALGORITHM SETTINGS                          #
# ===================================================================== #

# TD3 Algorithm Parameters
ACTION_SIZE = 2                # Linear and angular velocity
HIDDEN_SIZE = 512              # Number of neurons in hidden layers

BATCH_SIZE = 256               # Samples per training batch
BUFFER_SIZE = 1000000          # Replay buffer size
DISCOUNT_FACTOR = 0.99         # Gamma
LEARNING_RATE = 0.0003         # Learning rate for both actor and critic
TAU = 0.003                    # Soft update parameter

# TD3 specific
POLICY_NOISE = 0.2             # Noise added to target policy
POLICY_NOISE_CLIP = 0.5        # Range to clip target policy noise
POLICY_UPDATE_FREQUENCY = 2    # Delayed policy updates

# Training parameters
OBSERVE_STEPS = 10000          # Random actions at start for exploration (reduced for faster training start)
EPSILON_DECAY = 0.9995         # Not used in TD3 but kept for compatibility
EPSILON_MINIMUM = 0.05

STEP_TIME = 0.0                # Delay between steps (0 for max speed)

# Storage and logging
MODEL_STORE_INTERVAL = 100     # Save model every N episodes
GRAPH_DRAW_INTERVAL = 10       # Update graph every N episodes  
GRAPH_AVERAGE_REWARD = 10      # Average reward over N episodes for plotting

# Episode outcome enumeration
UNKNOWN = 0
SUCCESS = 1
COLLISION_WALL = 2
COLLISION_OBSTACLE = 3
TIMEOUT = 4
TUMBLE = 5
RESULTS_NUM = 6

# Outcome translation
OUTCOME_LABELS = {
    UNKNOWN: "Unknown",
    SUCCESS: "Success",
    COLLISION_WALL: "Collision Wall",
    COLLISION_OBSTACLE: "Collision Obstacle",
    TIMEOUT: "Timeout",
    TUMBLE: "Tumble"
}

# ===================================================================== #
#                    SAR WORLD OBSTACLE POSITIONS                       #
# ===================================================================== #

# Obstacle positions in SAR room world (to avoid spawning goals on them)
# Format: (x, y, radius)
OBSTACLE_POSITIONS = [
    # Room 1 - bars
    (2, 0, 2.2),       # bar3
    
    # Room 1 - boxes (big)
    (-4, 3, 0.8),      # b1
    (-4, -3, 0.8),     # b2
    (0, 3, 0.8),       # b3
    (0, -3, 0.8),      # b4
    
    # Room 1 - debris (small)
    (-3, 0, 0.4),      # d1
    (-1, 1.5, 0.4),    # d2
    (-1, -1.5, 0.4),   # d3
    (1.5, 2, 0.4),     # d4
    (1.5, -2, 0.4),    # d5
    (4, 0, 0.5),       # d6
    
    # Room 1 - extra obstacles
    (-4.5, 1.5, 0.3),  # x1
    (-3.5, -1.5, 0.3), # x2
    (-0.5, 0, 0.4),    # x3
    (3, 1.5, 0.3),     # x4
    (3, -1.5, 0.3),    # x5
    (6, 0, 0.6),       # door_block
    
    # Room 2 - cylinders
    (7, 0, 0.25),      # c1
    (8, 1.5, 0.25),    # c2
    (8, -1.5, 0.25),   # c3
    (9, 3, 0.25),      # c4
    (9, -3, 0.25),     # c5
    (10, 0, 0.25),     # c6
    (11, 2, 0.25),     # c7
    (11, -2, 0.25),    # c8
    (12, 0.5, 0.25),   # c9
    (12, -0.5, 0.25),  # c10
    (7.5, 2.5, 0.2),   # c11
    (7.5, -2.5, 0.2),  # c12
    (9.5, 1.5, 0.2),   # c13
    (9.5, -1.5, 0.2),  # c14
    
    # Room 2 - walls/bars
    (13, 3, 1.6),      # zw1
    (13, -3, 1.6),     # zw2
    (14, 0, 2.2),      # zw3
    
    # Room 2 - boxes
    (14, 3.5, 0.8),    # fb1
    (14, -3.5, 0.8),   # fb2
    (13.5, 1.5, 0.5),  # fb3
    (13.5, -1.5, 0.5), # fb4
    
    # Room 2 - extra debris
    (8.5, 3, 0.4),     # r2x1
    (10.5, 3, 0.4),    # r2x2
    (10.5, -3, 0.4),   # r2x3
]

# Wall boundaries
# Room 1: x: [-5, 5], y: [-4, 4]
# Room 2: x: [5, 15], y: [-4, 4]
# Door opening: x: 5, y: [-1, 1]
ROOM1_BOUNDS = {"x_min": -4.8, "x_max": 4.8, "y_min": -3.8, "y_max": 3.8}
ROOM2_BOUNDS = {"x_min": 5.2, "x_max": 14.8, "y_min": -3.8, "y_max": 3.8}

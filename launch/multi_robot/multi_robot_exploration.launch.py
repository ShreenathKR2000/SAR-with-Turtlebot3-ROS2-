#!/usr/bin/env python3
"""
Multi-Robot Autonomous Exploration Launch File with MPPI Controller

Features:
- Decentralized mapping (each robot runs own SLAM)
- Centralized map merging (multirobot_map_merge)
- MPPI controller for better dynamic obstacle avoidance
- TD3 support (optional)
- Cooperative frontier exploration

Architecture:
- Robot1 & Robot2: Local SLAM + Nav2 (MPPI) + Frontier Detection
- Map Merge Node: Combines local maps into global map
- Cooperative exploration avoiding duplicate work

Usage:
  ros2 launch SAR_TB3 multi_robot_exploration.launch.py
  ros2 launch SAR_TB3 multi_robot_exploration.launch.py use_td3:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate multi-robot exploration with map merging and MPPI"""
    
    # Package directories
    pkg_sar_tb3 = get_package_share_directory('SAR_TB3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Paths
    world_file = os.path.join(pkg_sar_tb3, 'worlds', 'sar_room_world.sdf')
    nav2_params_robot1 = os.path.join(pkg_sar_tb3, 'config', 'nav2_params_robot1.yaml')
    nav2_params_robot2 = os.path.join(pkg_sar_tb3, 'config', 'nav2_params_robot2.yaml')
    slam_params_robot1 = os.path.join(pkg_sar_tb3, 'config', 'slam_params_robot1.yaml')
    slam_params_robot2 = os.path.join(pkg_sar_tb3, 'config', 'slam_params_robot2.yaml')
    rviz_config = os.path.join(pkg_sar_tb3, 'config', 'multirobot_rviz.rviz')
    
    # Model paths for Gazebo
    sar_tb3_models = os.path.join(pkg_sar_tb3, 'models')
    tb3_models = os.path.join(pkg_turtlebot3_gazebo, 'models')
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    tb3_model_file = os.path.join(tb3_models, f'turtlebot3_{tb3_model}', 'model.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_td3 = LaunchConfiguration('use_td3', default='false')
    
    # Robot 1 spawn positions
    robot1_x = LaunchConfiguration('robot1_x', default='-3.5')
    robot1_y = LaunchConfiguration('robot1_y', default='1.7')
    robot1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')
    
    # Robot 2 spawn positions
    robot2_x = LaunchConfiguration('robot2_x', default='-3.0')
    robot2_y = LaunchConfiguration('robot2_y', default='-1.7')
    robot2_yaw = LaunchConfiguration('robot2_yaw', default='0.0')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_use_td3 = DeclareLaunchArgument(
        'use_td3',
        default_value='false',
        description='Use TD3 for local control (requires trained model)'
    )
    
    # Set GAZEBO_MODEL_PATH
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    merged_model_path = ':'.join([p for p in [existing_model_path, sar_tb3_models, tb3_models] if p])
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=merged_model_path
    )
    
    # ============================================
    # GAZEBO WORLD (SHARED)
    # ============================================
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # ============================================
    # ROBOT 1 - Complete Stack
    # ============================================
    
    # Robot 1 State Publisher
    robot1_state_publisher = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    ])
    
    # Spawn Robot 1
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', tb3_model_file,
            '-x', robot1_x,
            '-y', robot1_y,
            '-z', '0.01',
            '-Y', robot1_yaw,
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )
    
    # Robot 1 SLAM - Manual Launch for proper parameter loading
    robot1_slam = GroupAction([
        PushRosNamespace('robot1'),
        
        # SLAM Toolbox Node - async mode with proper lifecycle
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_robot1,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
    
    # Robot 1 Nav2 - Manual Node Launch with MPPI
    robot1_nav2 = GroupAction([
        PushRosNamespace('robot1'),
        
        # Controller (MPPI)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_robot1],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params_robot1],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params_robot1],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params_robot1],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ])
    
    # Robot 1 Frontier Detector
    robot1_frontier = Node(
        package='SAR_TB3',
        executable='frontier_detector',
        namespace='robot1',
        name='frontier_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': 'map',
            'min_frontier_size': 5,
            'frontier_search_radius': 10.0,
            'update_frequency': 2.0
        }],
        output='screen'
    )
    
    # Robot 1 Explorer
    robot1_explorer = Node(
        package='SAR_TB3',
        executable='autonomous_explorer',
        namespace='robot1',
        name='autonomous_explorer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_td3_control': use_td3,
            'exploration_timeout': 60.0,
            'retry_on_failure': True,
            'max_retries': 3
        }],
        output='screen'
    )
    
    # ============================================
    # ROBOT 2 - Complete Stack
    # ============================================
    
    # Robot 2 State Publisher
    robot2_state_publisher = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    ])
    
    # Spawn Robot 2
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-file', tb3_model_file,
            '-x', robot2_x,
            '-y', robot2_y,
            '-z', '0.01',
            '-Y', robot2_yaw,
            '-robot_namespace', 'robot2'
        ],
        output='screen'
    )
    
    # Robot 2 SLAM - Manual Launch for proper parameter loading
    robot2_slam = GroupAction([
        PushRosNamespace('robot2'),
        
        # SLAM Toolbox Node - async mode with proper lifecycle
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_robot2,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
    
    # Robot 2 Nav2 - Manual Node Launch with MPPI
    robot2_nav2 = GroupAction([
        PushRosNamespace('robot2'),
        
        # Controller (MPPI)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_robot2],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params_robot2],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params_robot2],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params_robot2],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ])
    
    # Robot 2 Frontier Detector
    robot2_frontier = Node(
        package='SAR_TB3',
        executable='frontier_detector',
        namespace='robot2',
        name='frontier_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': 'map',
            'min_frontier_size': 5,
            'frontier_search_radius': 10.0,
            'update_frequency': 2.0
        }],
        output='screen'
    )
    
    # Robot 2 Explorer
    robot2_explorer = Node(
        package='SAR_TB3',
        executable='autonomous_explorer',
        namespace='robot2',
        name='autonomous_explorer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_td3_control': use_td3,
            'exploration_timeout': 60.0,
            'retry_on_failure': True,
            'max_retries': 3
        }],
        output='screen'
    )
    
    # ============================================
    # MAP MERGE - Central Coordination
    # ============================================
    
    map_merge = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_map_topic': 'map',
            'robot_namespace': 'robot',
            'merged_map_topic': 'map',
            'world_frame': 'map',
            'known_init_poses': True,
            'merging_rate': 2.0,
            'discovery_rate': 0.05,
            'estimation_rate': 0.1,
            'estimation_confidence': 1.0,
            
            # Initial poses matching spawn positions
            'robot1/map_merge/init_pose_x': -3.5,
            'robot1/map_merge/init_pose_y': 1.7,
            'robot1/map_merge/init_pose_z': 0.0,
            'robot1/map_merge/init_pose_yaw': 0.0,
            
            'robot2/map_merge/init_pose_x': -3.0,
            'robot2/map_merge/init_pose_y': -1.7,
            'robot2/map_merge/init_pose_z': 0.0,
            'robot2/map_merge/init_pose_yaw': 0.0,
        }],
        remappings=[
            ('/robot1/map', '/robot1/map'),
            ('/robot2/map', '/robot2/map'),
        ],
        output='screen'
    )
    
    # ============================================
    # CENTRAL COORDINATOR
    # ============================================
    
    central_coordinator = Node(
        package='SAR_TB3',
        executable='map_fusion_coordinator',
        name='map_fusion_coordinator',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespaces': ['robot1', 'robot2'],
            'world_frame': 'map',
            'fusion_rate': 1.0,
            'frontier_min_size': 10,
            'min_robot_distance': 2.0
        }],
        output='screen'
    )
    
    # ============================================
    # RVIZ
    # ============================================
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ============================================
    # TIMED LAUNCH SEQUENCE
    # ============================================
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_use_td3,
        
        # Environment
        set_gazebo_model_path,
        
        # Step 0: Gazebo (t=0)
        LogInfo(msg='[0s] Starting Gazebo world with SAR room...'),
        gazebo_server,
        gazebo_client,
        
        # Step 1: Spawn robots (t=8s) - WAIT FOR GAZEBO SERVICE
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg='[8s] Spawning Robot 1 and Robot 2...'),
                robot1_state_publisher,
                robot2_state_publisher,
                spawn_robot1,
                spawn_robot2
            ]
        ),
        
        # Step 2: SLAM (t=14s) - WAIT FOR ODOMETRY TOPICS
        TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg='[6s] Starting SLAM Toolbox for both robots...'),
                robot1_slam,
                robot2_slam
            ]
        ),
        
        # Step 3: Map Merge (t=20s) - WAIT FOR SLAM MAPS
        TimerAction(
            period=20.0,
            actions=[
                LogInfo(msg='[20s] Starting Map Merge node...'),
                map_merge
            ]
        ),
        
        # Step 4: Nav2 with MPPI (t=22s) - WAIT FOR MAPS
        TimerAction(
            period=22.0,
            actions=[
                LogInfo(msg='[22s] Starting Nav2 stacks with MPPI controller...'),
                robot1_nav2,
                robot2_nav2
            ]
        ),
        
        # Step 5: RViz (t=24s)
        TimerAction(
            period=24.0,
            actions=[
                LogInfo(msg='[24s] Starting RViz2...'),
                rviz
            ]
        ),
        
        # Step 6: Frontier detection and exploration (t=15s)
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='[15s] Starting frontier detection and exploration...'),
                robot1_frontier,
                robot1_explorer,
                robot2_frontier,
                robot2_explorer
            ]
        ),
        
        # Step 7: Central coordinator (t=28s)
        TimerAction(
            period=28.0,
            actions=[
                LogInfo(msg='[28s] Starting central coordinator...'),
                central_coordinator,
                LogInfo(msg='Multi-robot autonomous exploration system ready!')
            ]
        ),
    ])

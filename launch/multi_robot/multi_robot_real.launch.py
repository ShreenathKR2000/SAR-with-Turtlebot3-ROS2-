#!/usr/bin/env python3
"""
Multi-Robot Autonomous Exploration Launch File (Real Hardware)

Deploys 2 TurtleBot3 robots with centralized coordination:
- Robot1 and Robot2: Run locally on each robot (SLAM + Nav2 + TD3)
- Central PC: Runs map fusion and coordination

DEPLOYMENT ARCHITECTURE:
┌─────────────────┐         ┌─────────────────┐
│   Robot 1 PC    │         │   Robot 2 PC    │
│  (On Robot 1)   │         │  (On Robot 2)   │
├─────────────────┤         ├─────────────────┤
│ • Lidar Driver  │         │ • Lidar Driver  │
│ • SLAM Toolbox  │         │ • SLAM Toolbox  │
│ • Nav2 Stack    │         │ • Nav2 Stack    │
│ • TD3 Control   │         │ • TD3 Control   │
│ • Frontier Det. │         │ • Frontier Det. │
│ • Explorer Node │         │ • Explorer Node │
└────────┬────────┘         └────────┬────────┘
         │                           │
         │    WiFi / ROS 2 DDS      │
         │                           │
         └───────────┬───────────────┘
                     │
         ┌───────────▼────────────┐
         │    Central PC          │
         ├────────────────────────┤
         │ • Map Fusion           │
         │ • Global Frontiers     │
         │ • Goal Assignment      │
         │ • RViz Dashboard       │
         └────────────────────────┘

USAGE:
1. On Central PC: Launch this file with mode:=central
2. On Robot 1: Launch this file with mode:=robot robot_name:=robot1
3. On Robot 2: Launch this file with mode:=robot robot_name:=robot2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_robot_nodes(context, *args, **kwargs):
    """Generate nodes for robot mode"""
    
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_td3 = LaunchConfiguration('use_td3').perform(context)
    td3_model = LaunchConfiguration('td3_model').perform(context)
    
    pkg_sar_tb3 = get_package_share_directory('SAR_TB3')
    pkg_turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    nodes = []
    
    # TurtleBot3 bringup (lidar driver, robot state publisher)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )
    
    robot_bringup_group = GroupAction([
        PushRosNamespace(robot_name),
        robot_bringup
    ])
    nodes.append(robot_bringup_group)
    
    # SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(pkg_sar_tb3, 'config', 'slam_params_real.yaml')
        }.items()
    )
    
    slam_group = GroupAction([
        PushRosNamespace(robot_name),
        slam
    ])
    nodes.append(slam_group)
    
    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_sar_tb3, 'config', 'nav2_params_real.yaml')
        }.items()
    )
    
    nav2_group = GroupAction([
        PushRosNamespace(robot_name),
        nav2
    ])
    nodes.append(nav2_group)
    
    # Laser scan adapter (optional - interpolate 223 → 360 samples)
    scan_adapter = Node(
        package='SAR_TB3',
        executable='laser_scan_adapter',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': False,
            'interpolation_method': 'linear'
        }],
        remappings=[
            ('/scan', f'/{robot_name}/scan_raw'),
            ('/scan_adapted', f'/{robot_name}/scan')
        ],
        output='screen'
    )
    # Uncomment if you want scan adaptation:
    # nodes.append(scan_adapter)
    
    # Frontier Detector
    frontier_detector = Node(
        package='SAR_TB3',
        executable='frontier_detector',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': False,
            'map_topic': f'/{robot_name}/map',
            'robot_base_frame': f'{robot_name}/base_link',
            'min_frontier_size': 5
        }],
        remappings=[
            ('/map', f'/{robot_name}/map'),
            ('/frontier_goal', f'/{robot_name}/frontier_goal'),
            ('/frontier_markers', f'/{robot_name}/frontier_markers')
        ],
        output='screen'
    )
    nodes.append(frontier_detector)
    
    # Autonomous Explorer
    autonomous_explorer = Node(
        package='SAR_TB3',
        executable='autonomous_explorer',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': False,
            'enable_td3_control': use_td3 == 'true',
            'exploration_timeout': 60.0
        }],
        remappings=[
            ('/map', f'/{robot_name}/map'),
            ('/frontier_goal', f'/{robot_name}/frontier_goal'),
            ('/exploration_status', f'/{robot_name}/exploration_status')
        ],
        output='screen'
    )
    nodes.append(autonomous_explorer)
    
    # TD3 Waypoint Controller (if enabled)
    if use_td3 == 'true' and td3_model:
        td3_controller = Node(
            package='SAR_TB3',
            executable='td3_waypoint_controller',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': False,
                'model_path': td3_model
            }],
            remappings=[
                ('/scan', f'/{robot_name}/scan'),
                ('/odom', f'/{robot_name}/odom'),
                ('/plan', f'/{robot_name}/plan'),
                ('/cmd_vel', f'/{robot_name}/cmd_vel')
            ],
            output='screen'
        )
        nodes.append(td3_controller)
    
    return nodes


def generate_central_nodes(context, *args, **kwargs):
    """Generate nodes for central PC mode"""
    
    pkg_sar_tb3 = get_package_share_directory('SAR_TB3')
    
    nodes = []
    
    # Map Fusion Coordinator
    coordinator = Node(
        package='SAR_TB3',
        executable='map_fusion_coordinator',
        parameters=[{
            'use_sim_time': False,
            'robot_namespaces': ['robot1', 'robot2'],
            'world_frame': 'world',
            'fusion_rate': 1.0,
            'frontier_min_size': 10,
            'min_robot_distance': 2.0
        }],
        output='screen'
    )
    nodes.append(coordinator)
    
    # RViz for visualization
    rviz_config = os.path.join(pkg_sar_tb3, 'config', 'multirobot_rviz.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    nodes.append(rviz)
    
    return nodes


def generate_launch_description():
    """Generate launch description based on mode"""
    
    # Launch arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='central',
        description='Launch mode: "central" for coordinator PC, "robot" for robot PC'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Robot namespace (robot1 or robot2) - only used in robot mode'
    )
    
    declare_use_td3 = DeclareLaunchArgument(
        'use_td3',
        default_value='false',
        description='Use TD3 for local control'
    )
    
    declare_td3_model = DeclareLaunchArgument(
        'td3_model',
        default_value='',
        description='Path to trained TD3 model'
    )
    
    mode = LaunchConfiguration('mode')
    
    # Robot mode nodes
    robot_nodes = OpaqueFunction(
        function=generate_robot_nodes,
        condition=IfCondition(
            [mode, ' == robot']  # Launch if mode is 'robot'
        )
    )
    
    # Central mode nodes
    central_nodes = OpaqueFunction(
        function=generate_central_nodes,
        condition=IfCondition(
            [mode, ' == central']  # Launch if mode is 'central'
        )
    )
    
    return LaunchDescription([
        # Arguments
        declare_mode,
        declare_robot_name,
        declare_use_td3,
        declare_td3_model,
        
        # Conditional node launches
        LogInfo(msg=['Launching in mode: ', mode]),
        robot_nodes,
        central_nodes,
    ])

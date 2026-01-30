#!/usr/bin/env python3
"""
Real TurtleBot3 Autonomous Exploration Launch File.
Brings up complete system on physical robot hardware.

Prerequisites:
- TurtleBot3 powered on and connected via network
- Robot's ROS 2 workspace sourced
- Correct TURTLEBOT3_MODEL set (burger/waffle/waffle_pi)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sar_tb3 = get_package_share_directory('SAR_TB3')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # Real robot!
    td3_model = LaunchConfiguration('td3_model', default='')
    use_td3 = LaunchConfiguration('use_td3', default='false')
    adapt_scan = LaunchConfiguration('adapt_scan', default='true')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real robot)'
    )
    
    declare_td3_model = DeclareLaunchArgument(
        'td3_model',
        default_value='',
        description='Path to trained TD3 model for waypoint following'
    )
    
    declare_use_td3 = DeclareLaunchArgument(
        'use_td3',
        default_value='false',
        description='Use TD3 for local control (false = use Nav2 DWB controller)'
    )
    
    declare_adapt_scan = DeclareLaunchArgument(
        'adapt_scan',
        default_value='true',
        description='Adapt 223-sample scan to 360 samples (for TD3 compatibility)'
    )
    
    # 1. Robot bringup (assuming robot drivers already running)
    # If not, you may need to launch turtlebot3_bringup here
    # This typically includes: robot_state_publisher, lidar driver, motor controllers
    
    # 2. Laser Scan Adapter (if needed)
    # Adapts real TurtleBot3's 223-sample scan to 360 samples for TD3 model
    laser_adapter = Node(
        package='SAR_TB3',
        executable='laser_scan_adapter',
        name='laser_scan_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_scan_topic': '/scan_raw'},
            {'output_scan_topic': '/scan'},
            {'target_samples': 360},  # Match training
            {'interpolation_method': 'linear'},
        ],
        remappings=[
            ('/scan_raw', '/scan'),  # Read from real scan
            ('/scan', '/scan_adapted'),  # Publish adapted scan
        ]
    )
    
    # 3. Launch SLAM
    slam_launch = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'base_frame': 'base_footprint'},
                    {'odom_frame': 'odom'},
                    {'map_frame': 'map'},
                    {'scan_topic': '/scan_adapted' if adapt_scan else '/scan'},
                    {'mode': 'localization'},  # or 'mapping'
                ]
            )
        ]
    )
    
    # 4. Launch Nav2 (needs params file configured for real robot)
    # Note: You'll need to create nav2_params_real.yaml with real robot settings
    default_params = os.path.join(pkg_sar_tb3, 'config', 'nav2_params_real.yaml')
    
    nav2_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                    '/bringup_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': default_params,
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # 5. RViz (optional, run on remote PC)
    # Not included here - run separately on your laptop/PC
    
    # 6. Frontier Detector
    frontier_detector = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='SAR_TB3',
                executable='frontier_detector',
                name='frontier_detector',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'map_topic': '/map'},
                    {'min_frontier_size': 5},
                    {'frontier_search_radius': 10.0},
                    {'update_frequency': 2.0},
                ]
            )
        ]
    )
    
    # 7. Autonomous Explorer
    autonomous_explorer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='SAR_TB3',
                executable='autonomous_explorer',
                name='autonomous_explorer',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'enable_td3_control': use_td3},
                    {'exploration_timeout': 60.0},
                    {'retry_on_failure': True},
                    {'max_retries': 3},
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_td3_model,
        declare_use_td3,
        declare_adapt_scan,
        
        # Nodes
        laser_adapter,  # Adapt 223 samples to 360
        slam_launch,
        nav2_launch,
        frontier_detector,
        autonomous_explorer,
    ])

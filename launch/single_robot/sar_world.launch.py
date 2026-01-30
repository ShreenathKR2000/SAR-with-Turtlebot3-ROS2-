#!/usr/bin/env python3
"""
SAR World Launch File
Launches only Gazebo with the SAR world - useful for TD3 training
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_sar_tb3 = get_package_share_directory('SAR_TB3')
    
    # Paths
    world_file = os.path.join(pkg_sar_tb3, 'worlds', 'sar_room_world.sdf')
    urdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    
    # Set Gazebo model path to include our custom models
    gazebo_model_path = os.path.join(pkg_sar_tb3, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot state publisher (TF tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file, 'r').read()
        }]
    )
    
    # Spawn TurtleBot3
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation clock'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                            description='Robot spawn X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                            description='Robot spawn Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.01',
                            description='Robot spawn Z position'),
        
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
    ])

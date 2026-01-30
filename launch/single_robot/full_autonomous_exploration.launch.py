#!/usr/bin/env python3
"""
Complete autonomous exploration launch with SAR world, SLAM, Nav2, and QR detection.
Fully independent - no dependencies on ai_rescuebot package.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_td3 = LaunchConfiguration('use_td3', default='false')
    td3_model = LaunchConfiguration('td3_model', default='')
    enable_qr_detection = LaunchConfiguration('enable_qr_detection', default='true')
    
    # Robot spawn pose
    x_pose = LaunchConfiguration('x_pose', default='-3.58')
    y_pose = LaunchConfiguration('y_pose', default='1.749')
    z_pose = LaunchConfiguration('z_pose', default='0.0078')
    yaw = LaunchConfiguration('yaw', default='0.0')
    
    # Get package directories
    pkg_autonomous_exploration = get_package_share_directory('autonomous_exploration_pkg')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    
    # Paths
    world_file_path = os.path.join(pkg_autonomous_exploration, 'worlds', 'sar_room_world.sdf')
    qr_model_path = os.path.join(pkg_autonomous_exploration, 'resource', 'qr_best.pt')
    qr_save_dir = os.path.join(
        os.path.dirname(os.path.dirname(pkg_autonomous_exploration)),
        'src', 'autonomous_exploration_pkg', 'qr_detections'
    )
    nav2_params = os.path.join(pkg_autonomous_exploration, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_tb3_navigation2, 'rviz', 'tb3_navigation2.rviz')
    
    # Set Gazebo model path to include our QR cube model and TB3 models
    autonomous_exploration_models = os.path.join(pkg_autonomous_exploration, 'models')
    tb3_models = os.path.join(pkg_tb3_gazebo, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    merged_model_path = ':'.join(
        [p for p in [existing_model_path, autonomous_exploration_models, tb3_models] if p]
    )
    
    # TurtleBot3 model path
    tb3_model_name = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    tb3_model_folder = 'turtlebot3_' + tb3_model_name
    tb3_model_path = os.path.join(pkg_tb3_gazebo, 'models', tb3_model_folder, 'model.sdf')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_use_td3 = DeclareLaunchArgument(
        'use_td3', default_value='false',
        description='Use TD3 controller for navigation'
    )
    
    declare_td3_model = DeclareLaunchArgument(
        'td3_model', default_value='',
        description='Path to TD3 model checkpoint'
    )
    
    declare_enable_qr = DeclareLaunchArgument(
        'enable_qr_detection', default_value='true',
        description='Enable QR code detection'
    )
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='-3.58',
        description='X position to spawn robot'
    )
    
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='1.749',
        description='Y position to spawn robot'
    )
    
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.0078',
        description='Z position to spawn robot'
    )
    
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Yaw orientation to spawn robot'
    )
    
    # Set environment variable for Gazebo models
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=merged_model_path
    )
    
    # 1. Start Gazebo Server with SAR world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # 2. Start Gazebo Client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # 3. Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 4. Spawn TurtleBot3
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', tb3_model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ],
        output='screen'
    )
    
    # 5. SLAM Toolbox (for mapping)
    slam_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )
    
    # 6. Nav2 (navigation stack)
    # Check if custom params exist, otherwise use default
    if os.path.exists(nav2_params):
        nav2_params_file = nav2_params
    else:
        nav2_params_file = os.path.join(pkg_tb3_navigation2, 'param', 'waffle_pi.yaml')
    
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_navigation2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )
    
    # 7. RViz (optional)
    rviz_cmd = ExecuteProcess(
        condition=lambda context: context.launch_configurations['use_rviz'] == 'true',
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )
    
    # 8. Frontier Detector
    frontier_detector_node = Node(
        package='autonomous_exploration_pkg',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 9. Autonomous Explorer
    autonomous_explorer_node = Node(
        package='autonomous_exploration_pkg',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 10. TD3 Controller (optional)
    td3_controller_node = Node(
        package='autonomous_exploration_pkg',
        executable='td3_waypoint_controller',
        name='td3_waypoint_controller',
        output='screen',
        condition=lambda context: context.launch_configurations['use_td3'] == 'true',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--model', td3_model]
    )
    
    # 11. QR Detector
    qr_detector_node = Node(
        package='autonomous_exploration_pkg',
        executable='qr_detector',
        name='qr_detector',
        output='screen',
        condition=lambda context: context.launch_configurations['enable_qr_detection'] == 'true',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': qr_model_path,
            'save_dir': qr_save_dir,
            'confidence_threshold': 0.6,
            'save_interval': 5.0,
            'camera_topic': '/camera/image_raw'
        }]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_td3)
    ld.add_action(declare_td3_model)
    ld.add_action(declare_enable_qr)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw)
    
    # Set environment
    ld.add_action(set_gazebo_model_path)
    
    # Add launch actions in order
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(frontier_detector_node)
    ld.add_action(autonomous_explorer_node)
    ld.add_action(td3_controller_node)
    ld.add_action(qr_detector_node)
    
    return ld

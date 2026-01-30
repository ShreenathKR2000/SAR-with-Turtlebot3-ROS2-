from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_td3 = LaunchConfiguration('use_td3', default='false')
    td3_model = LaunchConfiguration('td3_model', default='')
    enable_qr_detection = LaunchConfiguration('enable_qr_detection', default='true')
    
    # Get package directories
    pkg_dir = get_package_share_directory('autonomous_exploration_pkg')
    qr_model_path = os.path.join(pkg_dir, 'resource', 'qr_best.pt')
    qr_save_dir = os.path.join(os.path.dirname(pkg_dir), 'autonomous_exploration_pkg', 'qr_detections')

    ld = LaunchDescription([
        DeclareLaunchArgument('use_td3', default_value='false'),
        DeclareLaunchArgument('td3_model', default_value=''),
        DeclareLaunchArgument('enable_qr_detection', default_value='true',
                            description='Enable QR code detection during exploration'),

        Node(
            package='autonomous_exploration_pkg',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen'
        ),

        Node(
            package='autonomous_exploration_pkg',
            executable='autonomous_explorer',
            name='autonomous_explorer',
            output='screen'
        ),

        # TD3 controller optional
        Node(
            package='autonomous_exploration_pkg',
            executable='td3_waypoint_controller',
            name='td3_waypoint_controller',
            output='screen',
            condition=None,
            arguments=['--model', LaunchConfiguration('td3_model')]
        ),
        
        # QR Code Detector
        Node(
            package='autonomous_exploration_pkg',
            executable='qr_detector',
            name='qr_detector',
            output='screen',
            parameters=[{
                'model_path': qr_model_path,
                'save_dir': qr_save_dir,
                'confidence_threshold': 0.6,
                'save_interval': 5.0,
                'camera_topic': '/camera/image_raw'
            }]
        ),
    ])

    return ld

#!/usr/bin/env python3

import sys
import os

# Add SAR_TB3 to Python path
sys.path.insert(0, '/root/turtlebot3_ws/install/SAR_TB3/lib/python3.10/site-packages')

# Set environment
os.environ['AMENT_PREFIX_PATH'] = '/root/turtlebot3_ws/install/SAR_TB3:/opt/ros/humble'
os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

# Import launch
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

# Load and run launch file
launch_file = '/root/turtlebot3_ws/install/SAR_TB3/share/SAR_TB3/launch/single_robot/sar_td3_exploration.launch.py'

# Execute launch file  
import importlib.util
spec = importlib.util.spec_from_file_location("launch_file", launch_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

# Generate launch description
launch_description = module.generate_launch_description()

# Run launch service
ls = LaunchService()
ls.include_launch_description(launch_description)
result = ls.run()
sys.exit(result)

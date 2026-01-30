from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'SAR_TB3'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch', 'single_robot'), glob('launch/single_robot/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'multi_robot'), glob('launch/multi_robot/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resources'), ['resources/training_log.csv']),
        (os.path.join('share', package_name, 'resource'), ['resource/qr_best.pt']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'qr_cube'), glob('models/qr_cube/*.*')),
        (os.path.join('share', package_name, 'models', 'qr_cube', 'materials', 'scripts'), 
         glob('models/qr_cube/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'qr_cube', 'materials', 'textures'), 
         glob('models/qr_cube/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autonomous Explorer',
    maintainer_email='devnull@example.com',
    description='Packaged autonomous exploration nodes with TD3 loader',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_detector = SAR_TB3.frontier_detector:main',
            'autonomous_explorer = SAR_TB3.autonomous_explorer:main',
            'td3_waypoint_controller = SAR_TB3.td3_waypoint_controller:main',
            'qr_detector = SAR_TB3.qr_detector:main',
            'train_td3_sar = SAR_TB3.drl_training.train_td3:main',
        ],
    },
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ldlidar_dir = get_package_share_directory('ldlidar')
    launch_dir = os.path.join(ldlidar_dir, 'launch')
    return LaunchDescription([
        # Node(
        #     package='sensor_bringup',
        #     namespace='',
        #     executable='imu_publisher',
        # ),
        # Node(
        #     package='imu_filter_madgwick',
        #     namespace='',
        #     executable='imu_filter_madgwick_node',
        #     parameters=[
        #         {'publish_tf': True},
        #         {'frame_id': 'imu_link'},
        #         {'use_mag': False}
        #     ]
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ldlidar.launch.py')
            ),
            launch_arguments={
                'serial_port': "/dev/ttyUSB0",
            }.items(),
        )
    ])
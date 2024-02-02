from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            arguments=['0.135', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            arguments=['0.135', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
    ])
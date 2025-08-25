import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('wobl_control'),
        'config',
        'params.yaml'
    )
    return LaunchDescription(
        [
            Node(
                package="wobl_real",
                executable="imu_node",
                name="imu_node",
                output="screen",
            ),
            Node(
                package="wobl_real",
                executable="servo_node",
                name="servo_node",
                output="screen",
            ),
            Node(
                package="wobl_control",
                executable="wobl_controller",
                name="wobl_controller",
                output="screen",
                parameters=[config_file],
            )
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sim",
                executable="mujoco_bridge",
                name="sim",
                output="screen",
            )
        ]
    )

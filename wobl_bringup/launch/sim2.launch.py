from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mujoco_bridge",
                executable="mujoco_bridge",
                name="mujoco_bridge",
                output="screen",
            ),
            Node(
                package="wobl_controllers",
                executable="pid_balance_controller",
                name="pid_balance_controller",
                output="screen",
            )
        ]
    )

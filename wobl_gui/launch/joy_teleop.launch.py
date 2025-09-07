from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joystick bridge node (WSL version)
        Node(
            package="wobl_gui",
            executable="joy_bridge_wsl.py",
            name="joy_bridge",
            output="screen",
        ),
        
        # Joy to command velocity converter
        Node(
            package="wobl_gui",
            executable="joy2cmd_node",
            name="joy2cmd",
            output="screen",
        ),
    ])

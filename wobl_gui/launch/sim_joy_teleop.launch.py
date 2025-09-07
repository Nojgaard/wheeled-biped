import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joystick teleop with the full control system."""
    
    config_file = os.path.join(
        get_package_share_directory('wobl_control'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        # Robot control nodes (simulation)
        Node(
            package="wobl_sim",
            executable="mujoco_bridge_node",
            name="mujoco_bridge_node",
            output="screen",
        ),
        
        # Main controller
        Node(
            package="wobl_control",
            executable="wobl_controller",
            name="wobl_controller",
            output="screen",
            parameters=[config_file],
        ),
        
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

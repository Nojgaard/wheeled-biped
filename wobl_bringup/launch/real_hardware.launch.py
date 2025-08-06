from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wobl_hardware',
            executable='imu_node',
            name='imu_node',
        ),
        Node(
            package='wobl_hardware',
            executable='servo_node',
            name='servo_node',
        )
    ])

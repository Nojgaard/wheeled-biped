from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU Publisher Node (replace with your actual IMU driver)
        Node(
            package='real',
            executable='imu_node',
            name='imu_publisher',
        ),

        # Throttle Node from topic_tools
        Node(
            package='topic_tools',
            executable='throttle',
            name='imu_network',
            arguments=['messages', 'imu/data', '20.0', 'imu/data_network']
        )
    ])
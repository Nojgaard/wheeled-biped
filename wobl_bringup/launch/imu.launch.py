from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU Publisher Node (replace with your actual IMU driver)
        Node(
            package='wobl_hardware',
            executable='imu_node',
            name='imu_publisher',
        ),
        Node(
            package='topic_tools',
            executable='throttle',
            name='imu_network',
            arguments=['messages', 'imu/data', '20.0', 'remote/imu/data']
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
        )
    ])

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from wobl_msgs.msg import Topics


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
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="imu_remote_publisher",
                arguments=["messages", Topics.IMU, "20.0", Topics.IMU_REMOTE],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_state_remote_publisher",
                arguments=["messages", Topics.JOINT_STATE, "20.0", Topics.JOINT_STATE_REMOTE],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_command_remote_publisher",
                arguments=["messages", Topics.JOINT_COMMAND, "20.0", Topics.JOINT_COMMAND_REMOTE],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
            )
        ]
    )

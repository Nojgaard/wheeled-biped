from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
                executable="pid_balance_controller",
                name="pid_balance_controller",
                output="screen",
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="imu_remote_publisher",
                arguments=["messages", "imu/data", "20.0", "remote/imu/data"],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_state_remote_publisher",
                arguments=["messages", "joint_states", "20.0", "remote/joint_states"],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_command_remote_publisher",
                arguments=["messages", "joint_commands", "20.0", "remote/joint_commands"],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
            )
        ]
    )

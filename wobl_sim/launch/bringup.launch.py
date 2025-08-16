from launch import LaunchDescription
from launch_ros.actions import Node
from wobl_msgs.msg import Topics


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wobl_sim",
                executable="mujoco_bridge_node",
                name="mujoco_bridge_node",
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

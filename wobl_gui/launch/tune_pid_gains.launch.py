from launch import LaunchDescription
from launch_ros.actions import Node
from wobl_msgs.msg import Topics
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the params file
    config_file = os.path.join(
        get_package_share_directory("wobl_control"), "config", "params.yaml"
    )

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
                executable="wobl_controller",
                name="wobl_controller",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="wobl_gui",
                executable="tune_gains_node",
                name="tune_gains_node",
                output="screen",
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_state_remote_publisher",
                arguments=[
                    "messages",
                    Topics.JOINT_STATE,
                    "20.0",
                    Topics.JOINT_STATE_REMOTE,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="joint_command_remote_publisher",
                arguments=[
                    "messages",
                    Topics.JOINT_COMMAND,
                    "20.0",
                    Topics.JOINT_COMMAND_REMOTE,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="controller_inputs_remote_publisher",
                arguments=[
                    "messages",
                    Topics.CONTROLLER_INPUTS,
                    "20.0",
                    Topics.CONTROLLER_INPUTS_REMOTE,
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove",
                output="screen",
            ),
        ]
    )

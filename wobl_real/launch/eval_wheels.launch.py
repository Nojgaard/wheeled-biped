from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from wobl_msgs.msg import Topics

def generate_launch_description():
    # Start rosbag record process
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--topics",
            Topics.JOINT_COMMAND,
            Topics.JOINT_STATE
        ],
        output="screen"
    )

    # Nodes to bring up after rosbag is running
    bringup_nodes = [
        Node(
            package='wobl_real',
            executable='servo_node',
            name='servo_node',
        ),
        Node(
            package="wobl_control",
            executable="wheel_eval_node",
            name="wheel_eval",
            output="screen",
        ),
    ]

    # Start bringup nodes only after rosbag is running (with a short delay)
    bringup_event = RegisterEventHandler(
        OnProcessStart(
            target_action=rosbag_record,
            on_start=[TimerAction(period=2.0, actions=bringup_nodes)]
        )
    )

    return LaunchDescription([
        rosbag_record,
        bringup_event
    ])
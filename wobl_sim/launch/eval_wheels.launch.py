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
            Topics.JOINT_STATE,
            "-o", "data/bag_sim_wheel",
        ],
        output="screen"
    )

    # Start mujoco_bridge_node immediately after rosbag starts
    bridge_node = Node(
        package="wobl_sim",
        executable="mujoco_bridge_node",
        name="mujoco_bridge_node",
        output="screen",
        parameters=[{"eval_mode": True, "headless": True}],
    )

    # Start wheel_eval_node after a delay (after bridge_node is up)
    wheel_eval_node = Node(
        package="wobl_control",
        executable="wheel_eval_node",
        name="wheel_eval",
        output="screen",
    )

    # Event: Start bridge_node when rosbag_record starts
    bridge_event = RegisterEventHandler(
        OnProcessStart(
            target_action=rosbag_record,
            on_start=[bridge_node]
        )
    )

    # Event: Start wheel_eval_node after a delay (after bridge_node starts)
    eval_event = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge_node,
            on_start=[TimerAction(period=3.0, actions=[wheel_eval_node])]
        )
    )

    return LaunchDescription([
        rosbag_record,
        bridge_event,
        eval_event
    ])
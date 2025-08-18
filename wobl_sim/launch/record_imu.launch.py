from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from wobl_msgs.msg import Topics

def generate_launch_description():
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--topics",
            Topics.IMU,
            "-o", "data/bag_sim_imu",
        ],
        output="screen"
    )

    mjcf_bridge = Node(
        package="wobl_sim",
        executable="mujoco_bridge_node",
        name="mujoco_bridge_node",
        output="screen",
        parameters=[{"headless": False}],
    )
    crtl_node = Node(
        package="wobl_control",
        executable="pid_balance_controller",
        name="pid_balance_controller",
        output="screen",
    )

    return LaunchDescription([
        TimerAction(period=4.0, actions=[rosbag_record, crtl_node]),
        mjcf_bridge,
    ])

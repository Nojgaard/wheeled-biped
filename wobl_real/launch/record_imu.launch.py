from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from wobl_msgs.msg import Topics

def generate_launch_description():
    # Publish to enable servos using ros2 topic pub
    enable_servos_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            'servo/enabled', 'std_msgs/msg/Bool', '{data: true}'
        ],
        output='screen'
    )

    # Publish joint command (all zeros) using ros2 topic pub
    joint_command_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/joint_command', 'wobl_msgs/msg/JointCommand',
            '{position: [0.0, 0.0, 0.0, 0.0], velocity: [1.0, 1.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

    # rosbag2 record process (records imu topic)
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', Topics.IMU, '-o', 'data/bag_real_imu'],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='wobl_real',
            executable='imu_node',
            name='imu_node',
        ),
        Node(
            package='wobl_real',
            executable='servo_node',
            name='servo_node',
        ),
        TimerAction(period=2.0, actions=[enable_servos_pub]),
        TimerAction(period=2.0, actions=[joint_command_pub]),
        TimerAction(period=4.0, actions=[rosbag_record])
    ])

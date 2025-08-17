from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from wobl_msgs.msg import Topics

def generate_launch_description():
    # Node to enable servos
    enable_servos_node = Node(
        package='rclpy',
        executable='publisher',
        name='enable_servos_pub',
        arguments=[
            '--topic', 'servo/enabled',
            '--msg-type', 'std_msgs/msg/Bool',
            '--publish', 'data:=true'
        ],
        output='screen'
    )

    # Node to send joint command (all zeros)
    joint_command_node = Node(
        package='rclpy',
        executable='publisher',
        name='joint_command_pub',
        arguments=[
            '--topic', Topics.JOINT_COMMAND,
            '--msg-type', 'wobl_msgs/msg/JointCommand',
            '--publish', 'position:=[0.0,0.0,0.0,0.0]', 'velocity:=[1.0,1.0,0.0,0.0]'
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
        TimerAction(period=2.0, actions=[enable_servos_node]),
        TimerAction(period=1, actions=[joint_command_node]),
        TimerAction(period=1.0, actions=[rosbag_record])
    ])
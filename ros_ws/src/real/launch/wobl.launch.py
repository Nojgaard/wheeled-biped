import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('real')

    # Paths
    xacro_path = os.path.join(pkg_share, 'urdf', 'ros2_control.urdf.xacro')
    mjcf_path = os.path.join(pkg_share, 'mjcf', 'plane_world.xml')
    controller_yaml = os.path.join(pkg_share, 'config', 'wobl.yaml')

    # Process Xacro
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            controller_yaml,
            {'mujoco_model_path': mjcf_path}
        ]
    )

    # Node: robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller spawner nodes
    spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawner_velocity = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen'
    )

    spawner_position = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    # Register event handler to spawn controllers after mujoco starts
    spawn_controllers_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mujoco_node,
            on_start=[spawner_joint_state, spawner_velocity, spawner_position]
        )
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        mujoco_node,
        rsp_node,
        spawn_controllers_handler
    ])
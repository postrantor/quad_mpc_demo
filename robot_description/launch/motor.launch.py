#!/usr/bin/env python3

"""
@brief
@author postrantor@gmail.com
@date 2024-08-25 16:33:03

example:
> ros2 launch robot_description motor.launch.py
> ros2 run robot_description example_robot
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_name',
        default_value='robot_a1',
        description='robot namespace'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'),
    DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        choices=['true', 'false'],
        description='use gazebo simulation'),
    DeclareLaunchArgument(
        'debug',
        default_value='true',
        choices=['true', 'false'],
        description='debug'),
]


def generate_launch_description():
    # get package path
    pkg_description = get_package_share_directory('robot_description')

    # get config file path
    xacro_file = PathJoinSubstitution([pkg_description, 'config', 'xacro', 'robot.xacro'])
    robot_controllers = PathJoinSubstitution([pkg_description, 'config', 'controller', 'controller_real.yaml'])

    # load robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                'xacro', ' ', xacro_file, ' '
                'debug:=', LaunchConfiguration('debug'), ' '
                'use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'), ' '
                'gazebo:=ignition', ' ',
                'namespace:=', LaunchConfiguration('robot_name')])},
        ],
        output='screen',
    )

    # load controller_manager with controller.yaml config
    # the loading of control_nodel conflicts with `<gazebo>` in the urdf configuration file.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output="screen",
    )

    # load all unitree_joint_controller
    leg_controller_names = [
        f'{leg}_{part}_controller'
        for leg in ['FL']  # 'FL', 'FR', 'RL', 'RR'
        for part in ['hip', 'thigh', 'calf']  # 'hip', 'thigh', 'calf'
    ]
    controller_names = leg_controller_names
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        # load each controller use controller_manager service
        arguments=['--activate-as-group', *controller_names],
        output="screen",
    )

    ld = LaunchDescription([
        *ARGUMENTS,
        node_robot_state_publisher,
        control_node,
        load_controllers,
    ])

    return ld

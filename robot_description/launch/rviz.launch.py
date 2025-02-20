#!/usr/bin/env python3

"""
@brief load robot in rviz2
@author postrantor@gmail.com
@date 2025-02-13 09:30:30

example:
> ros2 launch robot_description rviz.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'),
    DeclareLaunchArgument(
        'robot_name',
        default_value='robot_description',
        description='robot name'),
    DeclareLaunchArgument(
        'namespace',
        default_value=LaunchConfiguration('robot_name'),
        description='robot namespace'),
    DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        choices=['true', 'false'],
        description='use gazebo simulation'
    ),
    DeclareLaunchArgument(
        'debug',
        default_value='false',
        choices=['true', 'false'],
        description='debug'
    ),
]


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')

    xacro_file = PathJoinSubstitution([pkg_robot_description, 'config', 'xacro', 'robot.xacro'])
    # urdf_file = PathJoinSubstitution([pkg_robot_description, 'config', 'urdf', 'robot.urdf'])
    rviz2_config = PathJoinSubstitution([pkg_robot_description, 'config', 'rviz', 'robot.rviz'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # {'robot_description': open(urdf_file.perform(None)).read()},
            {'robot_description': Command([
                'xacro ', xacro_file, ' ',
                'debug:=', LaunchConfiguration('debug'), ' ',
                'gazebo:=ignition', ' ',
                'use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'), ' ',
                'namespace:=', LaunchConfiguration('namespace')])
             },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')],
    )

    # visualize in rviz
    robot_rviz = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # delay launch of robot description to allow rviz2 to load first.
        # prevents visual bugs in the model.
        TimerAction(
            period=1.5,
            actions=[
                robot_state_publisher,
                joint_state_publisher
            ])
    ])

    # define launchdescription variable
    ld = LaunchDescription(ARGUMENTS)
    # add nodes to launchdescription
    ld.add_action(robot_rviz)

    return ld

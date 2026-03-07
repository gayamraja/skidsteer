#!/usr/bin/env python3
"""
Pivot Visual Launch: Gazebo + robot + 30-second pivot spin only.
No incline, no crop test - just watch the yellow dots on the tires spin.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('skid_steer_robot').find('skid_steer_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot.xacro')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True}
        ]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'false'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'agribot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    hardware_controller = Node(
        package='skid_steer_robot',
        executable='hardware_aware_controller.py',
        name='hardware_aware_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    pivot_test = Node(
        package='skid_steer_robot',
        executable='pivot_visual_test.py',
        name='pivot_visual_test',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        hardware_controller,
        # Wait 10s for Gazebo to load before starting pivot
        TimerAction(period=10.0, actions=[pivot_test]),
    ])

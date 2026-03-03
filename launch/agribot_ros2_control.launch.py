#!/usr/bin/env python3
"""
Agribot Launch with ros2_control
Proper motor control setup for 1.5kW motors with 22.5:1 reduction
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('skid_steer_robot').find('skid_steer_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_calibration.world')
    controllers_file = os.path.join(pkg_share, 'config', 'agribot_controllers.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process xacro file to get robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Gazebo launch with empty world
    # Set verbosity to 'error' to suppress model.config warnings
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'error'  # Only show errors, suppress warnings
        }.items()
    )
    
    # Spawn robot in Gazebo
    # Note: The gazebo_ros2_control plugin will automatically get robot_description
    # from the parameter server (published by robot_state_publisher)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agribot',
        arguments=[
            '-entity', 'agribot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.7'  # Spawn above ground
        ],
        output='screen'
    )
    
    # Spawn joint_state_broadcaster (ros2_control publishes /joint_states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn DiffDriveController
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])

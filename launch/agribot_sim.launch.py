#!/usr/bin/env python3
"""
Main simulation launcher for Agribot skid steer robot
Spawns robot in Gazebo empty world with hardware-aware controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import subprocess

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('skid_steer_robot').find('skid_steer_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot.xacro')
    # Use agricultural field world for terrain testing
    world_file = os.path.join(pkg_share, 'worlds', 'agriculture_field.world')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Process xacro and strip XML declaration.
    # gazebo_ros2_control spawns controller_manager with robot_description as --param,
    # and rcl's argument parser chokes on the <?xml ...?> prologue.
    robot_description_raw = subprocess.run(
        ['xacro', urdf_file], capture_output=True, text=True, check=True
    ).stdout
    robot_description = '\n'.join(
        line for line in robot_description_raw.splitlines()
        if not line.startswith('<?xml')
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )
    
    # Gazebo launch
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
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agribot',
        arguments=[
            '-entity', 'agribot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'  # 0.1m clearance so wheels settle onto ground
        ],
        output='screen'
    )
    
    # Hardware-aware controller
    hardware_controller = Node(
        package='skid_steer_robot',
        executable='hardware_aware_controller.py',
        name='hardware_aware_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz (optional) - launch without config file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz'
        ),
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        hardware_controller,
        rviz
    ])

#!/usr/bin/env python3
"""
Essential Safety Tests Launcher
Automated testing for 5 km/h max speed operation
Tests: Pivot Turn and Incline Start
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
    world_file = os.path.join(pkg_share, 'worlds', 'agribot_test_bench.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    run_tests = LaunchConfiguration('run_tests', default='true')
    
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
    
    # Gazebo launch with test bench world
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
            '-z', '0.1'  # Spawn just above ground so wheels settle
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
    
    # Essential safety check test (automated)
    # use_sim_time=False so wall-clock timers fire at the right time
    safety_test = Node(
        package='skid_steer_robot',
        executable='essential_safety_check.py',
        name='essential_safety_check',
        output='screen',
        condition=IfCondition(run_tests),
        parameters=[{'use_sim_time': False}]
    )
    
    # Belly clearance test (runs after safety tests)
    clearance_test = Node(
        package='skid_steer_robot',
        executable='belly_clearance_test.py',
        name='belly_clearance_test',
        output='screen',
        condition=IfCondition(run_tests),
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'run_tests',
            default_value='true',
            description='Run automated safety tests'
        ),
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        hardware_controller,
        # Wait 5 seconds for Gazebo to load before starting tests
        TimerAction(
            period=15.0,
            actions=[safety_test]
        ),
        # Run belly clearance test after safety tests complete (~20 seconds)
        TimerAction(
            period=40.0,
            actions=[clearance_test]
        )
    ])

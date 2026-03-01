#!/usr/bin/env python3
"""
Stress test suite launcher
Runs all three destruction tests sequentially
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('skid_steer_robot').find('skid_steer_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_calibration.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    run_tests = LaunchConfiguration('run_tests', default='true')
    
    # Robot description
    robot_description_content = f"""
    <robot name="agribot">
        <xacro:include filename="{urdf_file}"/>
    </robot>
    """
    
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
    
    # Gazebo
    gazebo_process = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gazebo',
        arguments=[world_file],
        output='screen'
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agribot',
        arguments=[
            '-entity', 'agribot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
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
    
    # Test nodes (run sequentially with delays)
    tip_over_test = Node(
        package='skid_steer_robot',
        executable='tip_over_test.py',
        name='tip_over_test',
        output='screen',
        condition=IfCondition(run_tests)
    )
    
    pivot_test = Node(
        package='skid_steer_robot',
        executable='pivot_stress_test.py',
        name='pivot_stress_test',
        output='screen',
        condition=IfCondition(run_tests)
    )
    
    latency_test = Node(
        package='skid_steer_robot',
        executable='latency_test.py',
        name='latency_test',
        output='screen',
        condition=IfCondition(run_tests)
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
            description='Run stress tests'
        ),
        robot_state_publisher,
        gazebo_process,
        spawn_entity,
        hardware_controller,
        # Run tests with delays between them
        TimerAction(
            period=5.0,
            actions=[tip_over_test]
        ),
        TimerAction(
            period=60.0,  # Start after tip-over test completes
            actions=[pivot_test]
        ),
        TimerAction(
            period=120.0,  # Start after pivot test completes
            actions=[latency_test]
        )
    ])

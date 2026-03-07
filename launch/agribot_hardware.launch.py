#!/usr/bin/env python3
"""
agribot_hardware.launch.py — Real hardware launcher for Agribot skid steer.

Launches on the Raspberry Pi (no Gazebo, no spawn_entity).
Nodes:
  1. robot_state_publisher  — publishes TF tree from URDF
  2. hardware_aware_controller — quantizer + soft-start (high-level)
  3. serial_bridge_node     — bridges /diff_cont/cmd_vel ↔ Arduino over USB serial
  4. rviz2                  — optional visualisation

Usage:
  ros2 launch skid_steer_robot agribot_hardware.launch.py
  ros2 launch skid_steer_robot agribot_hardware.launch.py use_rviz:=true
  ros2 launch skid_steer_robot agribot_hardware.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import subprocess


def generate_launch_description():
    pkg_share = FindPackageShare('skid_steer_robot').find('skid_steer_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot.xacro')
    params_file = os.path.join(pkg_share, 'config', 'hardware_params.yaml')

    use_rviz    = LaunchConfiguration('use_rviz',    default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')

    # Process URDF (strip XML declaration for compatibility)
    robot_description_raw = subprocess.run(
        ['xacro', urdf_file], capture_output=True, text=True, check=True
    ).stdout
    robot_description = '\n'.join(
        line for line in robot_description_raw.splitlines()
        if not line.startswith('<?xml')
    )

    # 1. Robot State Publisher — TF tree from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False}
        ]
    )

    # 2. Hardware-aware controller — quantizer (34-99) + soft-start (high-level)
    hardware_controller = Node(
        package='skid_steer_robot',
        executable='hardware_aware_controller.py',
        name='hardware_aware_controller',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False}
        ]
    )

    # 3. Serial bridge — /diff_cont/cmd_vel → Arduino → /odom + /serial_status
    serial_bridge = Node(
        package='skid_steer_robot',
        executable='serial_bridge_node.py',
        name='serial_bridge_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': False,
                'port': serial_port,
            }
        ]
    )

    # 4. RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for visualisation'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino USB serial port (e.g. /dev/ttyACM0 or /dev/ttyUSB0)'
        ),
        robot_state_publisher,
        hardware_controller,
        serial_bridge,
        rviz,
    ])

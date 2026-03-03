#!/bin/bash
# Diagnostic script to check ros2_control setup

echo "=== Checking ROS2 Control Setup ==="
echo ""

echo "1. Checking if controller_manager node is running:"
ros2 node list | grep controller_manager || echo "  ERROR: controller_manager node not found!"

echo ""
echo "2. Checking for controller_manager services:"
ros2 service list | grep controller_manager || echo "  ERROR: controller_manager services not found!"

echo ""
echo "3. Checking robot_description topic:"
timeout 2 ros2 topic echo /robot_description --once > /dev/null 2>&1 && echo "  OK: robot_description is published" || echo "  ERROR: robot_description not available"

echo ""
echo "4. Checking joint_states topic:"
timeout 2 ros2 topic echo /joint_states --once > /dev/null 2>&1 && echo "  OK: joint_states is published" || echo "  WARNING: joint_states not available (may need controllers)"

echo ""
echo "=== Diagnostic Complete ==="

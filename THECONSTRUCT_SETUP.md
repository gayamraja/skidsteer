# TheConstruct AI Setup Guide

## Quick Setup Steps

### Option 1: Clone from GitHub (Recommended)

1. **Open your TheConstruct AI workspace** (ROSject)

2. **Open a terminal** in TheConstruct AI

3. **Navigate to your ROS2 workspace**:
   ```bash
   cd ~/ros2_ws/src
   ```

4. **Clone the repository**:
   ```bash
   git clone https://github.com/gayamraja/skidsteer.git skid_steer_robot
   ```

5. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select skid_steer_robot
   source install/setup.bash
   ```

### Option 2: Manual Upload

If you prefer to upload files manually:

1. **Create the package structure** in TheConstruct AI:
   ```bash
   cd ~/ros2_ws/src
   mkdir -p skid_steer_robot
   cd skid_steer_robot
   ```

2. **Upload all files** from your local workspace to TheConstruct AI using the file browser or upload tool

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select skid_steer_robot
   source install/setup.bash
   ```

## Testing the Simulation

### 1. Launch the Simulation

```bash
ros2 launch skid_steer_robot agribot_sim.launch.py
```

This will:
- Start Gazebo with empty world
- Spawn the Agribot robot
- Launch the hardware-aware controller

### 2. Test Basic Movement

In a new terminal:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left (pivot)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3. Monitor Robot State

```bash
# Check odometry
ros2 topic echo /odom

# Check joint states
ros2 topic echo /joint_states

# Check controller output
ros2 topic echo /left_track_cmd
ros2 topic echo /right_track_cmd
```

### 4. Run Stress Tests

```bash
# Run all tests
ros2 launch skid_steer_robot stress_tests.launch.py

# Or run individual tests
ros2 run skid_steer_robot tip_over_test.py
ros2 run skid_steer_robot pivot_stress_test.py
ros2 run skid_steer_robot latency_test.py
```

## Troubleshooting

### Package Not Found
- Make sure you've built the package: `colcon build --packages-select skid_steer_robot`
- Source the workspace: `source install/setup.bash`

### Gazebo Not Starting
- Check if Gazebo is installed: `which gzserver`
- Try launching Gazebo manually first

### Robot Not Spawning
- Check Gazebo logs for errors
- Verify URDF is valid: `check_urdf install/skid_steer_robot/share/skid_steer_robot/urdf/agribot.xacro`

### Controller Not Working
- Check if controller node is running: `ros2 node list`
- Verify topics: `ros2 topic list`
- Check controller logs for errors

## TheConstruct AI Specific Notes

- TheConstruct AI typically uses ROS2 Humble
- Gazebo Classic or Gazebo 11+ should be available
- Make sure to source the workspace after building
- Use the web-based Gazebo viewer or connect via VNC if needed

## Next Steps

1. Test basic movement commands
2. Verify hardware constraints (quantizer, interlock, soft-start)
3. Run stress tests to validate stability
4. Calibrate physics parameters if needed

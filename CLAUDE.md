# Agribot Skid Steer Simulation - Claude Context

## Project
ROS2 + Gazebo simulation of a physical skid steer agricultural robot (Agribot), built on
TheConstruct AI before constructing the real hardware. Package name: `skid_steer_robot`.

## Two Claude Instances - Division of Labor

### Local Claude (Windows PC)
- Edit code, URDF, configs, launch files
- Commit and push to GitHub

### TheConstruct Claude (ROSject terminal)
- `git pull` to get latest code
- `colcon build` and source workspace
- Launch simulation, run tests
- Report errors back so Local Claude can fix them

**Sync command on TheConstruct:**
```bash
cd ~/ros2_ws/src/skid_steer_robot && git pull
cd ~/ros2_ws && colcon build --packages-select skid_steer_robot && source install/setup.bash
```

## Robot Physical Specs
- 4-wheel skid steer, V-stance legs, high-clearance chassis (0.6m)
- Wheelbase: 0.81m, Track width: 0.87m
- Tires: 4.00-8 (0.2025m radius)
- Drive: 22.5:1 total reduction (10:1 gearbox + 2.25:1 two-stage chain via jackshaft)
- Motors: 1.5kW per side, 300Nm max gearbox torque, max speed 5 km/h

## Architecture

### Command Flow (FIXED - fully wired)
```
teleop → /cmd_vel
           ↓
  hardware_aware_controller (scripts/hardware_aware_controller.py)
  [quantizer 34-99] + [relay interlock 150ms] + [soft-start 0.015/cycle]
  [inverse kinematics: left/right → linear.x / angular.z]
           ↓
  /diff_cont/cmd_vel
           ↓
  DiffDriveController (ros2_control, config/agribot_controllers.yaml)
           ↓
  4 wheel joints in Gazebo
```

### Hardware Constraints Simulated
- **Quantizer (34-99)**: Dead zone 0-0.34 → 0 output; 0.34-1.0 maps to 0.34-0.99
- **Relay Interlock**: 150ms pause on Forward↔Reverse direction change
- **Soft-Start**: Slew rate 0.015/update at 50Hz

## Key Files
| File | Purpose |
|------|---------|
| `urdf/agribot.xacro` | Robot model (base, V-legs, wheels, jackshafts, IMU) |
| `urdf/ros2_control.xacro` | ros2_control interfaces + Gazebo plugin |
| `config/agribot_controllers.yaml` | DiffDriveController "diff_cont" config |
| `config/hardware_params.yaml` | Quantizer, relay, soft-start parameters |
| `scripts/hardware_aware_controller.py` | Main Phase 2 controller |
| `scripts/agribot_physical_controller.py` | Simple Step-34 test controller (teleop test only) |

## Launch Files
| Launch file | Use |
|-------------|-----|
| `agribot_sim.launch.py` | Main: Gazebo + agricultural world + hardware controller |
| `agribot_ros2_control.launch.py` | Empty world + ros2_control only |
| `agribot_teleop_test.launch.py` | Empty world + physical controller for Step 34 test |
| `empty_world.launch.py` | Minimal: robot + ground plane only |
| `essential_tests.launch.py` | Automated safety test suite |

## 4-Phase Plan & Status
1. **Phase 1 (DONE)**: URDF digital twin with correct geometry
2. **Phase 2 (DONE)**: Hardware-aware controller (quantizer + relay + soft-start)
3. **Phase 3 (In Progress)**: Physics/traction calibration in Gazebo
4. **Phase 4 (Written, needs Gazebo validation)**: Stress tests in `scripts/`

## TheConstruct - Useful Commands
```bash
# Build and source
cd ~/ros2_ws && colcon build --packages-select skid_steer_robot && source install/setup.bash

# Launch main sim
ros2 launch skid_steer_robot agribot_sim.launch.py

# Verify controller topics are live
ros2 topic list | grep diff_cont
ros2 topic echo /diff_cont/cmd_vel

# Send a test command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Check active controllers
ros2 control list_controllers

# Monitor IMU (tip detection)
ros2 topic echo /imu/data
```

## GitHub Repo
https://github.com/gayamraja/skidsteer

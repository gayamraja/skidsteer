# Agribot Skid Steer - Claude Context (Sim + Real Hardware)

## Project
ROS2 + Gazebo simulation AND real hardware bringup for a skid steer agricultural robot
(Agribot). Simulation built on TheConstruct AI; hardware runs on Raspberry Pi + Arduino.
Package name: `skid_steer_robot`.

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

### Simulation Command Flow (WORKING - confirmed in Gazebo)
```
teleop → /cmd_vel
           ↓
  hardware_aware_controller (scripts/hardware_aware_controller.py)
  [quantizer 34-99] + [relay interlock 150ms] + [soft-start 0.015/cycle]
           ↓
  /diff_cont/cmd_vel
           ↓
  libgazebo_ros_diff_drive (num_wheel_pairs=2, urdf/ros2_control.xacro)
  [drives all 4 wheels, publishes /odom and /tf]
           ↓
  4 wheel joints in Gazebo
```

NOTE: gazebo_ros2_control 0.4.10 was abandoned — it passes robot_description
as rcl --param which cannot parse multi-line XML. libgazebo_ros_diff_drive
is used instead and works identically for our Phase 3 purposes.

### Real Hardware Command Flow (Phase 5 - branch: feature/hw-bringup-rpi-arduino)
```
teleop → /cmd_vel
           ↓
  hardware_aware_controller (RPi — same script as sim)
  [quantizer 34-99] + [relay signal] + [soft-start — belt]
           ↓
  /diff_cont/cmd_vel
           ↓
  serial_bridge_node (scripts/serial_bridge_node.py)
  [Twist → normalised left/right → CMD:<l> <r> over USB serial]
           ↓
  Arduino (firmware/agribot_firmware.ino)
  [PRIMARY: soft-start, relay 150ms, backlash comp, 40A cutoff, watchdog]
           ↓
  Motor drivers → 4 wheels
           ↑
  Encoder ticks → /odom  |  ACS758 current → /serial_status
```

### Hardware Constraints — Division of Enforcement
| Constraint | RPi (software) | Arduino (hardware, primary) |
|-----------|---------------|---------------------------|
| Quantizer dead zone 34-99 | hardware_aware_controller.py | — |
| Soft-start 0.015/cycle | belt (backup) | PRIMARY enforcer |
| Relay 150ms dead-time | signal only | PRIMARY enforcer |
| 40A current cutoff | — | ACS758 → instant PWM cut |
| Watchdog safe stop | — | 200ms timeout on serial |
| Backlash compensation | — | 20% pulse 50ms on dir change |

## Key Files
| File | Purpose |
|------|---------|
| `urdf/agribot.xacro` | Robot model (base, V-legs, wheels, jackshafts, IMU) |
| `urdf/ros2_control.xacro` | libgazebo_ros_diff_drive plugin (4WD skid steer, sim only) |
| `config/hardware_params.yaml` | Quantizer, relay, soft-start + serial_bridge params |
| `scripts/hardware_aware_controller.py` | High-level controller (sim + hardware) |
| `scripts/serial_bridge_node.py` | ROS2 ↔ Arduino USB serial bridge (hardware only) |
| `firmware/agribot_firmware.ino` | Arduino real-time firmware (hardware only) |

## Launch Files
| Launch file | Use |
|-------------|-----|
| `agribot_sim.launch.py` | Simulation: Gazebo + agricultural world |
| `agribot_hardware.launch.py` | **Real hardware**: RPi only, no Gazebo |
| `empty_world.launch.py` | Minimal sim: robot + ground plane |
| `essential_tests.launch.py` | Sim safety test suite |
| `stress_tests.launch.py` | Sim Phase 4 stress tests |

## 5-Phase Plan & Status
1. **Phase 1 (DONE)**: URDF digital twin with correct geometry
2. **Phase 2 (DONE)**: Hardware-aware controller (quantizer + relay + soft-start)
3. **Phase 3 (DONE)**: Physics/traction calibration in Gazebo
4. **Phase 4 (DONE)**: Stress test suite validated in sim
5. **Phase 5 (ACTIVE)**: Real hardware bringup — RPi + Arduino (branch: feature/hw-bringup-rpi-arduino)

## TheConstruct - Useful Commands (Simulation)
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

# Monitor IMU (tip detection)
ros2 topic echo /imu/data
```

## Raspberry Pi - Useful Commands (Real Hardware)
```bash
# Install pyserial (once)
pip3 install pyserial

# Build and source
cd ~/ros2_ws && colcon build --packages-select skid_steer_robot && source install/setup.bash

# Find Arduino port
ls /dev/ttyACM* /dev/ttyUSB*

# Launch real hardware
ros2 launch skid_steer_robot agribot_hardware.launch.py
# Override port if needed:
ros2 launch skid_steer_robot agribot_hardware.launch.py serial_port:=/dev/ttyUSB0

# Monitor Arduino serial status
ros2 topic echo /serial_status

# Monitor odometry from encoders
ros2 topic echo /odom

# Send a test command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## GitHub Repo
https://github.com/gayamraja/skidsteer

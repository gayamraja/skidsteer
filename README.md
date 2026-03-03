# Agribot Skid Steer Robot Simulation

Hardware-accurate simulation of the Agribot skid steer robot for TheConstruct AI workspace. This simulation models the specific electronic constraints and physical characteristics of the robot before building the physical hardware.

## Project Overview

This simulation implements a 4-phase approach to create a digital twin of the Agribot:

1. **Phase 1: Digital Twin Alignment (URDF)** - Accurate physical model with V-stance geometry
2. **Phase 2: Hardware-Aware Controller** - Simulates electronic constraints (quantizer, relay interlock, soft-start)
3. **Phase 3: Physics & Traction Calibration** - Friction and torque tuning for realistic behavior
4. **Phase 4: Stress Testing & Validation** - Destruction tests to validate stability and performance

## Physical Specifications

- **Wheelbase**: 81cm (0.81m) - front-to-back wheel centers
- **Track Width**: 87cm (0.87m) - left-to-right wheel centers
- **Chassis Height**: 450mm from ground (high-clearance design)
- **Base Mass**: 40kg
- **Total Mass**: 60kg (40kg base + 20kg wheels)
- **Tires**: 4.00-8 (405mm diameter, 0.2025m radius)
- **Wheel Width**: 0.101m (4 inches)
- **Motor Power**: 1.5kW per side
- **Total Reduction**: 22.5:1 (10:1 gearbox + 2.25:1 chain)
  - **Chain Drive**: 16T (Gearbox) → 36T (Jackshaft) → 16T (Wheel)
  - **Stage 1**: 16T→36T = 2.25:1
  - **Stage 2**: 16T→16T = 1:1
- **Max Torque**: 300Nm at gearbox output
- **Motor RPM at 5 km/h**: 1,462 RPM
- **Wheel RPM at 5 km/h**: 65 RPM
- **Jackshaft**: 36T sprocket positioned in middle for ground clearance

## Hardware Constraints Simulated

### 34-99 Quantizer
- Dead zone: 0.0 to 0.34 (robot stays still)
- Active range: 0.34 to 0.99 (mapped to motor commands)

### Relay Interlock
- 150ms hard pause when direction changes (Forward ↔ Reverse)
- Protects virtual relays and chains

### Soft-Start Ramping
- Slew rate limiter prevents chain snapping
- Gradual acceleration/deceleration

## Installation

### Prerequisites

- ROS2 (Humble or Foxy recommended)
- Gazebo (classic or Gazebo 11+)
- Python 3.8+

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone or copy this package
# (Assuming package is in current directory)

# Build the package
cd ~/ros2_ws
colcon build --packages-select skid_steer_robot

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Teleop Test (Step 34 Jump Test)

This is the simplest test to verify the 22.5:1 reduction doesn't cause excessive "jump" at Step 34.

**Terminal 1: Launch Gazebo with robot**
```bash
ros2 launch skid_steer_robot agribot_teleop_test.launch.py
```

**Terminal 2: Run keyboard teleop**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Test Procedure:**
- Tap the `w` key **once** (brief forward command)
- **Success**: Robot moves forward at a crawl without front wheels lifting
- **Fail**: Robot "hops" or front wheels leave the ground (need to decrease MIN_PWM_STEP)

### Launch Main Simulation

```bash
ros2 launch skid_steer_robot agribot_sim.launch.py
```

This will:
- Start Gazebo with agricultural field world
- Spawn the Agribot robot
- Launch the hardware-aware controller
- Publish robot state and TF transforms

### Launch Empty World for Calibration

```bash
ros2 launch skid_steer_robot empty_world.launch.py
```

Minimal setup for Phase 3 physics calibration.

### Run Essential Safety Tests (Automated)

```bash
ros2 launch skid_steer_robot essential_tests.launch.py
```

Runs automated safety tests for 5 km/h max speed operation:
1. **Pivot Turn Test**: 360° rotation in place - checks for lateral tipping
2. **Incline Start Test**: Drive up 15° ramp and emergency stop - checks for rear tip-over

These tests verify the 0.6m high-clearance design is stable at low speeds (5 km/h max).

### Manual Control

After launching the simulation, you can control the robot:

```bash
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Monitor odometry
ros2 topic echo /odom

# Check joint states
ros2 topic echo /joint_states
```

## Project Structure

```
skid_steer_robot/
├── package.xml              # ROS2 package manifest
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── urdf/
│   └── agribot.xacro       # Robot URDF model
├── launch/
│   ├── agribot_sim.launch.py      # Main simulation launcher
│   ├── empty_world.launch.py      # Empty world for calibration
│   └── stress_tests.launch.py     # Test suite launcher
├── config/
│   ├── controllers.yaml           # Gazebo controller config
│   ├── gazebo_physics.yaml         # Physics parameters
│   └── hardware_params.yaml       # Hardware controller parameters
├── scripts/
│   ├── hardware_aware_controller.py  # Phase 2 controller
│   ├── tip_over_test.py            # Phase 4 test 1
│   ├── pivot_stress_test.py        # Phase 4 test 2
│   └── latency_test.py             # Phase 4 test 3
└── worlds/
    └── empty_calibration.world     # Empty Gazebo world
```

## Configuration

### Hardware Parameters

Edit `config/hardware_params.yaml` to adjust:
- Quantizer thresholds (dead zone, max output)
- Relay interlock delay
- Soft-start ramp rates

### Physics Parameters

Edit `config/gazebo_physics.yaml` to tune:
- Friction coefficients (μ1, μ2)
- Physics engine settings
- Ground material properties

### Controller Configuration

Edit `config/controllers.yaml` to modify:
- Controller update rates
- Joint effort limits

## Testing

### Individual Tests

```bash
# Tip-over test
ros2 run skid_steer_robot tip_over_test.py

# Pivot stress test
ros2 run skid_steer_robot pivot_stress_test.py

# Latency test
ros2 run skid_steer_robot latency_test.py
```

## Troubleshooting

### Robot Not Spawning

- Check Gazebo is running: `gz stats`
- Verify URDF is valid: `check_urdf urdf/agribot.xacro`
- Check spawn entity logs for errors

### Controller Not Responding

- Verify hardware_aware_controller is running: `ros2 node list`
- Check cmd_vel topic: `ros2 topic echo /cmd_vel`
- Check controller output: `ros2 topic echo /left_track_cmd`

### Physics Issues

- Adjust friction in `config/gazebo_physics.yaml`
- Modify wheel friction in `urdf/agribot.xacro` (Gazebo material properties)
- Check mass distribution and inertia values

## License

MIT License

## Maintainer

User - user@example.com

## Acknowledgments

Designed for TheConstruct AI workspace simulation before physical robot construction.

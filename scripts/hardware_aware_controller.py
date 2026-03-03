#!/usr/bin/env python3
"""
Hardware-Aware Controller for Agribot Skid Steer Robot
Simulates the actual electronic constraints:
- 34-99 Quantizer (dead zone until step 34)
- Relay Interlock (150ms pause on direction change)
- Soft-Start Ramping (slew rate limiter)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import yaml
import os
from enum import Enum
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Direction(Enum):
    """Direction state for relay interlock"""
    FORWARD = 1
    REVERSE = -1
    STOPPED = 0

class HardwareAwareController(Node):
    """Hardware-aware controller with quantizer, interlock, and soft-start"""
    
    def __init__(self):
        super().__init__('hardware_aware_controller')
        
        # Load configuration
        self.load_config()
        
        # Controller state
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        self.target_left_cmd = 0.0
        self.target_right_cmd = 0.0
        self.direction_state = Direction.STOPPED
        self.interlock_active = False
        self.interlock_end_time = 0.0
        
        # Publishers for wheel commands
        self.left_cmd_pub = self.create_publisher(
            Float64,
            'left_track_cmd',
            10
        )
        self.right_cmd_pub = self.create_publisher(
            Float64,
            'right_track_cmd',
            10
        )
        
        # Subscriber for cmd_vel
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        # Timer for controller update loop
        update_period = 1.0 / self.config['soft_start']['update_rate']
        self.timer = self.create_timer(update_period, self.update_controller)
        
        self.get_logger().info('Hardware-aware controller started')
        self.get_logger().info(f'Quantizer: {self.config["quantizer"]["dead_zone_max"]}-{self.config["quantizer"]["max_output"]}')
        self.get_logger().info(f'Interlock delay: {self.config["relay_interlock"]["direction_switch_delay"]}s')
        
    def load_config(self):
        """Load hardware parameters from YAML file"""
        # Try multiple possible paths
        possible_paths = [
            # Installed path (share directory)
            os.path.join(
                os.path.dirname(__file__),
                '..', '..', '..', 'share', 'skid_steer_robot', 'config', 'hardware_params.yaml'
            ),
            # Source path
            os.path.join(
                os.path.dirname(__file__),
                '..', 'config', 'hardware_params.yaml'
            ),
            # Alternative installed path
            '/opt/ros/humble/share/skid_steer_robot/config/hardware_params.yaml'
        ]
        
        config_path = None
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                config_path = abs_path
                break
        
        if config_path is None:
            # Use first path as default (will use defaults if not found)
            config_path = os.path.abspath(possible_paths[0])
        
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                self.config = config_data['hardware_controller']
        except Exception as e:
            self.get_logger().warn(f'Could not load config file: {e}')
            # Default configuration
            self.config = {
                'quantizer': {
                    'dead_zone_min': 0.0,
                    'dead_zone_max': 0.34,
                    'max_output': 0.99,
                    'min_output': 0.34
                },
                'relay_interlock': {
                    'direction_switch_delay': 0.15,
                    'enabled': True
                },
                'soft_start': {
                    'max_acceleration': 0.5,
                    'max_deceleration': 0.5,
                    'update_rate': 50.0,
                    'enabled': True
                }
            }
    
    def quantize(self, value):
        """
        34-99 Quantizer: Dead zone until 0.34, then map to 0.34-0.99 range
        """
        abs_value = abs(value)
        
        # Dead zone: robot stays still
        if abs_value < self.config['quantizer']['dead_zone_max']:
            return 0.0
        
        # Map from [dead_zone_max, 1.0] to [min_output, max_output]
        dead_zone = self.config['quantizer']['dead_zone_max']
        min_out = self.config['quantizer']['min_output']
        max_out = self.config['quantizer']['max_output']
        
        # Normalize to [0, 1] range after dead zone
        normalized = (abs_value - dead_zone) / (1.0 - dead_zone)
        
        # Map to output range
        output = min_out + normalized * (max_out - min_out)
        
        # Clamp to max
        output = min(output, max_out)
        
        # Restore sign
        return output if value >= 0 else -output
    
    def check_relay_interlock(self, new_direction):
        """
        Relay Interlock: 150ms pause when direction changes
        Returns True if interlock is active (commands should be blocked)
        """
        if not self.config['relay_interlock']['enabled']:
            return False
        
        # Check if direction changed
        if new_direction != self.direction_state and new_direction != Direction.STOPPED:
            if self.direction_state != Direction.STOPPED:
                # Direction change detected - activate interlock
                delay = self.config['relay_interlock']['direction_switch_delay']
                current_time = self.get_clock().now().nanoseconds / 1e9
                self.interlock_end_time = current_time + delay
                self.interlock_active = True
                self.get_logger().info(f'Relay interlock activated: {delay}s delay')
        
        # Check if interlock period has passed
        if self.interlock_active:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time >= self.interlock_end_time:
                self.interlock_active = False
                self.get_logger().info('Relay interlock released')
            else:
                return True  # Interlock still active
        
        return False  # No interlock
    
    def apply_soft_start(self, target, current):
        """
        Soft-Start Ramping: Slew rate limiter to prevent chain snapping
        """
        if not self.config['soft_start']['enabled']:
            return target
        
        max_change = self.config['soft_start']['max_acceleration']
        if abs(target) < abs(current):
            max_change = self.config['soft_start']['max_deceleration']
        
        diff = target - current
        if abs(diff) <= max_change:
            return target
        
        # Limit the change
        return current + (max_change if diff > 0 else -max_change)
    
    def cmd_vel_callback(self, msg):
        """Process incoming cmd_vel commands"""
        # Convert Twist to left/right wheel velocities
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Skid steer kinematics: v_left = linear - angular * wheel_separation/2
        # Wheel separation: 0.87m (87cm track width)
        wheel_separation = 0.87
        left_vel = linear - angular * wheel_separation / 2.0
        right_vel = linear + angular * wheel_separation / 2.0
        
        # Account for 22.5:1 total reduction (10:1 gearbox + 2.25:1 chain)
        # At 5 km/h (1.38 m/s), motor runs at 1462 RPM
        # The "Step 34" jump now has 22.5x torque multiplier - very aggressive
        # Soft-start is critical to prevent torque-induced tip-over
        
        # Determine direction
        if left_vel > 0.01 or right_vel > 0.01:
            new_direction = Direction.FORWARD
        elif left_vel < -0.01 or right_vel < -0.01:
            new_direction = Direction.REVERSE
        else:
            new_direction = Direction.STOPPED
        
        # Apply quantizer
        left_quantized = self.quantize(left_vel)
        right_quantized = self.quantize(right_vel)
        
        # Check relay interlock
        if self.check_relay_interlock(new_direction):
            # Interlock active - set targets to zero
            self.target_left_cmd = 0.0
            self.target_right_cmd = 0.0
        else:
            # Update direction state
            self.direction_state = new_direction
            # Set target commands
            self.target_left_cmd = left_quantized
            self.target_right_cmd = right_quantized
    
    def update_controller(self):
        """Controller update loop: Apply soft-start ramping"""
        # Apply soft-start ramping
        self.current_left_cmd = self.apply_soft_start(
            self.target_left_cmd,
            self.current_left_cmd
        )
        self.current_right_cmd = self.apply_soft_start(
            self.target_right_cmd,
            self.current_right_cmd
        )
        
        # Publish commands
        left_msg = Float64()
        left_msg.data = self.current_left_cmd
        self.left_cmd_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = self.current_right_cmd
        self.right_cmd_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HardwareAwareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

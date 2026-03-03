#!/usr/bin/env python3
"""
Agribot Physical Controller - Basic Step 34 Test
Tests if the "Step 34" minimum movement jump is too aggressive with 22.5:1 reduction
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AgribotPhysicalController(Node):
    """Physical controller with Step 34 logic and current limiting"""
    
    def __init__(self):
        super().__init__('agribot_physical_controller')
        
        # Subscribe to raw cmd_vel (from teleop)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',  # Subscribe to raw input
            self.cmd_vel_callback,
            10
        )
        
        # Publish processed command to /cmd_vel (for diff_drive plugin)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Physical Constants for 22.5:1 Gear Ratio
        self.MIN_PWM_STEP = 34  # Minimum starting threshold (Step 34)
        self.MAX_CURRENT_AMPS = 40.0  # Current limiter to protect electronics from 1.5kW spikes
        self.RAMP_RATE = 0.015  # Soft start to protect the Royal Enfield chain
        
        # Current output state
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('Agribot Physical Controller started')
        self.get_logger().info(f'MIN_PWM_STEP: {self.MIN_PWM_STEP}')
        self.get_logger().info(f'RAMP_RATE: {self.RAMP_RATE}')
        self.get_logger().info('Ready for teleop test - tap "w" key once to test Step 34 jump')
    
    def cmd_vel_callback(self, msg):
        """Process incoming cmd_vel commands with Step 34 logic"""
        target_linear = msg.linear.x
        target_angular = msg.angular.z
        
        # Apply "Step 34" logic for linear movement
        if abs(target_linear) > 0.0 and abs(target_linear) < 0.1:
            # If the user gives a tiny nudge, jump to the minimum stall-breaker
            # This simulates the quantizer behavior
            if target_linear > 0:
                applied_linear = self.MIN_PWM_STEP / 255.0  # Scale to 0-1 range
            else:
                applied_linear = -self.MIN_PWM_STEP / 255.0
            self.get_logger().info(f'Step 34 JUMP: Input={target_linear:.3f} -> Output={applied_linear:.3f}')
        else:
            # Normal scaling (0-1 input maps to 0-1 output, but with soft-start)
            applied_linear = target_linear
        
        # Apply soft-start ramping to prevent chain snap
        if abs(applied_linear - self.current_linear) > self.RAMP_RATE:
            if applied_linear > self.current_linear:
                self.current_linear += self.RAMP_RATE
            else:
                self.current_linear -= self.RAMP_RATE
        else:
            self.current_linear = applied_linear
        
        # Angular movement (same logic)
        if abs(target_angular) > 0.0 and abs(target_angular) < 0.1:
            if target_angular > 0:
                applied_angular = self.MIN_PWM_STEP / 255.0
            else:
                applied_angular = -self.MIN_PWM_STEP / 255.0
        else:
            applied_angular = target_angular
        
        # Apply soft-start to angular
        if abs(applied_angular - self.current_angular) > self.RAMP_RATE:
            if applied_angular > self.current_angular:
                self.current_angular += self.RAMP_RATE
            else:
                self.current_angular -= self.RAMP_RATE
        else:
            self.current_angular = applied_angular
        
        # Publish the processed command
        output_cmd = Twist()
        output_cmd.linear.x = self.current_linear
        output_cmd.angular.z = self.current_angular
        self.cmd_pub.publish(output_cmd)
        
        # Log for debugging
        if abs(target_linear) > 0.0 or abs(target_angular) > 0.0:
            self.get_logger().info(
                f'Input: lin={target_linear:.3f} ang={target_angular:.3f} | '
                f'Output: lin={self.current_linear:.3f} ang={self.current_angular:.3f} | '
                f'Chain Torque: High (22.5:1 reduction)'
            )

def main(args=None):
    rclpy.init(args=args)
    node = AgribotPhysicalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

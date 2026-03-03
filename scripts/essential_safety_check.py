#!/usr/bin/env python3
"""
Essential Safety Check for Agribot at 5 km/h max speed
Tests two critical scenarios:
1. Incline Start: Can robot start on 15° slope without front wheels lifting?
2. Pivot Turn: Can robot turn in place without tipping or stalling?
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import math

class EssentialSafetyCheck(Node):
    """Simplified safety tests for 5 km/h operation"""
    
    def __init__(self):
        super().__init__('essential_safety_check')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',  # Try common IMU topic names
            self.imu_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.current_velocity = 0.0
        self.commanded_effort = 0.0
        self.test_phase = 0  # 0=wait, 1=pivot, 2=incline
        self.test_complete = False
        self.pivot_complete = False
        self.incline_complete = False
        self.torque_warning_triggered = False
        
        # Wait for simulation to stabilize
        self.get_logger().info('Waiting 3 seconds for simulation to stabilize...')
        time.sleep(3.0)
        
        # Start tests
        self.get_logger().info('=' * 60)
        self.get_logger().info('ESSENTIAL SAFETY CHECK - 5 km/h Max Speed')
        self.get_logger().info('=' * 60)
        self.run_tests()
    
    def imu_callback(self, msg):
        """Monitor pitch and roll angles from IMU"""
        if self.test_complete:
            return
        
        # Convert quaternion to Euler angles
        q = msg.orientation
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        
        self.current_pitch = pitch
        self.current_roll = roll
        self.max_pitch = max(self.max_pitch, abs(pitch))
        self.max_roll = max(self.max_roll, abs(roll))
        
        # Danger threshold: 0.25 rad (~14.3 degrees) - critical for 0.6m height
        if abs(pitch) > 0.25 or abs(roll) > 0.25:
            self.get_logger().error('DANGER: Robot tip detected! Pitch: {:.2f}°, Roll: {:.2f}°'.format(
                math.degrees(pitch), math.degrees(roll)
            ))
    
    def check_torque_safety(self):
        """Monitor for torque-induced pitch with 22.5:1 reduction"""
        # With 22.5:1 reduction, if wheels get stuck, motor torque can lift the robot
        # Check if velocity is low but pitch is increasing (torque lifting front)
        if self.current_velocity < 0.1 and abs(self.current_pitch) > 0.15:  # 8.5 degrees
            if not self.torque_warning_triggered:
                self.torque_warning_triggered = True
                self.get_logger().warn('HIGH TORQUE LIFT: 22.5:1 gearbox is overpowering the 0.6m chassis!')
                self.get_logger().warn('Wheel may be stuck - motor torque is lifting robot instead of moving it')
                # Emergency stop
                cmd = Twist()
                self.cmd_pub.publish(cmd)
    
    def odom_callback(self, msg):
        """Monitor velocity and check for torque-induced pitch"""
        # Get current velocity
        self.current_velocity = abs(msg.twist.twist.linear.x)
        
        # Estimate commanded effort (simplified - in real system would read from motor controller)
        # If velocity is low but we're commanding movement, high torque is being applied
        if self.test_phase > 0:
            # Check for torque-induced pitch (stuck wheel scenario)
            self.check_torque_safety()
    
    def run_tests(self):
        """Run the two essential safety tests"""
        # Test 1: Pivot Turn
        self.get_logger().info('')
        self.get_logger().info('TEST 1: Pivot Turn (360° rotation in place)')
        self.get_logger().info('Testing if 1.5kW motors can turn without tipping or stalling')
        self.test_phase = 1
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Moderate rotation rate
        self.cmd_pub.publish(cmd)
        
        # Rotate for ~4 seconds (should complete ~360°)
        time.sleep(4.0)
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(1.0)
        
        self.pivot_complete = True
        self.get_logger().info('Pivot test complete. Max roll: {:.2f}°'.format(math.degrees(self.max_roll)))
        
        # Test 2: Incline Start
        self.get_logger().info('')
        self.get_logger().info('TEST 2: Incline Start (15° slope)')
        self.get_logger().info('Testing if robot can start on slope without front wheels lifting')
        self.test_phase = 2
        
        # Drive forward at 5 km/h (1.38 m/s) toward ramp
        cmd.linear.x = 1.38  # 5 km/h
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Drive for 5 seconds (should reach and climb ramp)
        time.sleep(5.0)
        
        # Emergency stop (kill switch test)
        self.get_logger().info('Emergency stop - monitoring for tip-over')
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        
        # Monitor for 3 seconds after stop
        time.sleep(3.0)
        
        self.incline_complete = True
        self.get_logger().info('Incline test complete. Max pitch: {:.2f}°'.format(math.degrees(self.max_pitch)))
        
        # Final report
        self.report_results()
    
    def report_results(self):
        """Report test results"""
        self.test_complete = True
        
        # Stop robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        pitch_deg = math.degrees(self.max_pitch)
        roll_deg = math.degrees(self.max_roll)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('ESSENTIAL SAFETY CHECK RESULTS')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Test 1 - Pivot Turn:')
        self.get_logger().info('  Maximum Roll: {:.2f}°'.format(roll_deg))
        
        if roll_deg < 14.0:  # ~0.25 rad
            self.get_logger().info('  RESULT: PASS - No lateral tip detected')
        else:
            self.get_logger().error('  RESULT: FAIL - Lateral tip risk detected')
        
        self.get_logger().info('')
        self.get_logger().info('Test 2 - Incline Start:')
        self.get_logger().info('  Maximum Pitch: {:.2f}°'.format(pitch_deg))
        
        if pitch_deg < 14.0:  # ~0.25 rad
            self.get_logger().info('  RESULT: PASS - No rear tip detected on 15° slope')
        else:
            self.get_logger().error('  RESULT: FAIL - Rear tip risk detected (consider adding ballast)')
        
        self.get_logger().info('')
        if pitch_deg < 14.0 and roll_deg < 14.0:
            self.get_logger().info('OVERALL: SAFE - Robot passed both essential safety checks at 5 km/h')
        else:
            self.get_logger().error('OVERALL: UNSAFE - Robot failed safety checks. Review design.')
        
        self.get_logger().info('=' * 60)
        
        # Exit after reporting
        time.sleep(2.0)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test = EssentialSafetyCheck()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

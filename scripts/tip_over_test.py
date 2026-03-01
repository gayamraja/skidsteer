#!/usr/bin/env python3
"""
Tip-Over Test: Stress test to verify robot stability
Command: Standstill → Step 99 (max forward) in single update
Monitor: Base link orientation (roll/pitch angles)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import time
import math

class TipOverTest(Node):
    """Test robot stability with maximum acceleration"""
    
    def __init__(self):
        super().__init__('tip_over_test')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',  # Assuming IMU topic
            self.imu_callback,
            10
        )
        
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.test_started = False
        self.test_complete = False
        
        # Wait a bit for simulation to stabilize
        self.get_logger().info('Waiting 2 seconds for simulation to stabilize...')
        time.sleep(2.0)
        
        # Start test
        self.get_logger().info('Starting Tip-Over Test: Standstill → Max Forward')
        self.start_test()
    
    def start_test(self):
        """Send maximum forward command"""
        cmd = Twist()
        cmd.linear.x = 1.0  # Maximum forward (will be quantized to 0.99)
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.test_started = True
        self.get_logger().info('Max forward command sent')
        
        # Run test for 5 seconds
        self.create_timer(5.0, self.end_test)
    
    def imu_callback(self, msg):
        """Monitor orientation from IMU"""
        if not self.test_started or self.test_complete:
            return
        
        # Convert quaternion to Euler angles
        q = msg.orientation
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        
        # Track maximum angles
        self.max_roll = max(self.max_roll, abs(roll))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        
        # Check for tip-over (angles > 45 degrees)
        if abs(roll) > math.pi / 4 or abs(pitch) > math.pi / 4:
            self.get_logger().error('TIP-OVER DETECTED!')
            self.end_test()
    
    def end_test(self):
        """End test and report results"""
        if self.test_complete:
            return
        
        self.test_complete = True
        
        # Stop robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # Report results
        roll_deg = math.degrees(self.max_roll)
        pitch_deg = math.degrees(self.max_pitch)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('TIP-OVER TEST RESULTS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Maximum Roll: {roll_deg:.2f} degrees')
        self.get_logger().info(f'Maximum Pitch: {pitch_deg:.2f} degrees')
        
        # Success criteria: angles < 45 degrees
        if roll_deg < 45.0 and pitch_deg < 45.0:
            self.get_logger().info('RESULT: PASS - Robot remained stable')
        else:
            self.get_logger().error('RESULT: FAIL - Robot tipped over')
        
        self.get_logger().info('=' * 50)
        
        # Exit after reporting
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test = TipOverTest()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

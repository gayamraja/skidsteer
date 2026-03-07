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
        """Send maximum forward command and keep publishing to avoid 0.5s timeout"""
        self._cmd = Twist()
        self._cmd.linear.x = 1.0  # Maximum forward (will be quantized to 0.99)
        self.cmd_pub.publish(self._cmd)
        self.test_started = True
        self.get_logger().info('Max forward command sent')

        # Keep publishing at 10 Hz so hardware controller does not time out
        self._pub_timer = self.create_timer(0.1, self._republish)
        # Run test for 5 seconds
        self.create_timer(5.0, self.end_test)

    def _republish(self):
        self.cmd_pub.publish(self._cmd)
    
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
        
        # Check for tip-over (angles > 30 degrees for high-clearance robot)
        # At 0.6m height, even 30 degrees is dangerous
        tip_threshold = math.radians(30)  # 30 degrees
        if abs(roll) > tip_threshold or abs(pitch) > tip_threshold:
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
        
        # Success criteria: angles < 30 degrees (stricter for high-clearance 0.6m chassis)
        # At 0.6m height, 30 degrees is the practical limit before tip-over
        if roll_deg < 30.0 and pitch_deg < 30.0:
            self.get_logger().info('RESULT: PASS - Robot remained stable (high-clearance design)')
        else:
            self.get_logger().error('RESULT: FAIL - Robot exceeded 30 degree tip threshold')
        
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

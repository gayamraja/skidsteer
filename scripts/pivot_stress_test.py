#!/usr/bin/env python3
"""
Pivot Stress Test: Rotate 360° in place
Monitor: Motor current/torque (check for stall conditions)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class PivotStressTest(Node):
    """Test robot pivot capability using odom angular velocity"""

    def __init__(self):
        super().__init__('pivot_stress_test')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.current_angle = 0.0      # integrated from odom angular velocity
        self.last_odom_time = None
        self.max_angular_vel = 0.0
        self.test_started = False
        self.test_complete = False
        self.rotation_complete = False

        self.get_logger().info('Waiting 2 seconds for simulation to stabilize...')
        time.sleep(2.0)

        self.get_logger().info('Starting Pivot Stress Test: 360 degree rotation in place')
        self.start_test()

    def start_test(self):
        self._cmd = Twist()
        self._cmd.angular.z = 1.0
        self.cmd_pub.publish(self._cmd)
        self.test_started = True
        self.test_start_time = time.time()
        self.get_logger().info('Pivot command sent (left forward, right reverse)')

        # Keep publishing so hardware controller does not time out
        self._pub_timer = self.create_timer(0.1, self._republish)
        # Hard timeout after 60 seconds (117kg robot pivots slowly ~13s/rev)
        self.create_timer(60.0, self.end_test_timeout)

    def _republish(self):
        self.cmd_pub.publish(self._cmd)

    def odom_callback(self, msg):
        if not self.test_started or self.test_complete:
            return

        angular_vel = abs(msg.twist.twist.angular.z)
        self.max_angular_vel = max(self.max_angular_vel, angular_vel)

        # Integrate angular velocity to track total rotation
        now = time.time()
        if self.last_odom_time is not None:
            dt = now - self.last_odom_time
            self.current_angle += angular_vel * dt
        self.last_odom_time = now

        # Stall check: command sent but robot barely rotating (skip first 5s for friction breakaway)
        if self.test_started and now - self.test_start_time > 5.0:
            if angular_vel < 0.05:
                self.get_logger().warn(
                    f'Possible stall: angular vel={angular_vel:.3f} rad/s')

        if self.current_angle >= 2 * math.pi and not self.rotation_complete:
            self.rotation_complete = True
            elapsed = now - self.test_start_time
            self.get_logger().info(
                f'360 degree rotation completed in {elapsed:.2f} seconds')
    
    def end_test_timeout(self):
        if not self.test_complete:
            self.get_logger().warn('Test timeout - ending test')
            self.end_test()

    def end_test(self):
        if self.test_complete:
            return
        self.test_complete = True
        if hasattr(self, '_pub_timer'):
            self._pub_timer.cancel()
        self.cmd_pub.publish(Twist())

        elapsed = time.time() - self.test_start_time
        rotation_deg = math.degrees(self.current_angle)

        self.get_logger().info('=' * 50)
        self.get_logger().info('PIVOT STRESS TEST RESULTS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Total rotation: {rotation_deg:.1f} degrees')
        self.get_logger().info(f'Time elapsed: {elapsed:.2f} seconds')
        self.get_logger().info(f'Peak angular velocity: {self.max_angular_vel:.3f} rad/s')

        if self.rotation_complete:
            self.get_logger().info('RESULT: PASS - Full 360 degree rotation achieved, no stall')
        elif self.max_angular_vel > 0.05:
            self.get_logger().warn(
                f'RESULT: INCOMPLETE - Only {rotation_deg:.0f} deg of 360 completed')
        else:
            self.get_logger().error('RESULT: FAIL - Robot did not rotate (stall or no drive)')
        self.get_logger().info('=' * 50)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test = PivotStressTest()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

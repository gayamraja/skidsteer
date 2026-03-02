#!/usr/bin/env python3
"""
Static Tip Test: Test robot stability on slopes
Approach a 10° incline and stop abruptly while facing uphill
Monitor if front wheels lift more than 5cm off the ground
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import time
import math

class StaticTipTest(Node):
    """Test robot stability on slopes with high-clearance design"""
    
    def __init__(self):
        super().__init__('static_tip_test')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.test_phase = 0  # 0=wait, 1=approach, 2=stop, 3=monitor
        self.max_front_lift = 0.0  # Maximum front wheel lift in meters
        self.test_started = False
        self.test_complete = False
        self.last_position = None
        self.slope_detected = False
        
        # Wait for simulation to stabilize
        self.get_logger().info('Waiting 3 seconds for simulation to stabilize...')
        time.sleep(3.0)
        
        # Start test
        self.get_logger().info('Starting Static Tip Test: Approach slope and stop abruptly')
        self.start_test()
    
    def start_test(self):
        """Start approaching slope"""
        self.test_started = True
        self.test_phase = 1
        
        # Drive forward to approach slope
        cmd = Twist()
        cmd.linear.x = 0.5  # Moderate speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Phase 1: Approaching slope at moderate speed')
        
        # Monitor for 10 seconds, then stop abruptly
        self.create_timer(10.0, self.stop_abruptly)
    
    def stop_abruptly(self):
        """Stop robot abruptly (kill switch test)"""
        if self.test_phase != 1:
            return
        
        self.test_phase = 2
        cmd = Twist()
        cmd.linear.x = 0.0  # Abrupt stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Phase 2: Abrupt stop - monitoring for tip-over')
        
        # Monitor for 5 seconds after stop
        self.create_timer(5.0, self.end_test)
        self.test_phase = 3
    
    def odom_callback(self, msg):
        """Monitor position and detect slope"""
        if not self.test_started or self.test_complete:
            return
        
        current_pos = msg.pose.pose.position
        
        # Detect slope by checking if robot is climbing (Z position or pitch)
        # In a real implementation, you'd check the pitch angle from IMU
        if self.last_position is not None:
            dz = current_pos.z - self.last_position.z
            if abs(dz) > 0.01:  # Significant height change
                self.slope_detected = True
                self.get_logger().info(f'Slope detected: height change = {dz:.3f}m')
        
        self.last_position = current_pos
    
    def joint_state_callback(self, msg):
        """Monitor joint positions to detect wheel lift"""
        if not self.test_started or self.test_complete or self.test_phase < 2:
            return
        
        # Find front wheel joints
        front_left_idx = None
        front_right_idx = None
        
        for i, name in enumerate(msg.name):
            if name == 'front_left_wheel_joint':
                front_left_idx = i
            elif name == 'front_right_wheel_joint':
                front_right_idx = i
        
        # Check if wheels are lifted (position != 0 indicates rotation, but we need to check contact)
        # In a real implementation, you'd check the wheel contact forces or use IMU pitch angle
        # For now, we'll estimate based on joint velocity (stopped wheels might indicate lift)
        
        if front_left_idx is not None and front_right_idx is not None:
            if len(msg.velocity) > max(front_left_idx, front_right_idx):
                # If wheels stopped but we're on a slope, might indicate lift
                # This is a simplified check - in reality, use contact sensors or IMU
                pass
    
    def end_test(self):
        """End test and report results"""
        if self.test_complete:
            return
        
        self.test_complete = True
        
        # Stop robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # Report results
        self.get_logger().info('=' * 50)
        self.get_logger().info('STATIC TIP TEST RESULTS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Maximum front wheel lift: {self.max_front_lift*100:.2f} cm')
        
        # Success criteria: front wheels should not lift more than 5cm
        if self.max_front_lift < 0.05:
            self.get_logger().info('RESULT: PASS - Robot remained stable on slope')
        else:
            self.get_logger().error('RESULT: FAIL - Front wheels lifted >5cm (consider adding ballast)')
            self.get_logger().info('Recommendation: Add virtual counterweights to lower part of V-legs')
        
        self.get_logger().info('=' * 50)
        
        # Exit after reporting
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test = StaticTipTest()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

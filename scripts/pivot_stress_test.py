#!/usr/bin/env python3
"""
Pivot Stress Test: Rotate 360° in place
Monitor: Motor current/torque (check for stall conditions)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import math

class PivotStressTest(Node):
    """Test robot pivot capability and motor stall resistance"""
    
    def __init__(self):
        super().__init__('pivot_stress_test')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.start_angle = None
        self.current_angle = 0.0
        self.max_torque = 0.0
        self.test_started = False
        self.test_complete = False
        self.rotation_complete = False
        
        # Wait for simulation to stabilize
        self.get_logger().info('Waiting 2 seconds for simulation to stabilize...')
        time.sleep(2.0)
        
        # Start test
        self.get_logger().info('Starting Pivot Stress Test: 360° rotation in place')
        self.start_test()
    
    def start_test(self):
        """Send pivot command (left forward, right reverse)"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0  # Maximum rotation rate
        self.cmd_pub.publish(cmd)
        self.test_started = True
        self.test_start_time = time.time()
        self.get_logger().info('Pivot command sent (left forward, right reverse)')
        
        # Timeout after 30 seconds
        self.create_timer(30.0, self.end_test_timeout)
    
    def joint_state_callback(self, msg):
        """Monitor joint states for torque and rotation"""
        if not self.test_started or self.test_complete:
            return
        
        # Find wheel joints and track efforts (torque)
        for i, name in enumerate(msg.name):
            if 'wheel' in name:
                if len(msg.effort) > i:
                    torque = abs(msg.effort[i])
                    self.max_torque = max(self.max_torque, torque)
                    
                    # Check for stall (torque near max but velocity near zero)
                    if len(msg.velocity) > i:
                        velocity = abs(msg.velocity[i])
                        if torque > 250.0 and velocity < 0.1:  # Near max torque, low velocity
                            self.get_logger().warn(f'Potential stall detected on {name}: torque={torque:.2f}Nm, vel={velocity:.2f}rad/s')
        
        # Estimate rotation from angular velocity (simplified)
        # In a real implementation, you'd integrate or use odometry
        if len(msg.velocity) > 0:
            # Assuming first wheel joint velocity represents rotation
            angular_vel = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
            dt = 0.1  # Approximate update period
            self.current_angle += abs(angular_vel) * dt
        
        # Check if 360° rotation complete (2π radians)
        if self.current_angle >= 2 * math.pi and not self.rotation_complete:
            self.rotation_complete = True
            elapsed = time.time() - self.test_start_time
            self.get_logger().info(f'360° rotation completed in {elapsed:.2f} seconds')
            self.end_test()
    
    def end_test_timeout(self):
        """End test on timeout"""
        if not self.test_complete:
            self.get_logger().warn('Test timeout - ending test')
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
        elapsed = time.time() - self.test_start_time
        rotation_deg = math.degrees(self.current_angle)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('PIVOT STRESS TEST RESULTS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Rotation: {rotation_deg:.2f} degrees')
        self.get_logger().info(f'Time elapsed: {elapsed:.2f} seconds')
        self.get_logger().info(f'Maximum torque: {self.max_torque:.2f} Nm')
        
        # Success criteria: completed rotation without stall
        if self.rotation_complete and self.max_torque < 300.0:
            self.get_logger().info('RESULT: PASS - Motors did not stall')
        elif self.max_torque >= 300.0:
            self.get_logger().error('RESULT: FAIL - Motors reached torque limit (stall)')
        else:
            self.get_logger().warn('RESULT: INCOMPLETE - Rotation not completed')
        
        self.get_logger().info('=' * 50)
        
        # Exit after reporting
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

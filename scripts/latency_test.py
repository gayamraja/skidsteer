#!/usr/bin/env python3
"""
Latency Test: Measure delay between command and motion start
Command sequence: Rapid direction changes (Forward→Reverse→Forward)
Measure: Relay interlock delay (150ms) + soft-start ramp time
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from collections import deque

class LatencyTest(Node):
    """Test command-to-motion latency"""
    
    def __init__(self):
        super().__init__('latency_test')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.test_phase = 0
        self.command_times = []
        self.motion_start_times = []
        self.last_position = None
        self.motion_detected = False
        self.test_complete = False
        
        # Wait for simulation to stabilize
        self.get_logger().info('Waiting 2 seconds for simulation to stabilize...')
        time.sleep(2.0)
        
        # Start test sequence
        self.get_logger().info('Starting Latency Test: Direction change sequence')
        self.start_test_sequence()
    
    def _set_drive(self, linear_x):
        """Publish a command and keep republishing at 10 Hz until next phase."""
        if hasattr(self, '_pub_timer') and self._pub_timer is not None:
            self._pub_timer.cancel()
        self._current_cmd = Twist()
        self._current_cmd.linear.x = linear_x
        self.cmd_pub.publish(self._current_cmd)
        self._pub_timer = self.create_timer(0.1, self._republish)

    def _republish(self):
        self.cmd_pub.publish(self._current_cmd)

    def start_test_sequence(self):
        """Execute test sequence: Forward → Reverse → Forward"""
        self.get_logger().info('Phase 1: Sending FORWARD command')
        self._set_drive(1.0)
        self.command_times.append(time.time())
        self.test_phase = 1
        self.motion_detected = False
        self.create_timer(3.0, self.phase_2_reverse)

    def phase_2_reverse(self):
        if self.test_phase != 1:
            return
        self.get_logger().info('Phase 2: Sending REVERSE command (direction change)')
        self._set_drive(-1.0)
        self.command_times.append(time.time())
        self.test_phase = 2
        self.motion_detected = False
        self.last_position = None
        self.create_timer(3.0, self.phase_3_forward)

    def phase_3_forward(self):
        if self.test_phase != 2:
            return
        self.get_logger().info('Phase 3: Sending FORWARD command (direction change)')
        self._set_drive(1.0)
        self.command_times.append(time.time())
        self.test_phase = 3
        self.motion_detected = False
        self.last_position = None
        self.create_timer(3.0, self.end_test)
    
    def odom_callback(self, msg):
        """Monitor odometry for motion detection"""
        if self.test_complete:
            return
        
        current_pos = msg.pose.pose.position
        current_time = time.time()
        
        # Detect motion start (position change > threshold)
        if self.last_position is not None:
            dx = current_pos.x - self.last_position.x
            dy = current_pos.y - self.last_position.y
            distance = (dx**2 + dy**2)**0.5
            
            # Motion detected if moved more than 1cm
            if distance > 0.01 and not self.motion_detected:
                self.motion_detected = True
                self.motion_start_times.append(current_time)
                
                # Calculate latency
                if len(self.command_times) == len(self.motion_start_times):
                    latency = current_time - self.command_times[-1]
                    phase_name = ['', 'Forward', 'Reverse', 'Forward'][self.test_phase]
                    self.get_logger().info(
                        f'{phase_name} motion started: {latency*1000:.1f}ms latency'
                    )
        
        self.last_position = current_pos
    
    def end_test(self):
        """End test and report results"""
        if self.test_complete:
            return
        
        self.test_complete = True
        
        # Stop robot
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        # Calculate latencies
        latencies = []
        for i in range(min(len(self.command_times), len(self.motion_start_times))):
            latency = self.motion_start_times[i] - self.command_times[i]
            latencies.append(latency)
        
        # Report results
        self.get_logger().info('=' * 50)
        self.get_logger().info('LATENCY TEST RESULTS')
        self.get_logger().info('=' * 50)
        
        if len(latencies) > 0:
            avg_latency = sum(latencies) / len(latencies)
            max_latency = max(latencies)
            min_latency = min(latencies)
            
            self.get_logger().info(f'Average latency: {avg_latency*1000:.1f}ms')
            self.get_logger().info(f'Minimum latency: {min_latency*1000:.1f}ms')
            self.get_logger().info(f'Maximum latency: {max_latency*1000:.1f}ms')
            
            # Expected: 150ms relay interlock + soft-start ramp + quantizer dead zone
            # On direction change: stop → 150ms interlock → ramp from 0 → odom detects motion
            # This realistically takes 800-1300ms and is by design
            self.get_logger().info('Expected: 150ms interlock + soft-start ramp (~800-1300ms total)')

            # Success criteria: < 1500ms (anything longer means something is broken)
            if max_latency < 1.5:
                self.get_logger().info(
                    f'RESULT: PASS - Latency {max_latency*1000:.0f}ms within design limits')
            else:
                self.get_logger().warn(
                    f'RESULT: WARNING - Latency {max_latency*1000:.0f}ms exceeds 1500ms')
        else:
            self.get_logger().error('RESULT: FAIL - No motion detected')
        
        self.get_logger().info('=' * 50)
        
        # Exit after reporting
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test = LatencyTest()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

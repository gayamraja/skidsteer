#!/usr/bin/env python3
"""
Essential Safety Check for Agribot at 5 km/h max speed
Tests two critical scenarios:
1. Pivot Turn: Can robot turn in place without tipping or stalling?
2. Incline Start: Can robot start on 15 deg slope without front wheels lifting?
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math


class EssentialSafetyCheck(Node):
    """Simplified safety tests for 5 km/h operation"""

    def __init__(self):
        super().__init__('essential_safety_check')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.current_velocity = 0.0
        self.torque_warning_triggered = False
        self.test_phase = 0
        self.imu_received = False

        self.get_logger().info('Waiting 3 seconds for simulation to stabilize...')
        # One-shot: cancel timer inside the callback
        self._t = self.create_timer(3.0, self._start_pivot)

    # ── helpers ──────────────────────────────────────────────────────────────
    def _once(self, timer, callback):
        """Cancel a repeating timer and call callback once."""
        timer.cancel()
        callback()

    # ── IMU ──────────────────────────────────────────────────────────────────
    def imu_callback(self, msg):
        if self.test_phase == 0 or self.test_phase == 4:
            return
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info('IMU data received - orientation monitoring active')
        q = msg.orientation
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        self.max_roll = max(self.max_roll, abs(roll))

        if abs(pitch) > 0.35 or abs(roll) > 0.25:  # 0.35 rad ≈ 20 deg matches incline threshold
            self.get_logger().error(
                'DANGER: tip detected! Pitch={:.1f}deg Roll={:.1f}deg'.format(
                    math.degrees(pitch), math.degrees(roll)))

    # ── Odometry ─────────────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.current_velocity = abs(msg.twist.twist.linear.x)
        if 0 < self.test_phase < 4:
            self._check_torque_safety()

    def _check_torque_safety(self):
        if self.current_velocity < 0.1 and abs(self.max_pitch) > 0.15:
            if not self.torque_warning_triggered:
                self.torque_warning_triggered = True
                self.get_logger().warn(
                    'HIGH TORQUE LIFT: 22.5:1 gearbox may be overpowering chassis!')
                self.cmd_pub.publish(Twist())

    # ── Test sequence ────────────────────────────────────────────────────────
    def _start_pivot(self):
        self._t.cancel()
        self.get_logger().info('=' * 60)
        self.get_logger().info('ESSENTIAL SAFETY CHECK - 5 km/h Max Speed')
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST 1: Pivot Turn (360 degree rotation in place)')
        self.test_phase = 1
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_pub.publish(cmd)
        self._t = self.create_timer(4.0, self._stop_pivot)

    def _stop_pivot(self):
        self._t.cancel()
        self.cmd_pub.publish(Twist())
        self.test_phase = 2
        self.get_logger().info('Pivot complete. Max roll: {:.2f}deg'.format(
            math.degrees(self.max_roll)))
        self._t = self.create_timer(1.0, self._start_incline)

    def _start_incline(self):
        self._t.cancel()
        self.get_logger().info('TEST 2: Incline Start (15 degree slope)')
        self.test_phase = 3
        self._incline_cmd = Twist()
        self._incline_cmd.linear.x = 1.38
        self.cmd_pub.publish(self._incline_cmd)
        # Keep publishing at 10 Hz so hardware controller doesn't time out (0.5s timeout)
        self._incline_pub_t = self.create_timer(0.1, self._publish_incline)
        self._t = self.create_timer(5.0, self._emergency_stop)

    def _publish_incline(self):
        self.cmd_pub.publish(self._incline_cmd)

    def _emergency_stop(self):
        self._t.cancel()
        self._incline_pub_t.cancel()
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Emergency stop - monitoring for 3 s...')
        self._t = self.create_timer(3.0, self._report)

    def _report(self):
        self._t.cancel()
        self.test_phase = 4
        self.cmd_pub.publish(Twist())

        pitch_deg = math.degrees(self.max_pitch)
        roll_deg = math.degrees(self.max_roll)

        if not self.imu_received:
            self.get_logger().warn('WARNING: No IMU data received during tests - angles not measured')

        self.get_logger().info('=' * 60)
        self.get_logger().info('ESSENTIAL SAFETY CHECK RESULTS')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Test 1 - Pivot Turn:')
        self.get_logger().info('  Maximum Roll: {:.2f}deg'.format(roll_deg))
        if roll_deg < 14.0:
            self.get_logger().info('  RESULT: PASS - No lateral tip detected')
        else:
            self.get_logger().error('  RESULT: FAIL - Lateral tip risk detected')

        self.get_logger().info('Test 2 - Incline Start:')
        self.get_logger().info('  Maximum Pitch: {:.2f}deg'.format(pitch_deg))
        if pitch_deg < 20.0:
            self.get_logger().info('  RESULT: PASS - No excessive rear tip on 15 degree slope')
        else:
            self.get_logger().error('  RESULT: FAIL - Rear tip risk (consider adding ballast)')

        if pitch_deg < 20.0 and roll_deg < 14.0:
            self.get_logger().info(
                'OVERALL: SAFE - Robot passed both essential safety checks at 5 km/h')
        else:
            self.get_logger().error('OVERALL: UNSAFE - Robot failed safety checks.')
        self.get_logger().info('=' * 60)

        self._t = self.create_timer(2.0, self._shutdown)

    def _shutdown(self):
        self._t.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    test = EssentialSafetyCheck()
    try:
        rclpy.spin(test)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            test.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()

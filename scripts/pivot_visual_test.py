#!/usr/bin/env python3
"""
Pivot Visual Test: Spin in place for 30 seconds so you can watch the wheel dots.
Left wheels (red) spin forward, right wheels (blue) spin backward.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PivotVisualTest(Node):

    def __init__(self):
        super().__init__('pivot_visual_test')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._elapsed = 0
        self._duration = 30  # seconds of spinning

        self.get_logger().info('Waiting 3s for sim to stabilize...')
        self._start_timer = self.create_timer(3.0, self._start)

    def _start(self):
        self._start_timer.cancel()
        self.get_logger().info('=' * 50)
        self.get_logger().info('PIVOT VISUAL TEST - spinning for 30 seconds')
        self.get_logger().info('Watch the YELLOW DOTS on the tires:')
        self.get_logger().info('  RED wheels  (left)  -> dots spin FORWARD')
        self.get_logger().info('  BLUE wheels (right) -> dots spin BACKWARD')
        self.get_logger().info('=' * 50)

        self._cmd = Twist()
        self._cmd.angular.z = 0.8
        self.cmd_pub.publish(self._cmd)

        self._pub_timer = self.create_timer(0.1, self._publish)
        self._count_timer = self.create_timer(1.0, self._tick)

    def _publish(self):
        self.cmd_pub.publish(self._cmd)

    def _tick(self):
        self._elapsed += 1
        remaining = self._duration - self._elapsed
        if remaining > 0 and remaining % 5 == 0:
            self.get_logger().info(f'{remaining}s remaining...')
        if self._elapsed >= self._duration:
            self._pub_timer.cancel()
            self._count_timer.cancel()
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Pivot complete - robot stopped.')
            self.create_timer(2.0, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PivotVisualTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()

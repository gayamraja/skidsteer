#!/usr/bin/env python3
"""
serial_bridge_node.py — ROS2 ↔ Arduino USB serial bridge for Agribot real hardware.

Subscribes to /diff_cont/cmd_vel (Twist), converts to normalised left/right wheel
velocities, and sends CMD packets to the Arduino at 50 Hz.

Receives FB packets from the Arduino containing encoder ticks and current readings,
computes odometry, and publishes /odom and /serial_status.

Serial protocol (ASCII, newline-terminated):
  RPi  → Arduino : CMD:<left> <right>\\n   (float -1.0 to +1.0)
  Arduino → RPi  : FB:<l_ticks> <r_ticks> <l_amps> <r_amps> <status>\\n
  status values  : OK | ESTOP | WATCHDOG | OVERCURRENT
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros

try:
    import serial
except ImportError:
    raise SystemExit(
        "pyserial not installed. Run: pip3 install pyserial"
    )


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('command_rate', 50.0)
        self.declare_parameter('track_width', 0.87)
        self.declare_parameter('wheel_radius', 0.2025)
        self.declare_parameter('gear_ratio', 22.5)
        self.declare_parameter('encoder_ticks_per_rev', 360)
        self.declare_parameter('max_current_amps', 40.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.command_rate = self.get_parameter('command_rate').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        ticks_per_motor_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.max_current = self.get_parameter('max_current_amps').value

        # Ticks per wheel revolution (after full gear reduction)
        self.ticks_per_wheel_rev = ticks_per_motor_rev * self.gear_ratio
        # Metres per tick
        self.metres_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_wheel_rev
        # Max wheel surface speed (maps normalised 1.0 → this m/s)
        # At 5 km/h = 1.389 m/s, wheel RPM = 65, rad/s = 6.807
        self.max_wheel_speed = 1.389  # m/s at normalised 1.0

        # ── State ────────────────────────────────────────────────────────────
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.cmd_lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_odom_time = self.get_clock().now()

        # ── Serial port ──────────────────────────────────────────────────────
        self.ser = None
        self._open_serial()

        # ── ROS interfaces ───────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/diff_cont/cmd_vel', self._cmd_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/serial_status', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Timers ───────────────────────────────────────────────────────────
        period = 1.0 / self.command_rate
        self.cmd_timer = self.create_timer(period, self._send_command)

        # Serial read runs in a background thread to avoid blocking the event loop
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'SerialBridgeNode started on {self.port} @ {self.baud} baud, '
            f'{self.command_rate:.0f} Hz'
        )

    # ── Serial helpers ────────────────────────────────────────────────────────

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0)  # Allow Arduino to reset after DTR toggle
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial port {self.port} opened.')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {self.port}: {e}')
            self.ser = None

    # ── Command callback ──────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        """Convert Twist → normalised left/right wheel velocities."""
        v = msg.linear.x      # m/s forward
        w = msg.angular.z     # rad/s

        left_ms  = v - w * self.track_width / 2.0
        right_ms = v + w * self.track_width / 2.0

        left_norm  = max(-1.0, min(1.0, left_ms  / self.max_wheel_speed))
        right_norm = max(-1.0, min(1.0, right_ms / self.max_wheel_speed))

        with self.cmd_lock:
            self.left_cmd  = left_norm
            self.right_cmd = right_norm

    # ── 50 Hz send timer ──────────────────────────────────────────────────────

    def _send_command(self):
        if self.ser is None or not self.ser.is_open:
            return
        with self.cmd_lock:
            l, r = self.left_cmd, self.right_cmd
        try:
            packet = f'CMD:{l:.4f} {r:.4f}\n'
            self.ser.write(packet.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Background read loop ──────────────────────────────────────────────────

    def _read_loop(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('FB:'):
                    self._process_feedback(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    # ── Feedback parser + odometry ────────────────────────────────────────────

    def _process_feedback(self, line: str):
        """Parse FB:<l_ticks> <r_ticks> <l_amps> <r_amps> <status> and update odom."""
        try:
            # Strip prefix
            data = line[3:]  # remove 'FB:'
            parts = data.split()
            if len(parts) != 5:
                return

            l_ticks = int(parts[0])
            r_ticks = int(parts[1])
            l_amps  = float(parts[2])
            r_amps  = float(parts[3])
            status  = parts[4]

        except (ValueError, IndexError):
            self.get_logger().warn(f'Malformed FB packet: {line}')
            return

        # Publish status
        self.status_pub.publish(String(data=status))

        # Log warnings for non-OK status
        if status == 'OVERCURRENT':
            self.get_logger().warn(
                f'OVERCURRENT detected: L={l_amps:.1f}A R={r_amps:.1f}A (limit {self.max_current}A)'
            )
        elif status == 'WATCHDOG':
            self.get_logger().error('Arduino WATCHDOG fired — RPi command stream lost!')
        elif status == 'ESTOP':
            self.get_logger().error('Arduino ESTOP active!')

        # ── Odometry from encoder ticks ──────────────────────────────────────
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks  = l_ticks
            self.prev_right_ticks = r_ticks
            self.last_odom_time   = now
            return

        d_left  = (l_ticks - self.prev_left_ticks)  * self.metres_per_tick
        d_right = (r_ticks - self.prev_right_ticks) * self.metres_per_tick
        self.prev_left_ticks  = l_ticks
        self.prev_right_ticks = r_ticks

        d_centre = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.track_width

        self.x     += d_centre * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_centre * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        v_lin = d_centre / dt
        v_ang = d_theta  / dt

        self._publish_odom(now, v_lin, v_ang)

    def _publish_odom(self, stamp, v_lin: float, v_ang: float):
        q = _yaw_to_quaternion(self.theta)

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp    = stamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = v_lin
        odom.twist.twist.angular.z = v_ang
        self.odom_pub.publish(odom)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            # Send a zero command before closing so Arduino watchdog stops motors cleanly
            try:
                self.ser.write(b'CMD:0.0000 0.0000\n')
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
serial_bridge_node.py  (on/off hardware version)

Bridges ROS2 /cmd_vel -> real Arduino over USB serial. Since this rig's
L298N has no PWM (ENA/ENB jumpered permanently on), motors are ON/OFF
only. This node converts a continuous Twist (linear.x, angular.z) into
simple forward/reverse/stop/turn commands per side.

  /cmd_vel (Twist) --> this node --> serial "L:<val>,R:<val>\n" --> Arduino

Logic:
  v = linear.x, w = angular.z
  left_side  = v - w * (separation/2)
  right_side = v + w * (separation/2)
  Then each side is thresholded: >deadzone -> +100 (full fwd),
  < -deadzone -> -100 (full reverse), else 0 (stop).

This means the real car will feel "bang-bang": full speed or nothing,
which matches your hardware exactly (no PWM available). Turning in
place happens naturally when j/l keys are pressed in teleop, since that
produces opposite-sign v for each side.

USAGE:
  python3 serial_bridge_node.py --ros-args -p serial_port:=/dev/ttyACM0

NOTE ON FEEDBACK: no wheel encoders on this rig, so /odom_estimate here
is a dead-reckoning ESTIMATE (integrated from commanded velocity x time),
not measured truth. It WILL drift, especially since motion is bang-bang
rather than smooth. Treat it as approximate.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ---------------- parameters ----------------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.18)     # meters, tune to your chassis
        self.declare_parameter('deadzone', 0.05)             # m/s or rad/s below which = stop
        self.declare_parameter('estimated_speed', 0.25)      # m/s, real car's rough full speed (measure this!)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.deadzone = self.get_parameter('deadzone').value
        self.estimated_speed = self.get_parameter('estimated_speed').value

        # ---------------- serial connection ----------------
        self.ser = None
        if serial is None:
            self.get_logger().error("pyserial not installed. Run: pip install pyserial")
        else:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
                time.sleep(2.0)  # allow Arduino to reset after serial port opens
                self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            except Exception as e:
                self.get_logger().error(
                    f"Could not open serial port {self.serial_port}: {e}. "
                    f"Node will run but commands will not reach hardware."
                )

        # ---------------- dead-reckoning state ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_left_cmd = 0
        self.last_right_cmd = 0

        # ---------------- ROS interfaces ----------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_estimate', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.integrate_and_publish)  # 20 Hz

        self.get_logger().info(
            "serial_bridge_node (on/off hardware) started. "
            "/odom_estimate is a dead-reckoning estimate, not measured."
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        self.last_v = v
        self.last_w = w

        left_val  = v - (w * self.wheel_separation / 2.0)
        right_val = v + (w * self.wheel_separation / 2.0)

        left_cmd  = self._to_bang_bang(left_val)
        right_cmd = self._to_bang_bang(right_val)

        self.last_left_cmd = left_cmd
        self.last_right_cmd = right_cmd

        self.send_to_arduino(left_cmd, right_cmd)

    def _to_bang_bang(self, val: float) -> int:
        if val > self.deadzone:
            return 100
        elif val < -self.deadzone:
            return -100
        else:
            return 0

    def send_to_arduino(self, left_cmd: int, right_cmd: int):
        line = f"L:{left_cmd},R:{right_cmd}\n"
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
        self.get_logger().debug(f"-> Arduino: {line.strip()}")

    def integrate_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # approximate actual velocity from bang-bang command direction
        v_est = 0.0
        if self.last_left_cmd != 0 and self.last_right_cmd != 0:
            if self.last_left_cmd == self.last_right_cmd:
                v_est = self.estimated_speed * (1 if self.last_left_cmd > 0 else -1)
        w_est = 0.0
        if self.last_left_cmd != self.last_right_cmd:
            # opposite signs (or one zero) -> treat as turning in place
            direction = 1 if self.last_right_cmd > self.last_left_cmd else -1
            w_est = direction * (self.estimated_speed / max(self.wheel_separation, 0.01))

        self.x += v_est * math.cos(self.theta) * dt
        self.y += v_est * math.sin(self.theta) * dt
        self.theta += w_est * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom_estimate'
        odom.child_frame_id = 'base_link_estimate'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v_est
        odom.twist.twist.angular.z = w_est
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom_estimate'
        t.child_frame_id = 'base_link_estimate'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.send_to_arduino(0, 0)
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
serial_bridge_node.py

Bridges ROS2 <-> real Arduino over USB serial, so the SAME /cmd_vel
commands that drive the Gazebo simulation also drive the physical
L298N car once it's wired up.

  /cmd_vel (geometry_msgs/Twist)  --->  this node  --->  serial "L:x,R:y\n" --> Arduino
  Arduino "OK:l,r\n" ack          --->  this node  --->  /odom_estimate (nav_msgs/Odometry)

IMPORTANT: No wheel encoders on this rig. The published /odom_estimate is
a DEAD-RECKONING ESTIMATE from commanded velocity x elapsed time, not
ground truth. It will drift. Treat it as "best guess" feedback until
encoders are added. It is published on a different topic name
(/odom_estimate) than Gazebo's ground-truth /odom so you can compare them
side by side if running both.

USAGE:
  ros2 run diffbot_sim serial_bridge_node.py --ros-args -p serial_port:=/dev/ttyUSB0

Or just:
  python3 serial_bridge_node.py
(after `pip install pyserial` and editing SERIAL_PORT below, or passing
 the serial_port ROS2 parameter)

TUNING:
  WHEEL_SEPARATION, WHEEL_RADIUS, MAX_PWM, MAX_LINEAR_SPEED must be
  calibrated to your real car once wired. Defaults are reasonable
  guesses for a small 4WD hobby chassis.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

try:
    import serial
except ImportError:
    serial = None


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ---------- parameters (override via --ros-args -p name:=value) ----------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.18)   # meters, tune to your chassis
        self.declare_parameter('wheel_radius', 0.033)       # meters
        self.declare_parameter('max_linear_speed', 0.3)     # m/s at full PWM (calibrate!)
        self.declare_parameter('max_pwm', 255)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value

        # ---------- serial connection ----------
        self.ser = None
        if serial is None:
            self.get_logger().error(
                "pyserial not installed. Run: pip install pyserial")
        else:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
                time.sleep(2.0)  # allow Arduino to reset after serial open
                self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            except Exception as e:
                self.get_logger().error(
                    f"Could not open serial port {self.serial_port}: {e}. "
                    f"Node will run but commands will not reach hardware."
                )

        # ---------- dead-reckoning state ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_v = 0.0
        self.last_w = 0.0

        # ---------- ROS interfaces ----------
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom_estimate', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # dead-reckoning integration timer (independent of when commands arrive)
        self.timer = self.create_timer(0.05, self.integrate_and_publish)  # 20 Hz

        self.get_logger().info(
            "serial_bridge_node started. NOTE: /odom_estimate is an "
            "open-loop dead-reckoning estimate (no encoders on this rig)."
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x          # m/s
        w = msg.angular.z         # rad/s

        self.last_v = v
        self.last_w = w

        # convert body velocity (v, w) -> left/right wheel speeds (m/s)
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        # convert wheel speed -> PWM value, scaled by calibrated max speed
        pwm_left = int(max(-self.max_pwm, min(self.max_pwm,
                       (v_left / self.max_linear_speed) * self.max_pwm)))
        pwm_right = int(max(-self.max_pwm, min(self.max_pwm,
                        (v_right / self.max_linear_speed) * self.max_pwm)))

        self.send_to_arduino(pwm_left, pwm_right)

    def send_to_arduino(self, pwm_left: int, pwm_right: int):
        line = f"L:{pwm_left},R:{pwm_right}\n"
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
        # Always log so you can see commands even without hardware connected
        self.get_logger().debug(f"-> Arduino: {line.strip()}")

    def integrate_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # simple unicycle dead-reckoning integration
        self.x += self.last_v * math.cos(self.theta) * dt
        self.y += self.last_v * math.sin(self.theta) * dt
        self.theta += self.last_w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom_estimate'
        odom.child_frame_id = 'base_link_estimate'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.angular.z = self.last_w

        # dead-reckoning has no reliable covariance; mark it clearly uncertain
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1

        self.odom_pub.publish(odom)

        # broadcast TF too, so it can be visualized in RViz alongside sim
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
            node.send_to_arduino(0, 0)  # stop motors on shutdown
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

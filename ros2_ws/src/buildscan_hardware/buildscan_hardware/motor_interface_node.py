#!/usr/bin/env python3
"""
motor_interface_node.py  ── Stage 2
═════════════════════════════════════
Translates ROS2 /cmd_vel commands into motor control.

In the new architecture, this node acts as a RELAY:
  /cmd_vel → motor_interface_node → micro-ROS (ESP32 DevKit)

The ESP32 DevKit receives /cmd_vel directly via micro-ROS and drives L298N.
This node provides:
  1. Velocity clamping (safety limits)
  2. Emergency stop enforcement (/emergency_stop)
  3. Communication watchdog
  4. Mode management (/set_mode service)
  5. Logging

Subscribes :
  /cmd_vel          geometry_msgs/Twist
  /emergency_stop   std_msgs/Bool

Publishes  :
  /cmd_vel          geometry_msgs/Twist  (velocity-clamped, safe version)

Services   :
  /set_mode         buildscan_interfaces/srv/SetMode

Parameters :
  max_linear_velocity   : float
  max_angular_velocity  : float
  watchdog_timeout      : float (seconds)

Usage:
  ros2 run buildscan_hardware motor_interface_node
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from buildscan_interfaces.srv import SetMode
import time


class MotorInterfaceNode(Node):
    """
    Safety-aware velocity relay between ROS2 topics and ESP32 DevKit (micro-ROS).

    The ESP32 DevKit subscribes to /cmd_vel directly via micro-ROS agent.
    This node enforces velocity limits and emergency stop before commands
    reach the embedded controller.
    """

    VALID_MODES = {'MANUAL', 'AUTO (NOT IMPLEMENTED)', 'AUTO', 'INSPECTION', 'PARK'}

    def __init__(self):
        super().__init__('motor_interface_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('max_linear_velocity',  0.30)
        self.declare_parameter('max_angular_velocity', 1.00)
        self.declare_parameter('watchdog_timeout',     2.0)

        self.max_linear  = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.watchdog_to = self.get_parameter('watchdog_timeout').value

        # ── State ─────────────────────────────────────────────────────────────
        self.current_mode    = 'MANUAL'
        self.emergency_stop  = False
        self.last_cmd_time   = self.get_clock().now()

        # ── Publishers ────────────────────────────────────────────────────────
        # Safe /cmd_vel published back — ESP32 micro-ROS subscribes to this
        self.safe_cmd_pub = self.create_publisher(Twist,  '/cmd_vel_safe', 10)
        self.status_pub   = self.create_publisher(String, '/rover/status',  10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel',
                                 self._cmd_vel_callback, 10)
        self.create_subscription(Bool,  '/emergency_stop',
                                 self._emergency_stop_callback, 10)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(SetMode, '/set_mode', self._set_mode_callback)

        # ── Watchdog timer (10 Hz) ────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f'MotorInterfaceNode ready | '
            f'max_linear={self.max_linear} m/s | '
            f'max_angular={self.max_angular} rad/s | '
            f'watchdog={self.watchdog_to}s'
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        """Receive /cmd_vel, clamp velocities, apply safety, forward to ESP32."""
        self.last_cmd_time = self.get_clock().now()

        if self.emergency_stop or self.current_mode == 'PARK':
            self._publish_stop()
            return

        # Clamp velocities to safe limits
        safe = Twist()
        safe.linear.x  = self._clamp(msg.linear.x,  -self.max_linear,  self.max_linear)
        safe.angular.z = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)

        self.safe_cmd_pub.publish(safe)

    def _emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop signal from safety_node."""
        if msg.data and not self.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP activated!')
        elif not msg.data and self.emergency_stop:
            self.get_logger().info('Emergency stop cleared.')

        self.emergency_stop = msg.data

        if self.emergency_stop:
            self._publish_stop()

    def _set_mode_callback(self, request, response):
        """Handle /set_mode service calls."""
        mode = request.mode.upper()
        if mode not in self.VALID_MODES:
            response.success      = False
            response.message      = f'Invalid mode: {mode}. Valid: {self.VALID_MODES}'
            response.current_mode = self.current_mode
            return response

        prev = self.current_mode
        self.current_mode = mode

        if mode == 'PARK':
            self._publish_stop()

        self.get_logger().info(f'Mode changed: {prev} → {mode}')
        response.success      = True
        response.message      = f'Mode set to {mode}'
        response.current_mode = self.current_mode
        return response

    def _watchdog_callback(self):
        """If no /cmd_vel received within watchdog_timeout, publish stop."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_to:
            self._publish_stop()

        # Publish status
        status = String()
        status.data = (
            f'mode={self.current_mode} | '
            f'e_stop={self.emergency_stop} | '
            f'idle={elapsed:.1f}s'
        )
        self.status_pub.publish(status)

    def _publish_stop(self):
        """Publish zero velocity — stops the rover."""
        self.safe_cmd_pub.publish(Twist())

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = MotorInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

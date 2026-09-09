#!/usr/bin/env python3
"""
safety_node.py  ── Stage 3
════════════════════════════
Monitors HC-SR04 ultrasonic distance and triggers emergency stop.
Also implements communication watchdog.

Subscribes :
  /ultrasonic_range   sensor_msgs/Range  (from ESP32 micro-ROS)

Publishes  :
  /emergency_stop     std_msgs/Bool
  /inspection/status  std_msgs/String    (safety status updates)

Parameters :
  safety_distance     : float  — stop if distance < this (meters)
  watchdog_timeout    : float  — stop if no ultrasonic data (seconds)

Safety Logic (from original project methodology, now as a proper ROS2 node):
  - HC-SR04 publishes distance as sensor_msgs/Range via micro-ROS
  - If range < safety_distance → emergency_stop = True
  - If no message received within watchdog_timeout → emergency_stop = True
  - Emergency stop is cleared when distance returns to safe range

Usage:
  ros2 run buildscan_hardware safety_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


class SafetyNode(Node):
    """
    Safety watchdog for BuildScan Rover.
    Monitors ultrasonic range and triggers emergency stop when needed.
    """

    def __init__(self):
        super().__init__('safety_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('safety_distance',  0.30)   # meters
        self.declare_parameter('watchdog_timeout', 2.0)    # seconds

        self.safety_dist = self.get_parameter('safety_distance').value
        self.watchdog_to = self.get_parameter('watchdog_timeout').value

        # ── State ─────────────────────────────────────────────────────────────
        self.last_range_time   = None
        self.current_distance  = 999.0
        self.emergency_active  = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.estop_pub  = self.create_publisher(Bool,   '/emergency_stop',     10)
        self.status_pub = self.create_publisher(String, '/inspection/status',  10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Range, '/ultrasonic_range',
                                 self._range_callback, 10)

        # ── Watchdog timer (5 Hz) ─────────────────────────────────────────────
        self.create_timer(0.2, self._watchdog_callback)

        self.get_logger().info(
            f'SafetyNode ready | '
            f'safety_distance={self.safety_dist}m | '
            f'watchdog_timeout={self.watchdog_to}s'
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _range_callback(self, msg: Range):
        """Process incoming ultrasonic range reading."""
        self.last_range_time  = self.get_clock().now()
        self.current_distance = msg.range

        obstacle_near = (msg.range < self.safety_dist and msg.range > 0.01)

        if obstacle_near and not self.emergency_active:
            self.get_logger().warn(
                f'OBSTACLE DETECTED: {msg.range:.3f}m < {self.safety_dist}m — '
                'activating emergency stop!'
            )
            self._set_emergency(True, f'Obstacle at {msg.range:.3f}m')

        elif not obstacle_near and self.emergency_active:
            # Only clear if no other reason for stop
            self.get_logger().info(
                f'Obstacle cleared: {msg.range:.3f}m — releasing emergency stop'
            )
            self._set_emergency(False, 'Path clear')

    def _watchdog_callback(self):
        """Periodic watchdog — stop if ultrasonic data is lost."""
        if self.last_range_time is None:
            # No data received yet — don't stop immediately, log warning
            self.get_logger().warn(
                'No ultrasonic data received yet — waiting for ESP32 micro-ROS...',
                throttle_duration_sec=5.0
            )
            return

        elapsed = (self.get_clock().now() - self.last_range_time).nanoseconds / 1e9

        if elapsed > self.watchdog_to and not self.emergency_active:
            self.get_logger().error(
                f'Ultrasonic watchdog timeout ({elapsed:.1f}s) — '
                'communication lost, activating emergency stop!'
            )
            self._set_emergency(True, f'Communication lost ({elapsed:.1f}s)')

        elif elapsed <= self.watchdog_to and self.emergency_active:
            # Communication restored — check if distance is safe
            if self.current_distance >= self.safety_dist:
                self.get_logger().info('Communication restored — releasing emergency stop')
                self._set_emergency(False, 'Communication restored')

    def _set_emergency(self, active: bool, reason: str):
        """Publish emergency stop state and log status."""
        self.emergency_active = active

        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

        status = String()
        status.data = f'SAFETY|{"STOP" if active else "CLEAR"}|{reason}'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

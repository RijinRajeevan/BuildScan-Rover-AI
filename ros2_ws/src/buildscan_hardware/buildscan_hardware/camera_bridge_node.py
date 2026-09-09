#!/usr/bin/env python3
"""
camera_bridge_node.py  ── Stage 4
══════════════════════════════════
Bridges the ESP32-CAM MJPEG HTTP stream into ROS2.

Subscribes : nothing
Publishes  : /camera/image_raw  (sensor_msgs/Image)

The ESP32-CAM streams MJPEG over HTTP on port 81 (existing firmware unchanged).
This node reads each JPEG frame via OpenCV and republishes as a ROS2 Image.

Parameters (from robot_params.yaml):
  esp32_cam_stream_url    : URL of the MJPEG stream
  camera_fps_target       : target publishing rate (Hz)
  camera_frame_id         : TF frame id for camera

Usage:
  ros2 run buildscan_hardware camera_bridge_node
"""

import cv2
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraBridgeNode(Node):
    """
    Reads ESP32-CAM MJPEG stream and publishes sensor_msgs/Image on /camera/image_raw.
    Handles stream disconnection and reconnects automatically.
    """

    def __init__(self):
        super().__init__('camera_bridge_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('esp32_cam_stream_url', 'http://10.131.116.176:81/stream')
        self.declare_parameter('camera_fps_target', 10)
        self.declare_parameter('camera_frame_id', 'camera_link')

        self.stream_url  = self.get_parameter('esp32_cam_stream_url').value
        self.fps_target  = self.get_parameter('camera_fps_target').value
        self.frame_id    = self.get_parameter('camera_frame_id').value

        # ── Publisher ─────────────────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ── cv_bridge ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── State ─────────────────────────────────────────────────────────────
        self.cap     = None
        self.running = True

        # ── Timer-driven capture loop ─────────────────────────────────────────
        period = 1.0 / max(1, self.fps_target)
        self.timer = self.create_timer(period, self._capture_callback)

        self.get_logger().info(
            f'CameraBridgeNode started | stream: {self.stream_url} | '
            f'target fps: {self.fps_target}'
        )
        self._connect()

    # ──────────────────────────────────────────────────────────────────────────

    def _connect(self):
        """Open the MJPEG stream."""
        self.get_logger().info(f'Connecting to ESP32-CAM stream: {self.stream_url}')
        try:
            self.cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.cap.isOpened():
                self.get_logger().warn('Stream not immediately available — will retry')
                self.cap = None
        except Exception as e:
            self.get_logger().error(f'Stream connection failed: {e}')
            self.cap = None

    def _capture_callback(self):
        """Called by timer — capture one frame and publish."""
        if self.cap is None or not self.cap.isOpened():
            self._connect()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Frame read failed — attempting reconnect')
            self.cap.release()
            self.cap = None
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp    = self.get_clock().now().to_msg()
            ros_image.header.frame_id = self.frame_id
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Image publish failed: {e}')

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

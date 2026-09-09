#!/usr/bin/env python3
"""
crack_detection_node.py  ── Stage 5 & 6
════════════════════════════════════════
BuildScan Rover – YOLO Crack Detection ROS2 Node

Subscribes :
  /camera/image_raw         sensor_msgs/Image

Publishes  :
  /inspection/result        buildscan_interfaces/InspectionResult
  /inspection/status        std_msgs/String
  /camera/image_annotated   sensor_msgs/Image  (debug — annotated frames)

Services   :
  /set_inspection_config    buildscan_interfaces/SetInspectionConfig

This node preserves the complete crack analysis methodology from the original
app.py (analyze() function), refactored as a proper ROS2 node:
  ① cv_bridge: ROS Image → OpenCV frame
  ② YOLO26n-seg inference (existing best.pt weights)
  ③ Crack measurements (pixel-width/length)
  ④ Severity classification (LOW/MEDIUM/HIGH thresholds from original)
  ⑤ Repair recommendation
  ⑥ Publish InspectionResult

Parameters (from robot_params.yaml):
  model_path                 : path to best.pt
  yolo_confidence_threshold  : float (default 0.30)
  yolo_inference_image_size  : int   (default 320)
  ai_device                  : str   (cuda / cpu)
  severity_low_max_width     : int   (default 70 px)
  severity_medium_max_width  : int   (default 150 px)

Usage:
  ros2 run buildscan_perception crack_detection_node
"""

import os
import cv2
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from buildscan_interfaces.msg import InspectionResult
from buildscan_interfaces.srv import SetInspectionConfig

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class CrackDetectionNode(Node):
    """
    ROS2 node for real-time crack detection using YOLO26n-seg.

    Preserves the original BuildScan crack analysis methodology:
      - Bounding-box pixel width for severity classification
      - Total length accumulation across detections
      - Maximum width tracking
      - Severity: NONE / LOW / MEDIUM / HIGH
      - Recommendations from original project
    """

    def __init__(self):
        super().__init__('crack_detection_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_path', 'models/yolo26n-seg.pt')
        self.declare_parameter('yolo_confidence_threshold', 0.30)
        self.declare_parameter('yolo_inference_image_size', 320)
        self.declare_parameter('ai_device', 'cpu')
        self.declare_parameter('severity_low_max_width',    70)
        self.declare_parameter('severity_medium_max_width', 150)
        self.declare_parameter('yolo_model_name',    'YOLO26n-seg')
        self.declare_parameter('yolo_model_version', '1.0.0')
        self.declare_parameter('camera_frame_id', 'camera_link')

        self._load_parameters()

        # ── Load YOLO Model ────────────────────────────────────────────────────
        self.model = None
        self._load_model()

        # ── cv_bridge ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ────────────────────────────────────────────────────────
        self.result_pub = self.create_publisher(
            InspectionResult, '/inspection/result', 10)
        self.status_pub = self.create_publisher(
            String, '/inspection/status', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/camera/image_annotated', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(
            SetInspectionConfig, '/set_inspection_config',
            self._set_config_callback)

        # ── State ─────────────────────────────────────────────────────────────
        self.frame_count   = 0
        self.inference_hz  = 0.0
        self._last_inf_t   = time.time()

        self.get_logger().info(
            f'CrackDetectionNode ready | '
            f'model={self.model_path} | '
            f'conf={self.conf_threshold} | '
            f'imgsz={self.imgsz} | '
            f'device={self.ai_device}'
        )
        self._publish_status('READY|Crack detection node initialized')

    # ──────────────────────────────────────────────────────────────────────────
    # PARAMETER LOADING
    # ──────────────────────────────────────────────────────────────────────────

    def _load_parameters(self):
        self.model_path     = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('yolo_confidence_threshold').value
        self.imgsz          = self.get_parameter('yolo_inference_image_size').value
        self.ai_device      = self.get_parameter('ai_device').value
        self.sev_low_max    = self.get_parameter('severity_low_max_width').value
        self.sev_med_max    = self.get_parameter('severity_medium_max_width').value
        self.model_name     = self.get_parameter('yolo_model_name').value
        self.model_version  = self.get_parameter('yolo_model_version').value

    def _load_model(self):
        """Load YOLO model — resolves path relative to workspace if needed."""
        if not YOLO_AVAILABLE:
            self.get_logger().error(
                'ultralytics not installed! Install: pip install ultralytics')
            return

        # Try absolute path first, then relative to package
        paths_to_try = [
            self.model_path,
            os.path.join(os.getcwd(), self.model_path),
        ]

        for p in paths_to_try:
            if os.path.exists(p):
                try:
                    self.model = YOLO(p)
                    self.get_logger().info(f'YOLO model loaded: {p}')
                    return
                except Exception as e:
                    self.get_logger().error(f'Failed to load model from {p}: {e}')

        self.get_logger().error(
            f'Model not found at: {self.model_path}\n'
            'Set model_path parameter to the correct path of best.pt'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # IMAGE CALLBACK — main processing pipeline
    # ──────────────────────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """
        Called for each frame on /camera/image_raw.
        Runs YOLO inference and publishes InspectionResult.
        """
        self.frame_count += 1

        # ── Convert ROS Image → OpenCV ────────────────────────────────────────
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        if self.model is None:
            self.get_logger().warn(
                'YOLO model not loaded — skipping inference',
                throttle_duration_sec=5.0)
            return

        # ── Inference ─────────────────────────────────────────────────────────
        t0 = time.time()
        try:
            annotated, count, length, width, conf, severity, repair = \
                self._analyze(frame)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        inf_time = time.time() - t0
        self.inference_hz = 1.0 / max(inf_time, 0.001)

        # ── Publish Annotated Image (debug) ───────────────────────────────────
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().warn(f'Annotated image publish failed: {e}')

        # ── Publish InspectionResult ──────────────────────────────────────────
        result = InspectionResult()
        result.header.stamp        = self.get_clock().now().to_msg()
        result.header.frame_id     = self.get_parameter('camera_frame_id').value
        result.cracks_detected     = count
        result.average_confidence  = float(conf) / 100.0  # back to 0-1 range
        result.maximum_width       = float(width)
        result.total_length        = float(length)
        result.severity            = severity
        result.recommendation      = repair
        result.model_name          = self.model_name
        result.model_version       = self.model_version

        self.result_pub.publish(result)

        # ── Status ────────────────────────────────────────────────────────────
        self._publish_status(
            f'DETECTING|cracks={count}|conf={conf:.1f}%|'
            f'sev={severity}|fps={self.inference_hz:.1f}'
        )

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: cracks={count} | '
                f'conf={conf:.1f}% | severity={severity} | '
                f'inference={inf_time*1000:.0f}ms'
            )

    # ──────────────────────────────────────────────────────────────────────────
    # ANALYZE — preserves original app.py analyze() methodology exactly
    # ──────────────────────────────────────────────────────────────────────────

    def _analyze(self, frame: np.ndarray):
        """
        Perform YOLO inference and calculate crack measurements.

        Preserves original BuildScan methodology from app.py:
          - YOLO predict with configured threshold and image size
          - Extract bounding boxes for pixel measurement
          - Accumulate total_length = sum of max(w,h) per detection
          - Track maximum_width = max of min(w,h) across detections
          - Severity based on maximum_width thresholds
          - Repair recommendation text

        Returns:
          (annotated_frame, count, total_length, max_width,
           confidence_pct, severity_str, recommendation_str)
        """
        results = self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            verbose=False,
            device=self.ai_device,
        )

        r = results[0]
        annotated = r.plot()

        count  = 0
        length = 0
        width  = 0
        conf   = 0.0

        if r.boxes is not None and len(r.boxes) > 0:
            count = len(r.boxes)
            conf  = float(r.boxes.conf.max()) * 100.0  # → percentage

            for box in r.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box[:4]
                w = int(x2 - x1)
                h = int(y2 - y1)

                # Original methodology: length = sum of longest dimension
                length += max(w, h)

                # Original methodology: width = maximum shortest dimension
                if min(w, h) > width:
                    width = min(w, h)

        # ── Severity Classification (original thresholds) ──────────────────
        severity = 'NONE'
        repair   = 'No crack detected'

        if count > 0:
            if width < self.sev_low_max:
                severity = 'LOW'
                repair   = 'Apply surface filler and sealant'
            elif width < self.sev_med_max:
                severity = 'MEDIUM'
                repair   = 'Use epoxy injection repair'
            else:
                severity = 'HIGH'
                repair   = 'Urgent structural inspection required'

        return annotated, count, length, width, conf, severity, repair

    # ──────────────────────────────────────────────────────────────────────────
    # SERVICE HANDLERS
    # ──────────────────────────────────────────────────────────────────────────

    def _set_config_callback(self, request, response):
        """Handle /set_inspection_config service calls."""
        if 0.0 < request.confidence_threshold <= 1.0:
            self.conf_threshold = request.confidence_threshold
            self.get_logger().info(
                f'Confidence threshold updated: {self.conf_threshold}')

        if request.inference_image_size in (160, 320, 416, 512, 640, 1280):
            self.imgsz = request.inference_image_size
            self.get_logger().info(
                f'Inference image size updated: {self.imgsz}')

        response.success = True
        response.message = (
            f'Config updated: conf={self.conf_threshold}, '
            f'imgsz={self.imgsz}'
        )
        return response

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CrackDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

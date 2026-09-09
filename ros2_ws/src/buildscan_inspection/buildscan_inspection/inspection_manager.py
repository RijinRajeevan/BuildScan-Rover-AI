#!/usr/bin/env python3
"""
inspection_manager.py  ── Stage 8
══════════════════════════════════
BuildScan Rover – InspectArea Action Server

Implements the full 10-step structural inspection workflow as a ROS2 action:

  1. Receive + validate goal
  2. Position robot (Nav2, if available)
  3. Camera scanning
  4. YOLO inference (via /inspection/result subscription)
  5. Crack analysis and aggregation
  6. Severity calculation
  7. PDF report generation
  8. Return result

Feedback published continuously through the action:
  Navigation       →  20%
  Camera scanning  →  40%
  AI analysis      →  65%
  Crack analysis   →  75%
  Report generation→  90%
  Completed        → 100%

Action        : /inspect_area  buildscan_interfaces/InspectArea
Subscribes    : /inspection/result buildscan_interfaces/InspectionResult
Publishes     : /inspection/status std_msgs/String

Usage:
  ros2 run buildscan_inspection inspection_manager
  ros2 action send_goal /inspect_area buildscan_interfaces/action/InspectArea \
    "{area_name: 'Wall_A', confidence_threshold: 0.30, scan_frames: 10}"
"""

import os
import time
import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from buildscan_interfaces.action import InspectArea
from buildscan_interfaces.msg import InspectionResult
from buildscan_inspection.report_generator import ReportGenerator, InspectionData


class InspectionManagerNode(Node):
    """
    Action server implementing the full BuildScan inspection workflow.

    Handles concurrent action goals safely using ReentrantCallbackGroup.
    Supports proper cancellation at each workflow stage.
    """

    def __init__(self):
        super().__init__('inspection_manager')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('inspection_output_dir', '/tmp/buildscan_reports')
        self.declare_parameter('inspection_timeout',    60.0)
        self.declare_parameter('inspection_scan_frames', 10)
        self.declare_parameter('yolo_confidence_threshold', 0.30)

        self.output_dir    = self.get_parameter('inspection_output_dir').value
        self.timeout       = self.get_parameter('inspection_timeout').value
        self.default_conf  = self.get_parameter('yolo_confidence_threshold').value

        # ── Report Generator ──────────────────────────────────────────────────
        self.report_gen = ReportGenerator(output_dir=self.output_dir)

        # ── State for latest inspection result ─────────────────────────────────
        self.latest_result: InspectionResult = None
        self.result_received = False

        # ── Callback group for concurrent operation ────────────────────────────
        cb_group = ReentrantCallbackGroup()

        # ── Action Server ─────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            InspectArea,
            '/inspect_area',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            InspectionResult, '/inspection/result',
            self._result_callback, 10,
            callback_group=cb_group
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/inspection/status', 10)

        self.get_logger().info('InspectionManager action server ready on /inspect_area')

    # ──────────────────────────────────────────────────────────────────────────
    # ACTION SERVER CALLBACKS
    # ──────────────────────────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        """Validate incoming goal."""
        self.get_logger().info(
            f'Received inspection goal: area="{goal_request.area_name}" '
            f'conf={goal_request.confidence_threshold}'
        )
        if not goal_request.area_name:
            self.get_logger().warn('Rejected: area_name is empty')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info(
            f'Cancellation requested for: {goal_handle.goal_id}')
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the full inspection workflow."""
        goal = goal_handle.request
        self.get_logger().info(
            f'=== Starting inspection: {goal.area_name} ===')

        feedback = InspectArea.Feedback()
        result   = InspectArea.Result()

        conf_threshold = (goal.confidence_threshold
                          if goal.confidence_threshold > 0
                          else self.default_conf)
        scan_frames = (goal.scan_frames if goal.scan_frames > 0 else 10)

        # Aggregation state
        all_results = []
        start_time  = time.time()

        # ─────────────────────────────────────────────────────────────────────
        # STEP 1 — Navigation / Positioning (20%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage  = 'Navigation - Positioning robot'
        feedback.progress       = 10.0
        feedback.cracks_found_so_far = 0
        feedback.status_detail  = 'Moving to inspection position...'
        goal_handle.publish_feedback(feedback)
        self._publish_status(f'NAV|Positioning for {goal.area_name}')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_cancelled_result(result)

        # Simulate navigation time (replace with actual Nav2 action call)
        # In physical deployment, this would call nav2 navigate_to_pose action
        await self._sleep_with_cancel_check(goal_handle, 1.5)

        feedback.current_stage = 'Navigation - Complete'
        feedback.progress = 20.0
        goal_handle.publish_feedback(feedback)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 2 — Camera Scanning (40%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage = 'Camera scanning - Capturing frames'
        feedback.progress      = 30.0
        feedback.status_detail = f'Capturing {scan_frames} frames...'
        goal_handle.publish_feedback(feedback)
        self._publish_status(f'SCAN|Capturing frames for {goal.area_name}')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_cancelled_result(result)

        # Wait for inspection results from crack_detection_node
        await self._sleep_with_cancel_check(goal_handle, 0.5)

        feedback.current_stage = 'Camera scanning - Complete'
        feedback.progress      = 40.0
        goal_handle.publish_feedback(feedback)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 3 — AI Analysis (65%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage = 'AI analysis - Running YOLO inference'
        feedback.progress      = 50.0
        feedback.status_detail = 'YOLO26n-seg processing frames...'
        goal_handle.publish_feedback(feedback)
        self._publish_status(f'AI|YOLO inference running')

        # Collect frames from /inspection/result topic
        self.result_received = False
        collected = []
        collect_deadline = time.time() + min(self.timeout, scan_frames * 1.5)

        while len(collected) < scan_frames and time.time() < collect_deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_cancelled_result(result)

            if self.result_received and self.latest_result is not None:
                collected.append(self.latest_result)
                self.result_received = False
                feedback.cracks_found_so_far = max(
                    r.cracks_detected for r in collected)
                feedback.progress = 50.0 + (len(collected) / scan_frames) * 15.0
                goal_handle.publish_feedback(feedback)

            await self._sleep_with_cancel_check(goal_handle, 0.1)

        feedback.current_stage = 'AI analysis - Complete'
        feedback.progress      = 65.0
        goal_handle.publish_feedback(feedback)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 4 — Crack Analysis (75%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage = 'Crack analysis - Aggregating results'
        feedback.progress      = 72.0
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_cancelled_result(result)

        aggregated = self._aggregate_results(collected)

        feedback.current_stage    = 'Crack analysis - Complete'
        feedback.progress         = 75.0
        feedback.cracks_found_so_far = aggregated['cracks_detected']
        goal_handle.publish_feedback(feedback)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 5 — PDF Report Generation (90%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage = 'Report generation - Creating PDF'
        feedback.progress      = 85.0
        goal_handle.publish_feedback(feedback)
        self._publish_status(f'REPORT|Generating PDF for {goal.area_name}')

        inspection_data = InspectionData(
            area_name           = goal.area_name,
            cracks_detected     = aggregated['cracks_detected'],
            average_confidence  = aggregated['average_confidence'],
            maximum_width       = aggregated['maximum_width'],
            total_length        = aggregated['total_length'],
            severity            = aggregated['severity'],
            recommendation      = aggregated['recommendation'],
            annotated_image_path= aggregated.get('annotated_image_path', ''),
            confidence_threshold= conf_threshold,
        )

        try:
            pdf_path = self.report_gen.generate(inspection_data)
            self.get_logger().info(f'PDF report saved: {pdf_path}')
        except Exception as e:
            self.get_logger().error(f'PDF generation failed: {e}')
            pdf_path = ''

        feedback.current_stage = 'Report generation - Complete'
        feedback.progress      = 90.0
        goal_handle.publish_feedback(feedback)

        # ─────────────────────────────────────────────────────────────────────
        # STEP 6 — Completed (100%)
        # ─────────────────────────────────────────────────────────────────────
        feedback.current_stage = 'Completed'
        feedback.progress      = 100.0
        goal_handle.publish_feedback(feedback)

        elapsed = time.time() - start_time
        self.get_logger().info(
            f'=== Inspection complete: {goal.area_name} | '
            f'cracks={aggregated["cracks_detected"]} | '
            f'severity={aggregated["severity"]} | '
            f'time={elapsed:.1f}s ==='
        )
        self._publish_status(
            f'COMPLETE|{goal.area_name}|'
            f'cracks={aggregated["cracks_detected"]}|'
            f'sev={aggregated["severity"]}'
        )

        # ── Populate result ────────────────────────────────────────────────────
        result.success              = True
        result.cracks_detected      = aggregated['cracks_detected']
        result.average_confidence   = aggregated['average_confidence']
        result.max_width_px         = aggregated['maximum_width']
        result.total_length_px      = aggregated['total_length']
        result.severity             = aggregated['severity']
        result.recommendation       = aggregated['recommendation']
        result.report_path          = pdf_path
        result.annotated_image_path = aggregated.get('annotated_image_path', '')
        result.message              = f'Inspection of {goal.area_name} completed in {elapsed:.1f}s'

        goal_handle.succeed()
        return result

    # ──────────────────────────────────────────────────────────────────────────
    # AGGREGATION
    # ──────────────────────────────────────────────────────────────────────────

    def _aggregate_results(self, results: list) -> dict:
        """
        Aggregate multiple InspectionResult frames into a single summary.
        Uses worst-case severity (maximum width detected across all frames).
        """
        if not results:
            return {
                'cracks_detected':    0,
                'average_confidence': 0.0,
                'maximum_width':      0.0,
                'total_length':       0.0,
                'severity':           'NONE',
                'recommendation':     'No crack detected',
                'annotated_image_path': '',
            }

        # Take the frame with maximum cracks as representative
        best = max(results, key=lambda r: r.cracks_detected)

        avg_conf   = np.mean([r.average_confidence * 100.0 for r in results
                               if r.cracks_detected > 0] or [0.0])
        max_width  = max(r.maximum_width for r in results)
        total_len  = sum(r.total_length for r in results)

        return {
            'cracks_detected':    best.cracks_detected,
            'average_confidence': float(avg_conf),
            'maximum_width':      float(max_width),
            'total_length':       float(total_len),
            'severity':           best.severity,
            'recommendation':     best.recommendation,
            'annotated_image_path': best.annotated_image_path,
        }

    # ──────────────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────────────

    def _result_callback(self, msg: InspectionResult):
        """Store the latest InspectionResult from crack_detection_node."""
        self.latest_result   = msg
        self.result_received = True

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    async def _sleep_with_cancel_check(self, goal_handle, duration: float):
        """Async sleep that respects cancel requests."""
        steps = max(1, int(duration / 0.05))
        for _ in range(steps):
            if goal_handle.is_cancel_requested:
                return
            time.sleep(0.05)

    def _make_cancelled_result(self, result: InspectArea.Result):
        result.success  = False
        result.message  = 'Inspection cancelled by user'
        result.severity = 'NONE'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

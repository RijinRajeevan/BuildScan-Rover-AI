#!/usr/bin/env python3
"""
dashboard_node.py  ── Stage 9
══════════════════════════════
BuildScan Rover – Streamlit Dashboard (ROS2 UI Layer ONLY)

IMPORTANT DESIGN RULE:
  This dashboard does NOT run AI inference.
  It is a pure visualization/control UI that communicates via ROS2.

  OLD (wrong):  Streamlit → camera → YOLO → PDF
  NEW (correct): Streamlit ↔ ROS2 topics/actions/services

What this dashboard does:
  ✅ Displays /camera/image_raw (live camera feed via ROS2)
  ✅ Displays /camera/image_annotated (YOLO-annotated frames)
  ✅ Shows /inspection/result metrics in real time
  ✅ Shows /inspection/status
  ✅ Sends /cmd_vel for manual teleop
  ✅ Sends InspectArea action goals
  ✅ Calls /set_mode service
  ✅ Calls /set_inspection_config service
  ✅ Displays generated PDF path

What this dashboard does NOT do:
  ❌ Direct camera stream reading
  ❌ YOLO inference
  ❌ PDF generation
  ❌ OpenCV processing

Usage (on Ubuntu with ROS2 sourced):
  python3 -m buildscan_dashboard.dashboard_node
  or
  streamlit run dashboard_node.py
"""

import threading
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from buildscan_interfaces.msg import InspectionResult
from buildscan_interfaces.action import InspectArea
from buildscan_interfaces.srv import SetMode, SetInspectionConfig

try:
    import streamlit as st
    STREAMLIT_AVAILABLE = True
except ImportError:
    STREAMLIT_AVAILABLE = False


class DashboardBridgeNode(Node):
    """
    ROS2 node that bridges Streamlit UI ↔ ROS2 topics/actions/services.
    Runs in a background thread while Streamlit renders the UI.
    """

    def __init__(self):
        super().__init__('buildscan_dashboard')

        cb = ReentrantCallbackGroup()

        # ── Shared state (thread-safe via Python GIL for simple types) ────────
        self.latest_frame        = None   # numpy BGR image from /camera/image_raw
        self.latest_annotated    = None   # numpy BGR image from /camera/image_annotated
        self.latest_result       = None   # InspectionResult message
        self.status_text         = ''
        self.emergency_stop      = False
        self.commanded_tilt_angle = 90.0  # degrees — last commanded servo angle (no feedback)

        self.bridge = CvBridge()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw',
            self._image_callback, 10, callback_group=cb)
        self.create_subscription(Image, '/camera/image_annotated',
            self._annotated_callback, 10, callback_group=cb)
        self.create_subscription(InspectionResult, '/inspection/result',
            self._result_callback, 10, callback_group=cb)
        self.create_subscription(String, '/inspection/status',
            self._status_callback, 10, callback_group=cb)
        self.create_subscription(Bool, '/emergency_stop',
            self._estop_callback, 10, callback_group=cb)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_vel_pub   = self.create_publisher(Twist,   '/cmd_vel',          10)
        # /camera_tilt_cmd: Float32 value = desired servo angle in degrees.
        # The ESP32 firmware clamps incoming values to [SERVO_MIN, SERVO_MAX].
        # There is NO angle feedback sensor — commanded angle is displayed only.
        self.tilt_pub      = self.create_publisher(Float32, '/camera_tilt_cmd',  10)

        # ── Action Client ─────────────────────────────────────────────────────
        self.inspect_client = ActionClient(
            self, InspectArea, '/inspect_area', callback_group=cb)

        # ── Service Clients ───────────────────────────────────────────────────
        self.mode_client   = self.create_client(SetMode, '/set_mode', callback_group=cb)
        self.config_client = self.create_client(
            SetInspectionConfig, '/set_inspection_config', callback_group=cb)

        self.get_logger().info('Dashboard bridge node ready')

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _annotated_callback(self, msg):
        try:
            self.latest_annotated = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _result_callback(self, msg):
        self.latest_result = msg

    def _status_callback(self, msg):
        self.status_text = msg.data

    def _estop_callback(self, msg):
        self.emergency_stop = msg.data

    # ── Control API (called from Streamlit) ───────────────────────────────────

    def send_velocity(self, linear: float, angular: float):
        """Publish velocity command to /cmd_vel."""
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def send_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def send_tilt(self, angle_deg: float):
        """Publish camera tilt angle command to /camera_tilt_cmd.

        Args:
            angle_deg: Desired servo angle in degrees.
                       Safe range is determined by firmware constants
                       SERVO_MIN_ANGLE and SERVO_MAX_ANGLE (default 45–135°).
                       Out-of-range values are clamped by the ESP32 firmware.
        Note: This is open-loop — no servo angle feedback exists.
              Displayed angle is commanded angle, not measured angle.
        """
        self.commanded_tilt_angle = float(angle_deg)
        msg = Float32()
        msg.data = self.commanded_tilt_angle
        self.tilt_pub.publish(msg)

    def send_inspect_goal(self, area_name: str, confidence: float = 0.30,
                           frames: int = 10) -> None:
        """Send an InspectArea action goal (non-blocking)."""
        if not self.inspect_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('/inspect_area action server not available')
            return

        goal = InspectArea.Goal()
        goal.area_name            = area_name
        goal.confidence_threshold = confidence
        goal.scan_frames          = frames

        self.inspect_client.send_goal_async(goal)
        self.get_logger().info(f'Inspection goal sent: {area_name}')

    def set_mode(self, mode: str):
        """Call /set_mode service."""
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            return
        req = SetMode.Request()
        req.mode = mode
        self.mode_client.call_async(req)

    def set_config(self, confidence: float, safety_distance: float):
        """Call /set_inspection_config service."""
        if not self.config_client.wait_for_service(timeout_sec=2.0):
            return
        req = SetInspectionConfig.Request()
        req.confidence_threshold = confidence
        req.safety_distance      = safety_distance
        self.config_client.call_async(req)


# ═══════════════════════════════════════════════════════════════════════════════
# STREAMLIT UI
# ═══════════════════════════════════════════════════════════════════════════════

def run_ros_node(node):
    """Run ROS2 spin in a background thread."""
    rclpy.spin(node)


def main():
    if not STREAMLIT_AVAILABLE:
        print('Streamlit not installed. Run: pip install streamlit')
        return

    # ── Initialize ROS2 + node in background thread ────────────────────────
    if 'ros_node' not in st.session_state:
        rclpy.init()
        node = DashboardBridgeNode()
        st.session_state.ros_node = node

        t = threading.Thread(target=run_ros_node, args=(node,), daemon=True)
        t.start()
        st.session_state.ros_thread = t

    node: DashboardBridgeNode = st.session_state.ros_node

    # ── Page Config ────────────────────────────────────────────────────────────
    st.set_page_config(
        page_title='BuildScan Rover ROS2',
        layout='wide',
        page_icon='🏗️'
    )

    # ── CSS ───────────────────────────────────────────────────────────────────
    st.markdown("""
    <style>
    .metric-box{padding:15px;border-radius:15px;color:white;
      font-weight:bold;margin-bottom:10px;text-align:center;font-size:18px;}
    .blue{background:#2563eb;} .green{background:#16a34a;}
    .orange{background:#ea580c;} .red{background:#dc2626;}
    .purple{background:#7c3aed;}
    .estop{background:#dc2626;color:white;padding:10px;border-radius:10px;
      text-align:center;font-size:20px;font-weight:bold;}
    </style>
    """, unsafe_allow_html=True)

    # ── Header ────────────────────────────────────────────────────────────────
    st.title('🏗️ BuildScan Rover — ROS2 Dashboard')

    if node.emergency_stop:
        st.markdown('<div class="estop">🚨 EMERGENCY STOP ACTIVE</div>',
                    unsafe_allow_html=True)

    # Status bar
    if node.status_text:
        st.info(f'📡 ROS2 Status: {node.status_text}')

    tab1, tab2, tab3, tab4 = st.tabs([
        '📷 Live Camera', '🔍 Inspection', '🎮 Teleop', '⚙️ Settings'])

    # ── TAB 1: Live Camera ────────────────────────────────────────────────────
    with tab1:
        c1, c2 = st.columns(2)

        with c1:
            st.subheader('📷 Camera Feed (/camera/image_raw)')
            frame_box = st.empty()
            if node.latest_frame is not None:
                frame_box.image(node.latest_frame, channels='BGR', use_column_width=True)
            else:
                frame_box.warning('No camera feed — waiting for /camera/image_raw...')

        with c2:
            st.subheader('🔬 Annotated (/camera/image_annotated)')
            ann_box = st.empty()
            if node.latest_annotated is not None:
                ann_box.image(node.latest_annotated, channels='BGR', use_column_width=True)
            else:
                ann_box.info('No annotations yet — crack detection running...')

    # ── TAB 2: Inspection Results ─────────────────────────────────────────────
    with tab2:
        st.subheader('🔍 Inspection Mission')

        col1, col2 = st.columns([2, 1])
        with col1:
            area_name = st.text_input('Area Name', value='Wall_A',
                                       key='area_name_input')
            conf      = st.slider('Confidence Threshold', 0.1, 0.9, 0.3, 0.05,
                                   key='conf_slider')
            frames    = st.slider('Scan Frames', 5, 30, 10, 1,
                                   key='frames_slider')

        with col2:
            st.write('')
            st.write('')
            if st.button('🚀 Start Inspection', type='primary'):
                node.send_inspect_goal(area_name, conf, frames)
                st.success(f'Inspection goal sent: {area_name}')

        # Results
        r = node.latest_result
        if r is not None:
            st.divider()
            st.subheader('📊 Latest Inspection Result')

            a, b, c, d = st.columns(4)
            with a:
                st.markdown(
                    f'<div class="metric-box blue">🔍 Cracks<br>{r.cracks_detected}</div>',
                    unsafe_allow_html=True)
            with b:
                st.markdown(
                    f'<div class="metric-box green">🎯 Confidence<br>'
                    f'{r.average_confidence*100:.1f}%</div>',
                    unsafe_allow_html=True)
            with c:
                sev_color = {'NONE':'blue','LOW':'green',
                             'MEDIUM':'orange','HIGH':'red'}.get(r.severity, 'purple')
                st.markdown(
                    f'<div class="metric-box {sev_color}">⚠ Severity<br>{r.severity}</div>',
                    unsafe_allow_html=True)
            with d:
                st.markdown(
                    f'<div class="metric-box purple">📏 Width<br>'
                    f'{r.maximum_width:.0f} px</div>',
                    unsafe_allow_html=True)

            st.info(f'🛠 Recommendation: {r.recommendation}')
            st.caption(f'Model: {r.model_name} v{r.model_version}')

    # ── TAB 3: Teleop ─────────────────────────────────────────────────────────
    with tab3:
        st.subheader('🎮 Manual Rover Control')
        st.caption('Commands publish to /cmd_vel via ROS2')

        lin_spd = st.slider('Linear Speed (m/s)', 0.05, 0.30, 0.15, 0.05)
        ang_spd = st.slider('Angular Speed (rad/s)', 0.1, 1.0, 0.5, 0.1)

        r1, r2, r3 = st.columns(3)
        with r2:
            if st.button('⬆ Forward', use_container_width=True):
                node.send_velocity(lin_spd, 0.0)
        r4, r5, r6 = st.columns(3)
        with r4:
            if st.button('⬅ Left', use_container_width=True):
                node.send_velocity(0.0, ang_spd)
        with r5:
            if st.button('⏹ STOP', use_container_width=True, type='primary'):
                node.send_stop()
        with r6:
            if st.button('➡ Right', use_container_width=True):
                node.send_velocity(0.0, -ang_spd)
        r7, r8, r9 = st.columns(3)
        with r8:
            if st.button('⬇ Backward', use_container_width=True):
                node.send_velocity(-lin_spd, 0.0)

        st.divider()
        mode_col1, mode_col2 = st.columns(2)
        with mode_col1:
            selected_mode = st.selectbox('Mode', ['MANUAL', 'AUTO (NOT IMPLEMENTED)', 'INSPECTION', 'PARK'])
        with mode_col2:
            st.write('')
            st.write('')
            if st.button('Set Mode'):
                node.set_mode(selected_mode)

        # ── Camera Tilt Control ───────────────────────────────────────────────
        st.divider()
        st.subheader('📷 Camera Tilt Control')
        st.caption(
            'Publishes to /camera_tilt_cmd (std_msgs/Float32 — degrees).  '
            'Servo clamped to [45°–135°] by ESP32 firmware.  '
            '⚠ Commanded angle only — no feedback sensor on hardware.'
        )

        tilt_col1, tilt_col2 = st.columns([3, 2])

        with tilt_col1:
            tilt_angle = st.slider(
                'Tilt Angle (°)',
                min_value=45,
                max_value=135,
                value=int(node.commanded_tilt_angle),
                step=5,
                key='tilt_slider',
                help='Safe range: 45° (down) to 135° (up). 90° = neutral/center.'
            )

        with tilt_col2:
            st.write('')
            st.write('')
            tc1, tc2, tc3 = st.columns(3)
            with tc1:
                if st.button('⬆ Up', key='tilt_up', use_container_width=True):
                    node.send_tilt(max(45, node.commanded_tilt_angle - 10))
                    st.rerun()
            with tc2:
                if st.button('●', key='tilt_center', use_container_width=True,
                             help='Center (90°)'):
                    node.send_tilt(90.0)
                    st.rerun()
            with tc3:
                if st.button('⬇ Dn', key='tilt_down', use_container_width=True):
                    node.send_tilt(min(135, node.commanded_tilt_angle + 10))
                    st.rerun()

        if st.button('Send Tilt', key='btn_send_tilt', type='primary'):
            node.send_tilt(float(tilt_angle))
            st.success(f'Camera tilt command sent: {tilt_angle}°  '
                       f'(topic: /camera_tilt_cmd)')

        st.info(
            f'Last commanded angle: **{node.commanded_tilt_angle:.0f}°**  '
            f'(open-loop — ESP32 servo, no angle sensor)'
        )

    # ── TAB 4: Settings ───────────────────────────────────────────────────────
    with tab4:
        st.subheader('⚙️ Runtime Configuration')
        st.caption('Changes applied via ROS2 /set_inspection_config service')

        s_conf = st.slider('YOLO Confidence Threshold', 0.1, 0.9, 0.3, 0.05,
                            key='cfg_conf')
        s_dist = st.slider('Safety Distance (m)', 0.10, 1.0, 0.3, 0.05,
                            key='cfg_dist')

        if st.button('Apply Configuration'):
            node.set_config(s_conf, s_dist)
            st.success('Configuration sent via ROS2 service')

        st.divider()
        st.subheader('📡 ROS2 Topics')
        st.code("""
ros2 topic list
ros2 topic echo /inspection/result
ros2 topic hz /camera/image_raw
ros2 action list
ros2 action info /inspect_area
        """)

    # Auto-refresh
    time.sleep(0.1)
    st.rerun()


if __name__ == '__main__':
    main()

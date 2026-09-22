#!/usr/bin/env python3
"""
dashboard.py  ──  BuildScan Rover AI Dashboard
═══════════════════════════════════════════════
Standalone Streamlit dashboard that wires directly to the real
crack-detection inference pipeline.

Run:
    streamlit run dashboard.py

Two modes (auto-detected at startup):
  • ROS2 mode    – rclpy available and ROS2 is sourced:
                   reuses DashboardBridgeNode from dashboard_node.py
                   reads /camera/image_annotated + /inspection/result

Severity thresholds (mirror crack_detection_node.py defaults):
    LOW    : maximum_width  <  70 px   → "Apply surface filler and sealant"
    MEDIUM : maximum_width  < 150 px   → "Use epoxy injection repair"
    HIGH   : maximum_width  ≥ 150 px   → "Urgent structural inspection required"
These constants are defined at the top of this file so you can tune them
without touching the ROS2 node.

PDF export reuses:
    buildscan_inspection.report_generator.ReportGenerator
    buildscan_inspection.report_generator.InspectionData
Falls back to inline PDF construction if the package import fails
(e.g. workspace not built yet), using the same ReportLab calls.

No new pip dependencies — everything is already in requirements-laptop.txt.
"""

# ─────────────────────────────────────────────────────────────────────────────
# STDLIB
# ─────────────────────────────────────────────────────────────────────────────
import io
import os
import sys
import time
import uuid
import datetime
import tempfile
import threading
import dataclasses
from pathlib import Path
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# THIRD-PARTY
# ─────────────────────────────────────────────────────────────────────────────
import cv2
import numpy as np
import streamlit as st
from PIL import Image as PILImage

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 (required - source /opt/ros/humble/setup.bash before running)
# ─────────────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from buildscan_dashboard.dashboard_node import DashboardBridgeNode
    ROS2_AVAILABLE = True
except Exception as e:
    ROS2_AVAILABLE = False
    _ROS2_ERROR = str(e)

# ─────────────────────────────────────────────────────────────────────────────
# REPORT GENERATOR (optional import — falls back to inline PDF)
# ─────────────────────────────────────────────────────────────────────────────
try:
    from buildscan_inspection.report_generator import ReportGenerator, InspectionData
    REPORT_GEN_AVAILABLE = True
except Exception:
    REPORT_GEN_AVAILABLE = False

# ─────────────────────────────────────────────────────────────────────────────
# REPORTLAB (for inline PDF fallback)
# ─────────────────────────────────────────────────────────────────────────────
try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas as rl_canvas
    from reportlab.lib.colors import HexColor, white
    from reportlab.lib.utils import ImageReader
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False

# ═════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS  (mirror crack_detection_node.py defaults)
# Tune severity bands here — or via ROS2 parameter in crack_detection_node
# ═════════════════════════════════════════════════════════════════════════════
SEV_LOW_MAX_WIDTH    = 70    # px  — widths below this → LOW
SEV_MEDIUM_MAX_WIDTH = 150   # px  — widths below this → MEDIUM (else HIGH)

REPAIR_NONE   = "No crack detected"
REPAIR_LOW    = "Apply surface filler and sealant"
REPAIR_MEDIUM = "Use epoxy injection repair"
REPAIR_HIGH   = "Urgent structural inspection required"

# Model path (resolved relative to this file → ../../../../models/)
_THIS_DIR   = Path(__file__).resolve().parent
_MODEL_PATH = _THIS_DIR.parents[3] / "models" / "yolo26n-seg.pt"

# USB camera index (0 = first USB cam; override with env var BUILDSCAN_CAM_IDX)
_CAM_IDX = int(os.environ.get("BUILDSCAN_CAM_IDX", "0"))

# Auto-refresh interval while streaming (seconds)
_REFRESH_INTERVAL = 0.30

# PDF / report output directory
_REPORT_DIR = Path(tempfile.gettempdir()) / "buildscan_reports"
_REPORT_DIR.mkdir(parents=True, exist_ok=True)


# ═════════════════════════════════════════════════════════════════════════════
# DATA CONTAINER — shared between streaming thread and Streamlit main thread
# ═════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class DetectionSnapshot:
    """Thread-safe snapshot of the latest crack detection result."""
    # Annotated frame (BGR numpy array, YOLO bbox + label already baked in)
    annotated_frame: Optional[np.ndarray] = None
    # Raw frame saved for PDF export
    original_frame:  Optional[np.ndarray] = None

    # Metrics (exact field names mirror InspectionResult.msg)
    cracks_detected:    int   = 0
    average_confidence: float = 0.0   # 0–100 %
    maximum_width:      float = 0.0   # px
    total_length:       float = 0.0   # px
    severity:           str   = "NONE"
    recommendation:     str   = REPAIR_NONE
    model_name:         str   = "YOLO26n-seg"
    model_version:      str   = "1.0.0"

    timestamp: Optional[datetime.datetime] = None


# ═════════════════════════════════════════════════════════════════════════════
# SEVERITY HELPERS  (same logic as crack_detection_node._analyze)
# ═════════════════════════════════════════════════════════════════════════════

def classify_severity(max_width_px: float, crack_count: int):
    """Return (severity_str, recommendation_str) using the same thresholds
    as crack_detection_node.py — tunable via SEV_* constants above."""
    if crack_count == 0 or max_width_px == 0:
        return "NONE", REPAIR_NONE
    if max_width_px < SEV_LOW_MAX_WIDTH:
        return "LOW", REPAIR_LOW
    if max_width_px < SEV_MEDIUM_MAX_WIDTH:
        return "MEDIUM", REPAIR_MEDIUM
    return "HIGH", REPAIR_HIGH




def _ros2_stream_loop(stop_event: threading.Event):
    """
    ROS2 mode streaming loop.
    Spins the existing DashboardBridgeNode in this thread and copies its
    latest_annotated + latest_result into session_state each tick.
    """
    if "ros_node" not in st.session_state:
        rclpy.init()
        node = DashboardBridgeNode()
        st.session_state["ros_node"] = node

        ros_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        st.session_state["ros_thread"] = ros_thread

    node: DashboardBridgeNode = st.session_state["ros_node"]

    while not stop_event.is_set():
        r = node.latest_result
        ann = node.latest_annotated
        raw = node.latest_frame

        if r is not None:
            snap = DetectionSnapshot(
                annotated_frame     = ann,
                original_frame      = raw,
                cracks_detected     = r.cracks_detected,
                average_confidence  = r.average_confidence * 100.0,
                maximum_width       = r.maximum_width,
                total_length        = r.total_length,
                severity            = r.severity,
                recommendation      = r.recommendation,
                model_name          = r.model_name,
                model_version       = r.model_version,
                timestamp           = datetime.datetime.now(),
            )
            st.session_state["snapshot"] = snap
        elif ann is not None:
            # Frame arriving before first result — show it without metrics
            snap = st.session_state.get("snapshot", DetectionSnapshot())
            snap.annotated_frame = ann
            snap.original_frame  = raw
            st.session_state["snapshot"] = snap

        time.sleep(0.05)


def start_stream():
    """Launch the appropriate streaming thread and store stop_event."""
    if st.session_state.get("streaming"):
        return

    stop_event = threading.Event()
    st.session_state["stop_event"] = stop_event
    st.session_state["streaming"]  = True
    st.session_state["snapshot"]   = DetectionSnapshot()

    target = _ros2_stream_loop
    t = threading.Thread(target=target, args=(stop_event,), daemon=True)
    t.start()
    st.session_state["stream_thread"] = t


def stop_stream():
    """Signal the streaming thread to stop and wait for clean exit."""
    if not st.session_state.get("streaming"):
        return

    ev: threading.Event = st.session_state.get("stop_event")
    if ev:
        ev.set()

    t: threading.Thread = st.session_state.get("stream_thread")
    if t and t.is_alive():
        t.join(timeout=3.0)

    st.session_state["streaming"]     = False
    st.session_state["stop_event"]    = None
    st.session_state["stream_thread"] = None


# ═════════════════════════════════════════════════════════════════════════════
# PDF EXPORT
# ═════════════════════════════════════════════════════════════════════════════

def _export_pdf(snap: DetectionSnapshot) -> bytes:
    """
    Generate a PDF report for the current snapshot.
    Prefers the project's ReportGenerator; falls back to inline ReportLab.
    Returns the PDF as bytes for st.download_button.
    """
    buf = io.BytesIO()

    # ── Try project ReportGenerator first ────────────────────────────────────
    if REPORT_GEN_AVAILABLE:
        data = InspectionData(
            area_name           = "Live Dashboard Capture",
            cracks_detected     = snap.cracks_detected,
            average_confidence  = snap.average_confidence,
            maximum_width       = snap.maximum_width,
            total_length        = snap.total_length,
            severity            = snap.severity,
            recommendation      = snap.recommendation,
            original_image      = snap.original_frame,
            annotated_image     = snap.annotated_frame,
            model_name          = snap.model_name,
            model_version       = snap.model_version,
            confidence_threshold= 0.30,
        )
        gen      = ReportGenerator(output_dir=str(_REPORT_DIR))
        pdf_path = gen.generate(data)
        with open(pdf_path, "rb") as f:
            return f.read()

    # ── Inline ReportLab fallback ─────────────────────────────────────────────
    if not REPORTLAB_AVAILABLE:
        # Last resort: JSON
        import json
        payload = {
            "timestamp":          snap.timestamp.isoformat() if snap.timestamp else "",
            "cracks_detected":    snap.cracks_detected,
            "average_confidence": round(snap.average_confidence, 2),
            "maximum_width_px":   round(snap.maximum_width, 1),
            "total_length_px":    round(snap.total_length, 1),
            "severity":           snap.severity,
            "recommendation":     snap.recommendation,
            "model":              f"{snap.model_name} v{snap.model_version}",
        }
        return json.dumps(payload, indent=2).encode()

    # Save annotated image to temp file for embedding
    ann_path = ""
    if snap.annotated_frame is not None:
        tmp = tempfile.NamedTemporaryFile(
            delete=False, suffix=".jpg", prefix="buildscan_ann_")
        cv2.imwrite(tmp.name, snap.annotated_frame)
        ann_path = tmp.name

    c = rl_canvas.Canvas(buf, pagesize=A4)
    w, h = A4

    BLUE   = HexColor("#2563eb")
    GREEN  = HexColor("#16a34a")
    ORANGE = HexColor("#ea580c")
    RED    = HexColor("#dc2626")
    PURPLE = HexColor("#7c3aed")
    DARK   = HexColor("#111827")
    ACCENT = HexColor("#1e3a8a")
    BG     = HexColor("#f4f4f4")

    SEV_COLORS = {"NONE": HexColor("#6b7280"), "LOW": GREEN,
                  "MEDIUM": ORANGE, "HIGH": RED}

    # Background
    c.setFillColor(BG); c.rect(0, 0, w, h, fill=1, stroke=0)

    # Header
    c.setFillColor(ACCENT); c.rect(0, h - 80, w, 80, fill=1, stroke=0)
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 22)
    c.drawString(30, h - 45, "BuildScan Rover — Inspection Report")
    c.setFont("Helvetica", 11)
    ts_str = snap.timestamp.strftime("%d-%m-%Y  %I:%M %p") if snap.timestamp else ""
    c.drawString(30, h - 65, ts_str)
    iid = str(uuid.uuid4())[:8].upper()
    c.setFont("Helvetica-Bold", 9)
    c.drawRightString(w - 20, h - 45, f"ID: {iid}")

    # Severity badge
    sev_col = SEV_COLORS.get(snap.severity, RED)
    c.setFillColor(sev_col)
    c.roundRect(w - 160, h - 135, 130, 35, 8, fill=1, stroke=0)
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 15)
    c.drawCentredString(w - 95, h - 123, f"⚠  {snap.severity}")

    # Metric cards helper
    def card(cx, cy, color, title, value, cw=130, ch=45):
        c.setFillColor(color); c.roundRect(cx, cy, cw, ch, 8, fill=1, stroke=0)
        c.setFillColor(white)
        c.setFont("Helvetica-Bold", 10); c.drawString(cx + 8, cy + 28, title)
        c.setFont("Helvetica", 11);      c.drawString(cx + 8, cy + 10, str(value))

    cy1 = h - 185
    card(30,  cy1, BLUE,   "Size (Length)",     f"{snap.total_length:.0f} px")
    card(175, cy1, ORANGE, "Confidence",        f"{snap.average_confidence:.1f}%")
    card(320, cy1, RED,    "Repair Solution",   snap.recommendation[:22] +
         ("…" if len(snap.recommendation) > 22 else ""))

    cy2 = cy1 - 60
    card(30,  cy2, GREEN,  "Width",             f"{snap.maximum_width:.0f} px")
    card(175, cy2, PURPLE, "Severity",          snap.severity)

    # Annotated image
    img_y = cy2 - 200
    if ann_path and os.path.exists(ann_path):
        c.setFillColor(DARK); c.setFont("Helvetica-Bold", 10)
        c.drawString(30, img_y + 175, "Last Crack Detection")
        c.drawImage(ImageReader(ann_path),
                    30, img_y, width=w - 60, height=165,
                    preserveAspectRatio=True)

    # Footer
    c.setStrokeColor(ACCENT); c.line(30, 45, w - 30, 45)
    c.setFillColor(ACCENT); c.setFont("Helvetica-Oblique", 8)
    c.drawString(30, 30,
        "Generated by BuildScan Rover — AI Structural Health Inspection System")
    c.drawRightString(w - 30, 30, f"ID: {iid}")

    c.save()
    buf.seek(0)
    return buf.read()


# ═════════════════════════════════════════════════════════════════════════════
# CSS
# ═════════════════════════════════════════════════════════════════════════════

_CSS = """
<style>
/* ── Global ── */
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700;800&display=swap');

html, body, [class*="css"] {
    font-family: 'Inter', sans-serif;
    background-color: #ffffff;
}

/* ── Header strip ── */
.bs-header {
    display: flex;
    align-items: center;
    gap: 14px;
    padding: 18px 0 10px 0;
    border-bottom: 2px solid #e5e7eb;
    margin-bottom: 20px;
}
.bs-header-icon {
    font-size: 2.4rem;
    line-height: 1;
}
.bs-header-title {
    font-size: 1.75rem;
    font-weight: 800;
    color: #111827;
    letter-spacing: -0.5px;
    margin: 0;
}
.bs-header-sub {
    font-size: 0.82rem;
    color: #6b7280;
    margin: 0;
    font-weight: 400;
}
.bs-mode-badge {
    margin-left: auto;
    padding: 4px 12px;
    border-radius: 20px;
    font-size: 0.78rem;
    font-weight: 700;
    letter-spacing: 0.5px;
}
.bs-mode-ros2 { background: #dbeafe; color: #1d4ed8; }
.bs-mode-standalone { background: #fef3c7; color: #92400e; }

/* ── Controls row ── */
.bs-controls { display: flex; gap: 12px; margin-bottom: 22px; }

/* ── Image panel ── */
.bs-image-panel {
    border: 2px solid #e5e7eb;
    border-radius: 16px;
    padding: 14px;
    background: #f9fafb;
    box-shadow: 0 2px 12px rgba(0,0,0,0.06);
    margin-bottom: 26px;
    text-align: center;
}
.bs-image-caption {
    margin-top: 10px;
    font-size: 0.82rem;
    color: #6b7280;
    font-weight: 600;
    letter-spacing: 0.3px;
}

/* ── Metric cards ── */
.bs-metric-card {
    border-radius: 16px;
    padding: 18px 20px 16px 20px;
    color: #ffffff;
    box-shadow: 0 4px 16px rgba(0,0,0,0.13);
    min-height: 100px;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    margin-bottom: 14px;
    transition: transform 0.15s ease, box-shadow 0.15s ease;
}
.bs-metric-card:hover {
    transform: translateY(-2px);
    box-shadow: 0 8px 24px rgba(0,0,0,0.18);
}
.bs-metric-label {
    font-size: 0.78rem;
    font-weight: 700;
    letter-spacing: 0.8px;
    text-transform: uppercase;
    opacity: 0.88;
    display: flex;
    align-items: center;
    gap: 6px;
}
.bs-metric-value {
    font-size: 1.6rem;
    font-weight: 800;
    letter-spacing: -0.5px;
    line-height: 1.1;
    margin-top: 10px;
    word-break: break-word;
}
.bs-metric-value.small { font-size: 1.15rem; }

.card-blue   { background: linear-gradient(135deg, #2563eb, #1d4ed8); }
.card-orange { background: linear-gradient(135deg, #ea580c, #c2410c); }
.card-red    { background: linear-gradient(135deg, #dc2626, #b91c1c); }
.card-green  { background: linear-gradient(135deg, #16a34a, #15803d); }
.card-purple { background: linear-gradient(135deg, #7c3aed, #6d28d9); }

/* ── Download button ── */
.bs-download-row { margin-top: 8px; }
.bs-download-row .stDownloadButton > button {
    background: transparent !important;
    border: 2px solid #2563eb !important;
    color: #2563eb !important;
    border-radius: 12px !important;
    font-weight: 700 !important;
    font-size: 0.95rem !important;
    padding: 10px 28px !important;
    transition: all 0.18s ease !important;
}
.bs-download-row .stDownloadButton > button:hover {
    background: #2563eb !important;
    color: #ffffff !important;
}

/* ── Status bar ── */
.bs-status {
    background: #f0fdf4;
    border-left: 4px solid #16a34a;
    border-radius: 8px;
    padding: 8px 14px;
    font-size: 0.82rem;
    color: #15803d;
    font-weight: 600;
    margin-bottom: 18px;
}
.bs-status.warn {
    background: #fef3c7;
    border-color: #d97706;
    color: #92400e;
}
.bs-status.error {
    background: #fef2f2;
    border-color: #dc2626;
    color: #b91c1c;
}

/* ── No-data placeholder ── */
.bs-no-data {
    border: 2px dashed #d1d5db;
    border-radius: 14px;
    padding: 60px 20px;
    text-align: center;
    color: #9ca3af;
    background: #f9fafb;
    font-size: 1rem;
}

/* Streamlit button overrides */
div[data-testid="stButton"] > button[kind="primary"] {
    border-radius: 10px !important;
    font-weight: 700 !important;
    padding: 8px 22px !important;
}
div[data-testid="stButton"] > button[kind="secondary"] {
    border-radius: 10px !important;
    font-weight: 600 !important;
    padding: 8px 22px !important;
}
</style>
"""


# ═════════════════════════════════════════════════════════════════════════════
# HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def _severity_color(severity: str) -> str:
    return {
        "NONE":   "card-blue",
        "LOW":    "card-green",
        "MEDIUM": "card-orange",
        "HIGH":   "card-red",
    }.get(severity, "card-purple")


def _metric_card(icon: str, label: str, value: str,
                 color_cls: str, small: bool = False) -> str:
    val_cls = "bs-metric-value small" if small else "bs-metric-value"
    return f"""
<div class="bs-metric-card {color_cls}">
  <div class="bs-metric-label">{icon} {label}</div>
  <div class="{val_cls}">{value}</div>
</div>"""


def _bgr_to_pil(bgr: np.ndarray) -> PILImage.Image:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return PILImage.fromarray(rgb)


# ═════════════════════════════════════════════════════════════════════════════
# MAIN STREAMLIT APP
# ═════════════════════════════════════════════════════════════════════════════

def main():
    # ── Page config (must be first Streamlit call) ────────────────────────────
    st.set_page_config(
        page_title="BuildScan Rover AI Dashboard",
        page_icon="🏗️",
        layout="wide",
        initial_sidebar_state="collapsed",
    )

    # ── Inject CSS ────────────────────────────────────────────────────────────
    st.markdown(_CSS, unsafe_allow_html=True)

    # ── ROS2 availability check ───────────────────────────────────────────────
    if not ROS2_AVAILABLE:
        st.error(
            f"❌ ROS 2 is not available. Cannot start dashboard.\n\n"
            f"Error: {_ROS2_ERROR}\n\n"
            "**To fix:** Source your ROS 2 environment before running Streamlit:\n"
            "```bash\n"
            "source /opt/ros/humble/setup.bash\n"
            "source ~/ros2_ws/install/setup.bash\n"
            "export ROS_DOMAIN_ID=25\n"
            "export ROS_LOCALHOST_ONLY=0\n"
            "python -m streamlit run dashboard.py\n"
            "```"
        )
        return

    # ── Session state defaults ────────────────────────────────────────────────
    st.session_state.setdefault("streaming",     False)
    st.session_state.setdefault("snapshot",      DetectionSnapshot())
    st.session_state.setdefault("stop_event",    None)
    st.session_state.setdefault("stream_thread", None)
    st.session_state.setdefault("stream_error",  None)

    snap: DetectionSnapshot = st.session_state["snapshot"]

    # ─────────────────────────────────────────────────────────────────────────
    # HEADER ROW
    # ─────────────────────────────────────────────────────────────────────────
    mode_label = "ROS2 Mode"
    mode_cls   = "bs-mode-ros2"

    st.markdown(f"""
    <div class="bs-header">
      <div class="bs-header-icon">🏗️</div>
      <div>
        <p class="bs-header-title">BuildScan Rover AI Dashboard</p>
        <p class="bs-header-sub">AI-Assisted Structural Crack Detection  •  YOLO26n-seg</p>
      </div>
      <span class="bs-mode-badge {mode_cls}">{mode_label}</span>
    </div>
    """, unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # STATUS BAR
    # ─────────────────────────────────────────────────────────────────────────
    err = st.session_state.get("stream_error")
    if err:
        st.markdown(
            f'<div class="bs-status error">⚠ {err}</div>',
            unsafe_allow_html=True)
    elif st.session_state["streaming"]:
        src = "ROS2 topics"
        st.markdown(
            f'<div class="bs-status">● Streaming live from {src}</div>',
            unsafe_allow_html=True)
    else:
        st.markdown(
            '<div class="bs-status warn">◌ Stream stopped — press ▶ Start Stream to begin</div>',
            unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # CONTROLS ROW  (Start on left, Stop on right — same row)
    # ─────────────────────────────────────────────────────────────────────────
    ctrl_left, ctrl_spacer, ctrl_right = st.columns([2, 6, 2])

    with ctrl_left:
        if st.button(
            "▶  Start Stream",
            key="btn_start",
            type="primary",
            disabled=st.session_state["streaming"],
            use_container_width=True,
        ):
            start_stream()
            st.rerun()

    with ctrl_right:
        if st.button(
            "■  Stop Stream",
            key="btn_stop",
            type="secondary",
            disabled=not st.session_state["streaming"],
            use_container_width=True,
        ):
            stop_stream()
            st.rerun()

    st.markdown("<div style='margin-bottom:6px'></div>", unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # IMAGE PANEL
    # ─────────────────────────────────────────────────────────────────────────
    st.markdown('<div class="bs-image-panel">', unsafe_allow_html=True)

    frame = snap.annotated_frame
    if frame is not None:
        pil_img = _bgr_to_pil(frame)
        st.image(pil_img, use_column_width=True)
        st.markdown(
            '<div class="bs-image-caption">🔍 Last Crack Detection</div>',
            unsafe_allow_html=True)
    else:
        st.markdown("""
        <div class="bs-no-data">
          <div style="font-size:3rem; margin-bottom:12px">📷</div>
          <div style="font-weight:600; color:#374151;">No frame yet</div>
          <div style="font-size:0.85rem; margin-top:6px;">
            Press <strong>▶ Start Stream</strong> to begin capturing and detecting cracks
          </div>
        </div>""", unsafe_allow_html=True)

    st.markdown('</div>', unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # METRIC CARDS  (2 rows × 3 columns)
    # ─────────────────────────────────────────────────────────────────────────
    conf_str  = f"{snap.average_confidence:.1f}%" if snap.cracks_detected else "—"
    size_str  = f"{snap.total_length:.0f} px"     if snap.cracks_detected else "— px"
    width_str = f"{snap.maximum_width:.0f} px"    if snap.cracks_detected else "— px"
    sev_str   = snap.severity
    rep_str   = snap.recommendation

    # Row 1: Size (blue) | Confidence (orange) | Repair Solution (red)
    r1c1, r1c2, r1c3 = st.columns(3)

    with r1c1:
        st.markdown(
            _metric_card("📏", "Size", size_str, "card-blue"),
            unsafe_allow_html=True)

    with r1c2:
        st.markdown(
            _metric_card("🎯", "Confidence", conf_str, "card-orange"),
            unsafe_allow_html=True)

    with r1c3:
        st.markdown(
            _metric_card("🛠", "Repair Solution", rep_str, "card-red", small=True),
            unsafe_allow_html=True)

    # Row 2: Width (green) | Severity (purple) | [spacer]
    r2c1, r2c2, r2c3 = st.columns(3)

    with r2c1:
        st.markdown(
            _metric_card("↔", "Width", width_str, "card-green"),
            unsafe_allow_html=True)

    with r2c2:
        sev_color = _severity_color(sev_str)
        st.markdown(
            _metric_card("⚠", "Severity", sev_str, sev_color),
            unsafe_allow_html=True)

    with r2c3:
        # Cracks count — extra context card
        crack_str = str(snap.cracks_detected) if snap.cracks_detected else "0"
        st.markdown(
            _metric_card("🔍", "Cracks Found", crack_str, "card-blue"),
            unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # DOWNLOAD RESULT BUTTON
    # ─────────────────────────────────────────────────────────────────────────
    st.markdown('<div class="bs-download-row">', unsafe_allow_html=True)

    has_result = snap.cracks_detected > 0 or snap.annotated_frame is not None

    if has_result:
        try:
            pdf_bytes = _export_pdf(snap)
            ext       = "pdf" if pdf_bytes[:4] == b"%PDF" else "json"
            ts_tag    = (snap.timestamp.strftime("%Y%m%d_%H%M%S")
                         if snap.timestamp else "export")
            st.download_button(
                label     = "📥  Download Result",
                data      = pdf_bytes,
                file_name = f"BuildScan_{ts_tag}.{ext}",
                mime      = "application/pdf" if ext == "pdf" else "application/json",
                key       = "btn_download",
            )
        except Exception as exc:
            st.warning(f"Export failed: {exc}")
    else:
        st.download_button(
            label     = "📥  Download Result",
            data      = b"",
            file_name = "BuildScan_no_data.pdf",
            disabled  = True,
            key       = "btn_download_disabled",
        )

    st.markdown('</div>', unsafe_allow_html=True)

    # ─────────────────────────────────────────────────────────────────────────
    # AUTO-REFRESH while streaming
    # ─────────────────────────────────────────────────────────────────────────
    if st.session_state["streaming"]:
        time.sleep(_REFRESH_INTERVAL)
        st.rerun()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()

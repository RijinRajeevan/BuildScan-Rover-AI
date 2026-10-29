"""
BuildScan Rover AI  ·  Live Structural Health Inspection Dashboard
════════════════════════════════════════════════════════════════════
Target Architecture:
  Logitech Brio 100 (/dev/video0)
        │ (USB)
  Raspberry Pi 4 (Ubuntu 22.04, 10.21.210.129:8080)
        │ (MJPEG /video or ROS 2 /image_raw)
  Network / Wi-Fi
        │
  AI Software Laptop (Dell G15 / RTX 3050)
        ├── Camera Receiver (camera_receiver.py)
        ├── YOLO26n-seg Crack Segmentation (yolo_detector.py)
        ├── Google Gemini Flash Visual Analysis (gemini_analyzer.py)
        └── Live Streamlit Dashboard (demo_dashboard.py)
"""

import os
import sys
import io
import time
import math
import datetime
import tempfile
import threading
import json
import random
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

import cv2
import numpy as np
import streamlit as st
from PIL import Image as PILImage

# Modular Pipeline Components
import camera_receiver
import yolo_detector
import gemini_analyzer

logging.basicConfig(
    level=logging.INFO,
    format="[BuildScan %(asctime)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("buildscan.dashboard")

# ── ReportLab PDF Export ─────────────────────────────────────
try:
    from reportlab.lib.pagesizes import letter
    from reportlab.lib import colors
    from reportlab.lib.units import inch
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.platypus import (
        SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image as RLImage, HRFlowable
    )
    PDF_OK = True
except ImportError:
    PDF_OK = False

# ── Environment & Network Constants ──────────────────────────
PI_IP = os.getenv("BUILDSCAN_PI_IP", "10.21.210.129").strip()
PI_PORT = int(os.getenv("BUILDSCAN_PI_PORT", "8080"))
PI_STREAM = os.getenv("BUILDSCAN_CAMERA_URL", f"http://{PI_IP}:{PI_PORT}/video").strip()
AI_PORT = 8081
AI_STREAM = f"http://127.0.0.1:{AI_PORT}/video_feed"

# Dedup tracking
TRACK_RADIUS = 65    # px spatial radius
TRACK_TTL = 4.0       # seconds before track expires


@dataclass
class Defect:
    defect_id: str
    timestamp: str
    width_mm: float
    width_px: float
    length_mm: float
    conf: float
    severity: str
    action: str
    bbox: tuple = (0, 0, 0, 0)


@dataclass
class EngineState:
    connected: bool = False
    video_source: str = "pi"       # "pi", "webcam", "sim"
    pi_ip: str = PI_IP
    pi_port: int = PI_PORT
    custom_url: Optional[str] = None

    # YOLO & Inspection Metrics
    cracks_now: int = 0
    total_defects: int = 0
    avg_conf: float = 0.0
    max_width_mm: float = 0.0
    max_len_mm: float = 0.0
    severity: str = "NONE"
    health_idx: float = 100.0
    action: str = "System initialized — awaiting camera stream..."
    fps: float = 0.0
    latency_ms: float = 0.0
    device_name: str = "Initializing..."
    is_crack_trained: bool = True

    # Frame Buffers
    frame_ts: float = 0.0
    frame_iso: str = ""
    last_raw_frame: Optional[np.ndarray] = None
    last_frame: Optional[np.ndarray] = None
    last_jpeg: Optional[bytes] = None
    registry: List[Defect] = field(default_factory=list)

    # Configs
    yolo_conf: float = 0.25
    scale_px_to_mm: float = 0.025
    health_info: Optional[Dict[str, Any]] = None


# ── Process-Level Singleton State ─────────────────────────────
if not hasattr(sys, "_buildscan_state"):
    sys._buildscan_state = EngineState()
    sys._buildscan_lock = threading.Lock()
    sys._buildscan_svc_started = False

_state: EngineState = sys._buildscan_state
_lock: threading.Lock = sys._buildscan_lock


# ════════════════════════════════════════════════════════════
#   STANDBY & SIMULATION GENERATORS
# ════════════════════════════════════════════════════════════
def _make_standby_frame(pi_ip: str, pi_port: int, status_note: str = "") -> bytes:
    w, h = 640, 480
    frame = np.full((h, w, 3), (16, 12, 8), dtype=np.uint8)

    # Inspection grid
    for x in range(0, w, 40):
        cv2.line(frame, (x, 0), (x, h), (26, 20, 14), 1)
    for y in range(0, h, 40):
        cv2.line(frame, (0, y), (w, y), (26, 20, 14), 1)

    # Corner reticles
    c_len = 22
    c_col = (255, 185, 0)
    for cx, cy, dx, dy in [(18, 18, 1, 1), (w - 18, 18, -1, 1), (18, h - 18, 1, -1), (w - 18, h - 18, -1, -1)]:
        cv2.line(frame, (cx, cy), (cx + dx * c_len, cy), c_col, 2)
        cv2.line(frame, (cx, cy), (cx, cy + dy * c_len), c_col, 2)

    # Center card
    bx1, by1, bx2, by2 = 35, 70, w - 35, h - 70
    overlay = frame.copy()
    cv2.rectangle(overlay, (bx1, by1), (bx2, by2), (26, 20, 14), -1)
    cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, frame)
    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (70, 55, 38), 1)

    # Header banner
    cv2.rectangle(frame, (bx1 + 12, by1 + 12), (bx2 - 12, by1 + 44), (40, 30, 20), -1)
    cv2.putText(frame, "BUILDSCAN ROVER AI  |  CAMERA OFFLINE / STANDBY",
                (bx1 + 22, by1 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 215, 255), 1, cv2.LINE_AA)

    # Radar pulse
    t = time.time()
    pulse = int(abs(math.sin(t * 3.5)) * 8) + 6
    cv2.circle(frame, (bx1 + 35, by1 + 75), 6, (0, 140, 255), -1)
    cv2.circle(frame, (bx1 + 35, by1 + 75), pulse, (0, 140, 255), 1)

    cv2.putText(frame, f"Awaiting Pi Camera: http://{pi_ip}:{pi_port}/video",
                (bx1 + 55, by1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (245, 245, 245), 1, cv2.LINE_AA)

    # Diagnostics
    err_txt = status_note if status_note else "Probing http://" + pi_ip + ":" + str(pi_port) + "/health..."
    cv2.putText(frame, f"Status: {err_txt[:55]}",
                (bx1 + 35, by1 + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 180, 255), 1, cv2.LINE_AA)

    # Instructions box
    cv2.putText(frame, "To start camera service on Raspberry Pi SSH terminal:",
                (bx1 + 35, by1 + 155), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 210), 1, cv2.LINE_AA)

    cv2.rectangle(frame, (bx1 + 25, by1 + 170), (bx2 - 25, by1 + 205), (10, 8, 6), -1)
    cv2.putText(frame, "python3 ~/brio_stream.py",
                (bx1 + 35, by1 + 193), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 190), 1, cv2.LINE_AA)

    cv2.putText(frame, "Or for ROS 2 mode: ros2 launch buildscan_bringup camera_stream.launch.py",
                (bx1 + 35, by1 + 230), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (130, 145, 165), 1, cv2.LINE_AA)

    _, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return enc.tobytes()


def _make_sim_frame(step: int) -> np.ndarray:
    w, h = 640, 480
    base = np.full((h, w, 3), 160, dtype=np.uint8)
    np.random.seed(123)
    noise = np.random.normal(0, 10, (h, w, 3)).astype(np.int16)
    frame = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    ox = int(25 * math.sin(step * 0.06))
    pts = [
        (130 + ox, 60),
        (200 + ox, 150),
        (270 + ox + int(10 * math.cos(step * 0.05)), 240),
        (360 + ox, 340),
        (450 + ox, 430),
    ]
    for i in range(len(pts) - 1):
        cv2.line(frame, pts[i], pts[i+1], (32, 32, 32), 4)

    pb1 = pts[2]
    pb2 = (pb1[0] + 85, pb1[1] + 45)
    pb3 = (pb2[0] + 55, pb2[1] + 15)
    cv2.line(frame, pb1, pb2, (42, 42, 42), 3)
    cv2.line(frame, pb2, pb3, (48, 48, 48), 2)
    return frame


# ════════════════════════════════════════════════════════════
#   LOCAL MJPEG STREAMING SERVER  (port AI_PORT)
# ════════════════════════════════════════════════════════════
class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/video_feed"):
            self.send_response(200)
            self.send_header("Connection", "close")
            self.send_header("Server", "BuildScan-AI-Streamer")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-type", "multipart/x-mixed-replace;boundary=boundarydonotcross")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _state.last_jpeg
                        cur_ip = _state.pi_ip
                        cur_port = _state.pi_port
                        err = _state.action
                    if jpg is None:
                        jpg = _make_standby_frame(cur_ip, cur_port, err)
                    part = (
                        b"--boundarydonotcross\r\n"
                        b"Content-type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                    self.wfile.write(part)
                    time.sleep(0.033)   # ~30 fps cap
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        elif self.path.startswith("/snapshot"):
            with _lock:
                jpg = _state.last_jpeg
                cur_ip = _state.pi_ip
                cur_port = _state.pi_port
            if jpg is None:
                jpg = _make_standby_frame(cur_ip, cur_port)
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == "/healthz":
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with _lock:
                body = json.dumps({
                    "connected": _state.connected,
                    "video_source": _state.video_source,
                    "fps": round(_state.fps, 1),
                    "cracks_now": _state.cracks_now,
                    "total": _state.total_defects,
                    "device": _state.device_name,
                    "latency_ms": _state.latency_ms,
                }).encode()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def log_message(self, *_):
        pass


def _run_mjpeg_server():
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", AI_PORT), _MJPEGHandler)
        srv.daemon_threads = True
        log.info(f"MJPEG server started → http://0.0.0.0:{AI_PORT}/video_feed")
        srv.serve_forever()
    except OSError as e:
        log.warning(f"MJPEG server error: {e}")


# ════════════════════════════════════════════════════════════
#   BACKGROUND INFERENCE WORKER
# ════════════════════════════════════════════════════════════
def _run_inference_worker():
    """
    Dedicated background worker:
    - In Pi mode: receives frames from PiCameraReceiver.
    - In webcam mode: captures from local device 0.
    - In sim mode: generates synthetic concrete crack frames.
    - Executes YOLO crack segmentation and updates state atomically.
    """
    global _state

    log.info("Initializing YOLOCrackDetector...")
    detector = yolo_detector.YOLOCrackDetector()

    with _lock:
        _state.device_name = detector.device_name
        _state.is_crack_trained = detector.is_crack_trained

    rx = camera_receiver.PiCameraReceiver(pi_ip=_state.pi_ip, pi_port=_state.pi_port)
    rx.start()

    tracks: list = []
    defect_seq = 1
    fps_t0 = time.time()
    fps_frames = 0
    cur_fps = 0.0
    sim_step = 0
    webcam_cap = None
    last_processed_ts = 0.0

    last_pi_ip = _state.pi_ip
    last_pi_port = _state.pi_port

    while True:
        with _lock:
            src_mode = _state.video_source
            current_ip = _state.pi_ip
            current_port = _state.pi_port
            user_conf = _state.yolo_conf

        # Handle IP or port update
        if current_ip != last_pi_ip or current_port != last_pi_port:
            log.info(f"Updating camera receiver target → {current_ip}:{current_port}")
            rx.stop()
            rx = camera_receiver.PiCameraReceiver(pi_ip=current_ip, pi_port=current_port)
            rx.start()
            last_pi_ip = current_ip
            last_pi_port = current_port

        # Handle webcam lifecycle
        if src_mode != "webcam" and webcam_cap is not None:
            try:
                webcam_cap.release()
            except Exception:
                pass
            webcam_cap = None

        frame = None
        raw_frame = None
        is_connected = False
        frame_timestamp = time.time()
        iso_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(frame_timestamp))
        status_action = "Running normal inspection"

        if src_mode == "webcam":
            if webcam_cap is None or not webcam_cap.isOpened():
                webcam_cap = cv2.VideoCapture(0)
            ret, frame = webcam_cap.read() if (webcam_cap and webcam_cap.isOpened()) else (False, None)
            if ret and frame is not None:
                raw_frame = frame.copy()
                is_connected = True
            else:
                time.sleep(0.05)
                continue

        elif src_mode == "sim":
            sim_step += 1
            frame = _make_sim_frame(sim_step)
            raw_frame = frame.copy()
            is_connected = True
            time.sleep(0.033)

        else:  # "pi"
            cam_frame, cam_st = rx.get_latest_frame()
            is_connected = cam_st.connected
            frame_timestamp = cam_st.last_frame_ts or frame_timestamp
            iso_str = cam_st.last_frame_iso or iso_str

            if cam_st.error_message:
                status_action = cam_st.error_message

            with _lock:
                _state.health_info = cam_st.health_info

            if cam_frame is not None and is_connected:
                frame = cam_frame
                raw_frame = frame.copy()
            else:
                # Disconnected state: update state and do not substitute simulation
                tracks.clear()
                with _lock:
                    _state.connected = False
                    _state.cracks_now = 0
                    _state.action = "Pi camera disconnected. " + (cam_st.error_message or "Awaiting connection...")
                    _state.last_jpeg = _make_standby_frame(current_ip, current_port, _state.action)
                time.sleep(0.2)
                continue

        # Prevent duplicate inference on identical / stale frames
        if src_mode == "pi" and frame_timestamp == last_processed_ts:
            time.sleep(0.015)
            continue
        last_processed_ts = frame_timestamp

        # Run YOLO Detection
        det_result = detector.detect(frame, conf_override=user_conf)
        annotated = det_result.annotated_frame if det_result.annotated_frame is not None else frame

        # FPS calculation
        fps_frames += 1
        elapsed = time.time() - fps_t0
        if elapsed >= 1.0:
            cur_fps = fps_frames / elapsed
            fps_frames = 0
            fps_t0 = time.time()

        # Spatial tracking and registration of unique defects
        now_t = time.time()
        cracks_now = len(det_result.detections)
        max_w_px = 0.0
        max_l_px = 0.0
        confs = []

        for det in det_result.detections:
            x1, y1, x2, y2 = det.bbox
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            max_w_px = max(max_w_px, det.width_px)
            max_l_px = max(max_l_px, det.length_px)
            confs.append(det.confidence)

            # Match against existing tracks
            matched_id = None
            for idx, trk in enumerate(tracks):
                tx, ty, _, _, last_t, tid = trk
                if math.hypot(cx - tx, cy - ty) < TRACK_RADIUS and (now_t - last_t) < TRACK_TTL:
                    matched_id = tid
                    tracks[idx] = (cx, cy, bw, bh, now_t, tid)
                    break

            if matched_id is None:
                new_id = f"DF-{defect_seq:04d}"
                defect_seq += 1
                tracks.append((cx, cy, bw, bh, now_t, new_id))
                d = Defect(
                    defect_id=new_id,
                    timestamp=datetime.datetime.now().strftime("%H:%M:%S"),
                    width_px=det.width_px,
                    width_mm=det.estimated_width_mm,
                    length_mm=det.estimated_length_mm,
                    conf=round(det.confidence * 100, 1),
                    severity=det.severity,
                    action=det.action,
                    bbox=(x1, y1, x2, y2),
                )
                with _lock:
                    _state.registry.append(d)
                    _state.total_defects += 1

        # Clean expired tracks
        tracks[:] = [t for t in tracks if (now_t - t[4]) < TRACK_TTL]

        # Global severity calculation
        if cracks_now == 0:
            sev_g = "NONE"
            act_g = "Concrete surface nominal. No active cracks observed."
        else:
            sev_g, act_g = yolo_detector.classify_severity(max_w_px, detector.scale_px_to_mm)

        # HUD Overlay Top Banner
        h_img, w_img = annotated.shape[:2]
        hud_top = annotated.copy()
        cv2.rectangle(hud_top, (0, 0), (w_img, 32), (8, 14, 26), -1)
        cv2.addWeighted(hud_top, 0.82, annotated, 0.18, 0, annotated)

        hud_text = (
            f"BUILDSCAN ROVER AI  |  {iso_str[11:]}  |  "
            f"{detector.device_name[:18]} ({det_result.latency_ms:.0f}ms)  |  {cur_fps:.1f} FPS  |  "
            f"Cracks: {cracks_now}  |  Logged: {defect_seq - 1}"
        )
        cv2.putText(annotated, hud_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 215, 255), 1, cv2.LINE_AA)

        # HUD Overlay Bottom Banner
        health = max(10.0, 100.0 - (cracks_now * 12.0 + max_w_px * detector.scale_px_to_mm * 20.0))
        hud_bot = annotated.copy()
        cv2.rectangle(hud_bot, (0, h_img - 24), (w_img, h_img), (8, 14, 26), -1)
        cv2.addWeighted(hud_bot, 0.82, annotated, 0.18, 0, annotated)

        scol = (30, 220, 80) if sev_g in ("NONE", "LOW") else ((0, 140, 255) if sev_g == "MEDIUM" else (30, 30, 245))
        bot_text = f"HEALTH: {health:.1f}%  |  SEVERITY: {sev_g}  |  {act_g[:65]}"
        cv2.putText(annotated, bot_text, (10, h_img - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.42, scol, 1, cv2.LINE_AA)

        # Encode JPEG for web streaming
        _, enc = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 88])
        enc_bytes = enc.tobytes()

        # Update process state atomically
        with _lock:
            _state.connected = is_connected
            _state.last_raw_frame = raw_frame
            _state.last_frame = annotated
            _state.last_jpeg = enc_bytes
            _state.cracks_now = cracks_now
            _state.fps = cur_fps
            _state.latency_ms = det_result.latency_ms
            _state.frame_ts = frame_timestamp
            _state.frame_iso = iso_str
            if confs:
                _state.avg_conf = sum(confs) / len(confs) * 100.0
            if max_w_px * detector.scale_px_to_mm > _state.max_width_mm:
                _state.max_width_mm = max_w_px * detector.scale_px_to_mm
            if max_l_px * detector.scale_px_to_mm > _state.max_len_mm:
                _state.max_len_mm = max_l_px * detector.scale_px_to_mm
            _state.severity = sev_g
            _state.health_idx = round(health, 1)
            _state.action = act_g


def _ensure_services():
    if getattr(sys, "_buildscan_svc_started", False):
        return
    sys._buildscan_svc_started = True
    threading.Thread(target=_run_mjpeg_server, daemon=True, name="bscan-mjpeg").start()
    threading.Thread(target=_run_inference_worker, daemon=True, name="bscan-infer").start()
    log.info("BuildScan background services started (MJPEG Server + YOLO Inference Worker)")


# ════════════════════════════════════════════════════════════
#   PDF AUDIT REPORT GENERATOR
# ════════════════════════════════════════════════════════════
def _make_pdf(st_copy: EngineState, gemini_result: Optional[gemini_analyzer.GeminiAnalysisResult] = None) -> bytes:
    if not PDF_OK:
        return json.dumps({
            "timestamp": datetime.datetime.now().isoformat(),
            "total_defects": st_copy.total_defects,
            "severity": st_copy.severity,
            "health_index": st_copy.health_idx,
        }, indent=2).encode()

    buf = io.BytesIO()
    tmp_annotated = None
    if st_copy.last_frame is not None:
        try:
            f = tempfile.NamedTemporaryFile(delete=False, suffix=".jpg", prefix="bscan_audit_")
            cv2.imwrite(f.name, st_copy.last_frame)
            tmp_annotated = f.name
        except Exception:
            pass

    styles = getSampleStyleSheet()
    doc = SimpleDocTemplate(buf, pagesize=letter, rightMargin=36, leftMargin=36, topMargin=36, bottomMargin=36)
    story = []

    title_s = ParagraphStyle("T", parent=styles["Heading1"], fontSize=18, fontName="Helvetica-Bold", textColor=colors.HexColor("#0f172a"))
    sub_s = ParagraphStyle("S", parent=styles["Normal"], fontSize=8.5, textColor=colors.HexColor("#64748b"))
    h2_s = ParagraphStyle("H2", parent=styles["Heading2"], fontSize=12, fontName="Helvetica-Bold", textColor=colors.HexColor("#1e3a8a"), spaceBefore=10, spaceAfter=4)
    body_s = ParagraphStyle("B", parent=styles["Normal"], fontSize=8.5, textColor=colors.HexColor("#334155"))

    now_s = datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    cert_id = f"BS-{datetime.datetime.now().strftime('%Y%m%d')}-{random.randint(100, 999)}"

    story.append(Paragraph("BUILDSCAN ROVER AI  ·  STRUCTURAL HEALTH AUDIT REPORT", title_s))
    story.append(Paragraph(
        f"Audit ID: {cert_id}  ·  Generated: {now_s}  ·  "
        f"Rover Source: {st_copy.pi_ip}:{st_copy.pi_port}  ·  Device: {st_copy.device_name}",
        sub_s
    ))
    story.append(Spacer(1, 6))
    story.append(HRFlowable(width="100%", thickness=1.5, color=colors.HexColor("#2563eb"), spaceBefore=2, spaceAfter=10))

    sev_col = {"LOW": "#16a34a", "MEDIUM": "#ea580c", "HIGH": "#dc2626"}.get(st_copy.severity, "#334155")
    metrics = [
        ["METRIC", "VALUE", "STANDARD REFERENCE", "EVALUATION"],
        ["Cumulative Defects", str(st_copy.total_defects), "ACI 224R-01", "LOGGED"],
        ["Max Crack Aperture", f"{st_copy.max_width_mm:.2f} mm", "< 0.30mm Hairline / ≥ 0.55mm Critical", st_copy.severity],
        ["Max Crack Length", f"{st_copy.max_len_mm:.1f} mm", "Calibrated monocular geometry", "CALC"],
        ["Structural Health Index", f"{st_copy.health_idx:.1f} %", "100% = Nominal / Unblemished", "EVAL"],
        ["AI Inference Engine", st_copy.device_name, "YOLO26n-seg Instance Segmentation", "VERIFIED"],
    ]
    tbl = Table(metrics, colWidths=[2.2*inch, 2.0*inch, 2.2*inch, 0.9*inch])
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, 0), 8),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("FONTSIZE", (0, 1), (-1, -1), 7.5),
        ("TEXTCOLOR", (3, 2), (3, 2), colors.HexColor(sev_col)),
        ("FONTNAME", (3, 2), (3, 2), "Helvetica-Bold"),
    ]))
    story.append(tbl)
    story.append(Spacer(1, 10))

    if tmp_annotated:
        story.append(Paragraph("ANNOTATED INSPECTION FRAME  ·  CRACK SEGMENTATION MASK", h2_s))
        try:
            story.append(RLImage(tmp_annotated, width=6.6*inch, height=3.6*inch))
        except Exception:
            pass
        story.append(Spacer(1, 8))

    # Gemini Supplementary Section if available
    if gemini_result and gemini_result.success and gemini_result.data:
        gd = gemini_result.data
        story.append(Paragraph(f"GEMINI FLASH VISUAL ANALYSIS  ·  {gemini_result.model}", h2_s))
        g_rows = [
            ["PARAMETER", "GEMINI VISUAL OBSERVATION"],
            ["Visual Defects", ", ".join(gd.get("visible_defects", []))],
            ["Crack Visible", "YES" if gd.get("crack_visible") else "NO"],
            ["Confidence Assessment", gd.get("confidence_assessment", "")],
            ["Estimated Location", gd.get("approximate_location", "")],
            ["Surface Material", gd.get("possible_surface_material", "")],
            ["Visual Description", gd.get("visual_description", "")],
            ["Recommended Action", gd.get("recommended_next_step", "")],
            ["Sensor Limitations", gd.get("limitations", "")],
        ]
        gtbl = Table(g_rows, colWidths=[2.2*inch, 5.1*inch])
        gtbl.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#334155")),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTSIZE", (0, 0), (-1, -1), 7.5),
            ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ]))
        story.append(gtbl)
        story.append(Spacer(1, 8))

    if st_copy.registry:
        story.append(Paragraph("DEFECT REGISTRY & REMEDIATION SCHEDULE", h2_s))
        rows = [["ID", "TIME", "APERTURE", "LENGTH", "CONF", "SEV", "REMEDIATION ACTION"]]
        for d in st_copy.registry[-10:]:
            rows.append([
                d.defect_id, d.timestamp,
                f"{d.width_mm:.2f} mm", f"{d.length_mm:.1f} mm",
                f"{d.conf:.0f}%", d.severity,
                d.action[:45] + "…"
            ])
        rt = Table(rows, colWidths=[0.8*inch, 0.7*inch, 0.9*inch, 0.8*inch, 0.5*inch, 0.6*inch, 3.0*inch])
        rt.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f172a")),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTSIZE", (0, 0), (-1, 0), 7.5),
            ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
            ("FONTSIZE", (0, 1), (-1, -1), 7),
        ]))
        story.append(rt)

    # Engineering Disclaimer
    story.append(Spacer(1, 10))
    story.append(Paragraph(
        "<b>LEGAL DISCLAIMER:</b> Automated AI detection and multimodal visual outputs are for screening and preliminary scoping only. "
        "Photographic inspection cannot measure internal depth, rebar condition, or core compressive strength. "
        "A certified Professional Engineer (PE) must validate all remediation actions before structural modifications.",
        ParagraphStyle("Disc", parent=styles["Normal"], fontSize=6.5, textColor=colors.HexColor("#64748b"))
    ))

    doc.build(story)
    if tmp_annotated:
        try:
            os.remove(tmp_annotated)
        except Exception:
            pass
    return buf.getvalue()


# ════════════════════════════════════════════════════════════
#   CSS THEME STYLING
# ════════════════════════════════════════════════════════════
_CSS = """
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800;900&display=swap');
html,body,[class*="css"]{font-family:'Inter',-apple-system,sans-serif!important;color:#f1f5f9;background:#050b14!important;}
.stApp{background:radial-gradient(circle at 50% 0%,#0d1e3a 0%,#060c18 55%,#030609 100%)!important;}
header[data-testid="stHeader"]{background:transparent!important;}
.block-container{padding-top:.8rem!important;padding-bottom:2rem!important;max-width:1440px!important;}

/* HEADER */
.bsh{display:flex;align-items:center;justify-content:space-between;
     background:rgba(13,24,48,.78);border:1px solid rgba(6,182,212,.3);
     backdrop-filter:blur(16px);border-radius:16px;padding:13px 22px;
     margin-bottom:12px;box-shadow:0 8px 32px rgba(0,0,0,.45);}
.bsh-ico{font-size:2.1rem;margin-right:13px;filter:drop-shadow(0 0 10px rgba(6,182,212,.6));}
.bsh-title{font-size:1.5rem;font-weight:900;letter-spacing:-.4px;margin:0;
           background:linear-gradient(135deg,#38bdf8,#818cf8);
           -webkit-background-clip:text;-webkit-text-fill-color:transparent;}
.bsh-sub{font-size:.78rem;color:#94a3b8;font-weight:500;margin-top:2px;}
.bsh-right{display:flex;align-items:center;gap:10px;}
.pill{display:inline-flex;align-items:center;gap:6px;border-radius:20px;
      font-size:.73rem;font-weight:700;padding:5px 13px;}
.p-gpu{background:rgba(14,165,233,.15);border:1px solid #0284c7;color:#38bdf8;}
.p-live{background:rgba(34,197,94,.15);border:1px solid #22c55e;color:#4ade80;}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;animation:pdot 1.8s infinite;}
.d-g{background:#22c55e;box-shadow:0 0 8px #22c55e;}
.d-c{background:#06b6d4;box-shadow:0 0 8px #06b6d4;}

/* STATUS BAR */
.sbar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;
      background:rgba(10,18,36,.65);border:1px solid rgba(255,255,255,.07);
      border-radius:11px;padding:8px 15px;margin-bottom:14px;
      font-size:.76rem;color:#cbd5e1;}
.si{display:flex;align-items:center;gap:5px;font-weight:600;margin-right:12px;}

/* VIDEO BOX */
.vbox{background:#040811;border:1.5px solid rgba(6,182,212,.45);border-radius:14px;
      overflow:hidden;position:relative;box-shadow:0 10px 40px rgba(0,0,0,.6);}
.vbadge{position:absolute;top:10px;left:12px;z-index:10;
        background:rgba(220,38,38,.88);backdrop-filter:blur(6px);color:#fff;
        font-size:.7rem;font-weight:800;letter-spacing:.7px;padding:3px 10px;
        border-radius:6px;box-shadow:0 0 12px rgba(220,38,38,.55);}
.vstatus{font-size:.72rem;color:#94a3b8;margin-top:5px;padding:0 4px;}

/* METRIC CARDS */
.mc{border-radius:13px;padding:13px 16px;min-height:80px;display:flex;
    flex-direction:column;justify-content:space-between;margin-bottom:9px;
    transition:transform .2s,box-shadow .2s;}
.mc:hover{transform:translateY(-2px);}
.ml{font-size:.67rem;font-weight:700;letter-spacing:.9px;text-transform:uppercase;opacity:.88;}
.mv{font-size:1.6rem;font-weight:800;letter-spacing:-.5px;line-height:1;margin-top:5px;}
.mv.sm{font-size:.88rem;}
.ms{font-size:.7rem;opacity:.8;font-weight:500;margin-top:3px;}
.mc-bl{background:linear-gradient(135deg,#1e3a8a,#2563eb);box-shadow:0 4px 18px rgba(37,99,235,.3);}
.mc-cy{background:linear-gradient(135deg,#164e63,#0891b2);box-shadow:0 4px 18px rgba(6,182,212,.3);}
.mc-or{background:linear-gradient(135deg,#7c2d12,#ea580c);box-shadow:0 4px 18px rgba(234,88,12,.3);}
.mc-rd{background:linear-gradient(135deg,#7f1d1d,#dc2626);box-shadow:0 4px 18px rgba(220,38,38,.3);}
.mc-pu{background:linear-gradient(135deg,#4c1d95,#7c3aed);box-shadow:0 4px 18px rgba(124,58,237,.3);}
.mc-gr{background:linear-gradient(135deg,#14532d,#16a34a);box-shadow:0 4px 18px rgba(22,163,74,.3);}
.sev-none{background:linear-gradient(135deg,#1e293b,#334155);}
.sev-low{background:linear-gradient(135deg,#14532d,#16a34a);}
.sev-med{background:linear-gradient(135deg,#7c2d12,#ea580c);}
.sev-high{background:linear-gradient(135deg,#7f1d1d,#dc2626);animation:pulse 2s infinite;}

/* GEMINI CARD */
.gemini-card{background:rgba(15,23,42,.85);border:1.5px solid rgba(129,140,248,.35);
             border-radius:14px;padding:16px;margin-top:14px;box-shadow:0 8px 30px rgba(0,0,0,.5);}
.gemini-h{display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid rgba(255,255,255,.1);padding-bottom:10px;margin-bottom:12px;}
.gemini-title{font-size:1.05rem;font-weight:800;color:#818cf8;margin:0;}
.gemini-badge{padding:3px 10px;border-radius:12px;font-size:.72rem;font-weight:700;}

/* REGISTRY TABLE */
.reg{background:rgba(13,22,44,.7);border:1px solid rgba(6,182,212,.2);
     border-radius:12px;padding:14px;margin-top:14px;}
.reg-h{font-size:.87rem;font-weight:700;color:#38bdf8;margin-bottom:8px;}
.reg-empty{text-align:center;padding:22px;color:#64748b;font-size:.83rem;}
table.rt{width:100%;border-collapse:collapse;font-size:.74rem;text-align:left;}
table.rt th{background:rgba(30,41,59,.8);color:#94a3b8;padding:6px 9px;font-weight:600;}
table.rt td{padding:6px 9px;border-bottom:1px solid rgba(51,65,85,.4);color:#cbd5e1;}
.sp{padding:2px 8px;border-radius:10px;font-weight:700;font-size:.67rem;}
.sp-h{background:rgba(220,38,38,.25);color:#f87171;border:1px solid #ef4444;}
.sp-m{background:rgba(234,88,12,.25);color:#fb923c;border:1px solid #f97316;}
.sp-l{background:rgba(22,163,74,.25);color:#4ade80;border:1px solid #22c55e;}

@keyframes pdot{0%,100%{opacity:1;transform:scale(1);}50%{opacity:.5;transform:scale(.85);}}
@keyframes pulse{0%,100%{box-shadow:0 4px 18px rgba(220,38,38,.4);}
                 50%{box-shadow:0 4px 34px rgba(220,38,38,.95);}}
</style>
"""


def _card(ico, lbl, val, cls, sub="", sm=False):
    vc = "mv sm" if sm else "mv"
    s = f'<div class="ms">{sub}</div>' if sub else ""
    return (
        f'<div class="mc {cls}">'
        f'  <div class="ml">{ico} {lbl}</div>'
        f'  <div><div class="{vc}">{val}</div>{s}</div>'
        f'</div>'
    )


def _render_metrics(s: EngineState):
    cf = f"{s.avg_conf:.1f}%" if s.cracks_now > 0 else (
        f"{s.registry[-1].conf:.1f}% (last)" if s.registry else "--"
    )
    sw = (
        f"{s.max_width_mm:.2f} mm" if s.cracks_now > 0
        else (f"{s.registry[-1].width_mm:.2f} mm (last)" if s.registry else "--")
    )
    sl = (
        f"{s.max_len_mm:.1f} mm" if s.cracks_now > 0
        else (f"{s.registry[-1].length_mm:.1f} mm (last)" if s.registry else "--")
    )
    sc = {"NONE": "sev-none", "LOW": "sev-low", "MEDIUM": "sev-med", "HIGH": "sev-high"}.get(s.severity, "sev-none")
    rd = "mc-rd" if s.total_defects > 0 else "mc-cy"

    cracks_cls = "mc-rd" if s.cracks_now > 0 else "mc-bl"
    cracks_sub = f"🔴 {s.cracks_now} active detection(s)" if s.cracks_now > 0 else "Live YOLO inference active"

    fps_disp = f"{s.fps:.1f} FPS" if s.fps > 0 else "Warming up..."
    latency_disp = f"{s.latency_ms:.0f} ms" if s.latency_ms > 0 else "--"

    st.markdown(_card("🔍", "Cracks in Current View", str(s.cracks_now), cracks_cls, cracks_sub), unsafe_allow_html=True)
    st.markdown(_card("📊", "Total Defects Logged", str(s.total_defects), rd, f"{len(s.registry)} unique track IDs"), unsafe_allow_html=True)
    st.markdown(_card("🎯", "Model Confidence", cf, "mc-or", f"YOLO26n-seg · {s.device_name[:20]}"), unsafe_allow_html=True)
    st.markdown(_card("↔", "Peak Aperture Width", sw, "mc-rd", f"1 px ≈ {s.scale_px_to_mm} mm reference"), unsafe_allow_html=True)
    st.markdown(_card("📏", "Peak Crack Length", sl, "mc-cy", "Monocular pixel geometry"), unsafe_allow_html=True)
    st.markdown(_card("⚠️", "Structural Severity", s.severity, sc, f"Health Index: {s.health_idx:.1f}%"), unsafe_allow_html=True)
    st.markdown(_card("⚡", "Inference Latency & FPS", f"{latency_disp}  ·  {fps_disp}", "mc-gr", f"Device: {s.device_name[:22]}"), unsafe_allow_html=True)
    st.markdown(_card("🛠️", "Civil Remediation Guidance", s.action, "mc-pu", sm=True), unsafe_allow_html=True)


# ════════════════════════════════════════════════════════════
#   MAIN STREAMLIT APPLICATION
# ════════════════════════════════════════════════════════════
def main():
    st.set_page_config(
        page_title="BuildScan Rover AI — Structural Health Inspection",
        page_icon="🏗️",
        layout="wide",
        initial_sidebar_state="expanded",
    )

    # 1.5s refresh for metrics and telemetry (does not trigger Gemini or re-render camera stream)
    from streamlit_autorefresh import st_autorefresh
    st_autorefresh(interval=1500, limit=None, key="live_refresh")

    try:
        st.html(_CSS)
    except Exception:
        st.markdown(_CSS, unsafe_allow_html=True)

    _ensure_services()

    # Snapshot current thread state safely
    with _lock:
        s = EngineState(
            connected=_state.connected,
            video_source=_state.video_source,
            pi_ip=_state.pi_ip,
            pi_port=_state.pi_port,
            cracks_now=_state.cracks_now,
            total_defects=_state.total_defects,
            avg_conf=_state.avg_conf,
            max_width_mm=_state.max_width_mm,
            max_len_mm=_state.max_len_mm,
            severity=_state.severity,
            health_idx=_state.health_idx,
            action=_state.action,
            fps=_state.fps,
            latency_ms=_state.latency_ms,
            device_name=_state.device_name,
            is_crack_trained=_state.is_crack_trained,
            frame_ts=_state.frame_ts,
            frame_iso=_state.frame_iso,
            last_raw_frame=None if _state.last_raw_frame is None else _state.last_raw_frame.copy(),
            last_frame=None if _state.last_frame is None else _state.last_frame.copy(),
            last_jpeg=_state.last_jpeg,
            registry=list(_state.registry),
            yolo_conf=_state.yolo_conf,
            scale_px_to_mm=_state.scale_px_to_mm,
            health_info=_state.health_info,
        )

    # ── HEADER ───────────────────────────────────────────────
    conn_color = "#22c55e" if s.connected else "#ef4444"
    conn_text = "Pi Camera Live" if s.connected else "Pi Standby / Disconnected"

    st.markdown(f"""
    <div class="bsh">
      <div style="display:flex;align-items:center;">
        <div class="bsh-ico">🏗️</div>
        <div>
          <p class="bsh-title">BuildScan Rover AI &bull; Structural Health Inspection</p>
          <p class="bsh-sub">
            Real-Time Pi Camera Stream &bull; YOLO26n-seg Crack Detection &bull; Gemini Flash Visual Analysis
          </p>
        </div>
      </div>
      <div class="bsh-right">
        <div class="pill p-gpu">
          <div class="dot d-c"></div>{s.device_name[:22]}
        </div>
        <div class="pill p-live" style="border-color:{conn_color};color:{conn_color};">
          <div class="dot" style="background:{conn_color};box-shadow:0 0 8px {conn_color};"></div>
          {conn_text}
        </div>
      </div>
    </div>
    """, unsafe_allow_html=True)

    # ── STATUS BAR ───────────────────────────────────────────
    st.markdown(f"""
    <div class="sbar">
      <div class="si">{'🟢' if s.connected else '🔴'} <b>Source:</b> http://{s.pi_ip}:{s.pi_port}/video</div>
      <div class="si">⚡ <b>Inference Device:</b> {s.device_name[:20]}</div>
      <div class="si">⏱️ <b>Latency:</b> {s.latency_ms:.0f} ms ({s.fps:.1f} FPS)</div>
      <div class="si">🕒 <b>Frame Acquisition:</b> {s.frame_iso[11:] if s.frame_iso else '--'}</div>
      <div class="si">🛡️ <b>Model Verified:</b> {'Crack-Trained' if s.is_crack_trained else 'Generic Weights'}</div>
      <div class="si">🔢 <b>Logged Defects:</b> {s.total_defects}</div>
    </div>
    """, unsafe_allow_html=True)

    # ── SIDEBAR SETTINGS ─────────────────────────────────────
    with st.sidebar:
        st.markdown("### 🛰️ Rover Communication")
        new_ip = st.text_input("Raspberry Pi IP Address", value=s.pi_ip, help="Configurable Pi IP address")
        new_port = st.number_input("Camera Port", value=s.pi_port, min_value=1, max_value=65535)

        if new_ip != s.pi_ip or new_port != s.pi_port:
            with _lock:
                _state.pi_ip = new_ip
                _state.pi_port = int(new_port)
            st.rerun()

        st.markdown(f"**Endpoint:** `http://{s.pi_ip}:{s.pi_port}/video`")
        if s.health_info:
            h_open = s.health_info.get("camera_open", False)
            h_dev = s.health_info.get("device", "unknown")
            st.markdown(f"**Pi Health Status:** {'🟢 Camera Open (' + str(h_dev) + ')' if h_open else '🔴 Offline'}")
        else:
            st.markdown(f"**Pi Health Status:** {'🟢 Stream Active' if s.connected else '🔴 Standby'}")

        if st.button("🔄 Probe Pi Camera Reconnect", use_container_width=True):
            health = camera_receiver.check_pi_health(s.pi_ip, s.pi_port)
            if health.get("camera_open"):
                st.success(f"Pi Camera Healthy ({health.get('device')})")
            else:
                st.warning(f"Pi Health Probe: {health.get('error') or 'Camera device not open'}")
            st.rerun()

        st.markdown("---")
        st.markdown("### 🎯 Local YOLO AI Settings")
        conf_slider = st.slider("Confidence Threshold", min_value=0.10, max_value=0.90, value=float(s.yolo_conf), step=0.05)
        if conf_slider != s.yolo_conf:
            with _lock:
                _state.yolo_conf = conf_slider

        st.markdown(f"**Active Device:** `{s.device_name}`")
        st.markdown(f"**Model:** `models/yolo26n-seg.pt`")

        st.markdown("---")
        st.markdown("### 🧠 Gemini Flash Visual Analysis")
        enable_gemini = st.toggle("Enable Gemini Integration", value=True)
        chosen_gemini_model = st.selectbox(
            "Gemini Model",
            ["gemini-3.8-flash", "gemini-flash-latest", "gemini-2.5-flash", "gemini-2.0-flash"],
            index=0,
            help="Supported multimodal vision models"
        )
        api_key_set = gemini_analyzer.is_gemini_available()
        if api_key_set:
            st.caption("🟢 `GEMINI_API_KEY` configured in environment.")
        else:
            st.caption("⚠️ `GEMINI_API_KEY` not set. Export key or add to `.env`.")

        st.markdown("---")
        st.markdown("### 🚀 Pi Camera Startup Commands")
        st.markdown("**Native MJPEG Stream (brio_stream.py):**")
        st.code("python3 ~/brio_stream.py", language="bash")
        st.markdown("**ROS 2 Humble Stream:**")
        st.code("source /opt/ros/humble/setup.bash\nros2 launch buildscan_bringup camera_stream.launch.py", language="bash")

    # ── MAIN LAYOUT: VIDEO & METRICS ─────────────────────────
    col_vid, col_met = st.columns([3.2, 1.8], gap="medium")

    with col_vid:
        # Camera Source Selector
        src_opts = ["🛰️ Raspberry Pi 4 Camera", "💻 Laptop Local Camera", "🧪 Live Crack Simulation"]
        curr_idx = 0 if s.video_source == "pi" else (1 if s.video_source == "webcam" else 2)
        chosen_src = st.radio(
            "Video Source",
            src_opts,
            index=curr_idx,
            horizontal=True,
            help="Select live robot stream, local laptop webcam, or synthetic crack simulation"
        )
        new_src_mode = "pi" if "Raspberry Pi" in chosen_src else ("webcam" if "Laptop Local" in chosen_src else "sim")
        if new_src_mode != s.video_source:
            with _lock:
                _state.video_source = new_src_mode
                _state.connected = False
            st.rerun()

        # View Mode Selector (AI Annotated vs Raw)
        view_mode = st.radio(
            "View Mode",
            ["🎯 AI-Annotated Stream (YOLO26n-seg)", "📹 Raw Camera Feed"],
            horizontal=True,
            label_visibility="collapsed"
        )

        if "AI-Annotated" in view_mode:
            src = AI_STREAM
            badge = f"🔴 LIVE AI STREAM &bull; YOLO26n-seg ({s.latency_ms:.0f}ms)"
            note = f"Annotated at localhost:{AI_PORT} · {s.device_name}"
        else:
            if s.video_source == "pi":
                src = f"http://{s.pi_ip}:{s.pi_port}/video"
            else:
                src = AI_STREAM
            badge = "📹 RAW CAMERA FEED"
            note = f"Direct feed from {s.pi_ip}:{s.pi_port} (unprocessed)"

        st.markdown(f"""
        <div class="vbox">
          <div class="vbadge">{badge}</div>
          <img src="{src}"
               id="live-video-feed"
               onerror="if(!this.dataset.retried){{this.dataset.retried=1;this.src='http://127.0.0.1:{AI_PORT}/video_feed';}}"
               style="width:100%;display:block;aspect-ratio:4/3;
                      object-fit:contain;border-radius:12px;background:#040811;"
               alt="Live camera stream" />
        </div>
        <div class="vstatus">{note}</div>
        """, unsafe_allow_html=True)

        # Quick Control Buttons
        st.markdown("<div style='height:8px'></div>", unsafe_allow_html=True)
        b1, b2, b3 = st.columns([1.4, 1.4, 1.4])
        with b1:
            if st.button("🔄 Refresh View", use_container_width=True, type="primary"):
                st.rerun()
        with b2:
            if st.button("🧹 Reset Defect Count", use_container_width=True):
                with _lock:
                    _state.total_defects = 0
                    _state.registry.clear()
                    _state.max_width_mm = 0.0
                    _state.max_len_mm = 0.0
                st.rerun()
        with b3:
            if st.button("📄 Generate Audit PDF", use_container_width=True):
                g_res = st.session_state.get("gemini_result")
                st.session_state["pdf_data"] = _make_pdf(s, g_res)
                st.session_state["pdf_tag"] = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        if st.session_state.get("pdf_data"):
            st.download_button(
                "📥 Download Certified Inspection PDF",
                data=st.session_state["pdf_data"],
                file_name=f"BuildScan_Audit_{st.session_state.get('pdf_tag', 'report')}.pdf",
                mime="application/pdf",
                use_container_width=True,
            )

        # ── GEMINI FLASH VISUAL ANALYSIS PANEL ───────────────
        if enable_gemini:
            st.markdown("---")
            st.markdown("### 🧠 Gemini Flash Visual Analysis")
            st.caption("Perform selective, deep multimodal visual assessment on the current still inspection frame.")

            g_col1, g_col2 = st.columns([1.5, 2.5])
            with g_col1:
                analyze_btn = st.button("🔬 Analyze Current Frame with Gemini", type="secondary", use_container_width=True)

            if analyze_btn:
                # Capture current raw frame
                frame_to_analyze = s.last_raw_frame if s.last_raw_frame is not None else s.last_frame
                if frame_to_analyze is None:
                    st.warning("No camera frame available to analyze yet. Please ensure camera is connected.")
                else:
                    with st.spinner(f"Analyzing still frame with Google {chosen_gemini_model}..."):
                        t_call_start = time.time()
                        meta = {
                            "yolo_cracks_detected": s.cracks_now,
                            "yolo_severity": s.severity,
                            "acquisition_timestamp": s.frame_iso,
                            "device": s.device_name,
                        }
                        res = gemini_analyzer.analyze_inspection_frame(
                            frame_to_analyze,
                            metadata=meta,
                            model_name=chosen_gemini_model,
                            timeout_sec=15.0
                        )
                        st.session_state["gemini_result"] = res
                        st.session_state["gemini_analyzed_img"] = frame_to_analyze.copy()
                        st.session_state["gemini_call_duration"] = round(time.time() - t_call_start, 2)

            # Display previous or newly acquired Gemini result from session state
            saved_gemini = st.session_state.get("gemini_result")
            if saved_gemini is not None:
                if saved_gemini.success and saved_gemini.data:
                    data = saved_gemini.data
                    analyzed_img = st.session_state.get("gemini_analyzed_img")

                    st.markdown(f"""
                    <div class="gemini-card">
                      <div class="gemini-h">
                        <span class="gemini-title">🧠 Gemini Flash Inspection Analysis</span>
                        <span class="gemini-badge" style="background:#22c55e26;color:#4ade80;border:1px solid #22c55e;">
                          {saved_gemini.model} · {saved_gemini.timestamp[11:]}
                        </span>
                      </div>
                    </div>
                    """, unsafe_allow_html=True)

                    c_prev, c_diag = st.columns([1.6, 2.4])
                    with c_prev:
                        if analyzed_img is not None:
                            rgb_prev = cv2.cvtColor(analyzed_img, cv2.COLOR_BGR2RGB)
                            st.image(rgb_prev, caption="Analyzed Still Frame", use_container_width=True)

                    with c_diag:
                        is_crk = data.get("crack_visible", False)
                        crk_badge = "🔴 CRACK VISIBLE" if is_crk else "🟢 NO CRACK DETECTED"
                        st.markdown(f"**Structural Defect Status:** `{crk_badge}`")
                        st.markdown(f"**Defect Types:** {', '.join(data.get('visible_defects', ['none']))}")
                        st.markdown(f"**Approximate Location:** {data.get('approximate_location', 'N/A')}")
                        st.markdown(f"**Surface Material:** {data.get('possible_surface_material', 'N/A')}")
                        st.markdown(f"**Visual Confidence:** `{data.get('confidence_assessment', 'N/A')}`")

                    st.markdown(f"**Visual Description:** {data.get('visual_description', '')}")
                    st.markdown(f"**Engineering Next Step:** {data.get('recommended_next_step', '')}")
                    st.markdown(f"**Optical Limitations:** {data.get('limitations', '')}")

                    st.caption(f"⚠️ *{saved_gemini.disclaimer}*")
                else:
                    st.error(f"Gemini Analysis Request Failed: {saved_gemini.error}")
                    st.info("Ensure `GEMINI_API_KEY` is exported in your environment or defined in a local `.env` file.")

    with col_met:
        _render_metrics(s)

    # ── DEFECT REGISTRY TABLE ────────────────────────────────
    st.markdown("---")
    st.markdown('<div class="reg"><div class="reg-h">'
                '📋 REAL-TIME DEFECT REGISTRY &amp; STRUCTURAL ANOMALY LOG'
                '</div>', unsafe_allow_html=True)

    if s.registry:
        rows = ""
        for d in reversed(s.registry[-10:]):
            sc = ("sp-h" if d.severity == "HIGH"
                  else "sp-m" if d.severity == "MEDIUM" else "sp-l")
            rows += f"""
            <tr>
              <td><b>{d.defect_id}</b></td>
              <td>{d.timestamp}</td>
              <td>{d.width_mm:.2f} mm</td>
              <td>{d.length_mm:.1f} mm</td>
              <td>{d.conf:.0f}%</td>
              <td><span class="sp {sc}">{d.severity}</span></td>
              <td style="max-width:260px;word-wrap:break-word">{d.action}</td>
            </tr>"""
        st.markdown(f"""
        <table class="rt">
          <thead><tr>
            <th>DEFECT ID</th><th>TIME</th><th>APERTURE</th>
            <th>LENGTH</th><th>CONF</th><th>SEVERITY</th><th>REMEDIATION ACTION</th>
          </tr></thead>
          <tbody>{rows}</tbody>
        </table>
        """, unsafe_allow_html=True)
    else:
        st.markdown(
            '<div class="reg-empty">'
            '🔍 No cracks logged in registry yet. '
            'When the Pi camera detects structural cracks, '
            'YOLO26n-seg will segment them live and register them here automatically.'
            '</div>', unsafe_allow_html=True)

    st.markdown("</div>", unsafe_allow_html=True)


if __name__ == "__main__":
    main()

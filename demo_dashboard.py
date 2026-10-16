#!/usr/bin/env python3
"""
demo_dashboard.py  -  BuildScan Rover AI Dashboard (Standalone Demo)
Runs on Windows without ROS 2.
Two modes:
  * Start Camera -- Opens webcam, runs real YOLO26n-seg crack detection
  * Demo Mode    -- Animated synthetic crack data (no camera / hardware needed)

Run:
    streamlit run demo_dashboard.py
"""
import io, os, math, random, time, uuid, tempfile, datetime, threading, dataclasses
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import streamlit as st
from PIL import Image as PILImage

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# PDF
try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas as rl_canvas
    from reportlab.lib.colors import HexColor, white
    from reportlab.lib.utils import ImageReader
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False

# ---------- Constants ----------
SEV_LOW_MAX_WIDTH    = 70
SEV_MEDIUM_MAX_WIDTH = 150
REPAIR_NONE   = "No crack detected"
REPAIR_LOW    = "Apply surface filler and sealant"
REPAIR_MEDIUM = "Use epoxy injection repair"
REPAIR_HIGH   = "Urgent structural inspection required"

_THIS_DIR   = Path(__file__).resolve().parent
_MODEL_PATH = _THIS_DIR / "models" / "yolo26n-seg.pt"
_REPORT_DIR = Path(tempfile.gettempdir()) / "buildscan_reports"
_REPORT_DIR.mkdir(parents=True, exist_ok=True)
_CAM_IDX          = int(os.environ.get("BUILDSCAN_CAM_IDX", "0"))
_REFRESH_INTERVAL = 0.30   # seconds


# ---------- Data container ----------
@dataclasses.dataclass
class DetectionSnapshot:
    annotated_frame: Optional[np.ndarray] = None
    original_frame:  Optional[np.ndarray] = None
    cracks_detected:    int   = 0
    average_confidence: float = 0.0
    maximum_width:      float = 0.0
    total_length:       float = 0.0
    severity:           str   = "NONE"
    recommendation:     str   = REPAIR_NONE
    model_name:         str   = "YOLO26n-seg"
    model_version:      str   = "1.0.0"
    timestamp: Optional[datetime.datetime] = None


# ---------- Severity helpers ----------
def classify_severity(max_width_px: float, crack_count: int):
    if crack_count == 0 or max_width_px == 0:
        return "NONE", REPAIR_NONE
    if max_width_px < SEV_LOW_MAX_WIDTH:
        return "LOW", REPAIR_LOW
    if max_width_px < SEV_MEDIUM_MAX_WIDTH:
        return "MEDIUM", REPAIR_MEDIUM
    return "HIGH", REPAIR_HIGH


# ---------- Model loader (cached) ----------
@st.cache_resource(show_spinner="Loading YOLO26n-seg model...")
def load_model():
    if not YOLO_AVAILABLE or not _MODEL_PATH.exists():
        return None
    try:
        return YOLO(str(_MODEL_PATH))
    except Exception:
        return None


# ---------- YOLO frame analysis ----------
def analyze_frame(model, frame: np.ndarray) -> DetectionSnapshot:
    snap = DetectionSnapshot(
        original_frame=frame.copy(),
        timestamp=datetime.datetime.now(),
    )
    if model is None:
        snap.annotated_frame = frame.copy()
        return snap

    results  = model(frame, conf=0.30, imgsz=320, verbose=False)
    annotated = frame.copy()
    max_width = total_len = 0.0
    confs: list = []
    count = 0

    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            w = x2 - x1
            h = y2 - y1
            max_width = max(max_width, w)
            total_len += math.sqrt(w**2 + h**2)
            confs.append(conf)
            count += 1
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(annotated, f"Crack {conf:.0%}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    # HUD bar
    h_img, w_img = annotated.shape[:2]
    ov = annotated.copy()
    cv2.rectangle(ov, (0, 0), (w_img, 36), (20, 20, 20), -1)
    cv2.addWeighted(ov, 0.6, annotated, 0.4, 0, annotated)
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(annotated,
                "BuildScan Rover  |  " + ts + "  |  Cracks: " + str(count),
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    sev, rec = classify_severity(max_width, count)
    snap.annotated_frame    = annotated
    snap.cracks_detected    = count
    snap.average_confidence = (sum(confs) / len(confs) * 100) if confs else 0.0
    snap.maximum_width      = max_width
    snap.total_length       = total_len
    snap.severity           = sev
    snap.recommendation     = rec
    return snap


# ---------- Demo mode synthetic frames ----------
def _make_demo_frame(tick: int) -> DetectionSnapshot:
    h, w = 480, 640
    frame = np.full((h, w, 3), (140, 138, 135), dtype=np.uint8)
    noise = np.random.randint(-20, 20, (h, w, 3), dtype=np.int16)
    frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    t = tick * 0.04
    cracks = [
        (int(120 + 10 * math.sin(t)),     180, int(290 + 8 * math.cos(t)),       210, 0.87),
        (int(340 +  6 * math.sin(t + 1)), 260, int(480 + 5 * math.cos(t + 1)),   310, 0.72),
        (int( 80 +  4 * math.sin(t + 2)), 360, int(200 + 6 * math.cos(t + 2)),   395, 0.61),
    ]

    annotated = frame.copy()
    cv2.rectangle(annotated, (0, 0), (w, 36), (20, 20, 20), -1)
    ov = annotated.copy()
    cv2.addWeighted(ov, 0.6, annotated, 0.4, 0, annotated)

    max_width = total_len = 0.0
    confs: list = []
    for (x1, y1, x2, y2, conf) in cracks:
        bw = x2 - x1
        bl = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        max_width = max(max_width, bw)
        total_len += bl
        confs.append(conf)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.line(annotated,
                 (x1 + random.randint(0, 5), y1 + random.randint(0, 5)),
                 (x2 - random.randint(0, 5), y2 - random.randint(0, 5)),
                 (30, 30, 200), 1)
        cv2.putText(annotated, f"Crack {conf:.0%}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)

    ts = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(annotated,
                "BuildScan Rover  |  " + ts + "  |  Cracks: " + str(len(cracks)) + "  [DEMO]",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 220, 60), 1)

    sev, rec = classify_severity(max_width, len(cracks))
    return DetectionSnapshot(
        annotated_frame=annotated, original_frame=frame,
        cracks_detected=len(cracks),
        average_confidence=(sum(confs) / len(confs) * 100) if confs else 0.0,
        maximum_width=max_width, total_length=total_len,
        severity=sev, recommendation=rec,
        timestamp=datetime.datetime.now(),
    )


# ---------- Streaming threads ----------
def _camera_loop(stop_event: threading.Event, source):
    model = load_model()
    cap   = cv2.VideoCapture(source)
    if not cap.isOpened():
        src_label = f"Pi stream ({source})" if isinstance(source, str) else f"camera {source}"
        st.session_state["stream_error"] = (
            f"Cannot connect to {src_label}. Check that the Pi camera node and web_video_server are running.")
        st.session_state["streaming"] = False
        stop_event.set()
        return
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        st.session_state["snapshot"] = analyze_frame(model, frame)
        time.sleep(0.04)
    cap.release()


def _demo_loop(stop_event: threading.Event):
    tick = 0
    while not stop_event.is_set():
        st.session_state["snapshot"] = _make_demo_frame(tick)
        tick += 1
        time.sleep(0.12)


def start_stream(mode: str = "webcam", pi_ip: str = "10.120.68.129", pi_topic: str = "/camera/image_raw"):
    if st.session_state.get("streaming"):
        return
    ev = threading.Event()
    is_demo = (mode == "demo")
    st.session_state.update({
        "stop_event": ev, "streaming": True,
        "stream_mode": mode,
        "demo_mode": is_demo, "stream_error": None,
        "snapshot": DetectionSnapshot(),
    })
    
    if mode == "demo":
        target = _demo_loop
        args = (ev,)
    elif mode == "pi":
        # Connect to Raspberry Pi web_video_server stream
        clean_topic = pi_topic if pi_topic.startswith("/") else f"/{pi_topic}"
        stream_url = f"http://{pi_ip.strip()}:8080/stream?topic={clean_topic}"
        target = _camera_loop
        args = (ev, stream_url)
    else: # local webcam
        target = _camera_loop
        args = (ev, _CAM_IDX)

    t = threading.Thread(target=target, args=args, daemon=True)
    t.start()
    st.session_state["stream_thread"] = t


def stop_stream():
    if not st.session_state.get("streaming"):
        return
    ev = st.session_state.get("stop_event")
    if ev:
        ev.set()
    t = st.session_state.get("stream_thread")
    if t and t.is_alive():
        t.join(timeout=3.0)
    st.session_state.update({
        "streaming": False, "stop_event": None, "stream_thread": None})


# ---------- PDF export ----------
def export_pdf(snap: DetectionSnapshot) -> bytes:
    buf = io.BytesIO()
    if not REPORTLAB_AVAILABLE:
        import json
        return json.dumps({
            "timestamp": snap.timestamp.isoformat() if snap.timestamp else "",
            "cracks_detected": snap.cracks_detected,
            "average_confidence": round(snap.average_confidence, 2),
            "maximum_width_px": round(snap.maximum_width, 1),
            "total_length_px": round(snap.total_length, 1),
            "severity": snap.severity,
            "recommendation": snap.recommendation,
        }, indent=2).encode()

    ann_path = ""
    if snap.annotated_frame is not None:
        tmp = tempfile.NamedTemporaryFile(
            delete=False, suffix=".jpg", prefix="buildscan_")
        cv2.imwrite(tmp.name, snap.annotated_frame)
        ann_path = tmp.name

    c  = rl_canvas.Canvas(buf, pagesize=A4)
    pw, ph = A4

    BLUE   = HexColor("#2563eb"); GREEN  = HexColor("#16a34a")
    ORANGE = HexColor("#ea580c"); RED    = HexColor("#dc2626")
    PURPLE = HexColor("#7c3aed"); ACCENT = HexColor("#1e3a8a")
    BG     = HexColor("#f4f4f4")
    SEV_C  = {"NONE": HexColor("#6b7280"), "LOW": GREEN,
               "MEDIUM": ORANGE, "HIGH": RED}

    c.setFillColor(BG); c.rect(0, 0, pw, ph, fill=1, stroke=0)
    c.setFillColor(ACCENT); c.rect(0, ph - 80, pw, 80, fill=1, stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold", 22)
    c.drawString(30, ph - 45, "BuildScan Rover -- Inspection Report")
    c.setFont("Helvetica", 11)
    ts_str = snap.timestamp.strftime("%d-%m-%Y  %I:%M %p") if snap.timestamp else ""
    c.drawString(30, ph - 65, ts_str)
    iid = str(uuid.uuid4())[:8].upper()
    c.setFont("Helvetica-Bold", 9)
    c.drawRightString(pw - 20, ph - 45, "ID: " + iid)

    sev_col = SEV_C.get(snap.severity, RED)
    c.setFillColor(sev_col); c.roundRect(pw - 160, ph - 135, 130, 35, 8, fill=1, stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold", 15)
    c.drawCentredString(pw - 95, ph - 123, snap.severity)

    c.setFillColor(HexColor("#111827")); c.setFont("Helvetica-Bold", 11)
    c.drawString(30, ph - 115, "Project: BUILDScan Rover -- AI Structural Health Inspection")
    c.setFont("Helvetica", 10); c.setFillColor(HexColor("#374151"))
    c.drawString(30, ph - 132,
        "Model: " + snap.model_name + " v" + snap.model_version +
        "   |   Confidence Threshold: 30%")

    def card(cx, cy, color, title, value, cw=130, ch=48):
        c.setFillColor(color); c.roundRect(cx, cy, cw, ch, 8, fill=1, stroke=0)
        c.setFillColor(white)
        c.setFont("Helvetica-Bold", 10); c.drawString(cx + 8, cy + 30, title)
        c.setFont("Helvetica", 12);      c.drawString(cx + 8, cy + 10, str(value))

    cy1 = ph - 200
    card(30,  cy1, BLUE,   "Cracks Detected",   str(snap.cracks_detected))
    card(175, cy1, ORANGE, "Confidence",         f"{snap.average_confidence:.1f}%")
    card(320, cy1, RED,    "Max Width",          f"{snap.maximum_width:.0f} px")
    card(465, cy1, PURPLE, "Total Length",       f"{snap.total_length:.0f} px")

    cy2 = cy1 - 70
    card(30,  cy2, GREEN,  "Severity",           snap.severity)
    rec_short = (snap.recommendation[:22] + "..."
                 if len(snap.recommendation) > 22 else snap.recommendation)
    card(175, cy2, ACCENT, "Repair Solution",    rec_short, cw=310)

    img_y = cy2 - 220
    if ann_path and os.path.exists(ann_path):
        c.setFillColor(HexColor("#111827")); c.setFont("Helvetica-Bold", 10)
        c.drawString(30, img_y + 205,
                     "Crack Detection Frame (YOLO26n-seg annotated)")
        try:
            c.drawImage(ImageReader(ann_path),
                        30, img_y, width=pw - 60, height=195,
                        preserveAspectRatio=True)
        except Exception:
            pass

    rec_y = img_y - 55
    c.setFillColor(sev_col); c.roundRect(30, rec_y, pw - 60, 40, 8, fill=1, stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold", 11)
    c.drawString(42, rec_y + 24, "Engineering Recommendation:")
    c.setFont("Helvetica", 10); c.drawString(42, rec_y + 8, snap.recommendation)

    c.setStrokeColor(ACCENT); c.line(30, 45, pw - 30, 45)
    c.setFillColor(ACCENT); c.setFont("Helvetica-Oblique", 8)
    c.drawString(30, 30,
        "Generated by BuildScan Rover -- AI Structural Health Inspection System")
    c.drawRightString(pw - 30, 30, "Report ID: " + iid)
    c.save(); buf.seek(0)
    return buf.read()


# ---------- CSS ----------
_CSS = """<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700;800&display=swap');
html,body,[class*="css"]{font-family:'Inter',sans-serif;background-color:#ffffff;}
.bs-header{display:flex;align-items:center;gap:14px;padding:18px 0 10px 0;border-bottom:2px solid #e5e7eb;margin-bottom:20px;}
.bs-header-icon{font-size:2.4rem;line-height:1;}
.bs-header-title{font-size:1.75rem;font-weight:800;color:#111827;letter-spacing:-0.5px;margin:0;}
.bs-header-sub{font-size:0.82rem;color:#6b7280;margin:0;font-weight:400;}
.bs-mode-badge{margin-left:auto;padding:4px 12px;border-radius:20px;font-size:0.78rem;font-weight:700;letter-spacing:0.5px;}
.bs-mode-cam{background:#dbeafe;color:#1d4ed8;}
.bs-mode-demo{background:#fef3c7;color:#92400e;}
.bs-mode-idle{background:#f3f4f6;color:#6b7280;}
.bs-image-panel{border:2px solid #e5e7eb;border-radius:16px;padding:14px;background:#f9fafb;box-shadow:0 2px 12px rgba(0,0,0,0.06);margin-bottom:26px;text-align:center;}
.bs-image-caption{margin-top:10px;font-size:0.82rem;color:#6b7280;font-weight:600;}
.bs-metric-card{border-radius:16px;padding:18px 20px 16px 20px;color:#fff;box-shadow:0 4px 16px rgba(0,0,0,0.13);min-height:100px;display:flex;flex-direction:column;justify-content:space-between;margin-bottom:14px;transition:transform 0.15s ease,box-shadow 0.15s ease;}
.bs-metric-card:hover{transform:translateY(-2px);box-shadow:0 8px 24px rgba(0,0,0,0.18);}
.bs-metric-label{font-size:0.78rem;font-weight:700;letter-spacing:0.8px;text-transform:uppercase;opacity:0.88;}
.bs-metric-value{font-size:1.6rem;font-weight:800;letter-spacing:-0.5px;line-height:1.1;margin-top:10px;word-break:break-word;}
.bs-metric-value.small{font-size:1.1rem;}
.card-blue{background:linear-gradient(135deg,#2563eb,#1d4ed8);}
.card-orange{background:linear-gradient(135deg,#ea580c,#c2410c);}
.card-red{background:linear-gradient(135deg,#dc2626,#b91c1c);}
.card-green{background:linear-gradient(135deg,#16a34a,#15803d);}
.card-purple{background:linear-gradient(135deg,#7c3aed,#6d28d9);}
.bs-download-row .stDownloadButton>button{background:transparent !important;border:2px solid #2563eb !important;color:#2563eb !important;border-radius:12px !important;font-weight:700 !important;font-size:0.95rem !important;padding:10px 28px !important;transition:all 0.18s ease !important;}
.bs-download-row .stDownloadButton>button:hover{background:#2563eb !important;color:#fff !important;}
.bs-status{background:#f0fdf4;border-left:4px solid #16a34a;border-radius:8px;padding:8px 14px;font-size:0.82rem;color:#15803d;font-weight:600;margin-bottom:18px;}
.bs-status.warn{background:#fef3c7;border-color:#d97706;color:#92400e;}
.bs-status.error{background:#fef2f2;border-color:#dc2626;color:#b91c1c;}
.bs-no-data{border:2px dashed #d1d5db;border-radius:14px;padding:60px 20px;text-align:center;color:#9ca3af;background:#f9fafb;font-size:1rem;}
.bs-info-box{background:#eff6ff;border:1px solid #bfdbfe;border-radius:10px;padding:10px 16px;font-size:0.82rem;color:#1e40af;margin-bottom:16px;}
div[data-testid="stButton"]>button[kind="primary"]{border-radius:10px !important;font-weight:700 !important;padding:8px 22px !important;}
div[data-testid="stButton"]>button[kind="secondary"]{border-radius:10px !important;font-weight:600 !important;padding:8px 22px !important;}
</style>"""


# ---------- UI helpers ----------
def _sev_color(s: str) -> str:
    return {"NONE": "card-blue", "LOW": "card-green",
            "MEDIUM": "card-orange", "HIGH": "card-red"}.get(s, "card-purple")


def _card(icon: str, label: str, value: str,
          cls: str, small: bool = False) -> str:
    vc = "bs-metric-value small" if small else "bs-metric-value"
    return (f'<div class="bs-metric-card {cls}">'
            f'<div class="bs-metric-label">{icon} {label}</div>'
            f'<div class="{vc}">{value}</div></div>')


def _bgr2pil(bgr: np.ndarray) -> PILImage.Image:
    return PILImage.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))


# ---------- Main Streamlit App ----------
def main():
    st.set_page_config(
        page_title="BuildScan Rover AI Dashboard",
        page_icon="🏗️", layout="wide",
        initial_sidebar_state="collapsed",
    )
    # Inject CSS — use st.html if available (Streamlit>=1.31), else markdown
    try:
        st.html(_CSS)
    except AttributeError:
        st.markdown(_CSS, unsafe_allow_html=True)

    # Session defaults
    for k, v in [("streaming", False), ("demo_mode", False), ("stream_mode", "idle"),
                  ("snapshot", DetectionSnapshot()), ("stop_event", None),
                  ("stream_thread", None), ("stream_error", None),
                  ("pi_ip", "10.120.68.129"), ("pi_topic", "/image_raw")]:
        st.session_state.setdefault(k, v)

    snap:        DetectionSnapshot = st.session_state["snapshot"]
    is_demo:     bool              = st.session_state.get("demo_mode", False)
    streaming:   bool              = st.session_state.get("streaming", False)
    stream_mode: str               = st.session_state.get("stream_mode", "idle")

    # Header
    if streaming and stream_mode == "pi":
        ml, mc = "Raspberry Pi Stream", "bs-mode-cam"
    elif streaming and is_demo:
        ml, mc = "Demo Mode", "bs-mode-demo"
    elif streaming:
        ml, mc = "Webcam Live", "bs-mode-cam"
    else:
        ml, mc = "Idle", "bs-mode-idle"

    st.markdown(
        f'<div class="bs-header">'
        f'<div class="bs-header-icon">🏗️</div>'
        f'<div><p class="bs-header-title">BuildScan Rover AI Dashboard</p>'
        f'<p class="bs-header-sub">AI-Assisted Structural Crack Detection &nbsp;•&nbsp; YOLO26n-seg &nbsp;•&nbsp; Distributed System</p></div>'
        f'<span class="bs-mode-badge {mc}">{ml}</span></div>',
        unsafe_allow_html=True)

    # Info banner
    ms = ("✅ YOLO26n-seg loaded" if (YOLO_AVAILABLE and _MODEL_PATH.exists())
          else "⚠️ YOLO model not found — use Demo Mode")
    ps = "✅ PDF export ready" if REPORTLAB_AVAILABLE else "⚠️ ReportLab missing"
    st.markdown(
        f'<div class="bs-info-box">🖥️ <strong>Host Dashboard (Laptop — Windows)</strong>'
        f'&nbsp;|&nbsp; {ms} &nbsp;|&nbsp; {ps} &nbsp;|&nbsp; 📡 Pi IP: <code>{st.session_state.pi_ip}</code></div>',
        unsafe_allow_html=True)

    # Status bar
    err = st.session_state.get("stream_error")
    if err:
        st.markdown(f'<div class="bs-status error">⚠ {err}</div>',
                    unsafe_allow_html=True)
    elif streaming and stream_mode == "pi":
        st.markdown(
            f'<div class="bs-status">'
            f'● Streaming live from Raspberry Pi ({st.session_state.pi_ip}) — YOLO26n-seg running</div>',
            unsafe_allow_html=True)
    elif streaming and is_demo:
        st.markdown(
            '<div class="bs-status warn">'
            '🧪 Simulating crack detection — no real camera needed</div>',
            unsafe_allow_html=True)
    elif streaming:
        st.markdown(
            '<div class="bs-status">'
            '● Streaming live from local webcam — YOLO26n-seg running</div>',
            unsafe_allow_html=True)
    else:
        st.markdown(
            '<div class="bs-status warn">'
            '◌ Stopped — connect to 📡 Pi Camera, ▶ Local Webcam, or 🧪 Demo Mode</div>',
            unsafe_allow_html=True)

    # Pi Configuration Expander
    with st.expander("⚙️ Raspberry Pi Connection Settings", expanded=(not streaming)):
        c_ip, c_top = st.columns([1, 1])
        with c_ip:
            st.session_state["pi_ip"] = st.text_input("Raspberry Pi IP Address", value=st.session_state.get("pi_ip", "10.120.68.129"))
        with c_top:
            st.session_state["pi_topic"] = st.text_input("Camera ROS Topic", value=st.session_state.get("pi_topic", "/camera/image_raw"))
        st.caption("Make sure <code>web_video_server</code> is running on port 8080 of your Pi.")

    # Controls
    c1, c2, c3, c4 = st.columns([2.5, 2, 2, 1.5])
    with c1:
        if st.button("📡  Connect Pi Camera", key="btn_pi", type="primary",
                     disabled=streaming, use_container_width=True):
            start_stream(mode="pi", pi_ip=st.session_state.pi_ip, pi_topic=st.session_state.pi_topic)
            st.rerun()
    with c2:
        if st.button("▶  Local Webcam", key="btn_cam", type="secondary",
                     disabled=streaming, use_container_width=True):
            start_stream(mode="webcam")
            st.rerun()
    with c3:
        if st.button("🧪  Demo Mode", key="btn_demo", type="secondary",
                     disabled=streaming, use_container_width=True):
            start_stream(mode="demo")
            st.rerun()
    with c4:
        if st.button("■  Stop", key="btn_stop", type="secondary",
                     disabled=not streaming, use_container_width=True):
            stop_stream()
            st.rerun()

    st.markdown("<div style='margin-bottom:6px'></div>", unsafe_allow_html=True)

    # Two-column layout: image | metrics
    col_img, col_m = st.columns([3, 2])
    with col_img:
        st.markdown('<div class="bs-image-panel">', unsafe_allow_html=True)
        if snap.annotated_frame is not None:
            st.image(_bgr2pil(snap.annotated_frame), use_container_width=True)
            ts_s = snap.timestamp.strftime("%H:%M:%S") if snap.timestamp else ""
            st.markdown(
                f'<div class="bs-image-caption">🔍 Last Crack Detection &nbsp;•&nbsp; {ts_s}</div>',
                unsafe_allow_html=True)
        else:
            st.markdown(
                '<div class="bs-no-data">'
                '<div style="font-size:3rem;margin-bottom:12px">📷</div>'
                '<div style="font-weight:600;color:#374151;">No frame yet</div>'
                '<div style="font-size:0.85rem;margin-top:6px;">'
                'Press <strong>▶ Start Camera</strong> or <strong>🧪 Demo Mode</strong></div>'
                '</div>', unsafe_allow_html=True)
        st.markdown('</div>', unsafe_allow_html=True)

    with col_m:
        cf = f"{snap.average_confidence:.1f}%" if snap.cracks_detected else "—"
        sl = f"{snap.total_length:.0f} px"     if snap.cracks_detected else "— px"
        sw = f"{snap.maximum_width:.0f} px"    if snap.cracks_detected else "— px"
        st.markdown(_card("🔍", "Cracks Found",    str(snap.cracks_detected), "card-blue"),   unsafe_allow_html=True)
        st.markdown(_card("🎯", "Confidence",      cf,                        "card-orange"),  unsafe_allow_html=True)
        st.markdown(_card("↔",  "Max Width",       sw,                        "card-green"),   unsafe_allow_html=True)
        st.markdown(_card("📏", "Total Length",    sl,                        "card-blue"),    unsafe_allow_html=True)
        st.markdown(_card("⚠",  "Severity",        snap.severity,             _sev_color(snap.severity)), unsafe_allow_html=True)
        st.markdown(_card("🛠",  "Repair Solution", snap.recommendation,       "card-red", small=True), unsafe_allow_html=True)

    # Download
    st.markdown("---")
    has_r = snap.cracks_detected > 0 or snap.annotated_frame is not None
    dl, _ = st.columns([2, 8])
    with dl:
        st.markdown('<div class="bs-download-row">', unsafe_allow_html=True)
        if has_r:
            try:
                pdf  = export_pdf(snap)
                ext  = "pdf" if pdf[:4] == b"%PDF" else "json"
                tag  = (snap.timestamp.strftime("%Y%m%d_%H%M%S")
                        if snap.timestamp else "export")
                st.download_button(
                    label="📥  Download Inspection Report",
                    data=pdf,
                    file_name=f"BuildScan_{tag}.{ext}",
                    mime="application/pdf" if ext == "pdf" else "application/json",
                    key="btn_dl")
            except Exception as e:
                st.warning(f"Export error: {e}")
        else:
            st.download_button(
                label="📥  Download Inspection Report",
                data=b"", file_name="BuildScan_no_data.pdf",
                disabled=True, key="btn_dl_dis")
        st.markdown('</div>', unsafe_allow_html=True)

    # About
    with st.expander("ℹ️  About this Dashboard & Architecture"):
        st.markdown("""
**BUILDScan Rover** — distributed AI robotic system for structural health inspection.

| Component | Device | Role |
|---|---|---|
| ROS 2 Humble | Raspberry Pi 4 | Robot coordination |
| micro-ROS Agent | Raspberry Pi 4 | USB bridge to ESP32 |
| v4l2_camera_node | Raspberry Pi 4 | Camera → `/camera/image_raw` |
| safety_node | Raspberry Pi 4 | Ultrasonic E-Stop |
| motor_interface_node | Raspberry Pi 4 | Velocity clamp & watchdog |
| ESP32 micro-ROS | ESP32 DevKit | L298N motors, HC-SR04 |
| **YOLO26n-seg** | **This Dell G15 laptop** | Real-time crack detection |
| **inspection_manager** | **This laptop** | InspectArea Action Server |
| **Streamlit Dashboard** | **This laptop** | Operator UI + PDF reports |

**Severity bands:**
- 🟢 LOW — crack width < 70 px → Surface filler and sealant
- 🟠 MEDIUM — crack width < 150 px → Epoxy injection repair
- 🔴 HIGH — crack width ≥ 150 px → Urgent structural inspection
        """)

    # Auto-refresh
    if streaming:
        time.sleep(_REFRESH_INTERVAL)
        st.rerun()


if __name__ == "__main__":
    main()

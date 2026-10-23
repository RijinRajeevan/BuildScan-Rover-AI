"""
BuildScan Rover AI  ·  Live Structural Health Inspection
=========================================================
Architecture:
  Raspberry Pi 4  →  Logitech Brio 100  →  ROS2 web_video_server (port 8080)
           |
      Wi-Fi / LAN
           |
  Laptop  →  YOLO26n-seg on RTX 3050 GPU  →  Annotated MJPEG (port 8081)
           |
  Streamlit Dashboard  →  Browser (live stream + metrics + defect log)
"""

import os, sys, io, time, math, datetime, tempfile, threading
import urllib.request, json, random, logging
from dataclasses import dataclass, field
from typing import List, Optional
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

import cv2, numpy as np, streamlit as st
from PIL import Image as PILImage

logging.basicConfig(level=logging.INFO,
                    format="[BuildScan %(asctime)s] %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("buildscan")

# ── PyTorch / YOLO ──────────────────────────────────────────
try:
    import torch
    from ultralytics import YOLO
    _CUDA = torch.cuda.is_available()
    GPU_NAME = torch.cuda.get_device_name(0) if _CUDA else "CPU"
    _DEVICE = 0 if _CUDA else "cpu"
    YOLO_OK = True
except Exception:
    _CUDA = False; GPU_NAME = "CPU"; _DEVICE = "cpu"; YOLO_OK = False

# ── ReportLab (optional PDF) ────────────────────────────────
try:
    from reportlab.lib.pagesizes import letter
    from reportlab.lib import colors
    from reportlab.lib.units import inch
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer,
                                    Table, TableStyle, Image as RLImage, HRFlowable)
    PDF_OK = True
except ImportError:
    PDF_OK = False

# ── Constants ────────────────────────────────────────────────
PI_IP          = os.getenv("BUILDSCAN_PI_IP", "10.181.155.129")
PI_PORT        = int(os.getenv("BUILDSCAN_PI_PORT", "8080"))
PI_STREAM      = f"http://{PI_IP}:{PI_PORT}/stream?topic=/image_raw"
PI_SNAP        = f"http://{PI_IP}:{PI_PORT}/snapshot?topic=/image_raw"
AI_PORT        = 8081
AI_STREAM      = f"http://127.0.0.1:{AI_PORT}/video_feed"
MODEL_PATH     = Path("models/yolo26n-seg.pt")
PX_MM          = 0.025          # 1 pixel ≈ 0.025 mm (calibrated)
CONF_THRESH    = 0.25
TRACK_RADIUS   = 65             # px  — spatial dedup radius
TRACK_TTL      = 4.0            # sec — how long a track lives without a match

# Civil recommendations
_R = {
    "LOW":    "Apply surface sealant. Monitor periodically. (< 0.30 mm aperture)",
    "MEDIUM": "Low-pressure epoxy resin injection recommended. (0.30–0.55 mm)",
    "HIGH":   "⚠ Immediate structural engineering assessment required! (≥ 0.55 mm)",
}

# ── Data Classes ─────────────────────────────────────────────
@dataclass
class Defect:
    defect_id: str
    timestamp: str
    width_mm:  float
    width_px:  float
    length_mm: float
    conf:      float
    severity:  str
    action:    str
    bbox:      tuple = (0, 0, 0, 0)


@dataclass
class EngineState:
    connected:     bool  = False
    cracks_now:    int   = 0
    total_defects: int   = 0
    avg_conf:      float = 0.0
    max_width_mm:  float = 0.0
    max_len_mm:    float = 0.0
    severity:      str   = "NONE"
    health_idx:    float = 100.0
    action:        str   = "System initialising — connecting to Pi camera stream..."
    fps:           float = 0.0
    last_frame:    Optional[np.ndarray] = None
    last_jpeg:     Optional[bytes]      = None
    registry:      List[Defect]         = field(default_factory=list)

# ── Shared Process-Level State (process-level singleton across reruns) ───
if not hasattr(sys, "_buildscan_state"):
    sys._buildscan_state = EngineState()
    sys._buildscan_lock = threading.Lock()
    sys._buildscan_svc_started = False

_state = sys._buildscan_state
_lock  = sys._buildscan_lock


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
                    if jpg:
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
            if jpg:
                self.send_response(200)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
            else:
                self.send_error(503, "No frame available yet")
        elif self.path == "/healthz":
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with _lock:
                body = json.dumps({
                    "connected": _state.connected,
                    "fps": round(_state.fps, 1),
                    "cracks_now": _state.cracks_now,
                    "total": _state.total_defects,
                }).encode()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def log_message(self, *_):
        pass   # silence HTTP server noise


def _run_mjpeg_server():
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", AI_PORT), _MJPEGHandler)
        srv.daemon_threads = True
        log.info(f"MJPEG server started → http://0.0.0.0:{AI_PORT}/video_feed")
        srv.serve_forever()
    except OSError as e:
        log.warning(f"MJPEG server error (port already open?): {e}")


# ════════════════════════════════════════════════════════════
#   LIVE GPU INFERENCE WORKER  (runs in a daemon thread)
# ════════════════════════════════════════════════════════════
def _severity(aperture_px: float, n: int):
    if n == 0:
        return "NONE", "No structural defects visible. Concrete surface nominal."
    w = aperture_px * PX_MM
    if   w < 0.30: return "LOW",    _R["LOW"]
    elif w < 0.55: return "MEDIUM", _R["MEDIUM"]
    else:          return "HIGH",   _R["HIGH"]


def _run_inference():
    """
    Permanent daemon thread.
    Connects to Pi stream, pulls frames, runs YOLO on GPU,
    annotates and stores latest JPEG + metrics in _state.
    """
    global _state

    # ── Load YOLO model once ────────────────────────────────
    model = None
    if YOLO_OK and MODEL_PATH.exists():
        try:
            model = YOLO(str(MODEL_PATH))
            # Warm-up inference to initialise CUDA graph
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            model(dummy, conf=CONF_THRESH, device=_DEVICE, verbose=False)
            log.info(f"YOLO26n-seg loaded — device={_DEVICE}  GPU={GPU_NAME}")
        except Exception as e:
            log.error(f"YOLO load error: {e}")
            model = None
    else:
        log.warning(f"Model not found at {MODEL_PATH} — running raw stream only")

    # ── Spatial dedup tracker ───────────────────────────────
    tracks     = []   # [(cx,cy,bw,bh,last_t,defect_id), ...]
    defect_seq = 1

    # ── Outer reconnect loop ────────────────────────────────
    while True:
        try:
            log.info(f"Connecting to Pi stream → {PI_STREAM}")
            req = urllib.request.Request(
                PI_STREAM,
                headers={"User-Agent": "BuildScan-Rover-AI/3.0",
                         "Connection": "keep-alive"}
            )
            with urllib.request.urlopen(req, timeout=8.0) as stream:
                with _lock:
                    _state.connected = True
                log.info("Pi stream connected — live inference active")

                buf         = b""
                fps_t0      = time.time()
                fps_frames  = 0
                cur_fps     = 0.0

                # ── Inner frame loop ────────────────────────
                while True:
                    chunk = stream.read(16384)
                    if not chunk:
                        break
                    buf += chunk

                    # Find JPEG boundaries
                    soi = buf.find(b"\xff\xd8")
                    eoi = buf.find(b"\xff\xd9")
                    if soi == -1 or eoi == -1 or eoi <= soi:
                        # Keep buffer bounded
                        if len(buf) > 1_000_000:
                            buf = buf[-500_000:]
                        continue

                    jpg_bytes = buf[soi: eoi + 2]
                    buf       = buf[eoi + 2:]

                    frame = cv2.imdecode(
                        np.frombuffer(jpg_bytes, dtype=np.uint8),
                        cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    h, w = frame.shape[:2]
                    ann  = frame.copy()
                    t0   = time.time()

                    # ── YOLO GPU Inference ──────────────────
                    cracks_now = 0
                    confs      = []
                    max_w_px   = 0.0
                    max_l_px   = 0.0

                    if model is not None:
                        try:
                            if _CUDA:
                                with torch.no_grad():
                                    results = model(
                                        frame,
                                        conf=CONF_THRESH,
                                        device=_DEVICE,
                                        verbose=False
                                    )
                            else:
                                results = model(
                                    frame,
                                    conf=CONF_THRESH,
                                    device=_DEVICE,
                                    verbose=False
                                )
                            res = results[0]

                            if res.boxes is not None and len(res.boxes):
                                cracks_now = len(res.boxes)

                                # Segmentation mask overlay (translucent red)
                                if res.masks is not None:
                                    overlay = ann.copy()
                                    for mask_t in res.masks.data:
                                        m = (mask_t.cpu().numpy() * 255).astype(np.uint8)
                                        if m.shape != (h, w):
                                            m = cv2.resize(m, (w, h),
                                                           interpolation=cv2.INTER_NEAREST)
                                        overlay[m > 128] = (30, 30, 220)  # red overlay
                                    cv2.addWeighted(overlay, 0.40, ann, 0.60, 0, ann)

                                # Per-crack bounding boxes & labels
                                now_t = time.time()
                                for i, box in enumerate(res.boxes):
                                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                                    conf = float(box.conf[0])
                                    bw   = max(1, x2 - x1)
                                    bh   = max(1, y2 - y1)
                                    blen = math.hypot(bw, bh)

                                    # Accurate aperture via mask area / length
                                    if res.masks is not None and i < len(res.masks.data):
                                        m_arr    = res.masks.data[i].cpu().numpy()
                                        m_area   = float(np.sum(m_arr > 0))
                                        aperture = m_area / max(1.0, blen)
                                    else:
                                        aperture = min(bw, bh)

                                    max_w_px = max(max_w_px, aperture)
                                    max_l_px = max(max_l_px, blen)
                                    confs.append(conf)

                                    sev, act = _severity(aperture, 1)
                                    col = ((0, 210, 80)    if sev == "LOW"
                                           else (0, 140, 255) if sev == "MEDIUM"
                                           else (30, 30, 245))

                                    # Bounding box
                                    cv2.rectangle(ann, (x1, y1), (x2, y2), col, 2)

                                    # Label background + text
                                    lbl = (f"CRACK {conf:.0%}  "
                                           f"W:{aperture * PX_MM:.2f}mm  "
                                           f"L:{blen * PX_MM:.1f}mm")
                                    (tw, th), _ = cv2.getTextSize(
                                        lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1)
                                    cv2.rectangle(ann,
                                                  (x1, max(0, y1 - 20)),
                                                  (x1 + tw + 8, y1), col, -1)
                                    cv2.putText(ann, lbl,
                                                (x1 + 4, y1 - 5),
                                                cv2.FONT_HERSHEY_SIMPLEX,
                                                0.46, (255, 255, 255), 1,
                                                cv2.LINE_AA)

                                    # Spatial dedup: is this crack already tracked?
                                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                    matched_id = None
                                    for idx, trk in enumerate(tracks):
                                        tx, ty, _, _, last_t, tid = trk
                                        if (math.hypot(cx - tx, cy - ty) < TRACK_RADIUS
                                                and now_t - last_t < TRACK_TTL):
                                            matched_id = tid
                                            tracks[idx] = (cx, cy, bw, bh,
                                                           now_t, tid)
                                            break

                                    if matched_id is None:
                                        # Brand-new crack
                                        new_id = f"DF-{defect_seq:04d}"
                                        defect_seq += 1
                                        tracks.append((cx, cy, bw, bh,
                                                        now_t, new_id))
                                        d = Defect(
                                            defect_id = new_id,
                                            timestamp = datetime.datetime.now()
                                                            .strftime("%H:%M:%S"),
                                            width_px  = round(aperture, 1),
                                            width_mm  = round(aperture * PX_MM, 2),
                                            length_mm = round(blen * PX_MM, 1),
                                            conf      = round(conf * 100, 1),
                                            severity  = sev,
                                            action    = act,
                                            bbox      = (x1, y1, x2, y2),
                                        )
                                        with _lock:
                                            _state.registry.append(d)
                                            _state.total_defects += 1

                        except torch.cuda.OutOfMemoryError:
                            log.error("CUDA OOM — clearing cache and retrying")
                            torch.cuda.empty_cache()
                            time.sleep(1.0)
                            continue
                        except Exception as infer_err:
                            log.warning(f"Inference error: {infer_err}")

                    # Clean stale tracks
                    tracks = [t for t in tracks
                              if (time.time() - t[4]) < TRACK_TTL]

                    # ── FPS counter ─────────────────────────
                    fps_frames += 1
                    elapsed = time.time() - fps_t0
                    if elapsed >= 1.0:
                        cur_fps    = fps_frames / elapsed
                        fps_frames = 0
                        fps_t0     = time.time()

                    # ── HUD top banner ──────────────────────
                    sev_g, act_g = _severity(max_w_px, cracks_now)
                    with _lock:
                        total = _state.total_defects
                    ov = ann.copy()
                    cv2.rectangle(ov, (0, 0), (w, 34), (8, 14, 26), -1)
                    cv2.addWeighted(ov, 0.82, ann, 0.18, 0, ann)
                    ts = datetime.datetime.now().strftime("%H:%M:%S")
                    hud = (f"BUILDSCAN ROVER AI  |  {ts}  |  "
                           f"{GPU_NAME[:20]}  |  {cur_fps:.1f} FPS  |  "
                           f"Live Cracks: {cracks_now}  |  Total Logged: {total}")
                    cv2.putText(ann, hud, (10, 23),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                                (0, 215, 255), 1, cv2.LINE_AA)

                    # ── HUD bottom status bar ───────────────
                    health = max(10.0, 100.0 -
                                 (cracks_now * 12.0 + max_w_px * PX_MM * 20.0))
                    bot_ov = ann.copy()
                    cv2.rectangle(bot_ov, (0, h - 24), (w, h), (8, 14, 26), -1)
                    cv2.addWeighted(bot_ov, 0.82, ann, 0.18, 0, ann)
                    scol = ((30, 220, 80)    if sev_g in ("NONE", "LOW")
                            else (0, 140, 255) if sev_g == "MEDIUM"
                            else (30, 30, 245))
                    bot = (f"HEALTH: {health:.1f}%  |  "
                           f"SEVERITY: {sev_g}  |  {act_g[:65]}")
                    cv2.putText(ann, bot, (10, h - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                scol, 1, cv2.LINE_AA)

                    # ── Encode & store ──────────────────────
                    _, enc = cv2.imencode(
                        ".jpg", ann,
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
                    enc_b = enc.tobytes()

                    with _lock:
                        _state.last_frame   = ann
                        _state.last_jpeg    = enc_b
                        _state.cracks_now   = cracks_now
                        if confs:
                            _state.avg_conf = sum(confs) / len(confs) * 100
                        # max_width_mm / max_len_mm only ever increase (peak values)
                        if max_w_px * PX_MM > _state.max_width_mm:
                            _state.max_width_mm = max_w_px * PX_MM
                        if max_l_px * PX_MM > _state.max_len_mm:
                            _state.max_len_mm = max_l_px * PX_MM
                        _state.severity     = sev_g
                        _state.health_idx   = round(health, 1)
                        _state.action       = act_g
                        if cur_fps > 0:
                            _state.fps = cur_fps

        except Exception as conn_err:
            with _lock:
                _state.connected = False
            log.warning(f"Pi stream disconnected: {conn_err} — retrying in 3 s")
            time.sleep(3.0)


# ── Start Services (called once per process) ─────────────────
def _ensure_services():
    if getattr(sys, "_buildscan_svc_started", False):
        return
    sys._buildscan_svc_started = True
    threading.Thread(target=_run_mjpeg_server,  daemon=True, name="mjpeg-srv").start()
    threading.Thread(target=_run_inference,     daemon=True, name="yolo-gpu").start()
    log.info("BuildScan background services started (MJPEG server + GPU inference worker)")


# ════════════════════════════════════════════════════════════
#   PDF AUDIT GENERATOR
# ════════════════════════════════════════════════════════════
def _make_pdf(st_copy: EngineState) -> bytes:
    if not PDF_OK:
        return json.dumps({
            "timestamp":      datetime.datetime.now().isoformat(),
            "total_defects":  st_copy.total_defects,
            "severity":       st_copy.severity,
            "health_index":   st_copy.health_idx,
        }, indent=2).encode()

    buf = io.BytesIO()
    tmp = None
    if st_copy.last_frame is not None:
        try:
            f = tempfile.NamedTemporaryFile(delete=False, suffix=".jpg",
                                            prefix="bscan_audit_")
            cv2.imwrite(f.name, st_copy.last_frame)
            tmp = f.name
        except Exception:
            pass

    styles = getSampleStyleSheet()
    doc = SimpleDocTemplate(buf, pagesize=letter,
                            rightMargin=36, leftMargin=36,
                            topMargin=36, bottomMargin=36)
    story = []

    title_s = ParagraphStyle("T", parent=styles["Heading1"],
                              fontSize=20, fontName="Helvetica-Bold",
                              textColor=colors.HexColor("#0f172a"))
    sub_s   = ParagraphStyle("S", parent=styles["Normal"],
                              fontSize=9, textColor=colors.HexColor("#64748b"))
    h2_s    = ParagraphStyle("H2", parent=styles["Heading2"],
                              fontSize=13, fontName="Helvetica-Bold",
                              textColor=colors.HexColor("#1e3a8a"),
                              spaceBefore=10, spaceAfter=4)

    now_s   = datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    cert_id = f"BS-{datetime.datetime.now().strftime('%Y%m%d')}-{random.randint(100,999)}"

    story.append(Paragraph(
        "BUILDSCAN ROVER AI  ·  STRUCTURAL HEALTH AUDIT", title_s))
    story.append(Paragraph(
        f"Audit ID: {cert_id}  ·  Generated: {now_s}  ·  "
        f"Camera: {PI_IP}:{PI_PORT}  ·  Model: YOLO26n-seg  ·  GPU: {GPU_NAME}",
        sub_s))
    story.append(Spacer(1, 6))
    story.append(HRFlowable(width="100%", thickness=1.5,
                             color=colors.HexColor("#2563eb"),
                             spaceBefore=2, spaceAfter=12))

    sev_col = {"LOW": "#16a34a", "MEDIUM": "#ea580c",
               "HIGH": "#dc2626"}.get(st_copy.severity, "#334155")
    metrics = [
        ["METRIC", "VALUE", "STANDARD", "STATUS"],
        ["Cumulative Defects",  str(st_copy.total_defects),
         "ACI 224R-01", "LOGGED"],
        ["Max Crack Aperture",
         f"{st_copy.max_width_mm:.2f} mm",
         "< 0.30 mm hairline / ≥ 0.55 mm critical",
         st_copy.severity],
        ["Max Crack Length",   f"{st_copy.max_len_mm:.1f} mm",
         "Monocular calibrated", "CALC"],
        ["Structural Health",  f"{st_copy.health_idx:.1f} %",
         "100 % = undamaged", "EVAL"],
        ["GPU Inference",      GPU_NAME, f"YOLO26n-seg · {MODEL_PATH.name}", "OK"],
    ]
    tbl = Table(metrics, colWidths=[2.2*inch, 2.0*inch, 2.2*inch, 0.9*inch])
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#1e3a8a")),
        ("TEXTCOLOR",  (0,0), (-1,0), colors.white),
        ("FONTNAME",   (0,0), (-1,0), "Helvetica-Bold"),
        ("FONTSIZE",   (0,0), (-1,0), 8),
        ("GRID",       (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e1")),
        ("FONTSIZE",   (0,1), (-1,-1), 7.5),
        ("TEXTCOLOR",  (3,2), (3,2), colors.HexColor(sev_col)),
        ("FONTNAME",   (3,2), (3,2), "Helvetica-Bold"),
    ]))
    story.append(tbl)
    story.append(Spacer(1, 10))

    if tmp:
        story.append(Paragraph(
            "LIVE INSPECTION FRAME  ·  YOLO26n-seg CRACK SEGMENTATION", h2_s))
        try:
            story.append(RLImage(tmp, width=6.8*inch, height=3.8*inch))
        except Exception:
            pass
        story.append(Spacer(1, 10))

    if st_copy.registry:
        story.append(Paragraph(
            "CERTIFIED DEFECT REGISTRY  ·  REMEDIATION PLAN", h2_s))
        rows = [["ID", "TIME", "APERTURE", "LENGTH", "CONF", "SEV", "ACTION"]]
        for d in st_copy.registry[-12:]:
            rows.append([d.defect_id, d.timestamp,
                         f"{d.width_mm:.2f} mm", f"{d.length_mm:.1f} mm",
                         f"{d.conf:.0f}%", d.severity,
                         d.action[:42] + "…"])
        rt = Table(rows,
                   colWidths=[0.8*inch, 0.8*inch, 0.9*inch,
                               0.9*inch, 0.5*inch, 0.6*inch, 3.0*inch])
        rt.setStyle(TableStyle([
            ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#0f172a")),
            ("TEXTCOLOR",  (0,0), (-1,0), colors.white),
            ("FONTNAME",   (0,0), (-1,0), "Helvetica-Bold"),
            ("FONTSIZE",   (0,0), (-1,0), 7.5),
            ("GRID",       (0,0), (-1,-1), 0.5, colors.HexColor("#cbd5e1")),
            ("FONTSIZE",   (0,1), (-1,-1), 7),
        ]))
        story.append(rt)

    doc.build(story)
    if tmp:
        try: os.remove(tmp)
        except Exception: pass
    return buf.getvalue()


# ════════════════════════════════════════════════════════════
#   CSS
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

div[data-testid="stDownloadButton"]>button{
  background:linear-gradient(135deg,#1e3a8a,#2563eb)!important;color:#fff!important;
  border:none!important;border-radius:12px!important;font-weight:700!important;
  box-shadow:0 4px 20px rgba(37,99,235,.4)!important;width:100%!important;}
div[data-testid="stDownloadButton"]>button:hover{
  transform:translateY(-2px)!important;box-shadow:0 8px 30px rgba(37,99,235,.6)!important;}

@keyframes pdot{0%,100%{opacity:1;transform:scale(1);}50%{opacity:.5;transform:scale(.85);}}
@keyframes pulse{0%,100%{box-shadow:0 4px 18px rgba(220,38,38,.4);}
                 50%{box-shadow:0 4px 34px rgba(220,38,38,.95);}}
</style>
"""

# ════════════════════════════════════════════════════════════
#   HELPERS
# ════════════════════════════════════════════════════════════
def _card(ico, lbl, val, cls, sub="", sm=False):
    vc  = "mv sm" if sm else "mv"
    s   = f'<div class="ms">{sub}</div>' if sub else ""
    return (f'<div class="mc {cls}">'
            f'  <div class="ml">{ico} {lbl}</div>'
            f'  <div><div class="{vc}">{val}</div>{s}</div>'
            f'</div>')


def _render_metrics(s: EngineState):
    # Use live values when detecting; fall back to best-ever from registry
    ever_detected = s.total_defects > 0 or len(s.registry) > 0

    cf = f"{s.avg_conf:.1f}%" if s.cracks_now > 0 else (
         f"{s.registry[-1].conf:.1f}% (last)" if s.registry else "--")
    sw = (f"{s.max_width_mm:.2f} mm ({s.max_width_mm/PX_MM:.1f}px)" if s.cracks_now > 0
          else (f"{s.registry[-1].width_mm:.2f} mm (last)" if s.registry else "--"))
    sl = (f"{s.max_len_mm:.1f} mm" if s.cracks_now > 0
          else (f"{s.registry[-1].length_mm:.1f} mm (last)" if s.registry else "--"))
    sc = {"NONE":"sev-none","LOW":"sev-low",
          "MEDIUM":"sev-med","HIGH":"sev-high"}.get(s.severity,"sev-none")
    rd = "mc-rd" if s.total_defects > 0 else "mc-cy"

    # Cracks in view — pulse red if active
    cracks_cls = "mc-rd" if s.cracks_now > 0 else "mc-bl"
    cracks_sub = (f"🔴 {s.cracks_now} crack(s) actively detected" if s.cracks_now > 0
                  else "Live GPU inference running...")

    fps_disp = f"{s.fps:.1f} FPS" if s.fps > 0 else "Warming up..."
    action_disp = s.action if s.action else "System initialising..."

    st.markdown(_card("🔍", "Cracks in Current View",
                      str(s.cracks_now), cracks_cls, cracks_sub),
                unsafe_allow_html=True)
    st.markdown(_card("📊", "Total Defects Logged",
                      str(s.total_defects), rd,
                      f"{len(s.registry)} unique crack IDs registered"),
                unsafe_allow_html=True)
    st.markdown(_card("🎯", "Detection Confidence",
                      cf, "mc-or",
                      f"YOLO26n-seg  ·  {GPU_NAME[:22]}"),
                unsafe_allow_html=True)
    st.markdown(_card("↔", "Max Crack Aperture", sw, "mc-rd",
                      f"1 px = {PX_MM} mm calibrated"),
                unsafe_allow_html=True)
    st.markdown(_card("📏", "Max Crack Length", sl, "mc-cy",
                      "Monocular pixel geometry"), unsafe_allow_html=True)
    st.markdown(_card("⚠️", "Structural Severity", s.severity, sc,
                      f"Health Index: {s.health_idx:.1f}%"),
                unsafe_allow_html=True)
    st.markdown(_card("🛠️", "Civil Remediation Action",
                      action_disp, "mc-pu", sm=True),
                unsafe_allow_html=True)
    st.markdown(_card("⚡", "Inference Speed",
                      fps_disp, "mc-gr",
                      f"GPU: {GPU_NAME[:22]}"),
                unsafe_allow_html=True)


# ════════════════════════════════════════════════════════════
#   MAIN STREAMLIT APP
# ════════════════════════════════════════════════════════════
def main():
    st.set_page_config(
        page_title="BuildScan Rover AI — Live Inspection",
        page_icon="🏗️",
        layout="wide",
        initial_sidebar_state="collapsed",
    )

    # ── Auto-refresh every 1.5 s — must be called early, before any widget ──
    from streamlit_autorefresh import st_autorefresh
    st_autorefresh(interval=1500, limit=None, key="live_refresh")

    try:    st.html(_CSS)
    except: st.markdown(_CSS, unsafe_allow_html=True)

    # Boot singleton services (safe to call on every rerun)
    _ensure_services()

    # ── Snapshot latest state from GPU worker (called on EVERY rerun) ────
    with _lock:
        s = EngineState(
            connected     = _state.connected,
            cracks_now    = _state.cracks_now,
            total_defects = _state.total_defects,
            avg_conf      = _state.avg_conf,
            max_width_mm  = _state.max_width_mm,
            max_len_mm    = _state.max_len_mm,
            severity      = _state.severity,
            health_idx    = _state.health_idx,
            action        = _state.action,
            fps           = _state.fps,
            last_frame    = _state.last_frame,
            last_jpeg     = _state.last_jpeg,
            registry      = list(_state.registry),
        )

    # ── HEADER ───────────────────────────────────────────────
    conn_color = "#22c55e" if s.connected else "#ef4444"
    conn_text  = "Pi Connected" if s.connected else "Pi Reconnecting…"
    st.markdown(f"""
    <div class="bsh">
      <div style="display:flex;align-items:center;">
        <div class="bsh-ico">🏗️</div>
        <div>
          <p class="bsh-title">BuildScan Rover AI &bull; Live Structural Inspection</p>
          <p class="bsh-sub">
            Real-Time Pi Camera Stream &bull; YOLO26n-seg Crack Segmentation
            &bull; {GPU_NAME} GPU
          </p>
        </div>
      </div>
      <div class="bsh-right">
        <div class="pill p-gpu">
          <div class="dot d-c"></div>GPU: {GPU_NAME[:20]}
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
      <div class="si">{'🟢' if s.connected else '🔴'} <b>Pi Stream:</b>
            {PI_IP}:{PI_PORT}/image_raw</div>
      <div class="si">⚡ <b>AI Engine:</b> YOLO26n-seg on {GPU_NAME}</div>
      <div class="si">📍 <b>LiDAR:</b> Standby (Visual Monocular Active)</div>
      <div class="si">🛡️ <b>E-Stop:</b> Armed (&lt; 20 cm)</div>
      <div class="si">🔢 <b>Total Defects:</b> {s.total_defects}</div>
      <div class="si">⏱️ <b>FPS:</b> {s.fps:.1f}</div>
    </div>
    """, unsafe_allow_html=True)

    # ── MAIN LAYOUT: VIDEO  |  METRICS ───────────────────────
    col_vid, col_met = st.columns([3.2, 1.8], gap="medium")

    with col_vid:
        # Feed mode toggle
        mode = st.radio(
            "feed_mode",
            ["🎯 AI-Annotated Stream (YOLO26n-seg GPU)",
             "📹 Raw Pi 4 Camera Feed"],
            horizontal=True,
            label_visibility="collapsed",
        )

        if "AI-Annotated" in mode:
            src   = AI_STREAM
            badge = "🔴 LIVE GPU AI STREAM &bull; YOLO26n-seg CRACK DETECTION"
            note  = f"Annotated at localhost:{AI_PORT}  ·  Inference on {GPU_NAME}"
        else:
            src   = PI_STREAM
            badge = "📹 RAW PI 4 CAMERA STREAM &bull; 30 FPS"
            note  = f"Direct from {PI_IP}:{PI_PORT} (no AI overlay)"

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

        # Controls
        st.markdown("<div style='height:8px'></div>", unsafe_allow_html=True)
        b1, b2, b3 = st.columns([1.5, 1.5, 1.2])
        with b1:
            if st.button("🔄 Refresh Metrics", use_container_width=True,
                         type="primary"):
                st.rerun()
        with b2:
            if st.button("🧹 Reset Defect Count", use_container_width=True):
                with _lock:
                    _state.total_defects = 0
                    _state.registry.clear()
                st.rerun()
        with b3:
            if st.button("📄 Generate Audit PDF", use_container_width=True):
                st.session_state["pdf_data"] = _make_pdf(s)
                st.session_state["pdf_tag"]  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            if st.session_state.get("pdf_data"):
                st.download_button(
                    "📥 Download PDF",
                    data=st.session_state["pdf_data"],
                    file_name=f"BuildScan_Audit_{st.session_state.get('pdf_tag','report')}.pdf",
                    mime="application/pdf",
                    use_container_width=True,
                )

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
            '🔍 No cracks detected yet.  '
            'When the Pi camera detects a real structural crack, '
            'YOLO26n-seg will segment it live, measure its aperture, '
            'and log it here automatically.'
            '</div>', unsafe_allow_html=True)

    st.markdown("</div>", unsafe_allow_html=True)


if __name__ == "__main__":
    main()

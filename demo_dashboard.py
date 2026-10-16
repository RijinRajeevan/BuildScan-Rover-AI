#!/usr/bin/env python3
"""
demo_dashboard.py  -  BuildScan Rover AI Dashboard (Premium Edition)
Dark glassmorphism UI | Live Pi camera stream | Real-time YOLO crack detection

Pi Camera Stream: http://10.195.174.129:8080/stream?topic=/image_raw
"""
import io, os, math, random, time, uuid, tempfile, datetime, threading, dataclasses
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import streamlit as st
from PIL import Image as PILImage

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas as rl_canvas
    from reportlab.lib.colors import HexColor, white
    from reportlab.lib.utils import ImageReader
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False

# ── Constants ─────────────────────────────────────────────
PI_IP         = "10.195.174.129"
PI_PORT       = 8080
PI_TOPIC      = "/image_raw"
PI_STREAM_URL = f"http://{PI_IP}:{PI_PORT}/stream?topic={PI_TOPIC}"

SEV_LOW_W  = 70
SEV_MED_W  = 150
R_NONE   = "No crack detected"
R_LOW    = "Apply surface filler and sealant"
R_MEDIUM = "Use epoxy injection repair"
R_HIGH   = "Urgent structural inspection required"

_DIR         = Path(__file__).resolve().parent
_MODEL_PATH  = _DIR / "models" / "yolo26n-seg.pt"
_CAM_IDX     = int(os.environ.get("BUILDSCAN_CAM_IDX", "0"))
_REFRESH     = 0.25


# ── Data ───────────────────────────────────────────────────
@dataclasses.dataclass
class Snap:
    annotated_frame:    Optional[np.ndarray] = None
    original_frame:     Optional[np.ndarray] = None
    cracks_detected:    int   = 0
    average_confidence: float = 0.0
    maximum_width:      float = 0.0
    total_length:       float = 0.0
    severity:           str   = "NONE"
    recommendation:     str   = R_NONE
    timestamp:          Optional[datetime.datetime] = None
    fps:                float = 0.0


def classify(max_w, n):
    if n == 0 or max_w == 0: return "NONE", R_NONE
    if max_w < SEV_LOW_W:    return "LOW",  R_LOW
    if max_w < SEV_MED_W:    return "MEDIUM", R_MEDIUM
    return "HIGH", R_HIGH


# ── Model ──────────────────────────────────────────────────
@st.cache_resource(show_spinner="Loading YOLO26n-seg model...")
def load_model():
    if not YOLO_AVAILABLE or not _MODEL_PATH.exists():
        return None
    try:
        return YOLO(str(_MODEL_PATH))
    except Exception:
        return None


# ── Inference ──────────────────────────────────────────────
def analyze(model, frame, t0):
    snap = Snap(original_frame=frame.copy(), timestamp=datetime.datetime.now())
    if model is None:
        snap.annotated_frame = frame.copy()
        return snap
    results   = model(frame, conf=0.30, imgsz=320, verbose=False)
    annotated = frame.copy()
    max_w = tot = 0.0
    confs, count = [], 0
    for result in results:
        if result.boxes is None: continue
        for box in result.boxes:
            x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            w, h = x2-x1, y2-y1
            max_w = max(max_w, w)
            tot  += math.sqrt(w*w + h*h)
            confs.append(conf); count += 1
            cv2.rectangle(annotated, (x1,y1),(x2,y2),(0,80,255),2)
            cv2.rectangle(annotated, (x1,y1-22),(x1+110,y1),(0,80,255),-1)
            cv2.putText(annotated, f"Crack {conf:.0%}", (x1+4,y1-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255,255,255), 1)
    h_i,w_i = annotated.shape[:2]
    ov = annotated.copy()
    cv2.rectangle(ov,(0,0),(w_i,38),(10,14,30),-1)
    cv2.addWeighted(ov,0.75,annotated,0.25,0,annotated)
    fps = 1.0 / max(time.time()-t0, 0.001)
    ts  = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(annotated,
                f"BuildScan Rover  |  {ts}  |  Cracks: {count}  |  {fps:.1f} FPS",
                (10,26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,210,255), 1)
    sev, rec = classify(max_w, count)
    snap.annotated_frame    = annotated
    snap.cracks_detected    = count
    snap.average_confidence = (sum(confs)/len(confs)*100) if confs else 0.0
    snap.maximum_width      = max_w
    snap.total_length       = tot
    snap.severity           = sev
    snap.recommendation     = rec
    snap.fps                = fps
    return snap


# ── Demo frames ────────────────────────────────────────────
def demo_frame(tick):
    h, w = 480, 640
    frame = np.full((h,w,3),(128,126,122),dtype=np.uint8)
    noise = np.random.randint(-18,18,(h,w,3),dtype=np.int16)
    frame = np.clip(frame.astype(np.int16)+noise,0,255).astype(np.uint8)
    t = tick*0.04
    cracks = [
        (int(120+10*math.sin(t)),    180, int(290+8*math.cos(t)),    210, 0.87),
        (int(340+6*math.sin(t+1)),   260, int(480+5*math.cos(t+1)), 310, 0.72),
        (int(80+4*math.sin(t+2)),    360, int(200+6*math.cos(t+2)), 395, 0.61),
    ]
    ann = frame.copy()
    ov  = ann.copy()
    cv2.rectangle(ov,(0,0),(w,38),(10,14,30),-1)
    cv2.addWeighted(ov,0.75,ann,0.25,0,ann)
    max_w = tot = 0.0
    confs = []
    for (x1,y1,x2,y2,conf) in cracks:
        bw = x2-x1; bl = math.sqrt((x2-x1)**2+(y2-y1)**2)
        max_w = max(max_w,bw); tot+=bl; confs.append(conf)
        cv2.rectangle(ann,(x1,y1),(x2,y2),(0,80,255),2)
        cv2.rectangle(ann,(x1,y1-22),(x1+110,y1),(0,80,255),-1)
        cv2.line(ann,(x1+random.randint(0,5),y1+random.randint(0,5)),
                 (x2-random.randint(0,5),y2-random.randint(0,5)),(30,30,200),1)
        cv2.putText(ann,f"Crack {conf:.0%}",(x1+4,y1-6),
                    cv2.FONT_HERSHEY_SIMPLEX,0.52,(255,255,255),1)
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(ann,f"BuildScan Rover  |  {ts}  |  Cracks: {len(cracks)}  [DEMO]",
                (10,26),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,210,255),1)
    sev,rec = classify(max_w,len(cracks))
    return Snap(annotated_frame=ann,original_frame=frame,
                cracks_detected=len(cracks),
                average_confidence=(sum(confs)/len(confs)*100) if confs else 0.0,
                maximum_width=max_w,total_length=tot,
                severity=sev,recommendation=rec,
                timestamp=datetime.datetime.now())


# ── Threads ────────────────────────────────────────────────
def _cam_loop(ev, src):
    model = load_model()
    cap   = cv2.VideoCapture(src)
    if not cap.isOpened():
        label = f"Pi stream ({src})" if isinstance(src,str) else f"cam {src}"
        st.session_state["stream_error"] = f"Cannot connect to {label}."
        st.session_state["streaming"]    = False
        st.session_state["stream_mode"]  = "idle"
        ev.set(); return
    while not ev.is_set():
        t0 = time.time()
        ret, frame = cap.read()
        if not ret: time.sleep(0.05); continue
        st.session_state["snapshot"] = analyze(model, frame, t0)
        time.sleep(0.02)
    cap.release()

def _demo_loop(ev):
    tick = 0
    while not ev.is_set():
        st.session_state["snapshot"] = demo_frame(tick)
        tick += 1; time.sleep(0.12)

def start_stream(mode="pi"):
    if st.session_state.get("streaming"): return
    ev = threading.Event()
    st.session_state.update({"stop_event":ev,"streaming":True,"stream_mode":mode,
                              "demo_mode":(mode=="demo"),"stream_error":None,
                              "snapshot":Snap()})
    if   mode=="demo": t=threading.Thread(target=_demo_loop,args=(ev,),daemon=True)
    elif mode=="pi":   t=threading.Thread(target=_cam_loop,args=(ev,PI_STREAM_URL),daemon=True)
    else:              t=threading.Thread(target=_cam_loop,args=(ev,_CAM_IDX),daemon=True)
    t.start()
    st.session_state["stream_thread"] = t

def stop_stream():
    if not st.session_state.get("streaming"): return
    ev = st.session_state.get("stop_event")
    if ev: ev.set()
    t = st.session_state.get("stream_thread")
    if t and t.is_alive(): t.join(timeout=2.0)
    st.session_state.update({"streaming":False,"stream_mode":"idle",
                              "stop_event":None,"stream_thread":None})


# ── PDF ────────────────────────────────────────────────────
def export_pdf(snap):
    buf = io.BytesIO()
    if not REPORTLAB_AVAILABLE:
        import json
        return json.dumps({"timestamp":snap.timestamp.isoformat() if snap.timestamp else "",
                           "cracks":snap.cracks_detected,"confidence":round(snap.average_confidence,2),
                           "max_width_px":round(snap.maximum_width,1),"severity":snap.severity,
                           "recommendation":snap.recommendation},indent=2).encode()
    ann_path = ""
    if snap.annotated_frame is not None:
        tmp = tempfile.NamedTemporaryFile(delete=False,suffix=".jpg",prefix="bscan_")
        cv2.imwrite(tmp.name, snap.annotated_frame); ann_path=tmp.name
    c = rl_canvas.Canvas(buf,pagesize=A4); pw,ph=A4
    BL=HexColor("#2563eb"); CY=HexColor("#06b6d4"); OR=HexColor("#ea580c")
    RD=HexColor("#dc2626"); PU=HexColor("#7c3aed"); AC=HexColor("#0a0f1e")
    GR=HexColor("#16a34a"); BG=HexColor("#f0f4f8")
    SC={"NONE":HexColor("#6b7280"),"LOW":GR,"MEDIUM":OR,"HIGH":RD}
    c.setFillColor(BG);  c.rect(0,0,pw,ph,fill=1,stroke=0)
    c.setFillColor(AC);  c.rect(0,ph-90,pw,90,fill=1,stroke=0)
    c.setFillColor(CY);  c.rect(0,ph-92,pw,4,fill=1,stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold",22)
    c.drawString(30,ph-48,"BuildScan Rover - Inspection Report")
    c.setFont("Helvetica",10)
    ts_s = snap.timestamp.strftime("%d %b %Y  %I:%M %p") if snap.timestamp else ""
    c.drawString(30,ph-68,ts_s)
    iid = str(uuid.uuid4())[:8].upper()
    c.setFont("Helvetica-Bold",9); c.drawRightString(pw-20,ph-48,f"ID: {iid}")
    c.setFont("Helvetica",8);      c.drawRightString(pw-20,ph-62,f"Pi: {PI_IP} | YOLO26n-seg")
    sc = SC.get(snap.severity,RD)
    c.setFillColor(sc); c.roundRect(pw-165,ph-148,140,40,10,fill=1,stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold",17)
    c.drawCentredString(pw-95,ph-133,snap.severity)
    c.setFillColor(HexColor("#111827")); c.setFont("Helvetica-Bold",11)
    c.drawString(30,ph-118,"Project: BUILDScan Rover - AI Structural Health Inspection")
    c.setFont("Helvetica",9); c.setFillColor(HexColor("#374151"))
    c.drawString(30,ph-134,f"Confidence: 30% | Resolution: 1920x1080 | Report: {iid}")
    def pc(cx,cy,color,title,value,cw=130,ch=52):
        c.setFillColor(color); c.roundRect(cx,cy,cw,ch,8,fill=1,stroke=0)
        c.setFillColor(white); c.setFont("Helvetica-Bold",9); c.drawString(cx+8,cy+34,title)
        c.setFont("Helvetica-Bold",14); c.drawString(cx+8,cy+12,str(value))
    cy1=ph-215
    pc(30,cy1,BL,"Cracks",str(snap.cracks_detected))
    pc(175,cy1,OR,"Confidence",f"{snap.average_confidence:.1f}%")
    pc(320,cy1,RD,"Max Width",f"{snap.maximum_width:.0f}px")
    pc(465,cy1,PU,"Length",f"{snap.total_length:.0f}px")
    cy2=cy1-75
    pc(30,cy2,GR,"Severity",snap.severity)
    recs=(snap.recommendation[:28]+"..." if len(snap.recommendation)>28 else snap.recommendation)
    pc(175,cy2,AC,"Repair Action",recs,cw=320)
    img_y=cy2-235
    if ann_path and os.path.exists(ann_path):
        c.setFillColor(HexColor("#111827")); c.setFont("Helvetica-Bold",10)
        c.drawString(30,img_y+222,"Annotated Detection Frame (YOLO26n-seg)")
        try: c.drawImage(ImageReader(ann_path),30,img_y,width=pw-60,height=210,preserveAspectRatio=True)
        except: pass
    ry=img_y-60
    c.setFillColor(sc); c.roundRect(30,ry,pw-60,44,8,fill=1,stroke=0)
    c.setFillColor(white); c.setFont("Helvetica-Bold",11)
    c.drawString(42,ry+28,"Engineering Recommendation:")
    c.setFont("Helvetica",10); c.drawString(42,ry+10,snap.recommendation)
    c.setStrokeColor(CY); c.setLineWidth(1.5); c.line(30,50,pw-30,50)
    c.setFillColor(AC); c.setFont("Helvetica-Oblique",8)
    c.drawString(30,34,"Generated by BuildScan Rover - AI Structural Health Inspection System")
    c.drawRightString(pw-30,34,f"Report ID: {iid}")
    c.save(); buf.seek(0); return buf.read()


def bgr2pil(bgr):
    return PILImage.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB))

def sev_cls(s):
    return {"NONE":"sev-none","LOW":"sev-low","MEDIUM":"sev-med","HIGH":"sev-high"}.get(s,"sev-none")


_CSS = """<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700;800;900&family=JetBrains+Mono:wght@400;600&display=swap');
html,body,[class*="css"]{font-family:'Inter',sans-serif!important;background:#070b18!important;color:#e2e8f0!important;}
.stApp{background:linear-gradient(135deg,#070b18 0%,#0d1628 50%,#070b18 100%)!important;}
.bs-header{background:linear-gradient(90deg,rgba(13,22,44,.95),rgba(6,182,212,.08));border:1px solid rgba(6,182,212,.25);border-radius:16px;padding:18px 24px;display:flex;align-items:center;gap:16px;margin-bottom:20px;backdrop-filter:blur(20px);box-shadow:0 4px 32px rgba(6,182,212,.08);}
.bs-icon{font-size:2.6rem;filter:drop-shadow(0 0 12px rgba(6,182,212,.6));}
.bs-title{font-size:1.55rem;font-weight:900;color:#fff;letter-spacing:-.5px;margin:0;}
.bs-sub{font-size:.78rem;color:#94a3b8;margin:0;}
.bs-right{margin-left:auto;display:flex;flex-direction:column;align-items:flex-end;gap:4px;}
.pill{display:flex;align-items:center;gap:7px;padding:6px 14px;border-radius:50px;font-size:.78rem;font-weight:700;letter-spacing:.5px;}
.p-stream{background:rgba(6,182,212,.2);border:1px solid #06b6d4;color:#67e8f9;}
.p-demo{background:rgba(234,88,12,.2);border:1px solid #ea580c;color:#fb923c;}
.p-idle{background:rgba(100,116,139,.2);border:1px solid #475569;color:#94a3b8;}
.dot{width:8px;height:8px;border-radius:50%;}
.d-cyan{background:#67e8f9;box-shadow:0 0 8px #67e8f9;animation:pdot 1.5s infinite;}
.d-ora{background:#fb923c;box-shadow:0 0 8px #fb923c;animation:pdot 1s infinite;}
.d-gray{background:#94a3b8;}
.ip-tag{font-family:'JetBrains Mono',monospace;font-size:.72rem;color:#64748b;}
.ibar{display:flex;gap:18px;flex-wrap:wrap;align-items:center;background:rgba(13,22,44,.7);border:1px solid rgba(6,182,212,.15);border-radius:10px;padding:8px 16px;font-size:.75rem;color:#64748b;font-family:'JetBrains Mono',monospace;margin-bottom:16px;}
.ii{display:flex;align-items:center;gap:6px;}
.idot{width:6px;height:6px;border-radius:50%;}
.id-g{background:#4ade80;box-shadow:0 0 6px #4ade80;}
.id-c{background:#67e8f9;}
.id-gr{background:#475569;}
div[data-testid="stButton"]>button{border-radius:10px!important;font-weight:700!important;font-size:.85rem!important;transition:all .2s!important;height:44px!important;}
div[data-testid="stButton"]>button[kind="primary"]{background:linear-gradient(135deg,#0284c7,#06b6d4)!important;color:#fff!important;box-shadow:0 4px 20px rgba(6,182,212,.35)!important;}
div[data-testid="stButton"]>button[kind="primary"]:hover{transform:translateY(-2px)!important;box-shadow:0 8px 28px rgba(6,182,212,.55)!important;}
div[data-testid="stButton"]>button[kind="secondary"]{background:rgba(30,42,74,.9)!important;color:#cbd5e1!important;border:1px solid rgba(100,116,139,.4)!important;}
div[data-testid="stButton"]>button[kind="secondary"]:hover{background:rgba(51,65,105,.9)!important;border-color:rgba(6,182,212,.5)!important;color:#fff!important;}
.vout{background:rgba(10,15,30,.9);border:1px solid rgba(6,182,212,.2);border-radius:16px;padding:8px;box-shadow:0 8px 40px rgba(0,0,0,.5);}
.vbadge{display:inline-flex;align-items:center;gap:6px;background:rgba(220,38,38,.85);color:#fff;font-size:.65rem;font-weight:700;letter-spacing:1.5px;padding:3px 10px;border-radius:4px;margin-bottom:6px;}
.nofeed{background:rgba(10,15,30,.9);border:2px dashed rgba(6,182,212,.2);border-radius:16px;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:340px;text-align:center;}
.nfi{font-size:3.5rem;margin-bottom:12px;opacity:.5;}
.nft{font-size:1rem;font-weight:600;color:#64748b;}
.nfs{font-size:.8rem;margin-top:6px;color:#475569;}
.vts{font-family:'JetBrains Mono',monospace;font-size:.72rem;color:#475569;text-align:right;margin-top:6px;}
.mc{border-radius:14px;padding:16px 18px;min-height:90px;display:flex;flex-direction:column;justify-content:space-between;margin-bottom:12px;transition:transform .2s,box-shadow .2s;cursor:default;}
.mc:hover{transform:translateY(-3px);}
.ml{font-size:.7rem;font-weight:700;letter-spacing:1px;text-transform:uppercase;opacity:.85;}
.mv{font-size:1.7rem;font-weight:800;letter-spacing:-1px;line-height:1;margin-top:8px;}
.mv.sm{font-size:1rem;}
.mc-bl{background:linear-gradient(135deg,#1e3a8a,#2563eb);box-shadow:0 4px 20px rgba(37,99,235,.3);}
.mc-cy{background:linear-gradient(135deg,#164e63,#0891b2);box-shadow:0 4px 20px rgba(6,182,212,.3);}
.mc-or{background:linear-gradient(135deg,#7c2d12,#ea580c);box-shadow:0 4px 20px rgba(234,88,12,.3);}
.mc-rd{background:linear-gradient(135deg,#7f1d1d,#dc2626);box-shadow:0 4px 20px rgba(220,38,38,.3);}
.mc-gr{background:linear-gradient(135deg,#14532d,#16a34a);box-shadow:0 4px 20px rgba(22,163,74,.3);}
.mc-pu{background:linear-gradient(135deg,#4c1d95,#7c3aed);box-shadow:0 4px 20px rgba(124,58,237,.3);}
.sev-none{background:linear-gradient(135deg,#1e293b,#334155);box-shadow:0 4px 20px rgba(51,65,85,.4);}
.sev-low{background:linear-gradient(135deg,#14532d,#16a34a);box-shadow:0 4px 20px rgba(22,163,74,.4);}
.sev-med{background:linear-gradient(135deg,#7c2d12,#ea580c);box-shadow:0 4px 20px rgba(234,88,12,.4);}
.sev-high{background:linear-gradient(135deg,#7f1d1d,#dc2626);box-shadow:0 4px 20px rgba(220,38,38,.5);animation:pcrd 2s infinite;}
div[data-testid="stDownloadButton"]>button{background:linear-gradient(135deg,#1e3a8a,#2563eb)!important;color:#fff!important;border:none!important;border-radius:12px!important;font-weight:700!important;box-shadow:0 4px 20px rgba(37,99,235,.4)!important;width:100%!important;}
div[data-testid="stDownloadButton"]>button:hover{transform:translateY(-2px)!important;box-shadow:0 8px 30px rgba(37,99,235,.6)!important;}
hr{border-color:rgba(6,182,212,.15)!important;margin:16px 0!important;}
.stExpander{background:rgba(13,22,44,.7)!important;border:1px solid rgba(6,182,212,.2)!important;border-radius:12px!important;}
label{color:#94a3b8!important;font-size:.78rem!important;font-weight:600!important;}
@keyframes pdot{0%,100%{opacity:1;transform:scale(1);}50%{opacity:.6;transform:scale(.85);}}
@keyframes pcrd{0%,100%{box-shadow:0 4px 20px rgba(220,38,38,.5);}50%{box-shadow:0 4px 36px rgba(220,38,38,.9);}}
</style>"""


def main():
    st.set_page_config(page_title="BuildScan Rover - AI Dashboard",
                       page_icon="🏗️", layout="wide",
                       initial_sidebar_state="collapsed")
    try:    st.html(_CSS)
    except: st.markdown(_CSS, unsafe_allow_html=True)

    defs = {"streaming":False,"demo_mode":False,"stream_mode":"idle",
            "snapshot":Snap(),"stop_event":None,"stream_thread":None,"stream_error":None}
    for k,v in defs.items():
        st.session_state.setdefault(k,v)

    snap  = st.session_state["snapshot"]
    strm  = st.session_state.get("streaming",False)
    mode  = st.session_state.get("stream_mode","idle")
    demo  = st.session_state.get("demo_mode",False)
    err   = st.session_state.get("stream_error",None)

    if strm and mode=="pi":     pc,dc,pt="p-stream","d-cyan","📡 Pi Camera Live"
    elif strm and demo:         pc,dc,pt="p-demo","d-ora","🧪 Demo Mode"
    elif strm:                  pc,dc,pt="p-stream","d-cyan","🎥 Webcam Live"
    else:                       pc,dc,pt="p-idle","d-gray","● Standby"

    fps_t = f"{snap.fps:.1f} FPS" if snap.fps>0 else "-- FPS"
    ts_t  = snap.timestamp.strftime("%H:%M:%S") if snap.timestamp else "--:--:--"

    st.markdown(f"""
    <div class="bs-header">
      <div class="bs-icon">🏗️</div>
      <div><p class="bs-title">BuildScan Rover</p>
        <p class="bs-sub">AI Structural Health Inspection &middot; YOLO26n-seg &middot; Distributed Architecture</p></div>
      <div class="bs-right">
        <div class="pill {pc}"><div class="dot {dc}"></div>{pt}</div>
        <div class="ip-tag">Pi {PI_IP}:{PI_PORT} &nbsp;|&nbsp; {ts_t} &nbsp;|&nbsp; {fps_t}</div>
      </div>
    </div>""", unsafe_allow_html=True)

    mok = YOLO_AVAILABLE and _MODEL_PATH.exists()
    stx = err[:60] if err else ("Active" if strm else "Idle")
    gm  = "id-g" if mok else "id-gr"
    gp  = "id-g" if REPORTLAB_AVAILABLE else "id-gr"
    gs  = "id-g" if strm and not err else "id-gr"

    st.markdown(f"""
    <div class="ibar">
      <div class="ii"><div class="idot {gm}"></div>{"🟢" if mok else "🔴"} YOLO26n-seg {"Loaded" if mok else "Not Found"}</div>
      <div class="ii"><div class="idot {gp}"></div>{"🟢" if REPORTLAB_AVAILABLE else "🟡"} PDF {"Ready" if REPORTLAB_AVAILABLE else "Missing"}</div>
      <div class="ii"><div class="idot {gs}"></div>{"🟢" if strm and not err else "⚪"} Stream: {stx}</div>
      <div class="ii"><div class="idot id-c"></div>🌐 {PI_STREAM_URL}</div>
    </div>""", unsafe_allow_html=True)

    b1,b2,b3,b4 = st.columns([2.5,1.8,1.8,1.5])
    with b1:
        if st.button("📡  Connect Pi Camera",key="btn_pi",type="primary",
                     disabled=strm,use_container_width=True):
            start_stream("pi"); st.rerun()
    with b2:
        if st.button("▶  Local Webcam",key="btn_wc",type="secondary",
                     disabled=strm,use_container_width=True):
            start_stream("webcam"); st.rerun()
    with b3:
        if st.button("🧪  Demo Mode",key="btn_dm",type="secondary",
                     disabled=strm,use_container_width=True):
            start_stream("demo"); st.rerun()
    with b4:
        if st.button("■  Stop",key="btn_st",type="secondary",
                     disabled=not strm,use_container_width=True):
            stop_stream(); st.rerun()

    cv, cm = st.columns([3,2], gap="medium")
    with cv:
        if snap.annotated_frame is not None:
            st.markdown('<div class="vout"><div class="vbadge">🔴 CRACK DETECTION LIVE</div>',
                        unsafe_allow_html=True)
            st.image(bgr2pil(snap.annotated_frame), use_container_width=True)
            st.markdown('</div>', unsafe_allow_html=True)
        else:
            st.markdown("""<div class="nofeed"><div class="nfi">📡</div>
              <div class="nft">No Feed Active</div>
              <div class="nfs">Click <strong>📡 Connect Pi Camera</strong> to start streaming from the Raspberry Pi</div>
            </div>""", unsafe_allow_html=True)
        if snap.timestamp:
            st.markdown(f'<div class="vts">Last frame: {snap.timestamp.strftime("%H:%M:%S.%f")[:-3]} | {fps_t}</div>',
                        unsafe_allow_html=True)

    with cm:
        def card(ico,lbl,val,cls,sm=False):
            vc = "mv sm" if sm else "mv"
            return f'<div class="mc {cls}"><div class="ml">{ico} {lbl}</div><div class="{vc}">{val}</div></div>'
        cf = f"{snap.average_confidence:.1f}%" if snap.cracks_detected else "--"
        sl = f"{snap.total_length:.0f} px"     if snap.cracks_detected else "--"
        sw = f"{snap.maximum_width:.0f} px"    if snap.cracks_detected else "--"
        sc = sev_cls(snap.severity)
        st.markdown(card("🔍","Cracks Detected",str(snap.cracks_detected),"mc-bl"),unsafe_allow_html=True)
        st.markdown(card("🎯","Avg Confidence",cf,"mc-or"),unsafe_allow_html=True)
        st.markdown(card("↔","Max Crack Width",sw,"mc-rd"),unsafe_allow_html=True)
        st.markdown(card("📏","Total Length",sl,"mc-cy"),unsafe_allow_html=True)
        st.markdown(card("⚠️","Severity",snap.severity,sc),unsafe_allow_html=True)
        st.markdown(card("🛠","Recommendation",snap.recommendation,"mc-pu",sm=True),unsafe_allow_html=True)

    st.markdown("---")
    has = snap.cracks_detected>0 or snap.annotated_frame is not None
    dc1,dc2 = st.columns([2,5])
    with dc1:
        if has:
            try:
                pdf = export_pdf(snap)
                ext = "pdf" if pdf[:4]==b"%PDF" else "json"
                tag = snap.timestamp.strftime("%Y%m%d_%H%M%S") if snap.timestamp else "export"
                st.download_button("📥  Download Inspection Report (PDF)",pdf,
                                   f"BuildScan_{tag}.{ext}","application/pdf",
                                   key="btn_dl",use_container_width=True)
            except Exception as e: st.warning(f"Export error: {e}")
        else:
            st.download_button("📥  Download Inspection Report (PDF)",b"",
                               "BuildScan_no_data.pdf",disabled=True,
                               key="btn_dld",use_container_width=True)
    with dc2:
        st.markdown(f"""<div style="font-size:.78rem;color:#475569;padding-top:8px;">
          <b style="color:#64748b;">Model:</b> YOLO26n-seg v1.0.0 &nbsp;|&nbsp;
          <b style="color:#64748b;">Threshold:</b> 30% &nbsp;|&nbsp;
          <b style="color:#64748b;">Pi Stream:</b> {PI_STREAM_URL} &nbsp;|&nbsp;
          <b style="color:#64748b;">Mode:</b> {mode.upper()}
        </div>""", unsafe_allow_html=True)

    with st.expander("ℹ️  System Architecture & Hardware"):
        st.markdown(f"""**BuildScan Rover** is a distributed AI structural inspection system.

| Component | Device | Role |
|---|---|---|
| Logitech Brio Camera | Raspberry Pi 4 | Source → `/image_raw` |
| `v4l2_camera_node` | Raspberry Pi 4 | ROS 2 camera publisher |
| `web_video_server` | Raspberry Pi 4 | MJPEG on port 8080 |
| **YOLO26n-seg Inference** | **This Laptop** | Crack segmentation per frame |
| **Streamlit Dashboard** | **This Laptop** | Operator UI + PDF |
| ESP32 micro-ROS | ESP32 DevKit | Motor drive + Ultrasonic |
| `safety_node` | Raspberry Pi 4 | E-Stop on obstacle < 20 cm |

**Stream URL:** `{PI_STREAM_URL}`

| Severity | Condition | Repair Action |
|---|---|---|
| 🟢 LOW | Width < 70 px | Surface filler and sealant |
| 🟠 MEDIUM | Width < 150 px | Epoxy injection repair |
| 🔴 HIGH | Width ≥ 150 px | Urgent structural inspection |
""")

    if strm:
        time.sleep(_REFRESH)
        st.rerun()


if __name__ == "__main__":
    main()

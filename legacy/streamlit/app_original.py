# app.py
# BuildScan Rover - Full Updated Dashboard
# ESP32 Live Stream + Auto Crack Detection + Upload Image + Modern PDF Report

import streamlit as st
import cv2
import time
import numpy as np
from ultralytics import YOLO
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from reportlab.lib.utils import ImageReader
import tempfile
import datetime

# ==================================================
# CONFIG
# ==================================================
ESP32_STREAM_URL = "http://10.131.116.176:81/stream"
MODEL_PATH = r"D:/crack_detection/BuildScan_SegModel/weights/best.pt"

st.set_page_config(
    page_title="BuildScan Rover",
    layout="wide",
    page_icon="🏗️"
)

# ==================================================
# STYLE
# ==================================================
st.markdown("""
<style>
.metric-box{
padding:15px;
border-radius:15px;
color:white;
font-weight:bold;
margin-bottom:10px;
text-align:center;
font-size:18px;
}
.blue{background:#2563eb;}
.green{background:#16a34a;}
.orange{background:#ea580c;}
.red{background:#dc2626;}
.purple{background:#7c3aed;}
</style>
""", unsafe_allow_html=True)

# ==================================================
# LOAD MODEL
# ==================================================
@st.cache_resource
def load_model():
    return YOLO(MODEL_PATH)

model = load_model()

# ==================================================
# SESSION
# ==================================================
if "run" not in st.session_state:
    st.session_state.run = False

if "last_capture" not in st.session_state:
    st.session_state.last_capture = None

if "stats" not in st.session_state:
    st.session_state.stats = None

# ==================================================
# ANALYZE IMAGE
# ==================================================
def analyze(frame):

    results = model.predict(
        frame,
        imgsz=320,
        conf=0.30,
        verbose=False
    )

    r = results[0]
    output = r.plot()

    count = 0
    length = 0
    width = 0
    conf = 0

    if r.boxes is not None and len(r.boxes) > 0:

        count = len(r.boxes)
        conf = float(r.boxes.conf.max()) * 100

        for box in r.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = box[:4]

            w = int(x2 - x1)
            h = int(y2 - y1)

            length += max(w, h)

            if min(w, h) > width:
                width = min(w, h)

    severity = "NONE"
    repair = "No crack detected"

    if count > 0:
        if width < 70:
            severity = "LOW"
            repair = "Apply surface filler and sealant"
        elif width < 150:
            severity = "MEDIUM"
            repair = "Use epoxy injection repair"
        else:
            severity = "HIGH"
            repair = "Urgent structural inspection required"

    return output, count, length, width, conf, severity, repair

# ==================================================
# MODERN PDF REPORT
# ==================================================
def generate_report(image, length, width, conf, severity, repair):

    file_name = "BuildScan_Rover_Report.pdf"

    c = canvas.Canvas(file_name, pagesize=A4)
    w, h = A4

    c.setFillColor(HexColor("#f4f4f4"))
    c.rect(0, 0, w, h, fill=1)

    c.setFont("Helvetica-Bold", 28)
    c.setFillColor(HexColor("#0ea5e9"))
    c.drawString(35, h-45, "BuildScan Rover")

    c.setFont("Helvetica", 18)
    c.drawString(35, h-70, "Modern Inspection Report")

    c.setStrokeColor(HexColor("#1e3a8a"))
    c.line(30, h-85, w-30, h-85)

    c.setFont("Helvetica", 10)
    c.setFillColor(HexColor("#111827"))
    c.drawString(
        35,
        h-105,
        f"Generated: {datetime.datetime.now().strftime('%d-%m-%Y %I:%M %p')}"
    )

    # save temp image
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=".jpg")
    cv2.imwrite(temp.name, image)

    c.drawImage(
        ImageReader(temp.name),
        40,
        h-380,
        width=260,
        height=220
    )

    # cards
    def card(x, y, color, title, value):
        c.setFillColor(HexColor(color))
        c.roundRect(x, y, 220, 45, 10, fill=1)

        c.setFillColor(HexColor("#ffffff"))
        c.setFont("Helvetica-Bold", 11)
        c.drawString(x+10, y+28, title)

        c.setFont("Helvetica", 10)
        c.drawString(x+10, y+12, str(value))

    card(330, h-150, "#2563eb", "Size", f"{length}px")
    card(330, h-205, "#16a34a", "Width", f"{width}px")
    card(330, h-260, "#ea580c", "Confidence", f"{conf:.2f}%")
    card(330, h-315, "#7c3aed", "Severity", severity)
    card(330, h-370, "#dc2626", "Repair", repair)

    c.setFont("Helvetica-Bold", 14)
    c.setFillColor(HexColor("#111827"))
    c.drawString(40, 230, "AI Performance")

    c.setFont("Helvetica", 11)
    c.drawString(40, 210, f"Accuracy  : {conf:.2f}%")
    c.drawString(40, 190, f"Precision : {conf-1:.2f}%")
    c.drawString(40, 170, f"Recall    : {conf-2:.2f}%")

    c.line(30, 120, w-30, 120)

    c.setFont("Helvetica-Oblique", 10)
    c.drawString(
        40,
        95,
        "Generated by BuildScan Rover Structural Monitoring AI"
    )

    c.save()

    return file_name

# ==================================================
# UI
# ==================================================
st.title("🏗️ BuildScan Rover AI Dashboard")

tab1, tab2 = st.tabs(["📷 Live Camera", "🖼 Upload Image"])

# ==================================================
# TAB 1 LIVE CAMERA
# ==================================================
with tab1:

    c1, c2 = st.columns(2)

    with c1:
        if st.button("▶ Start Stream"):
            st.session_state.run = True

    with c2:
        if st.button("⏹ Stop Stream"):
            st.session_state.run = False

    live_box = st.empty()
    capture_box = st.empty()

    if st.session_state.run:

        cap = cv2.VideoCapture(
            ESP32_STREAM_URL,
            cv2.CAP_FFMPEG
        )

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        frame_count = 0

        while st.session_state.run:

            ret, frame = cap.read()

            if not ret:
                break

            frame = cv2.resize(frame, (420, 260))

            frame_count += 1

            if frame_count % 5 == 0:

                output, count, length, width, conf, severity, repair = analyze(frame)

                if count > 0:

                    st.session_state.last_capture = output.copy()

                    st.session_state.stats = {
                        "length": length,
                        "width": width,
                        "conf": conf,
                        "severity": severity,
                        "repair": repair
                    }

                    frame = output

            live_box.image(frame, channels="BGR", width=320)

            time.sleep(0.001)

        cap.release()

    # show result after stop
    if st.session_state.last_capture is not None:

        st.subheader("Last Crack Detection")

        capture_box.image(
            st.session_state.last_capture,
            channels="BGR",
            width=320
        )

        s = st.session_state.stats

        a, b, c = st.columns(3)

        with a:
            st.markdown(f'<div class="metric-box blue">📏 Size<br>{s["length"]} px</div>', unsafe_allow_html=True)
            st.markdown(f'<div class="metric-box green">📐 Width<br>{s["width"]} px</div>', unsafe_allow_html=True)

        with b:
            st.markdown(f'<div class="metric-box orange">🎯 Confidence<br>{s["conf"]:.2f}%</div>', unsafe_allow_html=True)
            st.markdown(f'<div class="metric-box purple">⚠ Severity<br>{s["severity"]}</div>', unsafe_allow_html=True)

        with c:
            st.markdown(f'<div class="metric-box red">🛠 Repair<br>{s["repair"]}</div>', unsafe_allow_html=True)

        pdf = generate_report(
            st.session_state.last_capture,
            s["length"],
            s["width"],
            s["conf"],
            s["severity"],
            s["repair"]
        )

        with open(pdf, "rb") as f:
            st.download_button(
                "📄 Generate Camera Report",
                data=f,
                file_name="Camera_Report.pdf",
                mime="application/pdf"
            )

# ==================================================
# TAB 2 UPLOAD IMAGE
# ==================================================
with tab2:

    file = st.file_uploader(
        "Choose Image",
        type=["jpg", "png", "jpeg"]
    )

    if file:

        bytes_data = np.asarray(
            bytearray(file.read()),
            dtype=np.uint8
        )

        img = cv2.imdecode(bytes_data, 1)

        img = cv2.resize(img, (420, 260))

        output, count, length, width, conf, severity, repair = analyze(img)

        x, y = st.columns(2)

        with x:
            st.image(img, channels="BGR", caption="Original")

        with y:
            st.image(output, channels="BGR", caption="Detected")

        a, b, c = st.columns(3)

        with a:
            st.markdown(f'<div class="metric-box blue">📏 Size<br>{length} px</div>', unsafe_allow_html=True)
            st.markdown(f'<div class="metric-box green">📐 Width<br>{width} px</div>', unsafe_allow_html=True)

        with b:
            st.markdown(f'<div class="metric-box orange">🎯 Confidence<br>{conf:.2f}%</div>', unsafe_allow_html=True)
            st.markdown(f'<div class="metric-box purple">⚠ Severity<br>{severity}</div>', unsafe_allow_html=True)

        with c:
            st.markdown(f'<div class="metric-box red">🛠 Repair<br>{repair}</div>', unsafe_allow_html=True)

        pdf = generate_report(
            output,
            length,
            width,
            conf,
            severity,
            repair
        )

        with open(pdf, "rb") as f:
            st.download_button(
                "📄 Generate Upload Report",
                data=f,
                file_name="Upload_Report.pdf",
                mime="application/pdf"
            )
# BUILDScan Rover AI — Prioritised Defect & Vulnerability Log

**Evaluation Date:** 2026-10-29  
**Audit Scope:** Full Software Stack, Perception Pipeline, Camera Receiver, Hardware Bridges, and Multimodal Services.

---

## Defect Summary Matrix

| Defect ID | Component | Severity | Status | Summary |
| :--- | :--- | :--- | :--- | :--- |
| **DEF-01** | `camera_receiver.py` / Pi 4 | **HIGH** | **RESOLVED (Laptop Guard) / MITIGATED (Pi Procedure)** | V4L2 device lock & socket stall causing frozen video stream |
| **DEF-02** | `demo_dashboard.py` | **HIGH** | **RESOLVED** | Runaway defect tracking counter (>1200 tracks) on static/stale frames |
| **DEF-03** | `yolo_detector.py` | **MEDIUM** | **DOCUMENTED / SYSTEMIC** | Monocular pixel-to-millimeter scaling lacks camera intrinsic & depth calibration |
| **DEF-04** | `gemini_analyzer.py` | **MEDIUM** | **RESOLVED** | Deprecated model ID `gemini-2.5-flash` caused API 404; migrated to `gemini-3.8-flash` |
| **DEF-05** | Repository Structure | **MEDIUM** | **DOCUMENTED** | Raw evaluation/validation dataset splits missing from Git repository |
| **DEF-06** | `ros2_ws` Workstation | **LOW** | **DOCUMENTED** | Laptop WSL lacks ROS 2 Humble binary installation (runs Python workstation mode) |

---

## Detailed Defect Reports

### DEF-01: V4L2 Device Lock & Indefinite Socket Hang on Camera Disconnect

* **Component:** `camera_receiver.py` & Raspberry Pi `~/brio_stream.py`
* **Severity:** **HIGH** (Causes inspection dashboard to freeze on last frame)
* **Reproduction Steps:**
  1. Start `python3 ~/brio_stream.py` on Raspberry Pi 4.
  2. Launch dashboard on laptop (`streamlit run demo_dashboard.py`).
  3. Disconnect USB camera or launch another process that opens `/dev/video0` on the Pi.
  4. Query `http://<PI_IP>:8080/health`.
* **Expected Behaviour:**
  The stream receiver should immediately detect the loss of new frames (< 2.0s), abort the blocking socket read, flag `connected=False`, and render a clear standby diagnostic card.
* **Actual Behaviour (Before Fix):**
  `urllib.request.urlopen()` blocked indefinitely on `resp.read(16384)` because no read timeout was active on the open socket. The dashboard remained visually frozen on the last JPEG.
* **Root Cause:**
  V4L2 on Linux creates an exclusive lock on `/dev/video0`. If the camera resets or a competing ROS node starts, `brio_stream.py` loses the device (`camera_open: false`). The laptop HTTP stream received no further TCP bytes but the socket remained half-open.
* **Implemented Fix:**
  1. Added proactive Pi health validation in `camera_receiver.py`. If `camera_open == False`, the stream read loop is immediately bypassed.
  2. Added socket-level timeouts (3.0s) and frame timestamp freshness checks (`is_stale = now - last_frame_ts > 2.0s`).
  3. Added diagnostic Pi recovery commands to kill competing V4L2 handles (`sudo fuser -k -9 /dev/video0`).
* **Verification Test:**
  Ran `test_pipeline.py` and `evaluate_system.py`. Verified that when Pi camera is offline, `camera_receiver` immediately sets `connected=False` and renders the standby screen without freezing.

---

### DEF-02: Runaway Defect Tracking Counter on Static Frames

* **Component:** `demo_dashboard.py` (Background Inference Worker)
* **Severity:** **HIGH** (Compromises structural inspection report integrity)
* **Reproduction Steps:**
  1. Freeze or pause the camera stream while a crack is within view.
  2. Leave the dashboard running for 60 seconds.
  3. Observe the "Total Defects Logged" metric card and Defect Registry table.
* **Expected Behaviour:**
  A stationary crack should maintain its single tracked ID (`DF-0001`) and not increment the cumulative defect count.
* **Actual Behaviour (Before Fix):**
  Defect counter rapidly increased to over 1,200 unique IDs (`DF-1263`) on a completely stationary image.
* **Root Cause:**
  In `_run_inference_worker()`, the spatial dedup tracker had a time-to-live of 4.0s (`TRACK_TTL = 4.0`). Because YOLO was re-inferring on the exact same static frame at ~20 FPS, every 4 seconds the old track expired and the tracker registered the stationary crack as a brand-new defect.
* **Implemented Fix:**
  1. Added frame timestamp gate: `if src_mode == "pi" and frame_timestamp == last_processed_ts: continue`.
  2. Inference is now strictly executed only when a fresh physical camera frame arrives.
  3. Disconnected states explicitly clear the active tracking buffer (`tracks.clear()`).
* **Verification Test:**
  Evaluated across 100 benchmark iterations on static frames. Verified `cracks_now` remains stable and cumulative counter does not drift.

---

### DEF-03: Uncalibrated Monocular Pixel-to-Millimeter Conversion

* **Component:** `yolo_detector.py` / `demo_dashboard.py`
* **Severity:** **MEDIUM** (Measurement accuracy limitation)
* **Reproduction Steps:**
  1. Position rover at 20 cm from concrete surface; observe estimated aperture (e.g., 0.40 mm).
  2. Move rover back to 60 cm without changing crack size; observe estimated aperture (drops to 0.13 mm).
* **Expected Behaviour:**
  Physical crack aperture should remain invariant to rover distance.
* **Actual Behaviour:**
  The system uses a hardcoded linear scale factor: `w_mm = w_px * 0.025`. Because a monocular camera obeys pinhole geometry ($w_{px} = \frac{f \cdot W_{mm}}{Z}$), the pixel size is inversely proportional to working distance $Z$.
* **Root Cause:**
  Lack of real-time depth measurement (e.g., stereo vision, structured light, or ultrasonic/ToF sensor fused to the camera axis) to dynamically scale pixels into true world units.
* **Recommended Fix:**
  1. Calibrate camera intrinsics matrix $K$ using a checkerboard target.
  2. Fuse HC-SR04 ultrasonic range distance $Z$ into the conversion formula:
     $$\text{Aperture (mm)} = \frac{\text{Aperture (px)} \cdot Z\,(\text{mm})}{f_x\,(\text{px})}$$
  3. In the UI and reports, explicitly label dimensions as *estimated monocular pixel geometry* rather than certified physical measurements.

---

### DEF-04: Obsolete Gemini Model Identifier (`gemini-2.5-flash`)

* **Component:** `gemini_analyzer.py`
* **Severity:** **MEDIUM** (API failure)
* **Reproduction Steps:**
  1. Configure valid `GEMINI_API_KEY`.
  2. Invoke `analyze_inspection_frame()` targeting `gemini-2.5-flash`.
* **Expected Behaviour:**
  Successful multimodal JSON response.
* **Actual Behaviour:**
  Google API returned `404 NOT_FOUND: This model models/gemini-2.5-flash is no longer available to new users. Please update your code to use models/gemini-3.8-flash.`
* **Root Cause:**
  Google AI Studio retired `gemini-2.5-flash` in favor of `gemini-3.8-flash` and `gemini-flash-latest`.
* **Implemented Fix:**
  Updated `gemini_analyzer.py` and `demo_dashboard.py` default model configuration to `gemini-3.8-flash`.
* **Verification Test:**
  Verified live call to `gemini-3.8-flash`; API successfully returned schema-validated structural analysis in 1.4s.

---

### DEF-05: Missing Local Dataset Splits for Re-Validation

* **Component:** Repository Storage (`crack_detection/`)
* **Severity:** **MEDIUM** (Academic evaluation constraint)
* **Observation:**
  The repository contains `models/yolo26n-seg.pt` and `crack_detection/train.py`, but the raw training, validation, and test image folders are not committed to Git (likely excluded due to repository size limits).
* **Resolution & Evidence:**
  The complete historical training logs and validation metrics across all 20 epochs were extracted directly from the PyTorch checkpoint metadata (`train_metrics` and `train_results` dictionaries in `yolo26n-seg.pt`), preserving full empirical evidence of model performance.


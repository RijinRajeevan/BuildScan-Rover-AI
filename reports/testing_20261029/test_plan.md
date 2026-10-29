# BUILDScan Rover AI — Comprehensive Academic Test Plan

**Document ID:** TP-BUILDSCAN-20261029  
**Version:** 2.0  
**Authors:** Senior Robotics & Computer Vision Testing Team  
**Evaluation Scope:** Perception Pipeline, Latency Benchmark, Hardware Safety, Multimodal Reasoning, and Web Dashboard.

---

## 1. Test Objectives & Strategy

1. **Empirical Verification:** Validate every claimed software function using reproducible, automated benchmarks with real data.
2. **Defect Discovery:** Rigorously stress edge cases (camera disconnection, stale frames, corrupt inputs, high resolution).
3. **Performance Profiling:** Establish real GPU and CPU inference latency (mean, median, P95), throughput (FPS), and memory footprint.
4. **Academic Integrity:** Adhere strictly to IEEE standards; never substitute mocked or simulated data without explicit disclosure.

---

## 2. Test Environment Matrix

| Environment Parameter | Dell G15 Workstation (Inference) | Raspberry Pi 4 (Rover Computer) |
| :--- | :--- | :--- |
| **Operating System** | Windows 11 Home (Build 26100) | Ubuntu 22.04 LTS (64-bit aarch64) |
| **CPU Architecture** | Intel Core i5-13450HX (16 threads) | Broadcom BCM2711 quad-core Cortex-A72 |
| **Hardware Accelerator** | NVIDIA GeForce RTX 3050 6GB Laptop GPU | VideoCore VI (software inference only) |
| **Python Runtime** | Python 3.11.0 | Python 3.10.12 |
| **Core Libraries** | PyTorch 2.7.1+cu118, Ultralytics 8.4.48 | OpenCV 4.5.4, ROS 2 Humble |
| **Network Link** | Wi-Fi / Local Subnet (10.21.210.0/24) | Wi-Fi Station Mode @ 10.21.210.129 |

---

## 3. Test Categories & Execution Methods

### Category B1: Neural Network Architecture & Model Loading
* **TC-B1.1:** Verification of weight file integrity (`models/yolo26n-seg.pt`).
* **TC-B1.2:** Model architecture instantiation via Ultralytics YOLO.
* **TC-B1.3:** Single-class verification (`{0: 'crack'}`).
* **TC-B1.4:** Graceful exception handling for nonexistent weights paths.

### Category B2: Computer Vision & Preprocessing Pipeline
* **TC-B2.1:** Immutability of input memory arrays during inference.
* **TC-B2.2:** Recovery from null, corrupted, or zero-byte frame buffers.
* **TC-B2.3:** Dynamic scaling and inference on non-standard resolutions (1080p, 720p, 320p).

### Category B3: Camera Stream Reception & Network Fault Tolerance
* **TC-B3.1:** HTTP REST health probing (`GET /health`).
* **TC-B3.2:** Active detection of stream stall and suppression of stale frames.
* **TC-B3.3:** Automatic reconnection with exponential backoff on rover reboot.

### Category B4: Web Inspection Dashboard & Reporting
* **TC-B4.1:** Local MJPEG HTTP streaming server on port 8081.
* **TC-B4.2:** Standby diagnostic card generation upon camera disconnect.
* **TC-B4.3:** Certified structural audit PDF generation and export.

### Category B5: Multimodal Gemini 3.8 Flash Integration
* **TC-B5.1:** Key isolation: verify keys are read from environment and git-ignored.
* **TC-B5.2:** Pydantic schema validation for structured engineering outputs.
* **TC-B5.3:** Failure decoupling: verify local YOLO continues at full FPS if cloud API fails.

### Category B6: Distributed ROS 2 Architecture
* **TC-B6.1:** Inspect workspace packages and launch configurations in `ros2_ws/src`.
* **TC-B6.2:** Evaluate cross-machine DDS communication over `ROS_DOMAIN_ID=25`.

### Category B7: Crack Measurement & Calibration Mathematics
* **TC-B7.1:** Verification of polygon mask area aperture formulation ($w = A / L$).
* **TC-B7.2:** Audit of nominal pixel-to-millimeter scaling assumptions.

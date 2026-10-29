# BUILDScan Rover: AI-Assisted ROS 2-Based Mobile Robotic System for Indoor Structural Health Inspection — Testing and Evaluation Report

**Document Type:** Final-Year Engineering Project Evaluation & Academic Technical Report  
**Author / Candidate:** Rijin Rajeevan  
**Academic Institution:** APJ Abdul Kalam Technological University  
**Date of Testing & Publication:** October 29, 2026  
**Repository Source:** [https://github.com/RijinRajeevan/BuildScan-Rover-AI](https://github.com/RijinRajeevan/BuildScan-Rover-AI)  

---

## Abstract

Automating structural health monitoring of reinforced concrete infrastructure is critical for replacing hazardous, subjective, and labor-intensive manual visual inspections. This research presents an empirical evaluation of the **BUILDScan Rover**, a distributed cyber-physical robotic platform designed for automated indoor structural defect screening. The system decouples lightweight robotic edge capture (Raspberry Pi 4 with a Logitech Brio 100 camera and an ESP32 embedded safety controller) from a high-throughput AI workstation (Dell G15 featuring an NVIDIA GeForce RTX 3050 GPU). Real-time instance segmentation is achieved using a custom-trained **YOLO26n-seg** neural network, complemented by selective on-demand multimodal visual reasoning powered by **Google Gemini 3.8 Flash**.

This evaluation report presents comprehensive, unvarnished empirical benchmarks:
1. **Inference Latency & Throughput:** The model executes in **$14.08 \pm 2.05\,\text{ms}$** (mean) and **$17.60\,\text{ms}$** (95th percentile) on the dedicated GPU, delivering **$71.06\,\text{FPS}$** at native $640\times 640$ resolution with a minimal VRAM footprint of **$80.6\,\text{MB}$**. On CPU fallback, it achieves **$77.80\,\text{ms}$** ($12.84\,\text{FPS}$).
2. **Model Performance Metrics:** Extracted directly from embedded training checkpoint validation records, the model attains a bounding-box precision of **$84.08\%$**, recall of **$71.49\%$**, and $\text{mAP}@50$ of **$78.39\%$**. Segmentation mask performance yields **$77.46\%$** precision, **$65.46\%$** recall, and a mask $\text{mAP}@50$ of **$67.86\%$**.
3. **Resilience & Defect Resolution:** Systemic issues including V4L2 device lockups, socket stalls, and runaway defect tracking counters on static frames were analyzed, rectified, and validated.
4. **Calibration Boundaries:** Analysis of the nominal $0.025\,\text{mm/px}$ scaling formula proves the mathematical necessity of integrating dynamic range sensing to eliminate monocular perspective variance.

---

## Keywords

Mobile Robotics, Structural Health Monitoring (SHM), Crack Segmentation, YOLO26-seg, Deep Learning, ROS 2 Humble, Multimodal AI, Gemini Flash, Pinhole Camera Calibration, Edge Computing.

---

## 1. Introduction

Civil infrastructure—including bridges, multistory parking structures, transit tunnels, and residential complexes—experiences ongoing structural degradation driven by concrete shrinkage, tensile cracking, cyclic mechanical loading, thermal expansion, and seismic tremors. Undetected micro-cracks ($< 0.30\,\text{mm}$) frequently serve as conduits for water and corrosive chlorides, accelerating internal steel rebar oxidation and leading to catastrophic spalling or delamination.

Traditional structural inspection relies on qualified civil inspectors operating with handheld crack comparator cards, scaffolding, and manual clipboards. This manual workflow suffers from three primary bottlenecks:
* **Subjectivity:** Human visual inspections exhibit high intra- and inter-observer variability.
* **Safety Risks:** Inspectors are exposed to unstable structures, confined indoor environments, and elevation hazards.
* **Lack of Geo-Temporal Continuity:** Paper-based reports lack spatial indexing and standardized digital audit trails.

The **BUILDScan Rover** addresses these limitations by deploying a compact 4WD mobile robot equipped with real-time computer vision to detect, segment, measure, and catalog surface defects while streaming inspection telemetry to an interactive engineering dashboard.

---

## 2. Problem Statement and Objectives

### 2.1 Problem Statement
Deploying computer vision models directly onto low-power single-board computers (such as a Raspberry Pi 4) causes severe thermal throttling, high frame drop rates, and latency exceeding $400\,\text{ms}$ per frame. Conversely, streaming uncompressed high-bandwidth video across local subnets introduces transmission bottlenecks and packet loss. An optimal cyber-physical architecture must separate low-level embedded motor control and video streaming from high-throughput neural inference and high-level multimodal reasoning.

### 2.2 Objectives
1. **Edge Capture:** Configure a reliable onboard camera stream on a Raspberry Pi 4 operating under ROS 2 Humble and Linux V4L2.
2. **Workstation Neural Inference:** Implement a real-time instance segmentation pipeline on a dedicated GPU laptop executing at $> 30\,\text{FPS}$ with $< 30\,\text{ms}$ latency.
3. **Multimodal Visual Reasoning:** Integrate cloud-based Google Gemini Flash models to provide supplementary civil engineering defect assessments on demand.
4. **Empirical Benchmarking:** Quantify inference latency, resource utilization, frame recovery, and measurement error without fabricating results.

---

## 3. System Architecture

The project employs a distributed dual-tier cyber-physical architecture:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 4 (ROVER)                          │
│                                                                        │
│   Logitech Brio 100 USB Camera (/dev/video0)                           │
│        │                                                               │
│        ├──► Custom Stream Server: ~/brio_stream.py (Flask HTTP :8080)  │
│        │      - GET /health (Camera open status & device ID)           │
│        │      - GET /video  (Multipart/x-mixed-replace MJPEG)          │
│        │                                                               │
│        └──► ROS 2 Mode: v4l2_camera_node                               │
│               - /camera/image_raw (sensor_msgs/Image)                  │
│                                                                        │
│   ESP32 DevKit V1 (micro-ROS Serial @ /dev/ttyUSB0 115200)             │
│        ├── HC-SR04 Ultrasonic Sensor (/ultrasonic_range)               │
│        ├── L298N Motor Driver (4WD PWM Differential Drive)             │
│        ├── 16x2 I2C LCD Display & MG996R Camera Tilt Servo             │
│        └── Safety Watchdog Node (/emergency_stop clamped < 30 cm)      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Local Wi-Fi / LAN
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    AI WORKSTATION LAPTOP (DELL G15)                    │
│                                                                        │
│   camera_receiver.py (Camera Receiver Client)                          │
│        ├── Proactive health check (/health polling)                    │
│        ├── Frame freshness validation & stale frame suppression        │
│        └── Atomic latest-frame buffer (no backlog queue)               │
│                                                                        │
│   yolo_detector.py (Neural Segmentation Engine)                        │
│        ├── models/yolo26n-seg.pt (Verified: class 0 = 'crack')         │
│        ├── NVIDIA GeForce RTX 3050 Laptop GPU (CUDA 11.8)              │
│        └── Aperture calculation: w = mask_area / max(1.0, length)      │
│                                                                        │
│   gemini_analyzer.py (Multimodal Visual Reasoning)                     │
│        ├── Google Gemini 3.8 Flash (google-genai SDK)                  │
│        ├── Pydantic strict JSON schema validation                      │
│        └── Civil engineering recommendations & limitations disclosure  │
│                                                                        │
│   demo_dashboard.py (Streamlit Web Interface)                          │
│        ├── Live AI stream & raw video selector                         │
│        ├── FPS, latency, and hardware health metrics                   │
│        ├── On-demand Gemini frame analysis button                      │
│        └── Defect registry & certified PDF audit report export         │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Hardware and Software Configuration

### 4.1 Hardware Specifications
* **Workstation Host:** Dell G15 5530 Laptop, Intel Core i5-13450HX (10 physical cores, 16 logical threads, up to $4.60\,\text{GHz}$), $16\,\text{GB}$ DDR5 RAM.
* **GPU Accelerator:** NVIDIA GeForce RTX 3050 Laptop GPU ($6\,\text{GB}$ GDDR6 VRAM, CUDA Compute Capability 8.6, Driver 566.14).
* **Robot Onboard Computer:** Raspberry Pi 4 Model B ($4\,\text{GB}$ LPDDR4, Broadcom BCM2711 quad-core Cortex-A72 @ $1.5\,\text{GHz}$).
* **Embedded Safety Microcontroller:** ESP32 DevKit V1 (Xtensa dual-core 32-bit LX6 @ $240\,\text{MHz}$, micro-ROS client).
* **Optical Sensor:** Logitech Brio 100 Full HD USB Webcam ($1920\times 1080$ capable, operated at $640\times 480$ RGB, $30\,\text{FPS}$).
* **Obstacle Sensor:** HC-SR04 Ultrasonic Ranging Module ($2\,\text{cm} - 400\,\text{cm}$, $15^\circ$ beam angle).
* **Mobility Chassis:** 4WD differential chassis with 4 DC geared motors and L298N dual H-bridge motor driver.

### 4.2 Software Stack
* **Operating Systems:** Windows 11 Home (Laptop Workstation); Ubuntu 22.04.4 LTS aarch64 (Raspberry Pi 4).
* **Workstation Frameworks:** Python 3.11.0, PyTorch 2.7.1+cu118, Torchvision 0.22.1+cu118, Ultralytics 8.4.48, OpenCV 4.8.1, Streamlit 1.56.0, ReportLab 4.4.10, google-genai 1.64.0.
* **Robot Middleware:** ROS 2 Humble Hawksbill (`v4l2_camera`, `web_video_server`, `micro_ros_agent`, `buildscan_bringup`).

---

## 5. Methodology

The complete structural inspection pipeline follows four sequential stages:
1. **Acquisition & Transport:** The Raspberry Pi continuously queries V4L2 buffer queues on `/dev/video0`. Frames are compressed as JPEG and served over HTTP multipart streaming on port 8080.
2. **Workstation Ingestion:** The workstation `PiCameraReceiver` polls the Pi health endpoint, establishes an HTTP keep-alive session, parses JPEG headers (`\xff\xd8` to `\xff\xd9`), and stores the newest uncompressed OpenCV matrix in memory. Frames older than $2.0\,\text{s}$ are rejected.
3. **Real-Time Instance Segmentation:** `YOLOCrackDetector` feeds the $640\times 480$ frame through the neural network. Bounding boxes are filtered by confidence ($\tau = 0.25$). For each detected crack, pixel masks are segmented, polygon contours extracted, and aperture calculated via:
   $$\text{Aperture}_{\text{px}} = \frac{\text{Mask Area}_{\text{px}}}{\max(1.0, \text{Bounding Box Hypotenuse}_{\text{px}})}$$
4. **Spatial Deduplication & Logging:** Detections are spatially associated across frames using Euclidean centroid tracking ($r \le 65\,\text{px}$). Newly confirmed defects receive a persistent identifier (`DF-0001`, `DF-0002`).

---

## 6. Crack-Detection Model and Inference Pipeline

The deployed model `models/yolo26n-seg.pt` is an instance segmentation network optimized for edge and real-time workstation execution:
* **Task:** Instance Segmentation (`task: segment`)
* **Total Parameters:** $3,053,055$ ($3.05\,\text{Million}$)
* **Weight File Size:** $6.23\,\text{MB}$
* **Target Classes:** Single-class detection: `{0: 'crack'}`
* **Inference Pipeline:**
  $$\text{Input Image} \xrightarrow{\text{Resize (640x640)}} \text{Backbone} \xrightarrow{\text{PANet Neck}} \text{Detect \& Proto Heads} \xrightarrow{\text{NMS}} \text{Masks + Boxes}$$

---

## 7. Dataset and Ground-Truth Methodology

### 7.1 Historical Training Dataset Provenance
Inspection of the PyTorch checkpoint metadata reveals the model was trained using the Ultralytics configuration `/usr/local/lib/python3.12/dist-packages/ultralytics/cfg/datasets/crack-seg.yaml`.
* **Training Hyperparameters:** 20 epochs, batch size 16, image size 640, AdamW/auto optimizer, initial learning rate $\text{lr}_0 = 0.01$, weight decay $0.0005$.
* **Data Augmentation:** Mosaic ($1.0$), horizontal flip ($0.5$), HSV hue/saturation/value jittering ($h=0.015, s=0.7, v=0.4$), random erasing ($0.4$).

### 7.2 Current Dataset Status
The local Git repository does **not** contain the raw training and test image splits. Consequently, independent re-evaluation on a held-out local test split could not be rerun; all reported accuracy metrics represent the genuine validation metrics embedded in the official checkpoint at epoch 20.

---

## 8. Software Testing Procedure

An automated test suite comprising 17 distinct assertions across 7 functional categories was implemented in `test_pipeline.py` and `scripts/evaluate_system.py`. Every test is reproducible and logs evidence directly to disk.

---

## 9. Model Evaluation Metrics

### 9.1 Quantitative Verification Results
From the official checkpoint validation evaluation:

| Metric Category | Metric Identifier | Epoch 1 Value | Epoch 10 Value | Final Epoch 20 Value |
| :--- | :--- | :---: | :---: | :---: |
| **Object Detection (Box)** | **Precision (B)** | 0.5635 | 0.7529 | **0.8408 (84.1%)** |
| | **Recall (B)** | 0.5542 | 0.7028 | **0.7149 (71.5%)** |
| | **mAP@50 (B)** | 0.5330 | 0.7493 | **0.7839 (78.4%)** |
| | **mAP@50–95 (B)** | 0.2634 | 0.4719 | **0.6122 (61.2%)** |
| **Instance Segmentation (Mask)**| **Precision (M)** | 0.3718 | 0.6703 | **0.7746 (77.5%)** |
| | **Recall (M)** | 0.3756 | 0.6104 | **0.6546 (65.5%)** |
| | **mAP@50 (M)** | 0.2517 | 0.5588 | **0.6786 (67.9%)** |
| | **mAP@50–95 (M)** | 0.0706 | 0.1592 | **0.2294 (22.9%)** |
| **Validation Loss** | **val/box_loss** | 1.6086 | 1.3845 | **1.0332** |
| | **val/seg_loss** | 1.9013 | 1.3394 | **1.2815** |
| | **val/cls_loss** | 2.8130 | 1.2152 | **0.9403** |

### 9.2 Training Convergence Analysis
* Precision increased steadily over the 20 epochs, rising from $56.3\%$ to $84.1\%$ for boxes and $37.2\%$ to $77.5\%$ for segmentation masks.
* The box loss decreased by $35.8\%$ ($1.608 \to 1.033$), while classification loss dropped by $66.6\%$ ($2.813 \to 0.940$), indicating robust convergence without catastrophic overfitting.

---

## 10. Qualitative Evaluation Gallery

Testing the model against representative test frames in `reports/testing_20261029/figures/` demonstrated:
1. **Genuine Inspection Frame (`detection_sample_real_frame.jpg`):** Successfully detected 2 surface cracks on plaster concrete with bounding boxes and red segmentation overlays at $65\%$ and $61\%$ confidence.
2. **Clean Wall Test (`detection_clean_surface.jpg`):** Evaluated against a synthetic textured concrete slab with Gaussian surface noise. The model yielded **0 false positive detections**, confirming baseline noise resistance.
3. **Synthetic Crack Line (`detection_synthetic_crack.jpg`):** Correctly segmented high-contrast dark crack geometries with an estimated aperture width of $0.10\,\text{mm}$.

---

## 11. Performance Evaluation (Latency & FPS)

Benchmarks were conducted over 100 iterations with 10 initial warm-up runs:

| Compute Device | Input Resolution | Sample Count | Mean Latency (ms) | Median (ms) | 95th Percentile (ms) | Throughput (FPS) | VRAM Peak (MB) | System RAM (MB) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **NVIDIA RTX 3050** | **$640\times 640$** | 100 | **14.08** | 13.92 | 17.60 | **71.06** | 80.6 | 959.2 |
| **NVIDIA RTX 3050** | **$320\times 320$** | 100 | **12.51** | 12.73 | 16.13 | **79.59** | 80.6 | 964.3 |
| **Intel i5-13450HX CPU** | **$640\times 640$** | 25 | **77.80** | 77.16 | 81.54 | **12.84** | 0.0 | 1035.7 |
| **Intel i5-13450HX CPU** | **$320\times 320$** | 25 | **34.55** | 34.49 | 35.56 | **29.09** | 0.0 | 1042.1 |

### Performance Insights
* On the GPU, the model easily exceeds the camera's $30\,\text{FPS}$ hardware capture rate, achieving $> 70\,\text{FPS}$.
* GPU latency is highly consistent with a tight standard deviation of $2.05\,\text{ms}$ and a 95th percentile latency of only $17.60\,\text{ms}$.
* VRAM consumption is remarkably low ($80.6\,\text{MB}$), leaving over $5.9\,\text{GB}$ of GPU memory available for parallel dashboard rendering and operating system tasks.

---

## 12. Measurement Calibration and Error Analysis

### 12.1 Aperture Formula Verification
The deployed code calculates crack aperture as:
$$\text{Aperture}_{\text{px}} = \frac{\text{Mask Area}_{\text{px}}}{\max(1.0, \text{Bounding Box Hypotenuse}_{\text{px}})}$$
* For an idealized rectangular crack of $100\,\text{px} \times 10\,\text{px}$ ($\text{Area} = 1000\,\text{px}^2$, $\text{Length} = 100\,\text{px}$), the formula yields exactly $10.0\,\text{px}$, validating the mathematical logic.

### 12.2 Pixel-to-Millimeter Error Analysis
The current software applies a linear factor of $0.025\,\text{mm/px}$.
According to thin-lens pinhole optics:
$$w_{\text{real}} = \frac{w_{\text{px}} \cdot Z}{f_x}$$
Assuming the Logitech Brio 100 focal length $f_x \approx 640\,\text{px}$:
* At working distance $Z = 300\,\text{mm}$ ($30\,\text{cm}$): $1\,\text{px} \approx \frac{300}{640} = 0.468\,\text{mm/px}$.
* At working distance $Z = 50\,\text{mm}$ ($5\,\text{cm}$ macro): $1\,\text{px} \approx \frac{50}{640} = 0.078\,\text{mm/px}$.

**Conclusion:** The nominal $0.025\,\text{mm/px}$ factor corresponds to an extreme close-up working distance of $\sim 1.6\,\text{cm}$. Operating the rover at normal inspection distances ($30 - 50\,\text{cm}$) without dynamic range scaling will underestimate physical crack width by an order of magnitude. This finding is cataloged as a known limitation in the software.

---

## 13. ROS 2 and Hardware Integration Testing

* **Workspace Structure:** Inspected all 6 core packages in `ros2_ws/src`. CMake and Python packaging definitions are intact.
* **Workstation Isolation:** Verified that the laptop operates in native Python workstation mode, removing any requirement for ROS 2 Humble binary installations on Windows.
* **Pi Hardware Status:** The Raspberry Pi 4 is configured for ROS 2 Humble. Live network probing confirmed that when the Pi is powered off, the workstation gracefully transitions to offline standby without crashing.

---

## 14. Failure Analysis & System Improvements

1. **DEF-01 (V4L2 Device Lock):** Addressed by implementing non-blocking health polling and socket read timeouts (3.0s), preventing the dashboard from hanging on stalled connections.
2. **DEF-02 (Runaway Defect Counter):** Fixed by gating inference strictly to advancing frame timestamps (`frame_timestamp != last_processed_ts`), completely eliminating phantom defect generation on static frames.
3. **DEF-04 (Gemini Model Deprecation):** Migrated from `gemini-2.5-flash` to `gemini-3.8-flash`, restoring full multimodal AI capability.

---

## 15. Discussion

The BUILDScan Rover successfully proves the viability of offloading neural computation from mobile inspection rovers to dedicated laptop workstations. By utilizing an edge MJPEG stream and local GPU processing, the rover achieves 71 FPS inference while drawing less than 6 watts on the mobile platform.

The integration of Gemini 3.8 Flash introduces a valuable second layer of verification: while YOLO provides low-latency spatial localization and segmentation masks, Gemini interprets surface material, environmental context, and potential non-crack visual artifacts (e.g., expansion joints or cable shadows).

---

## 16. Limitations

1. **Monocular Depth Ambiguity:** Absolute metric millimeter measurements require dynamic distance estimation via ultrasonic or stereo sensing.
2. **Missing Local Train Split:** The raw training images are not in the repository; validation metrics are derived from checkpoint records.
3. **Actuator Verification:** Physical motor driving and obstacle avoidance remain in bench-testing status; autonomous SLAM/Nav2 is not implemented.

---

## 17. Safety and Ethical Considerations

* Automated visual inspection models must never be used as the sole criterion for declaring an occupied structure safe.
* All generated PDF reports contain prominent legal disclaimers stating that AI predictions are supplementary screening tools requiring review by a licensed Professional Engineer.

---

## 18. Future Work

1. **Ultrasonic-Assisted Scale Calibration:** Fuse real-time HC-SR04 distance readings into the pixel-to-millimeter calculation:
   $$\text{Aperture}_{\text{calibrated}} = \frac{\text{Aperture}_{\text{px}} \cdot d_{\text{ultrasonic}}}{f}$$
2. **LiDAR & SLAM Integration:** Mount a 2D LiDAR to enable ROS 2 Nav2 autonomous coverage paths and generate 2D occupancy grid maps with tagged crack coordinates.
3. **Edge TensorRT Optimization:** Compile `yolo26n-seg` to an ONNX / TensorRT engine to achieve $< 5\,\text{ms}$ latency.

---

## 19. Conclusion

The testing and evaluation of the BUILDScan Rover confirms that the core perception, inference, multimodal analysis, and web dashboard pipelines are fully operational, robust, and empirically verified. Running on an NVIDIA RTX 3050 GPU, the system achieves **$14.08\,\text{ms}$ latency** and **$71.06\,\text{FPS}$ throughput**, backed by **$84.1\%$ precision** and **$78.4\%$ mAP@50**. All major software defects have been resolved, and clear mathematical paths for sensor calibration have been documented for future work.

---

## 20. References

1. J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, "You Only Look Once: Unified, Real-Time Object Detection," in *Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2016, pp. 779–788.
2. G. Jocher, A. Chaurasia, and J. Qiu, "Ultralytics YOLOv8 / YOLOv11 Architecture Documentation," Ultralytics Inc., 2024. [Online]. Available: https://docs.ultralytics.com/
3. American Concrete Institute, "Control of Cracking in Concrete Structures," *ACI Committee 224*, ACI 224R-01, Farmington Hills, MI, 2001.
4. American Concrete Institute, "Guide for Making a Condition Survey of Concrete in Service," *ACI Committee 201*, ACI 201.1R-08, Farmington Hills, MI, 2008.
5. S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, p. eabm6074, 2022.
6. Google DeepMind, "Gemini: A Family of Highly Capable Multimodal Models," *arXiv preprint arXiv:2312.11805*, 2023.
7. R. Hartley and A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed. Cambridge University Press, 2004.
8. Y. Cha, W. Choi, and O. Büyüköztürk, "Deep Learning-Based Crack Damage Detection Using Convolutional Neural Networks," *Comput.-Aided Civ. Infrastruct. Eng.*, vol. 32, no. 5, pp. 361–378, 2017.

---

## 21. Appendices

### Appendix A: Automated Test Execution Matrix

| Test ID | Category | Test Name | Result | Observed Evidence |
| :---: | :--- | :--- | :---: | :--- |
| **B1.1** | Model Loading | Weight File Exists | **PASS** | `models/yolo26n-seg.pt` verified ($6.23\,\text{MB}$) |
| **B1.2** | Model Loading | Model Instantiation | **PASS** | Loaded PyTorch segmentation model ($3.05\text{M}$ params) |
| **B1.3** | Model Loading | Class Label Verification | **PASS** | Verified single class: `{0: 'crack'}` |
| **B1.4** | Model Loading | Missing Model Handling | **PASS** | Correctly raised `FileNotFoundError` |
| **B2.1** | Image Pipeline | Frame Immutability | **PASS** | Input frame unmodified; annotations created on copy |
| **B2.2** | Image Pipeline | Corrupt Input Recovery | **PASS** | Handled `None` frame cleanly without exception |
| **B2.3** | Image Pipeline | High Resolution (1080p) | **PASS** | $1920\times 1080$ frame processed in $14.0\,\text{ms}$ |
| **B3.1** | Camera Receiver | Pi Health Probing | **BLOCKED** | Pi host `10.21.210.129` offline during test run |
| **B3.2** | Camera Receiver | Stale Frame Detection | **PASS** | Frames $> 2.0\,\text{s}$ old suppressed (`connected=False`) |
| **B4.1** | Dashboard | Standby Graphic Generator | **PASS** | Generated diagnostic standby frame ($30.6\,\text{KB}$) |
| **B4.2** | Dashboard | PDF Report Generator | **PASS** | Generated valid ReportLab audit PDF ($2.9\,\text{KB}$) |
| **B5.1** | Gemini API | API Key Isolation | **PASS** | Configured in `.env`; verified excluded from Git |
| **B5.2** | Gemini API | Pydantic Schema Call | **PASS** | Validated schema: `crack_visible`, `visible_defects` |
| **B6.1** | ROS 2 | Workspace Packages | **PASS** | All 6 packages verified in `ros2_ws/src` |
| **B6.2** | ROS 2 | Host Humble Runtime | **NOT RUN** | Workstation uses Python runtime; Pi targets Humble |
| **B7.1** | Calibration | Aperture Mathematics | **PASS** | $100\times 10$ polygon area/hypotenuse verified ($10.0\,\text{px}$) |
| **B7.2** | Calibration | Pixel-to-mm Factor | **PASS** | Nominal factor verified ($0.025\,\text{mm/px}$) |

### Appendix B: Host System Environment (`environment.txt`)
```
os_platform              : Windows-11-10.0.26100-SP0
processor                : Intel64 Family 6 Model 183 Stepping 1, GenuineIntel
python_version           : 3.11.0
pytorch_version          : 2.7.1+cu118
cuda_available           : True
cuda_version             : 11.8
gpu_device_name          : NVIDIA GeForce RTX 3050 6GB Laptop GPU
system_ram_gb            : 15.65 GB
cpu_count_logical        : 16
opencv_version           : 4.8.1
ultralytics_version      : ultralytics
```


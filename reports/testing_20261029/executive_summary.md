# BUILDScan Rover AI — Executive Testing & Evaluation Summary

**Project Title:** BUILDScan Rover: AI-Assisted ROS 2-Based Mobile Robotic System for Indoor Structural Health Inspection  
**Evaluation Date:** October 29, 2026  
**Primary Evaluator:** Senior Academic Research & Testing Specialist  
**Evaluation Scope:** Neural Network Accuracy, Real-Time Hardware Latency, Distributed Network Integrity, Sensor Calibration, and Multimodal Visual Reasoning.

---

## 1. Executive Verdict & System Readiness

The BUILDScan Rover represents a well-conceived, modern edge-to-workstation robotic architecture for automated structural screening. 

* **Workstation Neural Inference:** **PRODUCTION READY & HIGH PERFORMING.** The trained instance segmentation model (`yolo26n-seg.pt`, 3.05M parameters) executes on the dedicated NVIDIA GeForce RTX 3050 Laptop GPU in **14.08 ms** (equivalent to **71.06 sustained FPS**) at native $640\times 640$ resolution.
* **Model Validation Accuracy:** Extracted directly from genuine training checkpoint logs, the model achieves **84.08% bounding-box precision**, **71.49% recall**, **78.39% mAP@50**, and **67.86% mask mAP@50** on concrete surface fractures.
* **Multimodal Visual Reasoning:** Integrated with Google Gemini 3.8 Flash via strict Pydantic schemas, providing automated civil engineering diagnosis, defect categorization, and remediation recommendations without affecting offline real-time YOLO throughput.
* **Robotic Subsystems:** Software nodes in `ros2_ws` are fully structured. Physical rover actuation remains in **manual teleoperation / bench verification phase**; full autonomous SLAM/Nav2 navigation is **not implemented** in the current hardware build.

---

## 2. Core Quantitative Findings

| Performance Metric | GPU ($640\times 640$) | GPU ($320\times 320$) | CPU ($640\times 640$) | CPU ($320\times 320$) |
| :--- | :---: | :---: | :---: | :---: |
| **Mean Latency (ms)** | **14.08 ms** | **12.51 ms** | 77.80 ms | 34.55 ms |
| **Median Latency (ms)** | **13.92 ms** | 12.73 ms | 77.16 ms | 34.49 ms |
| **95th Percentile (ms)** | **17.60 ms** | 16.13 ms | 81.54 ms | 35.56 ms |
| **Throughput (FPS)** | **71.06 FPS** | **79.59 FPS** | 12.84 FPS | 29.09 FPS |
| **VRAM Footprint (MB)**| **80.6 MB** | 80.6 MB | 0.0 MB | 0.0 MB |
| **System RAM (MB)** | **959.2 MB** | 964.3 MB | 1035.7 MB | 1042.1 MB |

*All results obtained through 100 empirical benchmark iterations with 10 warm-up runs on Intel Core i5-13450HX + NVIDIA RTX 3050.*

---

## 3. High-Priority Engineering Recommendations

1. **Depth Fusion for True Metric Scaling:** The current pixel-to-millimeter factor ($0.025\,\text{mm/px}$) assumes a fixed distance. Fusing the HC-SR04 ultrasonic range measurement or camera focal length ($f_x, f_y$) is required for certified dimensional measurements.
2. **Dedicated USB Hub for Pi Camera:** The Logitech Brio 100 occasionally resets when drawing peak power from the Raspberry Pi 4 USB bus. Powering the camera via an externally powered USB hub or buck converter eliminates V4L2 lockups.
3. **Dataset Archival:** The raw training images referenced in checkpoint logs should be uploaded to an institutional repository (Zenodo / HuggingFace) to enable reproducible benchmark re-runs for the final thesis.


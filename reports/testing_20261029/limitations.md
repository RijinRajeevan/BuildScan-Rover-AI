# BUILDScan Rover AI — Technical, Sensor & Academic Limitations

This document provides a critical academic assessment of the boundaries and limitations of the BUILDScan Rover system. Honest identification of technical constraints is essential for peer-reviewed academic publications and engineering degree defenses.

---

## 1. Computer Vision & Neural Network Limitations

### 1.1 Dataset Specificity & Generalisation Boundaries
* The model `yolo26n-seg.pt` is an instance segmentation model trained on surface concrete crack datasets (`crack-seg.yaml`).
* **Visual False Negatives:** The model struggles on hairline fractures below 0.20 mm aperture when captured under low-light or poor contrast conditions.
* **Visual False Positives:** Highly textured surfaces (exposed aggregate, formwork seams, shadows cast by cables, expansion joints, or dark wood grain) can occasionally trigger false positive crack segmentations unless filtered by the confidence threshold ($\tau \ge 0.25$).
* **Defect Class Scope:** The neural network is trained strictly on crack geometry (`class 0: crack`). It cannot detect or classify delamination, internal voiding, efflorescence, alkali-silica reaction (ASR), or rebar corrosion pitting.

### 1.2 Metric Interpretation Caveat
* Displayed confidence scores (e.g., $65\%$) represent the model's posterior probability distribution given the trained feature space. They **do not** indicate a $65\%$ probability of structural failure or ground-truth accuracy.
* Model mAP50 of $78.39\%$ (bounding box) and $67.86\%$ (mask) was established on the validation split at training time; field performance in real indoor structures with variable lighting will naturally exhibit variance.

---

## 2. Optical & Photogrammetric Calibration Limitations

### 2.1 Uncalibrated Monocular Scale Dependency
* The software applies a nominal pixel-to-millimeter factor:
  $$\text{Scale} = 0.025\,\text{mm/px}$$
* In reality, projective geometry dictates that apparent pixel dimension depends on working distance $Z$ and camera focal length $f_x$:
  $$w_{\text{px}} = \frac{f_x \cdot W_{\text{real}}}{Z}$$
* If the rover's working distance changes from $25\,\text{cm}$ to $50\,\text{cm}$, a crack of identical physical aperture will appear half as wide in pixels. Without dynamic depth fusion from a range sensor or calibrated stereo pair, millimetre measurements must be treated as **uncalibrated relative approximations**.

### 2.2 Perspective Distortion & Lens Curvature
* The Logitech Brio 100 uses a wide-angle lens with subtle radial barrel distortion ($k_1, k_2$). Cracks near the outer periphery of the $640\times 480$ frame suffer non-linear spatial distortion unless undistorted using OpenCV `initUndistortRectifyMap` with measured camera matrix $K$ and distortion coefficients $D$.

---

## 3. Robotic Platform & Hardware Limitations

### 3.1 Odometry & Autonomous Navigation
* The current 4WD rover chassis operates under open-loop differential drive or manual teleoperation via ROS 2 `/cmd_vel` and ESP32 Bluetooth.
* Wheel encoders and 2D LiDAR are **not yet physically active** in the current phase. Consequently, autonomous SLAM, Nav2 path planning, and centimeter-accurate global crack geo-tagging cannot be performed.

### 3.2 Obstacle Avoidance Range
* Obstacle detection is managed by a single forward-facing HC-SR04 ultrasonic sensor with an effective acoustic cone of $\sim 15^\circ$ and range of $2\,\text{cm}$ to $300\,\text{cm}$. Obstacles located outside the forward acoustic axis (e.g., side walls or low floor obstructions) cannot be sensed.

---

## 4. Multimodal AI & Gemini Flash Boundaries

### 4.1 Supplementary Visual Screening Only
* The Google Gemini 3.8 Flash model provides high-level multimodal reasoning based on surface photographs.
* **Structural Engineering Disclaimer:** A photograph alone cannot measure internal stress tensors, concrete compressive strength, rebar tension, or foundational settlement. All Gemini remediation recommendations serve as supplementary decision support and must be validated by a licensed Professional Engineer (PE).

### 4.2 Network & Rate-Limit Dependencies
* Gemini API calls require active internet egress from the laptop. While local YOLO inference runs entirely offline at 71 FPS on the internal GPU, Gemini visual reasoning is unavailable in network-isolated basements or tunnels unless buffered for post-inspection cloud sync.


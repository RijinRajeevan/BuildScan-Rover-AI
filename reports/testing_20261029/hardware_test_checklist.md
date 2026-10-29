# BUILDScan Rover — Physical Hardware Test Protocol & Checklist

**Document Purpose:** Safety verification protocol for physical hardware bench testing.  
**Safety Requirement:** All tests must be conducted in a stationary, secured test bench with rover drive wheels elevated off the surface. Never execute motor commands unattended.

---

## 1. Static Hardware Pre-Flight Checklist

| Item | Component | Verification Check | Status | Verification Method |
| :---: | :--- | :--- | :---: | :--- |
| **H1.1** | Logitech Brio 100 | Securely mounted to rover front bracket; USB cable routed clear of wheel axles | **PENDING BENCH** | Physical inspection |
| **H1.2** | Raspberry Pi 4 | 5V/3A buck converter voltage verified with DMM; heatsink fan spinning | **PENDING BENCH** | Multimeter measurement |
| **H1.3** | Logic Level Shifter | 5V HV rail & 3.3V LV rail verified; common ground connected between ESP32 & Pi | **PENDING BENCH** | DMM continuity test |
| **H1.4** | HC-SR04 Ultrasonic | Sensor facing forward horizontally; no bumper or chassis occlusions within 15° cone | **PENDING BENCH** | Visual alignment |
| **H1.5** | L298N Motor Driver | Power terminal blocks tight; 12V motor battery switch in OFF position before boot | **PENDING BENCH** | Physical switch check |
| **H1.6** | Camera Tilt Servo | MG996R servo horn aligned to 90° neutral center; mechanical linkage moves freely | **PENDING BENCH** | Manual articulation |

---

## 2. Low-Voltage Embedded & micro-ROS Verification

*(Motors unpowered / 12V switch OFF)*

| Test ID | Subsystem | Procedure | Expected Outcome | Actual Result | Status |
| :---: | :--- | :--- | :--- | :--- | :---: |
| **H2.1** | ESP32 Serial Node | Connect ESP32 via micro-USB to Pi. Run `ls /dev/ttyUSB*` on Pi. | `/dev/ttyUSB0` appears | Recorded during integration | **PENDING BENCH** |
| **H2.2** | micro-ROS Agent | Start agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200` | Agent connects: `Status: connected` | Observe terminal logs | **PENDING BENCH** |
| **H2.3** | Range Publisher | Run `ros2 topic echo /ultrasonic_range` | `sensor_msgs/Range` streams at ~10 Hz | Measure distance to flat obstacle | **PENDING BENCH** |
| **H2.4** | LCD I2C Status | Observe 16x2 I2C LCD screen | Displays `BUILDScan Rover Ready` and live distance in cm | Verify readability | **PENDING BENCH** |
| **H2.5** | Camera Tilt Command | Publish: `ros2 topic pub --once /camera_tilt_cmd std_msgs/Float32 "data: 45.0"` | Servo tilts camera 45° downward smoothly without jitter | Observe servo sweep | **PENDING BENCH** |

---

## 3. Motor Actuation & Dynamic Safety Brake Verification

*(Rover chassis placed on test blocks with wheels elevated off the bench)*

> [!CAUTION]
> Always maintain a physical finger on the 12V battery kill switch during motor actuation tests.

| Test ID | Subsystem | Procedure | Expected Outcome | Safety Limit | Status |
| :---: | :--- | :--- | :--- | :--- | :---: |
| **H3.1** | Wheel Direction (Fwd) | Turn ON 12V switch. Pub slow forward: `ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"` | All 4 wheels rotate forward symmetrically | Max 0.15 m/s | **PENDING BENCH** |
| **H3.2** | Wheel Direction (Rev) | Pub reverse: `ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1}}"` | All 4 wheels rotate backward | Max 0.15 m/s | **PENDING BENCH** |
| **H3.3** | Differential Turn | Pub yaw: `ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{angular: {z: 0.3}}"` | Left wheels rotate back, right rotate fwd | Differential spin | **PENDING BENCH** |
| **H3.4** | Safety Stop (<30cm) | While wheels rotate forward, place flat board 20 cm in front of HC-SR04 | `safety_node` trips `/emergency_stop`; motors clamp to zero within 50 ms | Hard stop < 30 cm | **PENDING BENCH** |
| **H3.5** | Comm Watchdog Stop | Publish continuous motion, then pull USB cable or kill teleop node | Motors stop within 500 ms (`AUTO_WATCHDOG_STOP`) | Auto watchdog timeout | **PENDING BENCH** |

---

## 4. End-to-End Rover Inspection Walkthrough

1. **Power Sequence:**
   - Power Raspberry Pi 4 via 5V buck converter.
   - Boot Pi; observe green LED steady activity.
   - Verify Wi-Fi link: `ping 10.21.210.129` from laptop.
2. **Camera Initialization:**
   - On Pi SSH terminal: `python3 ~/brio_stream.py`.
   - Verify health: `curl http://10.21.210.129:8080/health`.
3. **Laptop AI Dashboard:**
   - On laptop: `streamlit run demo_dashboard.py`.
   - Confirm live camera stream displays in browser at 19–30 FPS.
   - Confirm YOLO26n-seg GPU inference executes in 14 ms.
4. **Defect Inspection & Gemini Screening:**
   - Point rover camera at concrete crack reference.
   - Confirm real-time bounding box and segmentation mask render on dashboard.
   - Click **"🔬 Analyze Current Frame with Gemini"**; confirm validated multimodal diagnosis and civil engineering recommendation appear.
   - Click **"📄 Generate Audit PDF"**; download and archive inspection certificate.

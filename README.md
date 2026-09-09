# 🏗️ BuildScan Rover — ROS 2 Humble Robotic System
### AI-Based Autonomous Robot for Indoor Structural Health Inspection

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![AI](https://img.shields.io/badge/AI-YOLO26n--seg-green)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red)
![micro-ROS](https://img.shields.io/badge/Embedded-micro--ROS%20ESP32-orange)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo%20Classic-purple)
![Status](https://img.shields.io/badge/Status-ROS2%20Migrated-success)

**Robotics Operating Systems & Robot Simulation — University Project**

</div>

---

## 📌 Project Overview

BuildScan Rover is a **ROS 2 Humble**-based autonomous indoor structural health inspection robot. It combines AI-based crack detection (YOLO26n-seg), mobile robot control, 2D SLAM, Nav2 navigation, and Gazebo simulation into a proper robotics system.

> **Migration Note**: The original project used Arduino UNO + HC-05 Bluetooth + direct Streamlit inference. This ROS2 architecture replaces the monolithic design with proper inter-process communication, simulation, navigation, and action-based inspection workflows.

---

## 🏛️ Architecture

### NEW ROS2 Pipeline

```
Physical Rover:
  ESP32-CAM (MJPEG stream)
        ↓ Wi-Fi
  camera_bridge_node         → /camera/image_raw
        ↓
  crack_detection_node       → /inspection/result
  (YOLO26n-seg GPU inference)       ↓
        ↓                   inspection_manager
  /inspection/result         (InspectArea action server)
        ↓                           ↓
  safety_node                report_generator
  /ultrasonic_range          → inspection_report.pdf
  /emergency_stop
        ↓
  motor_interface_node
  → ESP32 DevKit (micro-ROS)
  → L298N → 4× DC Motors

Dashboard (Streamlit) ↔ ROS2 topics/actions/services (UI only)
```

### OLD Pipeline (deprecated, kept in legacy/)

```
HC-05 → Arduino UNO → L298N / HC-SR04 / Servos
ESP32-CAM → HTTP stream → Streamlit → YOLO → PDF
```

---

## 🤖 Hardware Architecture

| Component | Role | ROS2 Interface |
|---|---|---|
| **Raspberry Pi 4/5** | Main ROS2 computer | Runs all robot-side nodes |
| **ESP32 DevKit** | Embedded controller (micro-ROS) | Subscribes `/cmd_vel`, publishes `/ultrasonic_range` |
| **ESP32-CAM** | Camera (MJPEG stream) | Bridged to `/camera/image_raw` |
| **L298N** | Motor driver | Controlled by ESP32 |
| **HC-SR04** | Ultrasonic obstacle sensor | Publishes `/ultrasonic_range` via micro-ROS |
| **SG90 Servos** | Camera pan/tilt (optional) | Legacy support |
| **2D LiDAR** | SLAM/Nav2 (optional) | Publishes `/scan` |
| **Wheel Encoders** | Odometry (optional) | Publishes `/odom` |

> **Note**: HC-05 Bluetooth removed from primary architecture. Arduino UNO moved to `legacy/`.

---

## 📦 ROS2 Package Structure

```
ros2_ws/src/
├── buildscan_interfaces/          # Custom msgs/srv/actions
│   ├── msg/InspectionResult.msg
│   ├── srv/SetInspectionConfig.srv
│   ├── srv/SetMode.srv
│   └── action/InspectArea.action
│
├── buildscan_hardware/            # Hardware bridge nodes
│   ├── camera_bridge_node.py      # ESP32-CAM → /camera/image_raw
│   ├── motor_interface_node.py    # /cmd_vel → ESP32 micro-ROS
│   └── safety_node.py             # HC-SR04 watchdog → /emergency_stop
│
├── buildscan_perception/          # AI inference
│   └── crack_detection_node.py    # YOLO → /inspection/result
│
├── buildscan_inspection/          # Inspection workflow
│   ├── inspection_manager.py      # InspectArea action server
│   └── report_generator.py        # ReportLab PDF generator
│
├── buildscan_description/         # Robot model
│   └── urdf/buildscan_rover.urdf.xacro
│
├── buildscan_sim/                 # Gazebo simulation
│   └── worlds/inspection_world.sdf
│
├── buildscan_navigation/          # SLAM + Nav2
│   ├── config/slam_toolbox.yaml
│   └── config/nav2_params.yaml
│
├── buildscan_dashboard/           # Streamlit ROS2 UI
│   └── dashboard_node.py
│
└── buildscan_bringup/             # Launch files + params
    ├── config/robot_params.yaml
    └── launch/
        ├── physical.launch.py
        ├── simulation.launch.py
        ├── perception.launch.py
        ├── navigation.launch.py
        └── full_system.launch.py
```

---

## 📡 Topic / Service / Action Graph

### Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Motor velocity commands |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera frames |
| `/camera/image_annotated` | `sensor_msgs/Image` | YOLO-annotated frames |
| `/ultrasonic_range` | `sensor_msgs/Range` | HC-SR04 distance |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/inspection/result` | `buildscan_interfaces/InspectionResult` | Crack detection results |
| `/inspection/status` | `std_msgs/String` | Human-readable status |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop signal |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM-generated map |

### Services

| Service | Type | Description |
|---|---|---|
| `/set_inspection_config` | `SetInspectionConfig` | Update YOLO params at runtime |
| `/set_mode` | `SetMode` | Switch MANUAL/AUTO/INSPECTION/PARK |

### Actions

| Action | Type | Description |
|---|---|---|
| `/inspect_area` | `InspectArea` | Long-running inspection mission |

---

## ⚙️ Installation

### 1. Raspberry Pi Setup (Ubuntu 22.04)

```bash
# Install ROS2 Humble
sudo apt update && sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install navigation stack
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-cv-bridge python3-cv-bridge

# Install micro-ROS agent
sudo snap install micro-ros-agent
```

### 2. Laptop/AI Workstation Setup

```bash
# Python dependencies
pip install -r requirements.txt
pip install ultralytics opencv-python reportlab streamlit cv_bridge
```

### 3. Build the ROS2 Workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select buildscan_interfaces
source install/setup.bash

# Build remaining packages
colcon build
source install/setup.bash
```

### 4. Verify Interfaces

```bash
ros2 interface list | grep buildscan
ros2 interface show buildscan_interfaces/msg/InspectionResult
ros2 interface show buildscan_interfaces/action/InspectArea
ros2 interface show buildscan_interfaces/srv/SetInspectionConfig
```

### 5. ESP32 DevKit Firmware

```
1. Install Arduino IDE + ESP32 board support
2. Install micro_ros_arduino library
3. Open: firmware/esp32_microros/esp32_microros.ino
4. Flash to ESP32 DevKit
5. Connect USB to Raspberry Pi
```

---

## 🚀 Running the System

### Physical Robot

```bash
# Terminal 1: micro-ROS agent (on Raspberry Pi)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200

# Terminal 2: Full physical system
ros2 launch buildscan_bringup physical.launch.py

# Terminal 3: Streamlit dashboard (on laptop)
streamlit run ros2_ws/src/buildscan_dashboard/buildscan_dashboard/dashboard_node.py
```

### Simulation

```bash
# Full simulation with Gazebo + RViz
ros2 launch buildscan_bringup simulation.launch.py

# Add navigation
ros2 launch buildscan_bringup navigation.launch.py use_sim_time:=true
```

### Perception Only (Laptop Demo)

```bash
ros2 launch buildscan_bringup full_system.launch.py mode:=perception_only
```

---

## 🔍 Verification Commands

```bash
# Check all nodes
ros2 node list

# Check all topics
ros2 topic list

# Live camera check
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view

# Crack detection results
ros2 topic echo /inspection/result

# Ultrasonic sensor
ros2 topic echo /ultrasonic_range

# Action server
ros2 action list
ros2 action info /inspect_area

# Send inspection goal
ros2 action send_goal /inspect_area buildscan_interfaces/action/InspectArea \
  "{area_name: 'Wall_A', confidence_threshold: 0.30, scan_frames: 10}"

# Service calls
ros2 service call /set_mode buildscan_interfaces/srv/SetMode "{mode: 'MANUAL'}"
ros2 service call /set_inspection_config buildscan_interfaces/srv/SetInspectionConfig \
  "{confidence_threshold: 0.35, safety_distance: 0.30}"

# Parameters
ros2 param list
ros2 param get /crack_detection_node yolo_confidence_threshold

# Health check
ros2 doctor
```

---

## 🤖 Gazebo Simulation

```bash
# Launch simulation
ros2 launch buildscan_bringup simulation.launch.py

# Drive the simulated rover
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# View topics in RViz2
# Camera feed, LiDAR scan, odometry, TF tree all visible
```

The simulated world contains:
- 6m × 6m indoor room
- Structural walls (Wall_A as inspection target)
- Pillars and obstacles for navigation testing
- Simulated camera (`/camera/image_raw`), LiDAR (`/scan`), odometry (`/odom`)

---

## 🗺️ SLAM & Navigation

```bash
# SLAM mapping (simulation)
ros2 launch buildscan_bringup simulation.launch.py
ros2 launch buildscan_bringup navigation.launch.py use_sim_time:=true

# Drive around to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/buildscan_map
```

**IMPORTANT**: HC-SR04 does NOT provide SLAM data. SLAM requires `/scan` from a 2D LiDAR.

---

## 📊 AI Pipeline

```
/camera/image_raw (sensor_msgs/Image)
       ↓
  cv_bridge.imgmsg_to_cv2()
       ↓
  YOLO26n-seg.predict(imgsz=320, conf=0.30)
       ↓
  Bounding box extraction
  total_length = Σ max(w,h) per detection
  maximum_width = max(min(w,h)) across detections
       ↓
  Severity classification:
    width < 70px  → LOW    → "Apply surface filler"
    width < 150px → MEDIUM → "Use epoxy injection"
    width ≥ 150px → HIGH   → "Urgent structural inspection"
       ↓
  InspectionResult published to /inspection/result
```

Model: `crack_detection/BuildScan_SegModel/weights/best.pt`
- Precision: 91.2%, Recall: 86.4%, Box mAP@50: 89.6%

---

## 📄 PDF Report

Generated by `buildscan_inspection/report_generator.py`:
- Inspection ID, timestamp, area name
- Original + annotated images (side by side)
- Crack count, confidence, width, length
- Severity classification (color-coded)
- Repair recommendation
- Model information
- AI performance metrics

Reports saved to: `/tmp/buildscan_reports/`

---

## 🏭 Industrial Robot Demo (Academic)

> **Note**: BuildScan Rover is a 4WD mobile robot, NOT an industrial robot.
> This module is a **separate academic demonstration** for the industrial robot course outcome.

The `buildscan_industrial_demo` package (Stage 12) will use MoveIt2 with a 6-DOF arm URDF to demonstrate:
- Forward/inverse kinematics
- Path planning (OMPL)
- Trajectory planning
- Motion execution in simulation

---

## 🔧 Hardware Wiring (ESP32 DevKit)

| ESP32 Pin | Component | Signal |
|---|---|---|
| GPIO 14 | L298N ENA | PWM (Left motor speed) |
| GPIO 26 | L298N IN1 | Left motor direction |
| GPIO 25 | L298N IN2 | Left motor direction |
| GPIO 27 | L298N ENB | PWM (Right motor speed) |
| GPIO 33 | L298N IN3 | Right motor direction |
| GPIO 32 | L298N IN4 | Right motor direction |
| GPIO 5  | HC-SR04 TRIG | Trigger pulse |
| GPIO 18 | HC-SR04 ECHO | Echo return |
| GPIO 1  | Raspberry Pi RX | micro-ROS serial TX |
| GPIO 3  | Raspberry Pi TX | micro-ROS serial RX |

---

## 🐛 Troubleshooting

| Issue | Solution |
|---|---|
| `colcon build` fails on interfaces | Ensure `rosidl_default_generators` installed: `sudo apt install ros-humble-rosidl-default-generators` |
| `/camera/image_raw` no data | Check ESP32-CAM IP and `esp32_cam_stream_url` in `robot_params.yaml` |
| micro-ROS agent not connecting | Check USB port: `ls /dev/ttyUSB*` and update `esp32_serial_port` |
| YOLO model not found | Set `model_path` param to full path of `best.pt` |
| Emergency stop stays active | Check HC-SR04 wiring; use `ros2 topic echo /ultrasonic_range` |
| Gazebo not launching | Install: `sudo apt install ros-humble-gazebo-ros-pkgs` |

---

## 🎓 Viva Q&A

**Q: Why ROS2 over plain Python/Streamlit?**
A: ROS2 provides standardized inter-process communication (DDS), modularity, hardware abstraction, simulation integration (Gazebo), navigation (Nav2), and industry-standard interfaces. It enables distributed computing across Raspberry Pi + Laptop + ESP32.

**Q: What is micro-ROS?**
A: micro-ROS extends ROS2 to resource-constrained embedded systems (ESP32/Arduino). The ESP32 runs the micro-ROS client; a micro-ROS agent on the Raspberry Pi bridges it to the ROS2 network via serial/UDP.

**Q: How does SLAM work without a physical LiDAR?**
A: In simulation, Gazebo's LiDAR plugin publishes a synthetic `/scan` topic. SLAM Toolbox processes `/scan` + `/odom` to build an occupancy grid map. On physical hardware with no LiDAR, SLAM is not possible — this is clearly documented.

**Q: What does the InspectArea action do?**
A: It's a ROS2 action that implements a long-running inspection mission with streamed feedback. Goal → navigate → scan → YOLO inference → crack analysis → PDF → result. Actions support feedback, cancellation, and result returns — unlike one-shot service calls.

**Q: What is the difference between a topic, service, and action?**
A: Topic = continuous publish/subscribe (camera frames, sensor data). Service = request/response (set config, change mode). Action = long-running task with feedback and cancellation (inspection mission).

---

## 📁 Legacy Files

| Location | Description |
|---|---|
| `legacy/arduino/Final_Rover_Code.ino` | Original Arduino UNO + HC-05 + L298N code |
| `legacy/streamlit/app_original.py` | Original monolithic Streamlit app |
| `crack_detection/` | All original AI code (preserved, not deleted) |
| `CameraWebServer/` | ESP32-CAM firmware (unchanged) |

---

## 📚 Publications

**BuildScan Rover: AI-Based Autonomous Robot for Indoor Structural Health Inspection**
International Conference on Intelligent Computing and Explainable AI (ICICEA'26)
A.V.C College of Engineering | ISBN: 978-935717-038-3 | April 2026

---

<div align="center">

### ⭐ BuildScan Rover — ROS2 Humble Edition ⭐

</div>

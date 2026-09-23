# BuildScan Rover — Physical Demonstration Procedure

This document outlines the step-by-step procedure to demonstrate the BuildScan Rover physical hardware, running the distributed ROS 2 architecture.

## Prerequisites

1. **Hardware**:
   - Dell G15 Laptop (Ubuntu 22.04 + ROS 2 Humble).
   - Raspberry Pi 4 on the Rover (Ubuntu 22.04 + ROS 2 Humble).
   - ESP32 DevKit V1 (flashed with `esp32_microros.ino`).
   - Logitech Brio 100 USB Camera (plugged into Raspberry Pi).
   - MG996R/MG995-class servo + external 5–6V servo supply.
   - HC-SR04 + 4-channel logic-level converter (wired per `docs/hardware/buildscan_wiring.md`).
   - 16×2 I2C LCD (wired per wiring doc).
   - Power supply (battery/buck converter for motors, separate servo supply).
2. **Network**: Laptop and Raspberry Pi on the **same Wi-Fi network**.
3. **Environment**: `ROS_DOMAIN_ID=25` and `ROS_LOCALHOST_ONLY=0` set on both machines.

---

## Step 1: Boot Up and Network Verification

1. Power on the Raspberry Pi and ESP32 (ESP32 LED should go solid after ~3s = micro-ROS ready).
2. Confirm 16×2 LCD shows `BuildScan Rover` on row 0 and `ROS2 Ready!` on row 1.
3. Connect servo external supply — servo should move to 90° (neutral) on boot.
4. Plug in Logitech Brio 100 to Raspberry Pi.
5. On the **Laptop**, ping the Pi:
   ```bash
   ping <raspberry_pi_ip>
   ```

---

## Step 2: Start Raspberry Pi Nodes

1. SSH into the Raspberry Pi:
   ```bash
   ssh ubuntu@<raspberry_pi_ip>
   ```
2. Source the workspace and launch Pi-side nodes:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch buildscan_bringup pi_robot.launch.py
   ```
3. **Expected Output**:
   - `micro_ros_agent` connects to ESP32 (`/buildscan_esp32` node appears).
   - `v4l2_camera_node` starts streaming.
   - `safety_node` initializes.
   - `motor_interface_node` initializes.

---

## Step 3: Verify Distributed ROS 2 Communication

1. On the **Laptop**, open a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic list
   ```
2. Expected topics from Pi/ESP32:
   - `/camera/image_raw`
   - `/ultrasonic_range`
   - `/cmd_vel_safe`
   - `/emergency_stop`
   - `/camera_tilt_cmd`
3. Test HC-SR04:
   ```bash
   ros2 topic echo /ultrasonic_range
   ```
   *Place your hand in front of the sensor — values should drop below 0.30m.*

---

## Step 4: Start Laptop AI & Dashboard

1. On the **Laptop**, launch AI inference:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch buildscan_bringup laptop_ai.launch.py
   ```
2. In a separate terminal, start the dashboard:
   ```bash
   source install/setup.bash
   streamlit run src/buildscan_dashboard/buildscan_dashboard/dashboard_node.py
   ```
3. Open the dashboard URL in your browser (default: `http://localhost:8501`).

---

## Step 5: Physical Hardware Demonstrations

### 5.1 Teleoperation & Safety Watchdog

1. In the dashboard, go to the **Teleop** tab.
2. Use the directional buttons to drive the rover forward, backward, left, and right.
3. **Safety Demo**: While driving forward, place an obstacle < 30cm in front of the HC-SR04.
   - The rover should immediately stop.
   - `/emergency_stop` publishes `True`.
   - The LCD row 0 changes to `*** E-STOP ***`.
   - Remove the obstacle — rover resumes normal operation.

### 5.2 Real-time AI Crack Detection

1. Drive the rover so the camera points at a structural surface with a crack (or print a crack image).
2. In the dashboard, go to the **Live Camera** tab.
3. You should see:
   - **Left**: Raw camera feed (`/camera/image_raw`).
   - **Right**: YOLO-annotated feed (`/camera/image_annotated`) with bounding boxes.

### 5.3 Automated Inspection Mission & PDF Report

1. In the dashboard, go to the **Inspection** tab.
2. Enter an **Area Name** (e.g., `Wall_A`).
3. Click **🚀 Start Inspection**.
4. Observe the ROS 2 action sequence (feedback shown on dashboard).
5. View the **Inspection Result** card (crack count, severity, recommendation).
6. Download the PDF report from the dashboard, or:
   ```bash
   xdg-open /tmp/buildscan_reports/BuildScan_*.pdf
   ```

### 5.4 Camera Servo Tilt Test

1. In the dashboard, go to the **Teleop** tab → scroll to **Camera Tilt Control**.
2. Verify the commanded angle display shows `90°` (neutral/boot).
3. Use the **UP (⬆)** button — servo should tilt upward (angle decreases toward 45°).
4. Use the **CENTER (●)** button — servo returns to 90°.
5. Use the **DOWN (⬇)** button — servo tilts downward (angle increases toward 135°).
6. Alternatively, test from command line:
   ```bash
   # Move to center
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 90.0"
   # Tilt up
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 60.0"
   # Tilt down
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 120.0"
   # Out-of-range (firmware clamps to 135°)
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 180.0"
   ```
7. Confirm the LCD row 1 updates to show the new tilt angle (e.g., `D:0.45m T:060`).

> ⚠ **Note**: The servo angle shown is the **commanded angle only** — there is no position feedback sensor. The displayed value reflects what was sent to the ESP32, not a measured angle.

---

## Step 6: System Visualization

1. On the **Laptop**, open rqt_graph:
   ```bash
   rqt_graph
   ```
2. Expected nodes visible: `/buildscan_esp32`, `/motor_interface_node`, `/safety_node`, `/v4l2_camera_node`, `/crack_detection_node`, `/inspection_manager`, `/buildscan_dashboard`.
3. Expected topic connections visible between all nodes.

---

**Demonstration Complete.**

For full hardware wiring details: [`docs/hardware/buildscan_wiring.md`](../hardware/buildscan_wiring.md)
For viva Q&A preparation: [`docs/demo/viva_qa.md`](viva_qa.md)

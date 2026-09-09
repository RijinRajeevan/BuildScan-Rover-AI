# BuildScan Rover — Physical Demonstration Procedure

This document outlines the step-by-step procedure to demonstrate the BuildScan Rover physical hardware, running the distributed ROS2 architecture.

## Prerequisites

1.  **Hardware**:
    *   Laptop (Ubuntu 22.04 + ROS2 Humble + GPU for YOLO).
    *   Raspberry Pi 4/5 on the Rover (Ubuntu 22.04 + ROS2 Humble).
    *   ESP32 DevKit (flashed with `esp32_microros.ino`).
    *   USB HD Camera (plugged into the Raspberry Pi).
    *   Power supply (batteries for motors and Pi).
2.  **Network**: Both Laptop and Raspberry Pi must be on the **same Wi-Fi network**.
3.  **Environment**: `ROS_DOMAIN_ID=25` and `ROS_LOCALHOST_ONLY=0` must be set on both machines.

---

## Step 1: Boot Up and Network Verification

1.  Power on the Raspberry Pi and ESP32 DevKit. Ensure the USB HD Camera is plugged in.
2.  On the **Laptop**, open a terminal and ping the Pi to verify connectivity:
    ```bash
    ping <raspberry_pi_ip>
    ```

## Step 2: Start Raspberry Pi Nodes

1.  SSH into the Raspberry Pi:
    ```bash
    ssh user@<raspberry_pi_ip>
    ```
2.  Source the workspace and launch the Pi-side nodes:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch buildscan_bringup pi_robot.launch.py
    ```
3.  **Expected Output**:
    *   `micro_ros_agent` connects to the ESP32.
    *   `v4l2_camera_node` connects to the USB camera.
    *   `safety_node` initializes.
    *   `motor_interface_node` initializes.

## Step 3: Verify Distributed ROS2 Communication

1.  On the **Laptop**, open a new terminal:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 topic list
    ```
2.  You should see the topics published by the Pi:
    *   `/camera/image_raw`
    *   `/ultrasonic_range`
    *   `/cmd_vel_safe`
3.  Test sensor data (HC-SR04):
    ```bash
    ros2 topic echo /ultrasonic_range
    ```
    *Place your hand in front of the sensor to see the values change.*

## Step 4: Start Laptop AI & UI Nodes

1.  On the **Laptop**, open a new terminal and launch the AI nodes:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch buildscan_bringup laptop_ai.launch.py
    ```
2.  On the **Laptop**, open *another* terminal and start the Streamlit Dashboard:
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    streamlit run src/buildscan_dashboard/buildscan_dashboard/dashboard_node.py
    ```

## Step 5: Physical Hardware Demonstration

### 5.1 Teleoperation & Safety Watchdog
1.  In the **Streamlit Dashboard**, go to the **Teleop** tab.
2.  Use the directional buttons to drive the rover forward, backward, left, and right.
3.  **Safety Demo**: While driving forward, place an obstacle (e.g., your hand or a box) < 30cm in front of the HC-SR04 sensor.
    *   The rover should immediately stop.
    *   The Dashboard will display a red `🚨 EMERGENCY STOP ACTIVE` banner.
    *   Remove the obstacle; the banner should disappear, allowing movement again.

### 5.2 Real-time AI Crack Detection
1.  Drive the rover so the camera points at a structural surface with a crack (or a printed image of a crack).
2.  In the **Streamlit Dashboard**, go to the **Live Camera** tab.
3.  You should see:
    *   **Left**: The raw camera feed (`/camera/image_raw`).
    *   **Right**: The YOLO-annotated feed (`/camera/image_annotated`) showing the detected crack bounding boxes and confidence.

### 5.3 Automated Inspection Mission & PDF Report
1.  In the **Streamlit Dashboard**, go to the **Inspection** tab.
2.  Enter an **Area Name** (e.g., "Wall_A").
3.  Click **🚀 Start Inspection**.
4.  Observe the ROS2 Action sequence:
    *   The system captures frames, runs YOLO inference, aggregates results, and calculates severity.
    *   A PDF report is generated.
5.  View the final **Inspection Result** card on the dashboard (showing crack count, max width, severity, and recommendation).
6.  Open the generated PDF report on the Laptop:
    ```bash
    xdg-open /tmp/buildscan_reports/BuildScan_*.pdf
    ```
    *Show the PDF containing the metadata, original/annotated images, and severity metrics.*

---
**Demonstration Complete.**

# BuildScan Rover — Viva Q&A Guide

This document contains potential questions and answers for a university project Viva/Defense regarding the BuildScan Rover.

## Architecture & Frameworks

**Q: Why did you migrate from Arduino/Streamlit to ROS 2?**
**A:** The original design was a monolithic architecture where Arduino handled hardware and Streamlit handled AI directly. Moving to ROS 2 provides standardized inter-process communication (DDS), hardware abstraction, distributed computing (running AI on the laptop while the Pi handles robot logic), and the ability to easily integrate complex robotics algorithms (like Nav2 or SLAM) in the future. It transforms a "remote-controlled camera" into a true robotic system.

**Q: What is micro-ROS and why did you use it on the ESP32?**
**A:** micro-ROS is an extension of ROS 2 designed for resource-constrained microcontrollers. Instead of writing custom serial parsing code on the Arduino (like in the original project), micro-ROS allows the ESP32 to act as a first-class ROS 2 node. It can subscribe directly to `/cmd_vel` to drive motors and publish `/ultrasonic_range` using standard ROS message types.

**Q: Explain the distributed computing architecture in your project.**
**A:** The system splits the workload across three devices on the same Wi-Fi network (ROS 2 DDS):
1.  **ESP32 DevKit (Embedded):** Handles real-time hardware control (PWM for motors, HC-SR04 pulse reading).
2.  **Raspberry Pi (Robot Onboard):** Runs the micro-ROS agent, manages the USB HD Camera to publish `/camera/image_raw`, and manages safety limits.
3.  **Laptop (Base Station):** Handles computationally heavy tasks: YOLO AI inference (using GPU), PDF report generation, and the Streamlit UI dashboard.

## AI & Perception

**Q: How does the crack detection work?**
**A:** We use a YOLO26n-seg model trained specifically for crack segmentation. The `crack_detection_node` subscribes to `/camera/image_raw`, converts the ROS Image to an OpenCV format, runs inference, and measures the bounding box of the detected segments.

**Q: How do you determine the "severity" of a crack?**
**A:** We preserved the methodology from the original project. We calculate the maximum pixel width across detected crack segments. 
*   Width < 70 pixels = LOW severity (Surface filler recommended).
*   Width < 150 pixels = MEDIUM severity (Epoxy injection recommended).
*   Width >= 150 pixels = HIGH severity (Urgent inspection required).

**Q: Why doesn't the Streamlit dashboard run the AI model anymore?**
**A:** Streamlit is designed for UI, not backend robotics processing. By moving YOLO to a dedicated ROS 2 node (`crack_detection_node`), the AI runs independently and continuously. The dashboard simply subscribes to the resulting `/inspection/result` topic, making the system much more modular and robust.

## Robotics & Control

**Q: How does the emergency stop feature work?**
**A:** The ESP32 publishes HC-SR04 distance data to `/ultrasonic_range`. The `safety_node` on the Pi monitors this. If the distance drops below 0.30 meters, it publishes `True` to `/emergency_stop`. The `motor_interface_node` listens to this and instantly overrides any incoming `/cmd_vel` commands, publishing zero velocity to the ESP32 to halt the rover.

**Q: What is a ROS 2 Action, and why use it for the inspection process?**
**A:** ROS 2 has Topics (continuous data), Services (quick request/response), and Actions (long-running tasks with feedback). An inspection mission takes time (positioning, scanning, AI processing, PDF generation). We use the `InspectArea` Action server so the UI can trigger the mission, receive real-time progress updates (e.g., "Scanning... 40%"), and cancel the mission if needed, before finally receiving the generated PDF path.

## Limitations & Future Work

**Q: Can this robot perform autonomous navigation and mapping (SLAM)?**
**A:** The current physical hardware relies on an HC-SR04 ultrasonic sensor, which only provides a single point of distance data. True 2D SLAM and Nav2 autonomous navigation require a 2D LiDAR (like an RPLIDAR) and wheel encoders for odometry. However, the software architecture is fully prepared for this; the ROS 2 environment, URDF, and TF tree are set up, and we have demonstrated this capability in Gazebo simulation.

**Q: Why did you switch from the ESP32-CAM to a direct USB HD Camera?**
**A:** The ESP32-CAM streams MJPEG over Wi-Fi, which introduces network latency, dropped frames, and compression artifacts, especially when sharing the same Wi-Fi bandwidth as the ROS 2 DDS network. By plugging a USB HD Camera directly into the Raspberry Pi, we achieve higher resolution, significantly lower latency, and greater reliability. The `v4l2_camera` node efficiently publishes this raw data to the ROS 2 network for the laptop to process.

# micro-ROS Agent Setup (Raspberry Pi)

The micro-ROS agent runs on the Raspberry Pi and acts as a bridge, allowing the ESP32 DevKit (which runs the micro-ROS client) to communicate with the main ROS 2 DDS network.

## Installation

The `pi_setup.sh` script installs the micro-ROS agent via `apt`. If you need to install it manually:

```bash
sudo apt update
sudo apt install ros-humble-micro-ros-agent
```

## Running the Agent

The agent must be started and configured to listen on the correct serial port (USB) that the ESP32 is connected to.

1.  Connect the ESP32 DevKit to the Raspberry Pi via USB.
2.  Find the correct serial port (usually `/dev/ttyUSB0` or `/dev/ttyACM0`):
    ```bash
    ls /dev/ttyUSB*
    ls /dev/ttyACM*
    ```
3.  Ensure your user has permission to read/write to the serial port:
    ```bash
    sudo usermod -a -G dialout $USER
    # You may need to log out and back in for this to take effect
    ```
4.  Start the agent:
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
    ```
    *(Replace `/dev/ttyUSB0` with the actual port found in step 2).*

**Note:** This is handled automatically by the `pi_robot.launch.py` launch file.

## Verification

Once the agent is running and the ESP32 is connected and powered on:

1.  Open a new terminal on the Pi.
2.  List active ROS 2 nodes:
    ```bash
    ros2 node list
    ```
    *You should see a node named `/buildscan_esp32`.*
3.  List active topics to verify the ESP32 is publishing/subscribing:
    ```bash
    ros2 topic list
    ```
    *You should see `/ultrasonic_range` and `/cmd_vel_safe`.*

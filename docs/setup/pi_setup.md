# Raspberry Pi Setup Guide (Ubuntu 22.04 + ROS 2 Humble)

This guide walks you through setting up the Raspberry Pi as the onboard robot computer for the BuildScan Rover.

## Prerequisites
*   Raspberry Pi 4 or 5.
*   MicroSD card (16GB or larger).
*   Internet connection via Wi-Fi.

## Step 1: Install Ubuntu 22.04 LTS

1.  Download the **Raspberry Pi Imager** tool from the official website.
2.  Select **Choose OS** -> **Other general-purpose OS** -> **Ubuntu** -> **Ubuntu Server 22.04.x LTS (64-bit)**.
3.  Select your MicroSD card.
4.  Click the gear icon (Settings) before writing:
    *   Set hostname: `buildscan-pi`
    *   Enable SSH.
    *   Configure Wi-Fi SSID and password (ensure it's the same network the laptop will use).
5.  Write the OS to the SD card.

## Step 2: Automated ROS 2 Installation

1.  Insert the SD card into the Pi and power it on.
2.  Find the Pi's IP address on your network and SSH into it:
    ```bash
    ssh ubuntu@<pi_ip_address>
    ```
3.  Copy the `pi_setup.sh` script to the Pi and make it executable:
    ```bash
    chmod +x pi_setup.sh
    ```
4.  Run the setup script:
    ```bash
    ./pi_setup.sh
    ```
    *This script installs ROS 2 Humble, the micro-ROS agent, required ROS 2 packages (cv_bridge, robot_state_publisher), and Python dependencies.*

## Step 3: Copy Workspace

1.  Copy the `ros2_ws` directory from your laptop to the Raspberry Pi:
    ```bash
    scp -r ros2_ws/ ubuntu@<pi_ip_address>:~/
    ```

## Step 4: Build Workspace

1.  On the Pi, build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select buildscan_interfaces
    source install/setup.bash
    colcon build
    ```

## Step 5: Network Configuration

Ensure that `ROS_DOMAIN_ID=25` and `ROS_LOCALHOST_ONLY=0` are set in the `~/.bashrc` file (the setup script should have done this).

```bash
source ~/.bashrc
```

The Pi is now ready to run the onboard robot nodes!

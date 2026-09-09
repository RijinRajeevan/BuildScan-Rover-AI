#!/bin/bash
# Laptop Setup Script for BuildScan Rover (Ubuntu 22.04 + ROS2 Humble)
# Run as standard user (script will prompt for sudo when needed)

set -e

echo "=== BuildScan Rover: Laptop Setup ==="

# 1. System Update
echo "[1/4] Updating system..."
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg lsb-release -y

# 2. Install ROS2 Humble (if not installed)
if ! command -v ros2 &> /dev/null; then
    echo "[2/4] Installing ROS2 Humble..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep -y
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
else
    echo "[2/4] ROS2 is already installed. Skipping."
fi

# 3. Install Python dependencies for AI and UI
echo "[3/4] Installing Python dependencies (YOLO, Streamlit, etc.)..."
pip3 install ultralytics opencv-python reportlab streamlit cv_bridge

# 4. Environment configuration
echo "[4/4] Configuring environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
if ! grep -q "ROS_DOMAIN_ID=25" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=25" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
fi

echo "=== Setup Complete! ==="
echo "Please restart your terminal or run: source ~/.bashrc"

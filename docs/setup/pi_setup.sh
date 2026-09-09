#!/bin/bash
# Raspberry Pi Setup Script for BuildScan Rover (Ubuntu 22.04 + ROS2 Humble)
# Run as standard user (script will prompt for sudo when needed)

set -e

echo "=== BuildScan Rover: Raspberry Pi Setup ==="

# 1. System Update
echo "[1/5] Updating system..."
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg lsb-release -y

# 2. Install ROS2 Humble
echo "[2/5] Installing ROS2 Humble..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep -y
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 3. Install ROS2 dependencies and micro-ROS agent
echo "[3/5] Installing ROS2 dependencies and micro-ROS agent..."
sudo apt install ros-humble-cv-bridge ros-humble-robot-state-publisher ros-humble-xacro python3-cv-bridge ros-humble-v4l2-camera -y
sudo apt install ros-humble-micro-ros-agent -y

# 4. Install Python dependencies
echo "[4/5] Installing Python dependencies..."
pip3 install opencv-python

# 5. Environment configuration
echo "[5/5] Configuring environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
if ! grep -q "ROS_DOMAIN_ID=25" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=25" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
fi

echo "=== Setup Complete! ==="
echo "Please restart your terminal or run: source ~/.bashrc"

"""
physical.launch.py
──────────────────
Full physical robot launch: hardware nodes + perception + inspection.
Requires:
  - Raspberry Pi 4/5 running Ubuntu 22.04 + ROS2 Humble
  - ESP32 DevKit running micro-ROS firmware connected via USB
  - ESP32-CAM connected to Wi-Fi

Usage:
  ros2 launch buildscan_bringup physical.launch.py
  ros2 launch buildscan_bringup physical.launch.py esp32_cam_url:=http://192.168.1.100:81/stream
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    args = [

        DeclareLaunchArgument('esp32_serial_port',
            default_value='/dev/ttyUSB0',
            description='ESP32 DevKit serial port for micro-ROS'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('buildscan_bringup'),
                'config', 'robot_params.yaml'
            ])),
    ]

    params_file = LaunchConfiguration('params_file')

    # ── micro-ROS Agent ────────────────────────────────────────────────────────
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('esp32_serial_port'),
                   '--baudrate', '115200'],
        output='screen',
    )

    # ── Motor Interface Node ───────────────────────────────────────────────────
    motor_interface = Node(
        package='buildscan_hardware',
        executable='motor_interface_node',
        name='motor_interface_node',
        output='screen',
        parameters=[params_file],
    )

    # ── Safety Node ───────────────────────────────────────────────────────────
    safety_node = Node(
        package='buildscan_hardware',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[params_file],
    )

    # ── USB Camera (v4l2_camera) ──────────────────────────────────────────────
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/image_raw', '/camera/image_raw')
        ]
    )

    # ── Crack Detection Node ───────────────────────────────────────────────────
    crack_detection = Node(
        package='buildscan_perception',
        executable='crack_detection_node',
        name='crack_detection_node',
        output='screen',
        parameters=[params_file],
    )

    # ── Inspection Manager (Action Server) ────────────────────────────────────
    inspection_manager = Node(
        package='buildscan_inspection',
        executable='inspection_manager',
        name='inspection_manager',
        output='screen',
        parameters=[params_file],
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription(
        args + [
            microros_agent,
            motor_interface,
            safety_node,
            camera_node,
            # delay perception 3 seconds to ensure camera is ready
            TimerAction(period=3.0, actions=[crack_detection]),
            TimerAction(period=4.0, actions=[inspection_manager]),
            robot_state_pub,
        ]
    )

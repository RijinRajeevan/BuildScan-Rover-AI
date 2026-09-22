"""
pi_robot.launch.py
──────────────────
Pi-side nodes for the distributed ROS2 physical robot.
Runs on: Raspberry Pi 4/5 (Ubuntu 22.04 + ROS2 Humble)

Includes:
  - micro-ROS Agent (bridge to ESP32)
  - motor_interface_node
  - safety_node
  - v4l2_camera_node
  - robot_state_publisher

Usage:
  export ROS_DOMAIN_ID=25
  ros2 launch buildscan_bringup pi_robot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    args = [

        DeclareLaunchArgument('esp32_serial_port',
            default_value='/dev/ttyUSB0',
            description='ESP32 DevKit serial port for micro-ROS'),
        DeclareLaunchArgument('camera_device',
            default_value='/dev/video0',
            description='USB camera device node'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('buildscan_bringup'),
                'config', 'robot_params.yaml'
            ])),
    ]

    params_file = LaunchConfiguration('params_file')

    # micro-ROS Agent
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('esp32_serial_port'),
                   '--baudrate', '115200'],
        output='screen',
    )

    # Motor Interface
    motor_interface = Node(
        package='buildscan_hardware',
        executable='motor_interface_node',
        name='motor_interface_node',
        output='screen',
        parameters=[params_file],
    )

    # Safety Watchdog
    safety_node = Node(
        package='buildscan_hardware',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[params_file],
    )

    # USB Camera (v4l2_camera)
    # HARDWARE DEPENDENT: requires USB camera at camera_device (/dev/video0 by default)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[
            params_file,
            {
                'video_device':  LaunchConfiguration('camera_device'),
                'image_width':   640,
                'image_height':  480,
                'image_format':  'MJPG',
            }
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw')
        ]
    )

    # Robot State Publisher (TF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription(args + [
        microros_agent,
        motor_interface,
        safety_node,
        camera_node,
        robot_state_pub,
    ])

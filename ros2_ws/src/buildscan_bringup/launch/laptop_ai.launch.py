"""
laptop_ai.launch.py
───────────────────
Laptop-side AI and UI nodes for the distributed ROS2 physical robot.
Runs on: Laptop (Ubuntu 22.04 + ROS2 Humble + GPU)

Includes:
  - crack_detection_node (YOLO inference)
  - inspection_manager (Action Server)
  - dashboard_node (Streamlit)

Usage:
  export ROS_DOMAIN_ID=25
  ros2 launch buildscan_bringup laptop_ai.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    args = [
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('buildscan_bringup'),
                'config', 'robot_params.yaml'
            ])),
    ]

    params_file = LaunchConfiguration('params_file')

    # Crack Detection Node (GPU)
    crack_detection = Node(
        package='buildscan_perception',
        executable='crack_detection_node',
        name='crack_detection_node',
        output='screen',
        parameters=[params_file],
    )

    # Inspection Manager (Action Server)
    inspection_manager = Node(
        package='buildscan_inspection',
        executable='inspection_manager',
        name='inspection_manager',
        output='screen',
        parameters=[params_file],
    )

    # Note: dashboard_node is typically run via 'streamlit run' directly for web UI, 
    # but we can launch the ROS2 backend here as well.
    # To run Streamlit properly: streamlit run src/buildscan_dashboard/buildscan_dashboard/dashboard_node.py

    return LaunchDescription(args + [
        crack_detection,
        inspection_manager,
    ])

"""
full_system.launch.py
──────────────────────
Master launch file for the complete BuildScan Rover system.

Physical mode (default):
  ros2 launch buildscan_bringup full_system.launch.py

Simulation mode:
  ros2 launch buildscan_bringup full_system.launch.py mode:=simulation

Perception only (laptop-only AI demo):
  ros2 launch buildscan_bringup full_system.launch.py mode:=perception_only
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_share = FindPackageShare('buildscan_bringup')

    # ── Arguments ─────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'mode',
            default_value='physical',
            choices=['physical', 'simulation', 'perception_only'],
            description='System launch mode'
        ),
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='false',
            description='Launch SLAM + Nav2 stack'
        ),
        DeclareLaunchArgument(
            'enable_dashboard',
            default_value='true',
            description='Launch Streamlit dashboard bridge'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'robot_params.yaml'
            ]),
        ),
    ]

    mode              = LaunchConfiguration('mode')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_dashboard  = LaunchConfiguration('enable_dashboard')
    params_file       = LaunchConfiguration('params_file')

    # ── Physical launch ───────────────────────────────────────────────────────
    physical_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'physical.launch.py'])
        ]),
        launch_arguments={'params_file': params_file}.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'physical')),
    )

    # ── Simulation launch ─────────────────────────────────────────────────────
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'simulation.launch.py'])
        ]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'simulation')),
    )

    # ── Perception-only launch (laptop AI workstation) ─────────────────────────
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'perception.launch.py'])
        ]),
        launch_arguments={'params_file': params_file}.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'perception_only')),
    )

    # ── Navigation (optional) ─────────────────────────────────────────────────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'navigation.launch.py'])
        ]),
        launch_arguments={'params_file': params_file}.items(),
        condition=IfCondition(enable_navigation),
    )

    # ── Inspection Manager (always needed) ────────────────────────────────────
    inspection_manager = Node(
        package='buildscan_inspection',
        executable='inspection_manager',
        name='inspection_manager',
        output='screen',
        parameters=[params_file],
        condition=UnlessCondition(EqualsSubstitution(mode, 'simulation')),
        # simulation.launch.py already starts it
    )

    return LaunchDescription(
        args + [
            physical_launch,
            simulation_launch,
            perception_launch,
            TimerAction(period=5.0, actions=[navigation_launch]),
            inspection_manager,
        ]
    )

"""
simulation.launch.py
────────────────────
Launches the BuildScan Rover in Gazebo simulation.
Provides simulated: camera, LiDAR (/scan), odometry (/odom), TF.

Usage:
  ros2 launch buildscan_bringup simulation.launch.py
  ros2 launch buildscan_bringup simulation.launch.py world:=inspection_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('world',
            default_value='inspection_world.sdf',
            description='Gazebo world file name (from buildscan_sim/worlds/)'),
        DeclareLaunchArgument('use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('buildscan_bringup'),
                'config', 'robot_params.yaml'
            ])),
        DeclareLaunchArgument('rviz',
            default_value='true',
            description='Launch RViz2'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')

    sim_share   = FindPackageShare('buildscan_sim')
    desc_share  = FindPackageShare('buildscan_description')

    # ── Robot URDF ────────────────────────────────────────────────────────────
    urdf_file = PathJoinSubstitution([
        desc_share, 'urdf', 'buildscan_rover.urdf.xacro'
    ])

    robot_description = Command(['xacro ', urdf_file,
                                 ' use_sim_time:=', use_sim_time])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    world_path = PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_path}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'buildscan_rover',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
    )

    # ── Simulated Crack Detection ─────────────────────────────────────────────
    crack_detection = Node(
        package='buildscan_perception',
        executable='crack_detection_node',
        name='crack_detection_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── Inspection Manager ────────────────────────────────────────────────────
    inspection_manager = Node(
        package='buildscan_inspection',
        executable='inspection_manager',
        name='inspection_manager',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([desc_share, 'rviz', 'buildscan.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(
        args + [
            robot_state_pub,
            gazebo,
            TimerAction(period=2.0, actions=[spawn_entity]),
            TimerAction(period=4.0, actions=[crack_detection]),
            TimerAction(period=4.0, actions=[inspection_manager]),
            TimerAction(period=3.0, actions=[rviz]),
        ]
    )

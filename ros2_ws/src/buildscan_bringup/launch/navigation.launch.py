"""
navigation.launch.py
─────────────────────
Launches SLAM Toolbox and Nav2 navigation stack.
Can be used with physical LiDAR or Gazebo simulated /scan.

Usage (simulation):
  ros2 launch buildscan_bringup navigation.launch.py use_sim_time:=true

Usage (physical with LiDAR):
  ros2 launch buildscan_bringup navigation.launch.py use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    nav_share = FindPackageShare('buildscan_navigation')
    params_file = LaunchConfiguration('params_file',
        default=PathJoinSubstitution([
            FindPackageShare('buildscan_bringup'), 'config', 'robot_params.yaml'
        ])
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── SLAM Toolbox (online async mapping) ───────────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution([
                nav_share, 'config', 'slam_toolbox.yaml'
            ]),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Nav2 Bringup ──────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                nav_share, 'config', 'nav2_params.yaml'
            ]),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox,
        nav2,
    ])

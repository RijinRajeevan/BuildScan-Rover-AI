"""
perception.launch.py
────────────────────
Launches the camera bridge and crack detection nodes.
Works in both physical and simulation mode.

Usage:
  ros2 launch buildscan_bringup perception.launch.py
  ros2 launch buildscan_bringup perception.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('buildscan_bringup'),
            'config', 'robot_params.yaml'
        ]),
        description='Path to robot parameter YAML file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')



    # ── Crack Detection Node ───────────────────────────────────────────────────
    crack_detection_node = Node(
        package='buildscan_perception',
        executable='crack_detection_node',
        name='crack_detection_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/camera/image_raw',   '/camera/image_raw'),
            ('/inspection/result',  '/inspection/result'),
            ('/inspection/status',  '/inspection/status'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,

        crack_detection_node,
    ])

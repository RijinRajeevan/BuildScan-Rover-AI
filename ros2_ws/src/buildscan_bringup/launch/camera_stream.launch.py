"""
camera_stream.launch.py
───────────────────────
Starts the Logitech Brio 100 camera and web_video_server on the Raspberry Pi.
Publishes /image_raw at 640x480 and serves MJPEG stream on port 8080.

Usage on Raspberry Pi:
  source /opt/ros/humble/setup.bash
  source ~/buildscan_ws/install/setup.bash  # or ~/ros2_ws/install/setup.bash
  ros2 launch buildscan_bringup camera_stream.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 video capture device'
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'time_per_frame': [1, 30],
            'pixel_format': 'YUYV',
        }],
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
        }],
    )

    return LaunchDescription([
        video_device_arg,
        camera_node,
        web_video_server,
    ])

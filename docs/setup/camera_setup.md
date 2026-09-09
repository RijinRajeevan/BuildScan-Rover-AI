# USB HD Camera Setup (Raspberry Pi)

The primary visual sensor for the BuildScan Rover is a USB HD Camera connected directly to the Raspberry Pi. This replaces the legacy ESP32-CAM HTTP stream, providing higher resolution, lower latency, and standardized ROS 2 integration.

## Hardware Connection

Plug the USB HD Camera into any available USB port on the Raspberry Pi.
*   *Note: USB 3.0 ports (the blue ones) are recommended if your camera supports USB 3.0 for higher bandwidth.*

## Verification

To ensure the Raspberry Pi recognizes the camera, open a terminal on the Pi and run:

```bash
ls /dev/video*
```

You should see at least one entry, such as `/dev/video0`. This is the device path for your camera.

## ROS 2 Integration

We use the standard `v4l2_camera` node provided by the ROS 2 community. This node reads from the V4L2 device (e.g., `/dev/video0`) and publishes the frames to the standard `sensor_msgs/Image` topic, `/camera/image_raw`.

### Configuration

The camera parameters are defined in `ros2_ws/src/buildscan_bringup/config/robot_params.yaml`:

```yaml
    # ── Camera ────────────────────────────────────────────────────────────
    camera_topic:             "/camera/image_raw"
    video_device:             "/dev/video0"
    image_width:              640
    image_height:             480
    camera_frame_id:          "camera_link"
```

If your camera appears as `/dev/video1` or another device, update the `video_device` parameter in this file.

### Running the Camera Node

The camera node is automatically started by the Pi bringup launch file:

```bash
ros2 launch buildscan_bringup pi_robot.launch.py
```

### Viewing the Stream

You can view the raw camera stream from the laptop (or any machine on the ROS 2 DDS network) using:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

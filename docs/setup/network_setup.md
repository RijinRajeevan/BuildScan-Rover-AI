# Distributed ROS 2 Network Setup (DDS)

The BuildScan Rover uses a distributed computing architecture where the Raspberry Pi and the Laptop communicate over the same Wi-Fi network using ROS 2 Data Distribution Service (DDS).

## Configuration

For nodes on the Raspberry Pi and Laptop to discover each other and communicate, they must share the same `ROS_DOMAIN_ID` and be on the same subnet.

### Step 1: Environment Variables

Add the following to the `~/.bashrc` file on **BOTH** the Raspberry Pi and the Laptop:

```bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
```

*   `ROS_DOMAIN_ID=25`: Isolates the ROS 2 traffic from other ROS 2 networks that might be on the same Wi-Fi. Both machines must use the same number (e.g., 25).
*   `ROS_LOCALHOST_ONLY=0`: Ensures ROS 2 traffic is broadcast over the network interfaces, not just the local loopback.

### Step 2: (Optional) Explicit IP Binding

Sometimes, if a machine has multiple network interfaces (e.g., Wi-Fi, Ethernet, VPN, Docker bridges), ROS 2 might choose the wrong one for communication.

If `ros2 topic list` on one machine doesn't show topics from the other, explicitly set the `ROS_IP` or `ROS_DISCOVERY_SERVER` to bind to the correct Wi-Fi IP address.

On the Raspberry Pi:
```bash
export ROS_IP=<pi_wifi_ip_address>
```

On the Laptop:
```bash
export ROS_IP=<laptop_wifi_ip_address>
```

## Verification

1.  Connect both the Raspberry Pi and Laptop to the same Wi-Fi network.
2.  On the Raspberry Pi, publish a test topic:
    ```bash
    ros2 topic pub /hello std_msgs/msg/String "data: 'Hello from Pi!'"
    ```
3.  On the Laptop, check if the topic is visible and echo the data:
    ```bash
    ros2 topic list
    ros2 topic echo /hello
    ```
    *If you see the "Hello from Pi!" messages, distributed communication is working.*

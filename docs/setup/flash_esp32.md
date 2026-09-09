# Flashing the ESP32 DevKit (micro-ROS)

The ESP32 DevKit requires custom firmware to run the micro-ROS client, which handles motor control (via L298N) and reads the HC-SR04 ultrasonic sensor.

## Prerequisites

1.  **Arduino IDE** installed on your Laptop.
2.  **ESP32 Board Support** installed in Arduino IDE (`Tools` -> `Board` -> `Boards Manager` -> search for `esp32`).
3.  **micro_ros_arduino** library installed in Arduino IDE.
    *   Download the precompiled library zip for Humble from the [micro_ros_arduino GitHub releases](https://github.com/micro-ROS/micro_ros_arduino/releases).
    *   In Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...` and select the downloaded zip.

## Flashing Procedure

1.  Open the `firmware/esp32_microros/esp32_microros.ino` file in the Arduino IDE.
2.  Connect the ESP32 DevKit to your Laptop via a USB cable.
3.  Select the correct board: `Tools` -> `Board` -> `ESP32 Arduino` -> `ESP32 Dev Module`.
4.  Select the correct port: `Tools` -> `Port` -> (Select the COM or USB port for your ESP32).
5.  Click the **Upload** arrow button.
    *   *Note: If the upload fails to connect, you may need to press and hold the "BOOT" button on the ESP32 DevKit when you see "Connecting..." in the console.*
6.  Once the upload is complete, the ESP32 is ready.

## Hardware Connections

Connect the ESP32 DevKit to the Rover hardware as follows:

| ESP32 Pin | Component | Signal |
|---|---|---|
| GPIO 14 | L298N ENA | PWM (Left motor speed) |
| GPIO 26 | L298N IN1 | Left motor direction |
| GPIO 25 | L298N IN2 | Left motor direction |
| GPIO 27 | L298N ENB | PWM (Right motor speed) |
| GPIO 33 | L298N IN3 | Right motor direction |
| GPIO 32 | L298N IN4 | Right motor direction |
| GPIO 5  | HC-SR04 TRIG | Trigger pulse |
| GPIO 18 | HC-SR04 ECHO | Echo return |
| USB Port| Raspberry Pi | Power + Serial Data (micro-ROS) |

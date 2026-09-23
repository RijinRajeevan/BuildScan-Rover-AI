# Flashing the ESP32 DevKit (micro-ROS)

The ESP32 DevKit requires custom firmware to run the micro-ROS client, which handles:
- Motor control via L298N
- HC-SR04 ultrasonic distance sensing → `/ultrasonic_range`
- Camera tilt servo (MG996R/MG995-class) ← `/camera_tilt_cmd`
- 16×2 I2C LCD status display

## Prerequisites

1. **Arduino IDE** installed on your Laptop.
2. **ESP32 Board Support** installed in Arduino IDE:
   - `Tools` → `Board` → `Boards Manager` → search for `esp32` → install.
3. **micro_ros_arduino** library installed:
   - Download the precompiled `.zip` for **Humble** from:
     [micro_ros_arduino GitHub releases](https://github.com/micro-ROS/micro_ros_arduino/releases)
   - In Arduino IDE: `Sketch` → `Include Library` → `Add .ZIP Library...`
4. **ESP32Servo** library installed:
   - In Arduino IDE: `Tools` → `Manage Libraries` → search `ESP32Servo` → install.
5. **LiquidCrystal_I2C** library installed:
   - In Arduino IDE: `Tools` → `Manage Libraries` → search `LiquidCrystal I2C` → install
     (by Frank de Brabander).

## Flashing Procedure

1. Open `firmware/esp32_microros/esp32_microros.ino` in Arduino IDE.
2. Connect the ESP32 DevKit to your Laptop via a USB cable.
3. Select the correct board: `Tools` → `Board` → `ESP32 Arduino` → `ESP32 Dev Module`.
4. Select the correct port: `Tools` → `Port` → (select COM or `/dev/ttyUSBx` for ESP32).
5. Click the **Upload** (▶) button.
   - *If upload fails to connect, hold the **BOOT** button on the ESP32 when you see "Connecting..." in the console.*
6. Once upload completes, the built-in LED should go solid — micro-ROS is ready.

## Hardware Connections

Connect the ESP32 DevKit to rover hardware as follows:

### L298N Motor Driver

| ESP32 GPIO | L298N Pin | Signal |
|------------|-----------|--------|
| GPIO 14 | ENA | Left motor PWM speed |
| GPIO 26 | IN1 | Left motor direction |
| GPIO 25 | IN2 | Left motor direction |
| GPIO 27 | ENB | Right motor PWM speed |
| GPIO 33 | IN3 | Right motor direction |
| GPIO 32 | IN4 | Right motor direction |

### HC-SR04 Ultrasonic Sensor (via Level Shifter)

> ⚠ **SAFETY**: HC-SR04 ECHO outputs 5V. ESP32 GPIOs are 3.3V tolerant ONLY.
> You MUST route ECHO through the 4-channel logic-level converter.

| Connection | Notes |
|-----------|-------|
| HC-SR04 VCC → 5V rail | HC-SR04 needs 5V |
| HC-SR04 GND → Common GND | — |
| HC-SR04 TRIG → ESP32 GPIO5 | Direct (3.3V trigger is fine) |
| HC-SR04 ECHO → Level Shifter HV CH1 | 5V side of converter |
| Level Shifter LV CH1 → ESP32 GPIO18 | 3.3V side → ESP32 |
| Level Shifter HV → 5V rail | — |
| Level Shifter LV → ESP32 3.3V | — |
| Level Shifter GND → Common GND | — |

### 4-Channel Logic-Level Converter Allocation

| Channel | HV side (5V) | LV side (3.3V) | Purpose |
|---------|-------------|----------------|---------|
| CH1 | HC-SR04 ECHO | ESP32 GPIO18 | Ultrasonic echo |
| CH2 | LCD SDA | ESP32 GPIO21 | I2C data |
| CH3 | LCD SCL | ESP32 GPIO22 | I2C clock |
| CH4 | (spare) | (spare) | Future use |

### Camera Tilt Servo (MG996R/MG995-class)

> ⚠ **POWER WARNING**: Servo requires a **dedicated external 5–6V supply (≥ 1A)**.
> Do NOT power servo from ESP32 3.3V or Raspberry Pi 5V — insufficient current will cause resets or damage.

| Servo Wire | Connection |
|-----------|-----------|
| Signal (orange/yellow) | ESP32 GPIO13 |
| Power (red) | External 5–6V supply positive |
| Ground (brown/black) | Common GND (shared with ESP32) |

### I2C LCD (16×2, PCF8574 backpack)

> **ASSUMPTION**: PCF8574-based backpack with 5V I2C pullups.
> If your backpack is 3.3V-native, connect SDA/SCL directly to GPIO21/22.

| LCD Pin | Connection |
|---------|-----------|
| VCC | 5V rail |
| GND | Common GND |
| SDA | Level Shifter HV CH2 (then LV CH2 → ESP32 GPIO21) |
| SCL | Level Shifter HV CH3 (then LV CH3 → ESP32 GPIO22) |

Default I2C address: **0x27**. If LCD is not detected, try **0x3F**.

### Serial (micro-ROS Agent)

| ESP32 | Connection |
|-------|-----------|
| USB port | Raspberry Pi USB-A port |
| Baud rate | 115200 |
| Device on Pi | `/dev/ttyUSB0` (most common) |

## Verification After Flashing

1. On Raspberry Pi, start the micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
   ```
2. Wait for the ESP32 LED to go solid.
3. Check nodes:
   ```bash
   ros2 node list
   # Expected: /buildscan_esp32
   ```
4. Check topics:
   ```bash
   ros2 topic list
   # Expected: /ultrasonic_range, /cmd_vel_safe, /emergency_stop, /camera_tilt_cmd
   ```
5. Test servo:
   ```bash
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 90.0"
   # Servo should move to 90° (neutral)
   ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 45.0"
   # Servo should tilt to 45°
   ```
6. Test HC-SR04:
   ```bash
   ros2 topic echo /ultrasonic_range
   # Should see Range messages at ~10 Hz
   ```

For full wiring reference, see: [`docs/hardware/buildscan_wiring.md`](../hardware/buildscan_wiring.md)

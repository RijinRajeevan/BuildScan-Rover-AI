# BuildScan Rover — Hardware Wiring Reference

**Status: Implemented in software. Hardware-dependent. Not fully physically validated.**

This document is the authoritative wiring reference for the BuildScan Rover Phase A hardware.
All GPIO numbers refer to the ESP32 DevKit V1 GPIO numbering.

---

## Quick Summary Table

| Component | Interface | Controller | Status |
|-----------|-----------|------------|--------|
| L298N + 4× DC motors | GPIO PWM + digital | ESP32 | ✅ Implemented |
| HC-SR04 | GPIO + level shifter | ESP32 | ✅ Implemented |
| Logitech Brio 100 camera | USB (V4L2) | Raspberry Pi 4 | ✅ Implemented |
| MG996R/MG995 servo | GPIO PWM | ESP32 | ✅ Implemented in firmware |
| 16×2 I2C LCD | I2C + level shifter | ESP32 | ✅ Implemented in firmware |
| 4-channel logic-level converter | Level shift | Between ESP32 and 5V peripherals | ✅ Wired (see below) |
| Raspberry Pi 4 | USB / Wi-Fi | Onboard compute | ✅ Implemented |

---

## 1. Raspberry Pi 4

| Pi Pin | Connection | Notes |
|--------|-----------|-------|
| USB-A port | ESP32 DevKit V1 (USB micro-B) | Serial: /dev/ttyUSB0 at 115200 baud (micro-ROS) |
| USB-A port | Logitech Brio 100 USB | /dev/video0 → /camera/image_raw |
| Wi-Fi | Dell G15 Laptop (same network) | ROS 2 DDS, ROS_DOMAIN_ID=25 |
| GPIO — NOT used for motors/sensors | — | All motor and sensor GPIO is on ESP32, NOT Raspberry Pi |

> ⚠ **IMPORTANT**: No motor or sensor GPIO is connected to the Raspberry Pi. All physical hardware control goes through the ESP32 micro-ROS.

---

## 2. ESP32 DevKit V1 — Full GPIO Map

| GPIO | Direction | Connected To | Signal |
|------|-----------|--------------|--------|
| **GPIO 14** | OUT | L298N ENA | Left motor PWM speed |
| **GPIO 26** | OUT | L298N IN1 | Left motor direction A |
| **GPIO 25** | OUT | L298N IN2 | Left motor direction B |
| **GPIO 27** | OUT | L298N ENB | Right motor PWM speed |
| **GPIO 33** | OUT | L298N IN3 | Right motor direction A |
| **GPIO 32** | OUT | L298N IN4 | Right motor direction B |
| **GPIO 5**  | OUT | HC-SR04 TRIG | Ultrasonic trigger pulse |
| **GPIO 18** | IN  | Level shifter LV side CH1 | HC-SR04 echo (level shifted from 5V) |
| **GPIO 13** | OUT | MG996R/MG995 servo signal | Camera tilt PWM (3.3V signal level) |
| **GPIO 21** | I/O | Level shifter LV side CH2 | I2C SDA for LCD |
| **GPIO 22** | I/O | Level shifter LV side CH3 | I2C SCL for LCD |
| **GPIO 1**  | OUT | Raspberry Pi UART RX | micro-ROS serial TX |
| **GPIO 3**  | IN  | Raspberry Pi UART TX | micro-ROS serial RX |

---

## 3. L298N Motor Driver

| L298N Terminal | Connection | Notes |
|----------------|-----------|-------|
| ENA | ESP32 GPIO14 | PWM signal — left motor speed (0–255 duty cycle) |
| IN1 | ESP32 GPIO26 | Left motor direction |
| IN2 | ESP32 GPIO25 | Left motor direction |
| OUT1, OUT2 | Left motor pair (M1, M2) | Both left-side DC motors in parallel |
| ENB | ESP32 GPIO27 | PWM signal — right motor speed |
| IN3 | ESP32 GPIO33 | Right motor direction |
| IN4 | ESP32 GPIO32 | Right motor direction |
| OUT3, OUT4 | Right motor pair (M3, M4) | Both right-side DC motors in parallel |
| VCC (12V) | Motor power supply (battery / buck converter) | 7–12V depending on motor specs |
| GND | Common ground | Must share GND with ESP32 and all other components |
| 5V out (onboard reg) | Optional ESP32 5V supply | Only if L298N regulator is sufficient (check datasheet) |

> ⚠ Do NOT exceed L298N rated current. For 4× DC motors, ensure your power supply provides sufficient current. Use a dedicated battery or buck converter.

---

## 4. HC-SR04 Ultrasonic Sensor

| HC-SR04 Pin | Connection | Notes |
|-------------|-----------|-------|
| VCC | 5V power rail | HC-SR04 requires 5V |
| GND | Common ground | Share with ESP32 GND |
| TRIG | ESP32 GPIO5 | 3.3V trigger is acceptable — 5V not required on TRIG |
| ECHO | 4-channel level shifter **HV side CH1** | ⚠ ECHO outputs 5V — NEVER connect directly to ESP32 GPIO |

### HC-SR04 ECHO Level Shifting (CRITICAL)

```
HC-SR04 ECHO (5V) ──→ Level Shifter HV side CH1 ──→ Level Shifter LV side CH1 ──→ ESP32 GPIO18 (3.3V)
```

- Level shifter HV rail: **5V** (from same 5V supply as HC-SR04)
- Level shifter LV rail: **3.3V** (from ESP32 3.3V pin)
- Level shifter GND: **common GND** with ESP32

> ⚠ **HARDWARE SAFETY**: Connecting HC-SR04 ECHO directly to ESP32 GPIO risks permanently damaging the ESP32 GPIO pin (5V input to 3.3V-tolerant GPIO). Always use the level shifter on CH1.

---

## 5. 4-Channel Logic-Level Converter

This single module handles all 5V ↔ 3.3V translation for the project.

| Converter Pin | Connected To | Voltage |
|--------------|-------------|---------|
| HV | 5V power rail | 5V |
| LV | ESP32 3.3V pin | 3.3V |
| GND | Common ground | 0V |
| **CH1 HV** | HC-SR04 ECHO (5V out) | 5V |
| **CH1 LV** | ESP32 GPIO18 (3.3V in) | 3.3V |
| **CH2 HV** | LCD I2C SDA (5V pullup) | 5V |
| **CH2 LV** | ESP32 GPIO21 (3.3V) | 3.3V |
| **CH3 HV** | LCD I2C SCL (5V pullup) | 5V |
| **CH3 LV** | ESP32 GPIO22 (3.3V) | 3.3V |
| **CH4 HV** | (spare — future use) | — |
| **CH4 LV** | (spare — future use) | — |

> **ASSUMPTION**: LCD backpack is PCF8574-based with 5V I2C pullups. If your backpack is 3.3V-native, CH2/CH3 are not required for the LCD — connect SDA/SCL directly to GPIO21/22. **Verify your exact backpack model before wiring.**

---

## 6. Camera Tilt Servo (MG996R/MG995-class)

| Servo Wire | Color (typical) | Connection | Notes |
|------------|----------------|-----------|-------|
| Signal | Orange / Yellow | ESP32 GPIO13 | 3.3V PWM signal — compatible with MG996R/MG995 class |
| Power (+) | Red | External 5–6V supply (⊕) | See power warning below |
| Ground (−) | Brown / Black | Common ground | Must share GND with ESP32 |

### Servo Power Warning

> ⚠ **CRITICAL**: Do NOT power the servo from:
> - ESP32 3.3V pin (insufficient current — will reset/damage ESP32)
> - Raspberry Pi 5V pin (insufficient current for MG996R/MG995 class — typically 1–2A stall current)
>
> **Use a dedicated external 5–6V supply capable of ≥ 1A continuous current.**
> A separate buck converter output or dedicated servo power rail is recommended.
> Connect the **negative terminal of the servo supply to ESP32 GND** (common ground).

### Servo Angle Range

- Safe operating range: **45° to 135°** (configurable in firmware)
- Default/neutral boot angle: **90°**
- Full 0°–180° is **NOT used** until the physical camera stand range is physically verified

### Servo Calibration Procedure

1. Flash firmware with default angles (45/90/135).
2. Power ESP32 and servo supply.
3. Publish center: `ros2 topic pub /camera_tilt_cmd std_msgs/msg/Float32 "data: 90.0"`
4. Confirm the camera stand is at mechanical center.
5. Test extremes: 45° and 135°.
6. If the stand hits a hard stop before 45° or 135°, adjust `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` in firmware and mirror to `robot_params.yaml`.

---

## 7. Camera — Logitech Brio 100 USB

| Connection | Details |
|-----------|---------|
| Interface | USB 2.0 / USB 3.0 |
| Connected to | Raspberry Pi 4 USB-A port |
| Linux device | `/dev/video0` (default) |
| ROS 2 node | `v4l2_camera_node` (package: `v4l2_camera`) |
| ROS 2 topic | `/camera/image_raw` (Image) |
| Resolution | 640×480, MJPG format |
| Power | Bus-powered from Raspberry Pi USB |

> The camera is NOT connected to the ESP32. It is a USB peripheral of the Raspberry Pi.

---

## 8. 16×2 I2C LCD

| LCD Pin | Connection | Notes |
|---------|-----------|-------|
| VCC | 5V power rail | LCD and backpack require 5V |
| GND | Common ground | Share with ESP32 GND |
| SDA | 4-channel level shifter **HV side CH2** | ⚠ 5V I2C pullup — must level-shift to 3.3V |
| SCL | 4-channel level shifter **HV side CH3** | ⚠ 5V I2C pullup — must level-shift to 3.3V |

**I2C Address**: 0x27 (PCF8574 default). If LCD is not detected, try 0x3F.

**ESP32 I2C pins**: SDA = GPIO21, SCL = GPIO22 (connected to LV side of level shifter CH2/CH3).

### LCD Display Content

| Row | Content | Notes |
|-----|---------|-------|
| Row 0 | `BuildScan Rover` or `*** E-STOP ***` | E-STOP takes priority |
| Row 1 | `D:X.XXm T:XXX` | Distance (meters) + Tilt angle (degrees) |

> ⚠ **ASSUMPTION**: PCF8574-based backpack at 5V. If your backpack is 3.3V-native, bypass level shifter for I2C lines. Verify backpack specs before wiring.

> ⚠ **LiDAR display**: LCD will NOT show "LiDAR: OK" until a physical LiDAR is connected and `use_physical_lidar = true` in params. Currently shows no LiDAR status (hardware not connected).

---

## 9. Power Rails

| Rail | Voltage | Source | Powers |
|------|---------|--------|--------|
| Motor rail | 7–12V | Battery / buck converter | L298N VCC, motor supply |
| 5V rail | 5V | L298N onboard reg OR separate buck converter | HC-SR04 VCC, LCD VCC, level shifter HV, servo supply |
| 3.3V rail | 3.3V | ESP32 onboard regulator | ESP32 GPIOs, level shifter LV |
| USB 5V | 5V | Raspberry Pi USB bus | Logitech Brio 100 camera |
| Servo supply | 5–6V, ≥1A | Dedicated buck converter output | MG996R/MG995-class servo only |

> ⚠ Do NOT use the L298N internal 5V regulator for the servo if motors are also running. The regulator may not supply sufficient current simultaneously.

---

## 10. Common Ground

**All components must share a common ground:**

- ESP32 GND
- L298N GND
- HC-SR04 GND
- Level shifter GND (both HV and LV sides share GND)
- LCD GND
- Servo power supply GND
- Raspberry Pi GND (if directly connected; otherwise isolated via USB)
- Battery / buck converter GND

> ⚠ **Floating ground** is the most common cause of erratic sensor behavior and motor noise. Verify continuity between all GND points with a multimeter before powering up.

---

## 11. Safety Warnings Summary

| Warning | Action |
|---------|--------|
| HC-SR04 ECHO is 5V | ALWAYS use level shifter CH1 — never connect directly to ESP32 |
| LCD I2C has 5V pullups (if PCF8574 backpack) | Use level shifter CH2 + CH3 |
| Servo requires 5–6V, ≥1A | Use dedicated supply — never ESP32 3.3V or Pi 5V |
| Servo signal from ESP32 GPIO13 (3.3V) | Acceptable for MG996R/MG995 class — verify your exact model |
| Common ground must be shared by all components | Missing ground = random failures |
| Do NOT exceed L298N current rating | Use appropriate motor supply |
| Do NOT exceed ESP32 GPIO current (max ~12mA per pin) | Only use GPIO for signal lines — never motor or servo power |

---

## Phase Status

| Feature | Status |
|---------|--------|
| Motor control (L298N + 4WD) | ✅ Implemented in software — hardware-dependent |
| HC-SR04 safety stop | ✅ Implemented in software — hardware-dependent |
| Camera (Logitech Brio 100 + v4l2) | ✅ Implemented in software — hardware-dependent |
| Camera tilt servo | ✅ Implemented in software — hardware-dependent, not physically tested |
| I2C LCD display | ✅ Implemented in software — hardware-dependent, not physically tested |
| Level shifter wiring | ✅ Documented — not physically validated |
| LiDAR (2D) | ❌ NOT implemented — hardware not yet connected |
| Autonomous navigation (Nav2) | ❌ NOT implemented |
| Wheel odometry | ❌ NOT implemented (no encoders) |
| Servo angle feedback | ❌ NOT available — open-loop commanded angle only |

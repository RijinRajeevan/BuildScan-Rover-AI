/**
 * esp32_microros.ino  ── Phase A (extended)
 * ══════════════════════════════════════════
 * BuildScan Rover – ESP32 DevKit micro-ROS Firmware
 *
 * Hardware controlled by this firmware:
 *   1. L298N motor driver  (4WD differential drive)
 *   2. HC-SR04 ultrasonic sensor  → /ultrasonic_range
 *   3. MG996R/MG995-class camera-tilt servo  ← /camera_tilt_cmd
 *   4. 16×2 I2C LCD (PCF8574 backpack assumed)
 *
 * micro-ROS subscriptions:
 *   /cmd_vel_safe      geometry_msgs/Twist   → motor drive
 *   /emergency_stop    std_msgs/Bool         → hardware stop
 *   /camera_tilt_cmd   std_msgs/Float32      → servo angle (degrees)
 *
 * micro-ROS publications:
 *   /ultrasonic_range  sensor_msgs/Range     → HC-SR04 distance @ 10 Hz
 *
 * ══════════════════════════════════════════════════════════════════════════
 * PIN MAP  (ESP32 DevKit V1)
 * ══════════════════════════════════════════════════════════════════════════
 *   L298N Motor Driver:
 *     ENA → GPIO 14    ENB → GPIO 27
 *     IN1 → GPIO 26    IN2 → GPIO 25
 *     IN3 → GPIO 33    IN4 → GPIO 32
 *
 *   HC-SR04 Ultrasonic Sensor:
 *     TRIG → GPIO 5
 *     ECHO → GPIO 18
 *     ⚠ SAFETY: HC-SR04 ECHO outputs 5V. ESP32 GPIOs are 3.3V tolerant only.
 *     Route ECHO through a 4-channel logic-level converter:
 *       HC-SR04 ECHO → level shifter HV side (5V) → LV side (3.3V) → GPIO 18
 *       Level shifter: HV = 5V rail, LV = 3.3V rail, GND common with ESP32.
 *
 *   Camera Tilt Servo (MG996R/MG995-class):
 *     Signal → GPIO 13  (3.3V PWM signal, compatible with most MG996R/MG995 servos)
 *     ⚠ POWER: Do NOT power the servo from ESP32 3.3V or Raspberry Pi 5V.
 *       Servo requires a dedicated 5–6V supply with ≥ 1A capacity.
 *       Common ground between external supply and ESP32 GND is mandatory.
 *
 *   I2C LCD (16×2, PCF8574 I2C backpack, assumed 5V backpack):
 *     SDA → GPIO 21  (route via level shifter LV side)
 *     SCL → GPIO 22  (route via level shifter LV side)
 *     ⚠ ASSUMPTION: This assumes a PCF8574-based LCD backpack powered from 5V.
 *       If the backpack's SDA/SCL pullups are 5V, use level shifter channels:
 *         CH2: LCD SDA (HV=5V) → ESP32 GPIO21 (LV=3.3V)
 *         CH3: LCD SCL (HV=5V) → ESP32 GPIO22 (LV=3.3V)
 *       If your backpack is 3.3V-native, connect directly (verify before powering).
 *     Default I2C address: 0x27 (common PCF8574). If LCD not found, try 0x3F.
 *
 *   Serial (micro-ROS agent):
 *     TX → GPIO 1    RX → GPIO 3 (default UART0 — shared with USB)
 *
 * ══════════════════════════════════════════════════════════════════════════
 * LIBRARY DEPENDENCIES (Arduino IDE / PlatformIO)
 * ══════════════════════════════════════════════════════════════════════════
 *   - micro_ros_arduino  (https://github.com/micro-ROS/micro_ros_arduino)
 *     Install: Sketch → Include Library → Add .ZIP Library (Humble release)
 *   - ESP32 Arduino core
 *   - ESP32Servo  (by Kevin Harrington / madhephaestus)
 *     Install: Arduino Library Manager → search "ESP32Servo"
 *   - LiquidCrystal_I2C  (by Frank de Brabander)
 *     Install: Arduino Library Manager → search "LiquidCrystal I2C"
 *
 * ══════════════════════════════════════════════════════════════════════════
 * micro-ROS Agent (Raspberry Pi):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
 * ══════════════════════════════════════════════════════════════════════════
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

// L298N Motor Driver
#define ENA 14
#define IN1 26
#define IN2 25
#define ENB 27
#define IN3 33
#define IN4 32

// HC-SR04 Ultrasonic Sensor
// WARNING: ECHO pin outputs 5V — must go through level-shifter LV→HV before GPIO18.
#define TRIG_PIN 5
#define ECHO_PIN 18

// Camera Tilt Servo (MG996R/MG995-class)
// WARNING: Servo power (5–6V, ≥1A) must come from an external supply.
// Do NOT use ESP32 3.3V or Raspberry Pi 5V to power the servo motor.
#define SERVO_PIN 13

// I2C LCD (PCF8574 backpack)
// Route through level-shifter if backpack pullups are 5V.
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22
#define LCD_I2C_ADDR 0x27   // Common PCF8574 address. Try 0x3F if not found.
#define LCD_COLS     16
#define LCD_ROWS     2

// Motor PWM channels
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8       // 8-bit = 0-255
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1

// ═══════════════════════════════════════════════════════════════════════════
// PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════

// Motor
#define MOTOR_SPEED_DEFAULT   160     // PWM value (0-255)
#define WHEEL_RADIUS_M        0.033f  // meters
#define WHEEL_BASE_M          0.160f  // meters (track width)
#define MAX_LINEAR_VEL        0.30f   // m/s
#define MAX_ANGULAR_VEL       1.00f   // rad/s

// HC-SR04
#define ULTRASONIC_FRAME_ID   "ultrasonic_link"
#define ULTRASONIC_MIN_RANGE  0.02f   // meters
#define ULTRASONIC_MAX_RANGE  4.00f   // meters
#define ULTRASONIC_FOV        0.2618f // radians (~15°)

// Watchdog
#define CMD_VEL_TIMEOUT_MS    2000    // stop if no /cmd_vel for 2 seconds

// Camera Tilt Servo
// ─────────────────
// These angles define the SAFE operating range for the physical camera stand.
// Calibration note: Start with SERVO_DEFAULT_ANGLE = 90 (neutral).
// Incrementally increase/decrease and observe the physical mount.
// Set MIN and MAX to the outermost angles that do NOT cause mechanical hard-stop
// or bind against the camera stand structure.
// ─────────────────
#define SERVO_MIN_ANGLE     45    // degrees — safest downward tilt limit
#define SERVO_MAX_ANGLE     135   // degrees — safest upward tilt limit
#define SERVO_DEFAULT_ANGLE 90    // degrees — neutral / boot position
// Note: Full 0-180° is NOT used. Stay within [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE]
// until you have physically verified the printed camera stand allows that range.

// LCD update rate — update LCD every N ultrasonic timer calls (1 call = 100ms)
// LCD_UPDATE_EVERY = 5 → LCD refresh ~0.5 Hz (slow enough not to flicker)
#define LCD_UPDATE_EVERY 5

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

Servo camera_servo;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// ═══════════════════════════════════════════════════════════════════════════
// micro-ROS OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

rcl_allocator_t        allocator;
rclc_support_t         support;
rcl_node_t             node;
rclc_executor_t        executor;

// Subscribers (3 total)
rcl_subscription_t     cmd_vel_sub;
rcl_subscription_t     estop_sub;
rcl_subscription_t     cam_tilt_sub;

// Publishers (1 total)
rcl_publisher_t        range_pub;

// Messages
geometry_msgs__msg__Twist   cmd_vel_msg;
sensor_msgs__msg__Range     range_msg;
std_msgs__msg__Bool         estop_msg;
std_msgs__msg__Float32      cam_tilt_msg;

// Timers (1 total)
rcl_timer_t  ultrasonic_timer;

// ═══════════════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════════════

bool      emergency_stop       = false;
uint32_t  last_cmd_vel_time    = 0;
float     commanded_tilt_angle = SERVO_DEFAULT_ANGLE;
uint8_t   lcd_update_counter   = 0;

// ═══════════════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════════════

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void error_loop() {
  // Flash built-in LED rapidly to signal micro-ROS init error.
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════

void motor_stop() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motor_drive(float linear_x, float angular_z) {
  if (emergency_stop) {
    motor_stop();
    return;
  }

  // Differential drive kinematics
  float left_vel  = linear_x - (angular_z * WHEEL_BASE_M / 2.0f);
  float right_vel = linear_x + (angular_z * WHEEL_BASE_M / 2.0f);

  // Clamp to max velocity
  left_vel  = constrain(left_vel,  -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  right_vel = constrain(right_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

  // Scale to PWM (0-255)
  int left_pwm  = (int)(fabs(left_vel)  / MAX_LINEAR_VEL * 255);
  int right_pwm = (int)(fabs(right_vel) / MAX_LINEAR_VEL * 255);

  // Left motors direction (IN1, IN2)
  if (left_vel > 0.01f) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (left_vel < -0.01f) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  }

  // Right motors direction (IN3, IN4)
  if (right_vel > 0.01f) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (right_vel < -0.01f) {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  }

  ledcWrite(PWM_CHANNEL_A, left_pwm);
  ledcWrite(PWM_CHANNEL_B, right_pwm);
}

// ═══════════════════════════════════════════════════════════════════════════
// SERVO CONTROL
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Move camera servo to the requested angle (degrees).
 * Clamps to [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE] to protect the physical camera
 * stand from mechanical hard-stop damage.
 * The servo is open-loop — no angle feedback sensor is present.
 */
void servo_set_angle(float angle_deg) {
  float clamped = constrain(angle_deg, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
  commanded_tilt_angle = clamped;
  camera_servo.write((int)clamped);
}

// ═══════════════════════════════════════════════════════════════════════════
// LCD DISPLAY
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Update the 16×2 LCD with current rover state.
 * Called from the ultrasonic timer callback at a slow rate (LCD_UPDATE_EVERY).
 *
 * Row 0: Mode + e-stop indicator
 * Row 1: Distance + servo angle
 *
 * Layout example:
 *   "MANUAL   [STOP]  "
 *   "Dist:0.45 T:090  "
 *
 * IMPORTANT: "LiDAR: N/A" is shown until hardware is physically connected.
 * Do NOT change this to "LiDAR: OK" without a real LiDAR providing /scan.
 */
void lcd_update(float distance_m) {
  // ── Row 0: Mode + emergency stop flag ───────────────────────────────────
  lcd.setCursor(0, 0);
  if (emergency_stop) {
    lcd.print("*** E-STOP ***  ");
  } else {
    // Show first 8 chars of a fixed status label
    lcd.print("BUILDSCAN ROVER ");
  }

  // ── Row 1: Distance + commanded servo angle ──────────────────────────────
  lcd.setCursor(0, 1);
  char line2[17];
  // Format: "D:X.XXm  T:XXX  " (16 chars)
  snprintf(line2, sizeof(line2), "D:%4.2fm T:%3d   ",
           distance_m,
           (int)commanded_tilt_angle);
  lcd.print(line2);
}

// ═══════════════════════════════════════════════════════════════════════════
// ULTRASONIC SENSOR
// ═══════════════════════════════════════════════════════════════════════════

float get_distance_m() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // 25ms timeout
  if (duration == 0) return ULTRASONIC_MAX_RANGE;

  float dist = (duration * 0.034f) / 2.0f / 100.0f;  // cm → meters
  return constrain(dist, ULTRASONIC_MIN_RANGE, ULTRASONIC_MAX_RANGE);
}

// ═══════════════════════════════════════════════════════════════════════════
// micro-ROS CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
    (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_vel_time = millis();
  motor_drive(msg->linear.x, msg->angular.z);
}

void estop_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  emergency_stop = msg->data;

  if (emergency_stop) {
    motor_stop();
    // Note: servo is NOT stopped — it holds its position safely.
    // Camera tilt is not a safety hazard; stopping it is unnecessary.
  }
}

/**
 * Camera tilt callback — /camera_tilt_cmd (std_msgs/Float32).
 *
 * The Float32 value is the requested servo angle in degrees.
 * Valid range: [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE].
 * Out-of-range values are CLAMPED (not rejected) to protect the servo mount.
 * Emergency stop does NOT block servo movement — tilt is not a drive system.
 */
void cam_tilt_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  servo_set_angle(msg->data);
}

void ultrasonic_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  float dist = get_distance_m();

  // Populate Range message
  range_msg.range = dist;
  range_msg.header.stamp.sec     = 0;
  range_msg.header.stamp.nanosec = 0;

  RCSOFTCHECK(rcl_publish(&range_pub, &range_msg, NULL));

  // Update LCD every LCD_UPDATE_EVERY calls (throttle to avoid flicker)
  lcd_update_counter++;
  if (lcd_update_counter >= LCD_UPDATE_EVERY) {
    lcd_update_counter = 0;
    lcd_update(dist);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // ── GPIO Setup ───────────────────────────────────────────────────────────
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Ensure motors are stopped on boot before anything else runs
  motor_stop();

  // ── PWM for motor speed ───────────────────────────────────────────────────
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

  // ── Camera Servo Init ────────────────────────────────────────────────────
  // Attach servo and move to safe neutral position on boot.
  // This prevents the servo from jerking to an unknown position at startup.
  camera_servo.attach(SERVO_PIN);
  servo_set_angle(SERVO_DEFAULT_ANGLE);
  delay(500);  // Allow servo to reach neutral before micro-ROS init starts

  // ── LCD Init ─────────────────────────────────────────────────────────────
  // Wire.begin() uses the default SDA/SCL pins (GPIO21/GPIO22 on ESP32).
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("BuildScan Rover ");
  lcd.setCursor(0, 1);
  lcd.print("Initializing... ");

  // ── Serial for micro-ROS agent ────────────────────────────────────────────
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);  // Wait for micro-ROS agent connection

  // ── micro-ROS Init ────────────────────────────────────────────────────────
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node, "buildscan_esp32", "", &support));

  // ── Subscribers (3) ──────────────────────────────────────────────────────

  // /cmd_vel_safe — motor drive commands (safety-clamped by motor_interface_node)
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel_safe"
  ));

  // /emergency_stop — hardware-level motor stop
  RCCHECK(rclc_subscription_init_default(
    &estop_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/emergency_stop"
  ));

  // /camera_tilt_cmd — servo angle command (Float32, degrees)
  RCCHECK(rclc_subscription_init_default(
    &cam_tilt_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/camera_tilt_cmd"
  ));

  // ── Publisher (1) ─────────────────────────────────────────────────────────

  // /ultrasonic_range — HC-SR04 distance readings
  RCCHECK(rclc_publisher_init_default(
    &range_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/ultrasonic_range"
  ));

  // ── Pre-fill Range message static fields ──────────────────────────────────
  static char frame_id_buf[] = ULTRASONIC_FRAME_ID;
  range_msg.header.frame_id.data     = frame_id_buf;
  range_msg.header.frame_id.size     = strlen(frame_id_buf);
  range_msg.header.frame_id.capacity = strlen(frame_id_buf) + 1;
  range_msg.radiation_type           = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view            = ULTRASONIC_FOV;
  range_msg.min_range                = ULTRASONIC_MIN_RANGE;
  range_msg.max_range                = ULTRASONIC_MAX_RANGE;

  // ── Ultrasonic Timer (10 Hz = 100ms) ─────────────────────────────────────
  RCCHECK(rclc_timer_init_default(
    &ultrasonic_timer,
    &support,
    RCL_MS_TO_NS(100),
    ultrasonic_timer_callback
  ));

  // ── Executor: 3 subscriptions + 1 timer = 4 handles ─────────────────────
  // IMPORTANT: handle count must match exactly (3 subs + 1 timer = 4).
  // If you add more subs or timers, increase this number accordingly.
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &estop_sub, &estop_msg, &estop_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &cam_tilt_sub, &cam_tilt_msg, &cam_tilt_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &ultrasonic_timer));

  // ── Boot complete ─────────────────────────────────────────────────────────
  digitalWrite(LED_BUILTIN, HIGH);  // Solid LED = micro-ROS ready

  lcd.setCursor(0, 0);
  lcd.print("BuildScan Rover ");
  lcd.setCursor(0, 1);
  lcd.print("ROS2 Ready!     ");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // ── cmd_vel Watchdog ──────────────────────────────────────────────────────
  // If no /cmd_vel_safe received within CMD_VEL_TIMEOUT_MS, stop motors.
  // This protects against communication loss (Wi-Fi drop, agent crash).
  if ((millis() - last_cmd_vel_time) > CMD_VEL_TIMEOUT_MS) {
    motor_stop();
  }

  // ── micro-ROS spin ────────────────────────────────────────────────────────
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(5);
}

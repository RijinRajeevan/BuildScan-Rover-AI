/**
 * esp32_microros.ino  ── Stage 2 & 3
 * ═════════════════════════════════════
 * BuildScan Rover – ESP32 DevKit micro-ROS Firmware
 * 
 * Replaces: Arduino UNO + HC-05 Bluetooth (moved to legacy/)
 * 
 * This firmware runs micro-ROS on an ESP32 DevKit and:
 *   1. Subscribes to /cmd_vel → drives L298N motor driver
 *   2. Subscribes to /cmd_vel_safe → alternative safe velocity topic
 *   3. Publishes /ultrasonic_range → HC-SR04 distance as sensor_msgs/Range
 *   4. Implements hardware-level stop on /emergency_stop
 * 
 * Hardware connections (ESP32 DevKit):
 *   L298N Motor Driver:
 *     ENA → GPIO 14    ENB → GPIO 27
 *     IN1 → GPIO 26    IN2 → GPIO 25
 *     IN3 → GPIO 33    IN4 → GPIO 32
 *   HC-SR04 Ultrasonic:
 *     TRIG → GPIO 5    ECHO → GPIO 18
 *   Serial to Raspberry Pi (micro-ROS agent):
 *     TX → GPIO 1      RX → GPIO 3 (default UART0)
 * 
 * micro-ROS Agent on Raspberry Pi:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
 * 
 * Dependencies (Arduino IDE / PlatformIO):
 *   - micro_ros_arduino (https://github.com/micro-ROS/micro_ros_arduino)
 *   - ESP32 Arduino core
 * 
 * IMPORTANT: Replace "buildscan" with actual micro_ros_arduino namespace if needed.
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
// WARNING: HC-SR04 ECHO pin outputs 5V. ESP32 GPIOs are 3.3V tolerant only.
// You MUST use a voltage divider or logic-level converter between ECHO and GPIO 18.
#define TRIG_PIN 5
#define ECHO_PIN 18

// Motor PWM channels
#define PWM_FREQ      1000
#define PWM_RESOLUTION 8    // 8-bit = 0-255
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1

// ═══════════════════════════════════════════════════════════════════════════
// PARAMETERS (matches robot_params.yaml)
// ═══════════════════════════════════════════════════════════════════════════

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

// ═══════════════════════════════════════════════════════════════════════════
// micro-ROS OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

rcl_allocator_t        allocator;
rclc_support_t         support;
rcl_node_t             node;
rclc_executor_t        executor;

// Subscribers
rcl_subscription_t     cmd_vel_sub;
rcl_subscription_t     estop_sub;

// Publishers
rcl_publisher_t        range_pub;

// Messages
geometry_msgs__msg__Twist  cmd_vel_msg;
sensor_msgs__msg__Range    range_msg;
std_msgs__msg__Bool        estop_msg;

// Timers
rcl_timer_t  ultrasonic_timer;

// ═══════════════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════════════

bool      emergency_stop     = false;
uint32_t  last_cmd_vel_time  = 0;

// ═══════════════════════════════════════════════════════════════════════════
// ERROR HANDLING
// ═══════════════════════════════════════════════════════════════════════════

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){ error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){} }

void error_loop() {
  // Flash LED to indicate error
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
  // left_vel  = linear_x - (angular_z * WHEEL_BASE_M / 2.0)
  // right_vel = linear_x + (angular_z * WHEEL_BASE_M / 2.0)
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
// ULTRASONIC
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
  }
}

void ultrasonic_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  float dist = get_distance_m();

  // Populate Range message
  range_msg.range = dist;
  // Timestamp via POSIX time is optional in micro-ROS; use 0 for now
  range_msg.header.stamp.sec     = 0;
  range_msg.header.stamp.nanosec = 0;

  RCSOFTCHECK(rcl_publish(&range_pub, &range_msg, NULL));
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // ── GPIO Setup ─────────────────────────────────────────────────────────
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // PWM setup for motor speed
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

  motor_stop();  // Ensure motors are stopped on boot

  // ── Serial for micro-ROS agent ─────────────────────────────────────────
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);  // Wait for agent connection

  // ── micro-ROS Init ─────────────────────────────────────────────────────
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node, "buildscan_esp32", "", &support));

  // ── Subscribers ────────────────────────────────────────────────────────
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel_safe"
  ));

  RCCHECK(rclc_subscription_init_default(
    &estop_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/emergency_stop"
  ));

  // ── Publisher ──────────────────────────────────────────────────────────
  RCCHECK(rclc_publisher_init_default(
    &range_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/ultrasonic_range"
  ));

  // ── Pre-fill Range message static fields ───────────────────────────────
  // frame_id string allocation
  static char frame_id_buf[] = ULTRASONIC_FRAME_ID;
  range_msg.header.frame_id.data = frame_id_buf;
  range_msg.header.frame_id.size = strlen(frame_id_buf);
  range_msg.header.frame_id.capacity = strlen(frame_id_buf) + 1;
  range_msg.radiation_type  = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view   = ULTRASONIC_FOV;
  range_msg.min_range       = ULTRASONIC_MIN_RANGE;
  range_msg.max_range       = ULTRASONIC_MAX_RANGE;

  // ── Ultrasonic Timer (10 Hz = 100ms) ───────────────────────────────────
  RCCHECK(rclc_timer_init_default(
    &ultrasonic_timer,
    &support,
    RCL_MS_TO_NS(100),
    ultrasonic_timer_callback
  ));

  // ── Executor: 2 subscriptions + 1 timer ────────────────────────────────
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &estop_sub, &estop_msg, &estop_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &ultrasonic_timer));

  digitalWrite(LED_BUILTIN, HIGH);  // Signal ready
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // ── cmd_vel Watchdog ───────────────────────────────────────────────────
  // Stop motors if no /cmd_vel received within timeout
  if ((millis() - last_cmd_vel_time) > CMD_VEL_TIMEOUT_MS) {
    motor_stop();
  }

  // ── micro-ROS spin ─────────────────────────────────────────────────────
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(5);
}

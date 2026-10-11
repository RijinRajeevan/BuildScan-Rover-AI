/**
 * motor_diagnostic.ino
 * ════════════════════════════════════════════════════════════════════════════
 * BUILDScan Rover — TEMPORARY Motor Hardware Diagnostic
 * ════════════════════════════════════════════════════════════════════════════
 *
 * PURPOSE  : Standalone motor test. Flash temporarily to verify all four DC
 *            geared motors and the HW-095/L298N driver work correctly.
 *            Restore esp32_microros.ino when done.
 *
 * HARDWARE : ESP32 DevKit V1 + HW-095 / L298N dual H-bridge
 *            4WD chassis - two motors per side, wired in parallel per channel.
 *
 * EXCLUDED : No ROS 2, micro-ROS, Wi-Fi, Bluetooth, servo, LCD, PCA9685,
 *            ultrasonic sensor. Pure motor + serial only.
 *
 * GPIO MAP (identical to esp32_microros.ino - do NOT rewire):
 *   LEFT SIDE  : ENA = GPIO 14   IN1 = GPIO 26   IN2 = GPIO 25
 *   RIGHT SIDE : ENB = GPIO 27   IN1 = GPIO 33   IN2 = GPIO 32
 *
 * SERIAL COMMANDS (115200 baud):
 *   F = Forward       B = Backward
 *   L = Turn Left     R = Turn Right
 *   S = Stop
 *
 * PWM API : ESP32 Arduino Core 3.x -> ledcAttach() / ledcWrite()
 *           NOT using deprecated ledcSetup() / ledcAttachPin()
 *
 * SAFETY  : Motors start STOPPED. PWM = 80/255 (~31%) for controlled test.
 * ════════════════════════════════════════════════════════════════════════════
 */

// ─────────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS  (mirrors esp32_microros.ino - do NOT change)
// ─────────────────────────────────────────────────────────────────────────────
#define LEFT_PWM   14   // ENA  - left  channel enable / speed
#define LEFT_IN1   26   // IN1  - left  direction bit A
#define LEFT_IN2   25   // IN2  - left  direction bit B

#define RIGHT_PWM  27   // ENB  - right channel enable / speed
#define RIGHT_IN1  33   // IN3  - right direction bit A
#define RIGHT_IN2  32   // IN4  - right direction bit B

// ─────────────────────────────────────────────────────────────────────────────
//  PWM CONFIGURATION  (ESP32 Core 3.x ledcAttach API)
// ─────────────────────────────────────────────────────────────────────────────
#define PWM_FREQ_HZ    1000   // 1 kHz - well within L298N spec
#define PWM_RESOLUTION 8      // 8-bit -> duty range 0-255
#define MOTOR_SPEED    80     // 80/255 ~31% - conservative starting speed

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR PRIMITIVES
// ─────────────────────────────────────────────────────────────────────────────

void motorsStop() {
    ledcWrite(LEFT_PWM,  0);
    ledcWrite(RIGHT_PWM, 0);
    digitalWrite(LEFT_IN1,  LOW);
    digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
}

void motorsForward() {
    digitalWrite(LEFT_IN1,  HIGH); digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);
    ledcWrite(LEFT_PWM,  MOTOR_SPEED);
    ledcWrite(RIGHT_PWM, MOTOR_SPEED);
}

void motorsBackward() {
    digitalWrite(LEFT_IN1,  LOW);  digitalWrite(LEFT_IN2,  HIGH);
    digitalWrite(RIGHT_IN1, LOW);  digitalWrite(RIGHT_IN2, HIGH);
    ledcWrite(LEFT_PWM,  MOTOR_SPEED);
    ledcWrite(RIGHT_PWM, MOTOR_SPEED);
}

void motorsTurnLeft() {
    digitalWrite(LEFT_IN1,  LOW);  digitalWrite(LEFT_IN2,  HIGH);
    digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);
    ledcWrite(LEFT_PWM,  MOTOR_SPEED);
    ledcWrite(RIGHT_PWM, MOTOR_SPEED);
}

void motorsTurnRight() {
    digitalWrite(LEFT_IN1,  HIGH); digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, LOW);  digitalWrite(RIGHT_IN2, HIGH);
    ledcWrite(LEFT_PWM,  MOTOR_SPEED);
    ledcWrite(RIGHT_PWM, MOTOR_SPEED);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LEFT_IN1,  OUTPUT);
    pinMode(LEFT_IN2,  OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    // ESP32 Arduino Core 3.x PWM API
    // ledcAttach(pin, frequency_Hz, resolution_bits)
    ledcAttach(LEFT_PWM,  PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttach(RIGHT_PWM, PWM_FREQ_HZ, PWM_RESOLUTION);

    motorsStop();

    Serial.println();
    Serial.println("============================================");
    Serial.println("  BUILDScan Rover -- Motor Diagnostic");
    Serial.println("  HW-095 / L298N  |  4WD  |  115200 baud");
    Serial.println("============================================");
    Serial.println("  GPIO layout:");
    Serial.println("    LEFT   PWM=GPIO14  IN1=GPIO26  IN2=GPIO25");
    Serial.println("    RIGHT  PWM=GPIO27  IN1=GPIO33  IN2=GPIO32");
    Serial.println("  Speed: 80/255 (31%) -- conservative test");
    Serial.println("============================================");
    Serial.println("  Commands (send single character):");
    Serial.println("    F = Forward");
    Serial.println("    B = Backward");
    Serial.println("    L = Turn Left");
    Serial.println("    R = Turn Right");
    Serial.println("    S = Stop");
    Serial.println("============================================");
    Serial.println("  MOTORS: STOPPED -- awaiting command...");
    Serial.println("============================================");
}

// ─────────────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();

        while (Serial.available() > 0) {
            char peek = (char)Serial.peek();
            if (peek == '\n' || peek == '\r' || peek == ' ') {
                Serial.read();
            } else {
                break;
            }
        }

        if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

        switch (cmd) {

            case 'F': case 'f':
                motorsForward();
                Serial.println();
                Serial.println("F");
                Serial.println("FORWARD -- LEFT + RIGHT MOTOR SIDES");
                break;

            case 'B': case 'b':
                motorsBackward();
                Serial.println();
                Serial.println("B");
                Serial.println("BACKWARD -- LEFT + RIGHT MOTOR SIDES");
                break;

            case 'L': case 'l':
                motorsTurnLeft();
                Serial.println();
                Serial.println("L");
                Serial.println("TURN LEFT -- LEFT REVERSE, RIGHT FORWARD");
                break;

            case 'R': case 'r':
                motorsTurnRight();
                Serial.println();
                Serial.println("R");
                Serial.println("TURN RIGHT -- LEFT FORWARD, RIGHT REVERSE");
                break;

            case 'S': case 's':
                motorsStop();
                Serial.println();
                Serial.println("S");
                Serial.println("STOPPED");
                break;

            default:
                Serial.print("UNKNOWN: '");
                Serial.print(cmd);
                Serial.println("'  ->  use F, B, L, R, or S");
                break;
        }
    }
}

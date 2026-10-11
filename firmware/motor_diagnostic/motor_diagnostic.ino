/**
 * motor_diagnostic.ino
 * ============================================================================
 * BUILDScan Rover — Standalone Motor Hardware Diagnostic
 * ============================================================================
 * 
 * AUTOMATIC CYCLING + INTERACTIVE SERIAL CONTROL
 * 
 * Works whether:
 *  1) ENA/ENB have jumper caps attached (full 5V)
 *  2) ENA/ENB are wired to GPIO 14 & GPIO 27 (driven with PWM / HIGH)
 * 
 * WIRING:
 *   LEFT SIDE  : ENA -> GPIO 14, IN1 -> GPIO 26, IN2 -> GPIO 25
 *   RIGHT SIDE : ENB -> GPIO 27, IN3 -> GPIO 33, IN4 -> GPIO 32
 *   COMMON GND : ESP32 GND -> L298N GND (MANDATORY)
 *   POWER      : Battery (+) -> L298N 12V, Battery (-) -> L298N GND
 * ============================================================================
 */

#include <Arduino.h>

// Pin definitions
#define LEFT_PWM   14   // ENA
#define LEFT_IN1   26   // IN1
#define LEFT_IN2   25   // IN2

#define RIGHT_PWM  27   // ENB
#define RIGHT_IN1  33   // IN3
#define RIGHT_IN2  32   // IN4

// PWM parameters
#define PWM_FREQ_HZ    1000
#define PWM_RESOLUTION 8
// Increased speed to 200/255 (~78%) to ensure gearboxes have enough torque to spin
#define MOTOR_SPEED    200

bool autoMode = true;
unsigned long lastStepTime = 0;
int autoStep = 0;

void setSpeed(uint32_t speed) {
    ledcWrite(LEFT_PWM, speed);
    ledcWrite(RIGHT_PWM, speed);
}

void motorsStop() {
    setSpeed(0);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
}

void motorsForward() {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    setSpeed(MOTOR_SPEED);
}

void motorsBackward() {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    setSpeed(MOTOR_SPEED);
}

void motorsTurnLeft() {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    setSpeed(MOTOR_SPEED);
}

void motorsTurnRight() {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    setSpeed(MOTOR_SPEED);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    // ESP32 Arduino Core 3.x PWM
    ledcAttach(LEFT_PWM, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttach(RIGHT_PWM, PWM_FREQ_HZ, PWM_RESOLUTION);

    motorsStop();

    Serial.println("\n============================================");
    Serial.println("  BUILDScan Rover -- Motor Diagnostic (AUTO)");
    Serial.println("============================================");
    Serial.println("  WIRING CHECK:");
    Serial.println("    LEFT : ENA=GPIO14, IN1=GPIO26, IN2=GPIO25");
    Serial.println("    RIGHT: ENB=GPIO27, IN3=GPIO33, IN4=GPIO32");
    Serial.println("    GND  : ESP32 GND <-> L298N GND");
    Serial.println("    POWER: Battery connected to L298N 12V + GND");
    Serial.println("--------------------------------------------");
    Serial.println("  AUTO SEQUENCE STARTING in 2 SECONDS...");
    Serial.println("  (Type 'S' to STOP, 'A' to re-enable AUTO)");
    Serial.println("  (Type 'F', 'B', 'L', 'R' for manual control)");
    Serial.println("============================================\n");

    lastStepTime = millis();
}

void loop() {
    // Check for user manual serial commands
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        while (Serial.available() > 0) Serial.read(); // flush

        if (cmd == 'S' || cmd == 's') {
            autoMode = false;
            motorsStop();
            Serial.println(">> STOPPED (Auto mode paused. Send 'A' to resume)");
        } else if (cmd == 'F' || cmd == 'f') {
            autoMode = false;
            motorsForward();
            Serial.println(">> FORWARD (Speed: 200/255)");
        } else if (cmd == 'B' || cmd == 'b') {
            autoMode = false;
            motorsBackward();
            Serial.println(">> BACKWARD (Speed: 200/255)");
        } else if (cmd == 'L' || cmd == 'l') {
            autoMode = false;
            motorsTurnLeft();
            Serial.println(">> TURN LEFT");
        } else if (cmd == 'R' || cmd == 'r') {
            autoMode = false;
            motorsTurnRight();
            Serial.println(">> TURN RIGHT");
        } else if (cmd == 'A' || cmd == 'a') {
            autoMode = true;
            autoStep = 0;
            lastStepTime = millis();
            Serial.println(">> AUTO MODE RESUMED");
        }
    }

    // Auto test sequence
    if (autoMode) {
        unsigned long now = millis();
        switch (autoStep) {
            case 0: // Startup delay
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] 1. FORWARD (2 seconds)...");
                    motorsForward();
                    autoStep = 1;
                    lastStepTime = now;
                }
                break;

            case 1: // Forward running
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] STOP (2 seconds)...");
                    motorsStop();
                    autoStep = 2;
                    lastStepTime = now;
                }
                break;

            case 2: // Stopped after forward
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] 2. REVERSE (2 seconds)...");
                    motorsBackward();
                    autoStep = 3;
                    lastStepTime = now;
                }
                break;

            case 3: // Reverse running
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] STOP (2 seconds)...");
                    motorsStop();
                    autoStep = 4;
                    lastStepTime = now;
                }
                break;

            case 4: // Stopped after reverse
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] 3. TURN LEFT (2 seconds)...");
                    motorsTurnLeft();
                    autoStep = 5;
                    lastStepTime = now;
                }
                break;

            case 5: // Turn left running
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] STOP (2 seconds)...");
                    motorsStop();
                    autoStep = 6;
                    lastStepTime = now;
                }
                break;

            case 6: // Stopped after left
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] 4. TURN RIGHT (2 seconds)...");
                    motorsTurnRight();
                    autoStep = 7;
                    lastStepTime = now;
                }
                break;

            case 7: // Turn right running
                if (now - lastStepTime >= 2000) {
                    Serial.println("[AUTO] ALL TESTS COMPLETE. Pausing 5 seconds before repeating...");
                    motorsStop();
                    autoStep = 8;
                    lastStepTime = now;
                }
                break;

            case 8: // Pause before loop
                if (now - lastStepTime >= 5000) {
                    autoStep = 0;
                    lastStepTime = now;
                }
                break;
        }
    }
}

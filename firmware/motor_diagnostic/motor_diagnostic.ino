#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// BUILDScan ROVER - ESP32 MASTER CONTROLLER
// Dynamic Braking Distance + Bluetooth + Motors + Ultrasonic
// =====================================================

// ---------------- BLUETOOTH ----------------
HardwareSerial BT(2);
const int BT_RX_PIN = 16;
const int BT_TX_PIN = 17;
const uint32_t BT_BAUD = 9600;

// ---------------- MOTOR DRIVER ----------------
const int ENA = 14;
const int IN1 = 26;
const int IN2 = 25;
const int ENB = 27;
const int IN3 = 33;
const int IN4 = 32;

// ---------------- PWM SPEED CONFIG ----------------
const int PWM_FREQ = 1000;         // 1 kHz PWM frequency
const int PWM_RES  = 8;            // 8-bit resolution (0 to 255)

int currentSpeed = 160;            // Default speed (0 - 255)

// ---------------- ULTRASONIC & DYNAMIC BRAKING ----------------
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// Dynamic Braking Limits:
// At minimum speed (~80 PWM), stop if obstacle is within 15 cm.
// At maximum speed (255 PWM), stop early if obstacle is within 45 cm.
const float MIN_BRAKE_DIST_CM = 15.0;
const float MAX_BRAKE_DIST_CM = 45.0;

float getDynamicStopDistance() {
  // Linearly scale required stopping distance based on current speed
  float safeDistance = map(currentSpeed, 80, 255, (long)MIN_BRAKE_DIST_CM, (long)MAX_BRAKE_DIST_CM);
  return constrain(safeDistance, MIN_BRAKE_DIST_CM, MAX_BRAKE_DIST_CM);
}

// ---------------- LCD ----------------
const int LCD_SDA = 19;
const int LCD_SCL = 23;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- PCA9685 ----------------
const int PCA_SDA = 21;
const int PCA_SCL = 22;
TwoWire PCAWire(1);
Adafruit_PWMServoDriver pca(0x40, PCAWire);
const int SERVO_CHANNEL = 0;

// ---------------- STATUS ----------------
bool lcdFound = false;
bool pcaFound = false;
bool pcaInitialized = false;
bool ultrasonicHasEcho = false;

float distanceCm = -1.0;
String currentMotion = "STOP";
String usbBuffer;
String btBuffer;

unsigned long lastDistanceTime = 0;
unsigned long lastLCDTime = 0;

// =====================================================
// OUTPUT MESSAGES
// =====================================================
void report(const String &message) {
  Serial.println(message);
  BT.println(message);
}

// =====================================================
// MOTOR SPEED & CONTROL (ESP32 Core 3.x Compatible)
// =====================================================
void applySpeed(int speedVal) {
  currentSpeed = constrain(speedVal, 0, 255);
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(ENA, currentSpeed);
    ledcWrite(ENB, currentSpeed);
  #else
    ledcWrite(0, currentSpeed);
    ledcWrite(1, currentSpeed);
  #endif
}

void stopMotors() {
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
  #else
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  #endif

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  currentMotion = "STOP";
}

void driveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  applySpeed(currentSpeed);
  currentMotion = "FORWARD";
}

void driveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  applySpeed(currentSpeed);
  currentMotion = "BACKWARD";
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  applySpeed(currentSpeed);
  currentMotion = "LEFT";
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  applySpeed(currentSpeed);
  currentMotion = "RIGHT";
}

bool forwardBlocked() {
  float threshold = getDynamicStopDistance();

  if (ultrasonicHasEcho && distanceCm > 0 && distanceCm <= threshold) {
    return true;
  }
  return false;
}

void commandForward() {
  float threshold = getDynamicStopDistance();
  if (forwardBlocked()) {
    stopMotors();
    report("SAFETY STOP: obstacle at " + String(distanceCm, 1) + 
           " cm (Min safe dist for speed " + String(currentSpeed) + " is " + String(threshold, 0) + " cm)");
    return;
  }
  driveForward();
  report("Motor command: FORWARD (Speed: " + String(currentSpeed) + 
         "/255 | Safe Braking Dist: " + String(threshold, 0) + " cm)");
}

void commandBackward() {
  driveBackward();
  report("Motor command: BACKWARD (Speed: " + String(currentSpeed) + "/255)");
}

void commandLeft() {
  turnLeft();
  report("Motor command: LEFT (Speed: " + String(currentSpeed) + "/255)");
}

void commandRight() {
  turnRight();
  report("Motor command: RIGHT (Speed: " + String(currentSpeed) + "/255)");
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout set to 20ms (approx 3.4 meters max range)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);

  if (duration == 0) {
    return -1.0;
  }
  return (duration * 0.0343f) / 2.0f;
}

void updateUltrasonic() {
  float reading = readDistanceCM();
  distanceCm = reading;
  ultrasonicHasEcho = (reading > 0);

  // Fast-acting Emergency Braking if moving FORWARD
  if (currentMotion == "FORWARD" && forwardBlocked()) {
    stopMotors();
    float threshold = getDynamicStopDistance();
    report("EMERGENCY BRAKE! Obstacle at " + String(distanceCm, 1) + 
           " cm (Braking threshold: " + String(threshold, 0) + " cm)");
  }
}

// =====================================================
// I2C BUS SCANNING
// =====================================================
bool deviceResponds(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void scanI2CBuses() {
  report("");
  report("========== I2C SCAN ==========");
  lcdFound = false;
  for (uint8_t address = 1; address < 127; address++) {
    if (deviceResponds(Wire, address)) {
      if (address == 0x27 || address == 0x3F) {
        lcdFound = true;
      }
    }
  }

  report(String("LCD: ") + (lcdFound ? "FOUND" : "NOT DETECTED"));
  pcaFound = deviceResponds(PCAWire, 0x40);
  report(String("PCA9685: ") + (pcaFound ? "FOUND" : "NOT DETECTED"));
  report("==============================");
}

// =====================================================
// LCD UPDATE
// =====================================================
void updateLCD() {
  if (!lcdFound) return;

  float safeDist = getDynamicStopDistance();

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  if (distanceCm > 0) {
    lcd.print("D:");
    lcd.print(distanceCm, 0);
    lcd.print("cm Safe:");
    lcd.print((int)safeDist);
    lcd.print("c");
  } else {
    lcd.print("D:-- Safe:");
    lcd.print((int)safeDist);
    lcd.print("c");
  }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(currentMotion);
  lcd.print(" S:");
  lcd.print(currentSpeed);
}

// =====================================================
// STATUS REPORT
// =====================================================
void showStatus() {
  float safeDist = getDynamicStopDistance();
  report("");
  report("========== HARDWARE STATUS ==========");
  report("Current Speed: " + String(currentSpeed) + " / 255");
  report("Safe Braking Distance: " + String(safeDist, 1) + " cm");
  report("Motor state: " + currentMotion);
  report("HC-SR04: " + (ultrasonicHasEcho ? ("Distance: " + String(distanceCm, 1) + " cm") : "NO ECHO"));
  report("=====================================");
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
void processCommand(String cmd, bool fromBluetooth) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  Serial.println(String(fromBluetooth ? "[BT RX] " : "[USB RX] ") + cmd);

  // Speed commands
  if (cmd.startsWith("SPEED ") || cmd.startsWith("SPD ")) {
    int spaceIdx = cmd.indexOf(' ');
    int newSpeed = cmd.substring(spaceIdx + 1).toInt();
    applySpeed(newSpeed);
    report("Speed: " + String(currentSpeed) + " | Safe brake limit: " + String(getDynamicStopDistance(), 0) + " cm");
  }
  else if (cmd == "SLOW" || cmd == "1") {
    applySpeed(110);
    report("Speed SLOW (110) | Safe brake limit: " + String(getDynamicStopDistance(), 0) + " cm");
  }
  else if (cmd == "MED" || cmd == "2") {
    applySpeed(170);
    report("Speed MEDIUM (170) | Safe brake limit: " + String(getDynamicStopDistance(), 0) + " cm");
  }
  else if (cmd == "FAST" || cmd == "3") {
    applySpeed(255);
    report("Speed MAXIMUM (255) | Safe brake limit: " + String(getDynamicStopDistance(), 0) + " cm");
  }
  // Direction commands
  else if (cmd == "F" || cmd == "FORWARD" || cmd == "W") {
    commandForward();
  }
  else if (cmd == "B" || cmd == "BACK" || cmd == "BACKWARD" || cmd == "S_REV") {
    commandBackward();
  }
  else if (cmd == "L" || cmd == "LEFT" || cmd == "A") {
    commandLeft();
  }
  else if (cmd == "R" || cmd == "RIGHT" || cmd == "D") {
    commandRight();
  }
  else if (cmd == "S" || cmd == "STOP" || cmd == "X") {
    stopMotors();
    report("STOP: motors disabled.");
  }
  // Diagnostic commands
  else if (cmd == "PING") {
    report("PONG");
  }
  else if (cmd == "STATUS") {
    showStatus();
  }
  else if (cmd == "DIST") {
    updateUltrasonic();
    report("Distance: " + String(distanceCm, 1) + " cm | Safe threshold: " + String(getDynamicStopDistance(), 0) + " cm");
  }
  else if (cmd == "SCAN") {
    scanI2CBuses();
  }
  else if (cmd == "HELP") {
    report("--- Commands ---");
    report("Motion: F, B, L, R, S");
    report("Speed:  SPEED 80-255, SLOW, MED, FAST");
    report("Status: STATUS, DIST, PING, SCAN");
  }
  else {
    report("Unknown command: " + cmd);
  }
}

// =====================================================
// SERIAL READER
// =====================================================
void readCommandStream(Stream &port, String &buffer, bool isBluetooth) {
  while (port.available() > 0) {
    char c = (char)port.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        processCommand(buffer, isBluetooth);
        buffer = "";
      }
    } else if (isPrintable(c)) {
      if (buffer.length() < 80) {
        buffer += c;
      } else {
        buffer = "";
      }
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  BT.begin(BT_BAUD, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, 0);
    ledcAttachPin(ENB, 1);
  #endif

  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(100000);

  PCAWire.begin(PCA_SDA, PCA_SCL);
  PCAWire.setClock(100000);

  scanI2CBuses();

  if (lcdFound) {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BUILDScan Rover");
    lcd.setCursor(0, 1);
    lcd.print("Adaptive Braking");
  }

  if (pcaFound) {
    pcaInitialized = pca.begin();
    if (pcaInitialized) {
      pca.setPWMFreq(50);
    }
  }

  updateUltrasonic();
  updateLCD();
  report("Adaptive Safety Braking Ready!");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  readCommandStream(Serial, usbBuffer, false);
  readCommandStream(BT, btBuffer, true);

  // Fast ultrasonic check: 50 ms (20Hz) when driving FORWARD, 500 ms when stopped
  unsigned long distanceInterval = (currentMotion == "FORWARD") ? 50 : 500;
  if (millis() - lastDistanceTime >= distanceInterval) {
    lastDistanceTime = millis();
    updateUltrasonic();
  }

  // Update LCD twice per second
  if (millis() - lastLCDTime >= 500) {
    lastLCDTime = millis();
    updateLCD();
  }
}

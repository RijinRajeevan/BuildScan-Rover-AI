#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// BUILDScan ROVER - ESP32 MASTER CONTROLLER
// Bluetooth + 4 DC motors + HC-SR04 + LCD + PCA9685
// =====================================================

// ---------------- BLUETOOTH ----------------
// Module TXD -> ESP32 GPIO16
// Module RXD <- ESP32 GPIO17
// Cross TX and RX. Common GND required.
HardwareSerial BT(2);
const int BT_RX_PIN = 16;
const int BT_TX_PIN = 17;
const uint32_t BT_BAUD = 9600;

// ---------------- MOTOR DRIVER ----------------
// Reported wiring: HW-095 / L298N-type driver
const int ENA = 14;
const int IN1 = 26;
const int IN2 = 25;
const int ENB = 27;
const int IN3 = 33;
const int IN4 = 32;

// ---------------- ULTRASONIC ----------------
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// Stop forward movement when an obstacle is closer
// than this distance. Set to 0 to disable this check.
const float OBSTACLE_LIMIT_CM = 20.0;

// ---------------- LCD ----------------
// LCD uses GPIO19/23 through your logic-level converter.
// This constructor assumes LCD address 0x27.
const int LCD_SDA = 19;
const int LCD_SCL = 23;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- PCA9685 ----------------
// PCA9685 uses its own I2C bus on GPIO21/22.
const int PCA_SDA = 21;
const int PCA_SCL = 22;
TwoWire PCAWire(1);
Adafruit_PWMServoDriver pca(0x40, PCAWire);

// PCA channel used by the MG996R signal wire.
// This sketch does NOT move the servo automatically.
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
unsigned long lastStatusTime = 0;

// =====================================================
// OUTPUT MESSAGES
// Send reports to USB Serial Monitor and Bluetooth.
// =====================================================
void report(const String &message) {
  Serial.println(message);
  BT.println(message);
}

void reportUSB(const String &message) {
  Serial.println(message);
}

// =====================================================
// MOTOR CONTROL
// The exact direction depends on motor wiring.
// Test with wheels raised off the floor.
// =====================================================
void stopMotors() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

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

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  currentMotion = "FORWARD";
}

void driveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  currentMotion = "BACKWARD";
}

void turnLeft() {
  // Left side backward, right side forward.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  currentMotion = "LEFT";
}

void turnRight() {
  // Left side forward, right side backward.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  currentMotion = "RIGHT";
}

bool forwardBlocked() {
  if (OBSTACLE_LIMIT_CM <= 0) return false;

  // Only block when a valid, recent reading indicates an obstacle.
  if (ultrasonicHasEcho &&
      distanceCm > 0 &&
      distanceCm <= OBSTACLE_LIMIT_CM) {
    return true;
  }

  return false;
}

void commandForward() {
  if (forwardBlocked()) {
    stopMotors();
    report("SAFETY STOP: obstacle at " +
           String(distanceCm, 1) + " cm. Forward blocked.");
    return;
  }

  driveForward();
  report("Motor command: FORWARD");
}

void commandBackward() {
  driveBackward();
  report("Motor command: BACKWARD");
}

void commandLeft() {
  turnLeft();
  report("Motor command: LEFT");
}

void commandRight() {
  turnRight();
  report("Motor command: RIGHT");
}

// =====================================================
// ULTRASONIC SENSOR
// ECHO must use your voltage divider before GPIO18.
// =====================================================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0;
  }

  return (duration * 0.0343f) / 2.0f;
}

void updateUltrasonic() {
  float reading = readDistanceCM();
  distanceCm = reading;
  ultrasonicHasEcho = (reading > 0);

  if (reading > 0) {
    Serial.print("[HC-SR04] PASS | Distance: ");
    Serial.print(reading, 1);
    Serial.println(" cm");
  } else {
    Serial.println(
      "[HC-SR04] CHECK | No echo or timeout");
  }

  // Safety stop if forward motion is active and
  // a valid reading detects an obstacle.
  if (currentMotion == "FORWARD" &&
      forwardBlocked()) {
    stopMotors();
    report("SAFETY STOP: obstacle detected!");
  }
}

// =====================================================
// I2C BUS SCANNING
// LCD: Wire on GPIO19/23
// PCA9685: PCAWire on GPIO21/22
// =====================================================
bool deviceResponds(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void scanI2CBuses() {
  report("");
  report("========== I2C SCAN ==========");

  lcdFound = false;

  report("Scanning LCD bus GPIO19/23...");
  for (uint8_t address = 1; address < 127; address++) {
    if (deviceResponds(Wire, address)) {
      report("LCD bus device ACK at 0x" +
             String(address, HEX));

      if (address == 0x27 || address == 0x3F) {
        lcdFound = true;
      }
    }
  }

  if (lcdFound) {
    report("LCD BUS: PASS - common LCD address found.");
    report("LCD constructor currently uses 0x27.");
    report("If detected address is 0x3F, change it in code.");
  } else {
    report("LCD BUS: CHECK - no 0x27/0x3F device.");
    report("Check 5V, GND, level converter, SDA/SCL.");
  }

  report("Scanning PCA bus GPIO21/22...");
  pcaFound = deviceResponds(PCAWire, 0x40);

  if (pcaFound) {
    report("PCA9685: PASS - ACK at 0x40.");
  } else {
    report("PCA9685: FAIL - no ACK at 0x40.");
    report("Check VCC, GND, SDA21, SCL22.");
  }

  report("==============================");
}

// =====================================================
// LCD UPDATE
// =====================================================
void updateLCD() {
  if (!lcdFound) return;

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  if (distanceCm > 0) {
    lcd.print("Dist:");
    lcd.print(distanceCm, 1);
    lcd.print("cm");
  } else {
    lcd.print("Dist: NO ECHO");
  }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(currentMotion);
}

// =====================================================
// STATUS REPORT
// =====================================================
void showStatus() {
  report("");
  report("========== HARDWARE STATUS ==========");

  report("ESP32: RUNNING");
  report("Bluetooth UART: GPIO16/17 at " +
         String(BT_BAUD) + " baud");
  report("Motor command state: " + currentMotion);

  report(String("HC-SR04: ") +
         (ultrasonicHasEcho ? "PASS" : "CHECK"));

  if (distanceCm > 0) {
    report("Distance: " + String(distanceCm, 1) + " cm");
  }

  report(String("LCD I2C response: ") +
         (lcdFound ? "PASS" : "CHECK"));

  report(String("PCA9685 I2C response: ") +
         (pcaFound ? "PASS" : "FAIL"));

  report(String("PCA9685 initialization: ") +
         (pcaInitialized ? "PASS" : "CHECK"));

  report("Motor wiring/electrical condition: NOT proven by software.");
  report("Servo motion and servo supply: NOT tested.");
  report("Raspberry Pi / LiDAR / fan: NOT tested by this sketch.");

  report("=====================================");
}

// =====================================================
// COMMAND PROCESSOR
// Accepts commands from USB or Bluetooth.
// =====================================================
void processCommand(String cmd, bool fromBluetooth) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  Serial.println(
    String(fromBluetooth ? "[BT RX] " : "[USB RX] ") + cmd);

  if (cmd == "PING") {
    report("PONG - Bluetooth/serial command received.");
  }
  else if (cmd == "HELP") {
    report("COMMANDS: F/FORWARD, B/BACKWARD, L/LEFT, R/RIGHT, S/STOP");
    report("More: PING, STATUS, DIST, SCAN, SERVO, HELP");
  }
  else if (cmd == "F" || cmd == "FORWARD" ||
           cmd == "FRONT" || cmd == "W") {
    commandForward();
  }
  else if (cmd == "B" || cmd == "BACK" ||
           cmd == "BACKWARD" || cmd == "REVERSE") {
    commandBackward();
  }
  else if (cmd == "L" || cmd == "LEFT" ||
           cmd == "A") {
    commandLeft();
  }
  else if (cmd == "R" || cmd == "RIGHT" ||
           cmd == "D") {
    commandRight();
  }
  else if (cmd == "S" || cmd == "STOP" ||
           cmd == "X") {
    stopMotors();
    report("STOP: motors disabled.");
  }
  else if (cmd == "PING") {
    report("PONG");
  }
  else if (cmd == "STATUS") {
    showStatus();
  }
  else if (cmd == "DIST") {
    updateUltrasonic();
  }
  else if (cmd == "SCAN") {
    scanI2CBuses();
  }
  else if (cmd == "SERVO") {
    if (pcaInitialized) {
      report("PCA9685 initialized. Servo signal channel is 0.");
      report("Servo has NOT been moved by this test.");
      report("Connect verified regulated servo power before testing.");
    } else {
      report("Servo test unavailable: PCA9685 not initialized.");
    }
  }
  else {
    report("Unknown command: " + cmd);
    report("Send HELP to list commands.");
  }
}

// =====================================================
// SERIAL INPUT READER
// Send commands ending with newline.
// =====================================================
void readCommandStream(Stream &port,
                       String &buffer,
                       bool isBluetooth) {
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
        report("Command too long; input cleared.");
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

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // HC-SR04 pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // LCD I2C bus
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(100000);

  // PCA9685 I2C bus
  PCAWire.begin(PCA_SDA, PCA_SCL);
  PCAWire.setClock(100000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" BUILDScan Rover - ESP32 Diagnostics");
  Serial.println(" USB Serial Monitor: 115200 baud");
  Serial.println(" Bluetooth UART: 9600 baud");
  Serial.println(" Motors are OFF at startup.");
  Serial.println("==========================================");

  report("ESP32 booted. Starting hardware checks...");

  scanI2CBuses();

  if (lcdFound) {
    // The LCD library uses Wire (GPIO19/23).
    // Ensure the constructor address matches your scan.
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BUILDScan Rover");
    lcd.setCursor(0, 1);
    lcd.print("Diagnostics");
    report("LCD initialization attempted.");
  }

  if (pcaFound) {
    pcaInitialized = pca.begin();

    if (pcaInitialized) {
      pca.setPWMFreq(50);
      report("PCA9685 initialization: PASS.");
      report("No servo movement commanded at startup.");
    } else {
      report("PCA9685 initialization: CHECK.");
    }
  }

  updateUltrasonic();
  updateLCD();

  report("Startup checks finished.");
  report("Send HELP from Android Bluetooth terminal.");
  report("Bluetooth test: send PING and look for PONG.");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  readCommandStream(Serial, usbBuffer, false);
  readCommandStream(BT, btBuffer, true);

  // Print sensor readings periodically.
  if (millis() - lastDistanceTime >= 1000) {
    lastDistanceTime = millis();
    updateUltrasonic();
  }

  // Refresh LCD twice per second.
  if (millis() - lastLCDTime >= 500) {
    lastLCDTime = millis();
    updateLCD();
  }
}
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// BUILDScan Rover - ESP32 Motor Controller
// HC-05 Bluetooth + ROS 2 Bridge Protocol + Safety Watchdog
// ============================================================

// ---------------- BLUETOOTH ----------------
HardwareSerial BT(2);

const int BT_RX_PIN = 16;   // ESP32 RX2 <- HC-05 TXD
const int BT_TX_PIN = 17;   // ESP32 TX2 -> HC-05 RXD
const uint32_t BT_BAUD = 9600;

// ---------------- MOTOR DRIVER: L298N ----------------
const int ENA = 14;
const int IN1 = 26;
const int IN2 = 25;

const int ENB = 27;
const int IN3 = 33;
const int IN4 = 32;

// ---------------- PWM ----------------
const int PWM_FREQ = 1000;
const int PWM_RES = 8;

int currentSpeed = 255;

// ---------------- ULTRASONIC ----------------
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

const float MIN_BRAKE_DIST_CM = 15.0;
const float MAX_BRAKE_DIST_CM = 45.0;

float distanceCm = -1.0;
bool ultrasonicHasEcho = false;

unsigned long lastDistanceTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 100;

// ---------------- LCD ----------------
const int LCD_SDA = 19;
const int LCD_SCL = 23;

LiquidCrystal_I2C lcd(0x27, 16, 2);

bool lcdFound = false;
unsigned long lastLCDTime = 0;

// ---------------- PCA9685 ----------------
const int PCA_SDA = 21;
const int PCA_SCL = 22;

TwoWire PCAWire(1);
Adafruit_PWMServoDriver pca(0x40, PCAWire);

bool pcaFound = false;
bool pcaInitialized = false;

// ---------------- MOTION STATE ----------------
enum MotionState {
  MOTION_STOP,
  MOTION_FORWARD,
  MOTION_BACKWARD,
  MOTION_LEFT,
  MOTION_RIGHT
};

MotionState currentMotion = MOTION_STOP;

// ---------------- CONTROL MODE ----------------
enum ControlMode {
  MODE_MANUAL,
  MODE_AUTO
};

ControlMode controlMode = MODE_MANUAL;

// ---------------- COMMUNICATION WATCHDOG ----------------
// A new valid AUTO movement command refreshes this timestamp.
// If commands stop arriving, the motors are stopped.
unsigned long lastAutoCommandMs = 0;
const unsigned long AUTO_WATCHDOG_MS = 500;

// A command is only accepted in AUTO mode if it comes from
// the HC-05. USB serial cannot directly drive the motors.
String btBuffer;
String usbBuffer;

// ---------------- OUTPUT ----------------
void reportBT(const String &message) {
  BT.println(message);
}

void reportUSB(const String &message) {
  Serial.println(message);
}

void reportBoth(const String &message) {
  reportUSB(message);
  reportBT(message);
}

const char *motionName() {
  switch (currentMotion) {
    case MOTION_FORWARD:  return "FORWARD";
    case MOTION_BACKWARD: return "BACKWARD";
    case MOTION_LEFT:     return "LEFT";
    case MOTION_RIGHT:    return "RIGHT";
    default:              return "STOP";
  }
}

const char *modeName() {
  return controlMode == MODE_AUTO ? "AUTO" : "MANUAL";
}

float getDynamicStopDistance() {
  float value = 15.0f +
      ((constrain(currentSpeed, 80, 255) - 80) / 175.0f) * 30.0f;

  return constrain(value, MIN_BRAKE_DIST_CM, MAX_BRAKE_DIST_CM);
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void applySpeed(int value) {
  currentSpeed = constrain(value, 80, 255);

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

  currentMotion = MOTION_STOP;
}

bool obstacleDetected() {
  return ultrasonicHasEcho &&
         distanceCm > 0 &&
         distanceCm <= getDynamicStopDistance();
}

void driveForward() {
  if (obstacleDetected()) {
    stopMotors();
    reportBoth("ERR,OBSTACLE");
    return;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  applySpeed(currentSpeed);
  currentMotion = MOTION_FORWARD;
}

void driveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  applySpeed(currentSpeed);
  currentMotion = MOTION_BACKWARD;
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  applySpeed(currentSpeed);
  currentMotion = MOTION_LEFT;
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  applySpeed(currentSpeed);
  currentMotion = MOTION_RIGHT;
}

// ============================================================
// ULTRASONIC SENSOR
// ============================================================

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

void updateUltrasonic() {
  float reading = readDistanceCM();

  distanceCm = reading;
  ultrasonicHasEcho = reading > 0;

  // Conservative behavior:
  // If an ultrasonic reading is missing while driving forward,
  // stop rather than assume the path is clear.
  if (currentMotion == MOTION_FORWARD) {
    if (!ultrasonicHasEcho || obstacleDetected()) {
      stopMotors();

      if (!ultrasonicHasEcho) {
        reportBoth("ERR,ULTRASONIC_NO_ECHO");
      } else {
        reportBoth("ERR,OBSTACLE");
      }
    }
  }
}

// ============================================================
// STATUS AND DIAGNOSTICS
// ============================================================

void showStatus() {
  reportBT("STATUS_BEGIN");
  reportBT(String("MODE=") + modeName());
  reportBT(String("MOTION=") + motionName());
  reportBT(String("SPEED=") + String(currentSpeed));
  reportBT(String("SAFE_DISTANCE_CM=") +
           String(getDynamicStopDistance(), 1));

  if (ultrasonicHasEcho) {
    reportBT(String("DISTANCE_CM=") + String(distanceCm, 1));
  } else {
    reportBT("DISTANCE_CM=UNKNOWN");
  }

  reportBT(String("LCD=") + (lcdFound ? "OK" : "NOT_FOUND"));
  reportBT(String("PCA9685=") +
           (pcaInitialized ? "OK" : "NOT_FOUND"));

  reportBT("STATUS_END");
}

void scanI2CBuses() {
  bool lcdDetected = false;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0 &&
        (address == 0x27 || address == 0x3F)) {
      lcdDetected = true;
    }
  }

  lcdFound = lcdDetected;

  PCAWire.beginTransmission(0x40);
  pcaFound = (PCAWire.endTransmission() == 0);

  reportBT(String("LCD=") + (lcdFound ? "OK" : "NOT_FOUND"));
  reportBT(String("PCA9685=") + (pcaFound ? "OK" : "NOT_FOUND"));
}

// ============================================================
// COMMAND HANDLING
// ============================================================

// USB commands are diagnostics only.
// Bluetooth commands can control motors.
void processCommand(String cmd, bool fromBluetooth) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.isEmpty()) {
    return;
  }

  if (cmd == "PING") {
    reportBT("PONG");
    return;
  }

  if (cmd == "HELP") {
    reportBT("CMD,PING");
    reportBT("CMD,STATUS");
    reportBT("CMD,DIST");
    reportBT("CMD,MODE");
    reportBT("CMD,AUTO");
    reportBT("CMD,MANUAL");
    reportBT("CMD,F,B,L,R,STOP");
    reportBT("CMD,SPEED,80-255");
    reportBT("CMD,TWIST,<linear>,<angular>");
    return;
  }

  if (cmd == "STATUS") {
    showStatus();
    return;
  }

  if (cmd == "MODE") {
    reportBT(String("MODE=") + modeName());
    return;
  }

  if (cmd == "DIST") {
    updateUltrasonic();

    if (ultrasonicHasEcho) {
      reportBT(String("DISTANCE_CM=") + String(distanceCm, 1));
    } else {
      reportBT("DISTANCE_CM=UNKNOWN");
    }
    return;
  }

  if (cmd == "SCAN") {
    scanI2CBuses();
    return;
  }

  // Everything that changes motion requires Bluetooth.
  if (!fromBluetooth) {
    reportUSB("ERROR: Motion commands require Bluetooth");
    return;
  }

  if (cmd == "AUTO") {
    stopMotors();
    controlMode = MODE_AUTO;
    lastAutoCommandMs = millis();
    reportBT("OK,MODE_AUTO");
    return;
  }

  if (cmd == "MANUAL") {
    stopMotors();
    controlMode = MODE_MANUAL;
    reportBT("OK,MODE_MANUAL");
    return;
  }

  if (cmd == "STOP" || cmd == "S" || cmd == "X") {
    stopMotors();
    reportBT("OK,STOP");
    return;
  }

  // Autonomous movement requires AUTO mode.
  // Manual mode retains direct Bluetooth movement commands.
  if (cmd.startsWith("SPEED ") || cmd.startsWith("SPD ")) {
    int separator = cmd.indexOf(' ');
    int value = cmd.substring(separator + 1).toInt();

    if (value < 80 || value > 255) {
      reportBT("ERR,SPEED_RANGE_80_255");
      return;
    }

    applySpeed(value);
    reportBT(String("OK,SPEED=") + String(currentSpeed));
    return;
  }

  if (cmd == "SLOW") {
    applySpeed(110);
    reportBT("OK,SPEED=110");
    return;
  }

  if (cmd == "MED") {
    applySpeed(170);
    reportBT("OK,SPEED=170");
    return;
  }

  if (cmd == "FAST") {
    applySpeed(255);
    reportBT("OK,SPEED=255");
    return;
  }

  if (controlMode == MODE_AUTO) {
    // The ROS bridge sends fresh movement commands periodically.
    // Only recognized movement commands refresh the watchdog.
    if (cmd != "F" && cmd != "FORWARD" &&
        cmd != "B" && cmd != "BACKWARD" &&
        cmd != "L" && cmd != "LEFT" &&
        cmd != "R" && cmd != "RIGHT") {
      reportBT("ERR,UNKNOWN_OR_DISABLED_COMMAND");
      return;
    }

    lastAutoCommandMs = millis();
  }

  if (cmd == "F" || cmd == "FORWARD" || cmd == "W") {
    driveForward();
  } else if (cmd == "B" || cmd == "BACK" || cmd == "BACKWARD") {
    driveBackward();
  } else if (cmd == "L" || cmd == "LEFT" || cmd == "A") {
    turnLeft();
  } else if (cmd == "R" || cmd == "RIGHT" || cmd == "D") {
    turnRight();
  } else {
    reportBT("ERR,UNKNOWN_COMMAND");
    return;
  }

  reportBT(String("OK,MOTION=") + motionName());
}

// ============================================================
// SERIAL READERS
// ============================================================

void readCommandStream(Stream &port, String &buffer,
                       bool isBluetooth) {
  while (port.available() > 0) {
    char c = static_cast<char>(port.read());

    if (c == '\n' || c == '\r') {
      if (!buffer.isEmpty()) {
        processCommand(buffer, isBluetooth);
        buffer = "";
      }
    } else if (isPrintable(c)) {
      if (buffer.length() < 100) {
        buffer += c;
      } else {
        buffer = "";
      }
    }
  }
}

// ============================================================
// LCD
// ============================================================

void updateLCD() {
  if (!lcdFound) {
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  if (ultrasonicHasEcho) {
    lcd.print("D:");
    lcd.print(distanceCm, 0);
    lcd.print("cm Safe:");
    lcd.print((int)getDynamicStopDistance());
  } else {
    lcd.print("Distance UNKNOWN");
  }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);

  lcd.print(motionName());
  lcd.print(" ");
  lcd.print(currentSpeed);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

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
    lcd.print("BT Controller");
  }

  if (pcaFound) {
    pcaInitialized = pca.begin();

    if (pcaInitialized) {
      pca.setPWMFreq(50);
    }
  }

  updateUltrasonic();
  updateLCD();

  lastAutoCommandMs = millis();

  reportBT("BUILDScan Rover Ready");
  reportBT("MODE=MANUAL");
  reportBT("SAFETY=ENABLED");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  readCommandStream(BT, btBuffer, true);
  readCommandStream(Serial, usbBuffer, false);

  unsigned long now = millis();

  // Sensor update.
  if (now - lastDistanceTime >= SENSOR_INTERVAL_MS) {
    lastDistanceTime = now;
    updateUltrasonic();
  }

  // Independent autonomous communication watchdog.
  if (controlMode == MODE_AUTO &&
      currentMotion != MOTION_STOP &&
      now - lastAutoCommandMs > AUTO_WATCHDOG_MS) {

    stopMotors();
    reportBT("ERR,AUTO_WATCHDOG_STOP");
  }

  // LCD update.
  if (now - lastLCDTime >= 500) {
    lastLCDTime = now;
    updateLCD();
  }
}
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== BLUETOOTH =====
SoftwareSerial BT(2, 3);

// ===== SERVO =====
Servo myServo;
Servo panServo;    // camera pan
Servo tiltServo;   // camera tilt

int panAngle = 90;
int currentAngle = 90;

// ===== MOTOR =====
int ENA = 5;
int IN1 = 8;
int IN2 = 9;
int ENB = 6;
int IN3 = 10;
int IN4 = 11;

int speedVal = 130;

// ===== ULTRASONIC =====
int trigPin = 4;
int echoPin = 12;

// ===== FAN =====
int fanPin = 13;
bool fanState = false;

// ===== CONTROL =====
char command;
bool autoMode = false;
bool parkMode = false;

// ===== FAN ICON =====
byte fan1[8] = {B00100,B01110,B00100,B00000,B00100,B01110,B00100,B00000};
byte fan2[8] = {B00000,B00100,B01110,B00100,B01110,B00100,B00000,B00000};

// ===== SETUP =====
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(fanPin, OUTPUT);

  myServo.attach(7);
  panServo.attach(A0);
  tiltServo.attach(A1);

  panServo.write(panAngle);
  tiltServo.write(125);
  myServo.write(90);

  BT.begin(9600);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  lcd.createChar(0, fan1);
  lcd.createChar(1, fan2);

  startupAnimation();
}

// ===== LOOP =====
void loop() {

  if (BT.available()) {
    command = BT.read();

    // ===== PARK MODE =====
    if (command == 'W') {
      parkMode = true;
      autoMode = false;
      smoothStop();
      digitalWrite(fanPin, LOW);
      showParkMode();
    }

    // ===== AUTO MODE =====
    if (command == '1') {
      parkMode = false;
      autoMode = true;
      showAutoMode();
    }

    // ===== MANUAL MODE =====
    if (command == '2') {
      parkMode = false;
      autoMode = false;
      showManualMode();
      smoothStop();
    }

    if (parkMode) return;

    // ===== FAN CONTROL =====
    if (!autoMode && command == 'Y') {
      fanState = !fanState;
      digitalWrite(fanPin, fanState);

      if (fanState) showCooling();
      else showFanOff();
    }

    // ===== AUTO FAN =====
    if (autoMode) digitalWrite(fanPin, HIGH);

    // ===== SPEED CONTROL =====
    if (command == '3') {
      speedVal = 100;
      updateDisplay("READY");
    }

    if (command == '4') {
      speedVal = 125;
      updateDisplay("READY");
    }

    // ===== CAMERA PAN =====
    if (command == 'U') {
      panAngle += 15;
      if (panAngle > 180) panAngle = 180;
      panServo.write(panAngle);
    }

    if (command == 'V') {
      panAngle -= 15;
      if (panAngle < 0) panAngle = 0;
      panServo.write(panAngle);
    }

    // ===== MANUAL CONTROL =====
    if (!autoMode) {
      switch (command) {
        case 'F': smoothForward(); break;
        case 'B': smoothBackward(); break;
        case 'L': smoothLeftTurn(); break;
        case 'R': smoothRightTurn(); break;
        case 'Z': smoothStop(); break;
      }
    }
  }

  // ===== FAN ANIMATION =====
  if (fanState || autoMode) animateFan();

  // ===== AUTO MODE =====
  if (autoMode) {
    int d = getDistance();

    if (d > 50) {
      smoothForward();
    } else {
      smoothStop();

      moveServoSmooth(180);
      int left = getDistance();

      moveServoSmooth(0);
      int right = getDistance();

      moveServoSmooth(90);

      if (left > right) smoothLeftTurn();
      else smoothRightTurn();

      delay(600);
    }
  }
}

// ===== DISPLAY =====
void updateDisplay(String dir) {
  lcd.setCursor(0, 0);
  lcd.print("Dir:");
  lcd.print(dir);
  lcd.print("     ");

  lcd.setCursor(0, 1);
  lcd.print("Speed:");
  lcd.print(speedVal);
  lcd.print("   ");
}

// ===== LCD FUNCTIONS =====
void startupAnimation() {
  lcd.clear();
  lcd.print("BuildScan Rover");
  lcd.setCursor(0,1);
  lcd.print("Initializing...");
  delay(1500);
  lcd.clear();
}

void showAutoMode() {
  lcd.clear();
  lcd.print("AUTO MODE");
  lcd.setCursor(0,1);
  lcd.print("Scanning...");
  delay(800);
}

void showManualMode() {
  lcd.clear();
  lcd.print("MANUAL MODE");
  lcd.setCursor(0,1);
  lcd.print("Ready...");
  delay(800);
}

void showCooling() {
  lcd.clear();
  lcd.print("Cooling Rover");
  lcd.setCursor(0,1);
  lcd.print("Fan Running");
}

void showFanOff() {
  lcd.clear();
  lcd.print("Fan Stopped");
  lcd.setCursor(0,1);
  lcd.print("Cooling Off");
}

void showParkMode() {
  lcd.clear();
  lcd.print("-- PARK MODE --");
  lcd.setCursor(0,1);
  lcd.print("Standby...");
}

// ===== FAN ANIMATION =====
void animateFan() {
  lcd.setCursor(15,0);
  lcd.write(byte(0));
  delay(120);
  lcd.setCursor(15,0);
  lcd.write(byte(1));
}

// ===== SERVO =====
void moveServoSmooth(int target) {
  if (currentAngle < target)
    for (int i=currentAngle;i<=target;i++){ myServo.write(i); delay(4); }
  else
    for (int i=currentAngle;i>=target;i--){ myServo.write(i); delay(4); }

  currentAngle = target;
}

// ===== MOTOR =====
void smoothForward() {
  updateDisplay("FORWARD ^");
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA,speedVal); analogWrite(ENB,speedVal);
}

void smoothBackward() {
  updateDisplay("BACKWARD v");
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  analogWrite(ENA,speedVal); analogWrite(ENB,speedVal);
}

void smoothLeftTurn() {
  updateDisplay("LEFT <");
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA,0); analogWrite(ENB,speedVal);
}

void smoothRightTurn() {
  updateDisplay("RIGHT >");
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  analogWrite(ENA,speedVal); analogWrite(ENB,0);
}

void smoothStop() {
  updateDisplay("STOP -");
  analogWrite(ENA,0); analogWrite(ENB,0);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}

// ===== ULTRASONIC =====
int getDistance() {
  int readings[3];

  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000);

    if (duration == 0) readings[i] = 400;
    else {
      int d = duration * 0.034 / 2;
      if (d < 5) d = 5;
      if (d > 400) d = 400;
      readings[i] = d;
    }

    delay(20);
  }

  if (readings[0] > readings[1]) {
    int t = readings[0]; readings[0] = readings[1]; readings[1] = t;
  }
  if (readings[1] > readings[2]) {
    int t = readings[1]; readings[1] = readings[2]; readings[2] = t;
  }
  if (readings[0] > readings[1]) {
    int t = readings[0]; readings[0] = readings[1]; readings[1] = t;
  }

  return readings[1];
}
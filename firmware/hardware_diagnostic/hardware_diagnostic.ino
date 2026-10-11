/*
 * ============================================================
 *  BUILDScan Rover — Hardware Diagnostic Firmware v2
 *  PCA9685 Servo Control Edition
 * ============================================================
 *
 * CHANGE FROM v1:
 *   Servo is controlled via PCA9685 (I2C, 0x40) on shared I2C bus
 *   with LCD (0x27). ESP32Servo on GPIO13 is NOT used.
 *
 * PHYSICAL CONNECTIONS CONFIRMED:
 *   ESP32 GPIO21 (SDA) ─── I2C bus ─── LCD (0x27) + PCA9685 (0x40)
 *   ESP32 GPIO22 (SCL) ─── I2C bus ─── LCD (0x27) + PCA9685 (0x40)
 *   PCA9685 CH0 ─────────────────────── SG90 Servo Signal
 *   PCA9685 V+  ─────────────────────── 5V (servo power rail)
 *   PCA9685 GND ─────────────────────── Common GND
 *
 * IF SERVO IS ON A DIFFERENT CHANNEL, CHANGE THIS LINE:
 *   #define SERVO_CHANNEL  0
 *
 * SG90 PULSE CALIBRATION:
 *   SERVO_MIN_PULSE  102  (≈500µs  = 0°)
 *   SERVO_MAX_PULSE  491  (≈2400µs = 180°)
 *   Adjust if servo does not reach full range.
 *
 * HC-SR04: TRIG=GPIO5, ECHO=GPIO18 (via level shifter)
 * LCD:     SDA=GPIO21, SCL=GPIO22, I2C address auto-detected
 * MOTORS:  All STOPPED — GPIO14,26,25,27,33,32 held LOW
 *
 * Libraries required:
 *   Adafruit PWM Servo Driver Library v3.0.3
 *   Adafruit BusIO v1.17.4
 *   LiquidCrystal_I2C (Frank de Brabander)
 *   Wire (built-in)
 *
 * NOTE: ESP32Servo is NOT used in this version.
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PWMServoDriver.h>

// ── PCA9685 Configuration ─────────────────────────────────────────────────────
#define PCA9685_ADDR    0x40          // Default I2C address of PCA9685
#define SERVO_CHANNEL   0             // ← CHANGE THIS if servo is on CH1, CH2, etc.

// SG90 pulse width calibration (12-bit, 4096 ticks at 50Hz = 20ms period)
// 1 tick = 20ms / 4096 = 4.88µs
// 500µs  / 4.88 ≈ 102   (0°)
// 2400µs / 4.88 ≈ 491   (180°)
#define SERVO_MIN_PULSE  102          // pulse count for 0° (≈500µs)
#define SERVO_MAX_PULSE  491          // pulse count for 180° (≈2400µs)
#define SERVO_FREQ_HZ    50           // standard servo frequency

// Safe angle limits
#define SERVO_MIN_DEG    10
#define SERVO_MAX_DEG    170

// ── Motor pins (ALL STOPPED) ──────────────────────────────────────────────────
#define LEFT_PWM   14
#define LEFT_IN1   26
#define LEFT_IN2   25
#define RIGHT_PWM  27
#define RIGHT_IN1  33
#define RIGHT_IN2  32

// ── HC-SR04 ───────────────────────────────────────────────────────────────────
#define TRIG_PIN   5
#define ECHO_PIN   18   // via level shifter → 3.3V safe

// ── I2C ───────────────────────────────────────────────────────────────────────
#define LCD_SDA    21
#define LCD_SCL    22

// ── Objects ───────────────────────────────────────────────────────────────────
LiquidCrystal_I2C* lcd = nullptr;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

bool lcd_found    = false;
bool pca_found    = false;
uint8_t lcd_addr  = 0x00;

// ── Timing ────────────────────────────────────────────────────────────────────
#define ULTRASONIC_INTERVAL_MS  500
#define SERVO_STEP_DELAY_MS     900

const int SERVO_POSITIONS[] = {20, 90, 160, 90};
const int SERVO_POS_COUNT   = 4;
int servoIndex = 0;

unsigned long lastUltrasonicMs = 0;
unsigned long lastServoMs      = 0;

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION: I2C Full Scanner — scans all addresses, reports everything found
// ─────────────────────────────────────────────────────────────────────────────
void runI2CScan() {
    Serial.println("[I2C SCAN] Scanning all addresses 0x01 to 0x7F...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C SCAN] Device found at 0x");
            Serial.print(addr, HEX);
            // Identify known devices
            if (addr == 0x27 || addr == 0x3F) Serial.print("  ← LCD backpack");
            if (addr == 0x40)                  Serial.print("  ← PCA9685 (servo driver)");
            if (addr == 0x41 || addr == 0x42)  Serial.print("  ← PCA9685 (alt address)");
            if (addr == 0x60)                  Serial.print("  ← PCA9685 (alt address)");
            Serial.println();
            found++;
        }
        delay(3);
    }
    if (found == 0) {
        Serial.println("[I2C SCAN] WARNING: No I2C devices found at all!");
        Serial.println("[I2C SCAN] Check: SDA=GPIO21, SCL=GPIO22, power, common GND");
    } else {
        Serial.print("[I2C SCAN] Total devices found: ");
        Serial.println(found);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION: Convert degrees to PCA9685 12-bit pulse count
// ─────────────────────────────────────────────────────────────────────────────
uint16_t angleToPulse(int degrees) {
    degrees = constrain(degrees, SERVO_MIN_DEG, SERVO_MAX_DEG);
    return map(degrees, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION: Set servo angle via PCA9685
// ─────────────────────────────────────────────────────────────────────────────
void servoWrite(int degrees) {
    if (!pca_found) return;
    uint16_t pulse = angleToPulse(degrees);
    pwm.setPWM(SERVO_CHANNEL, 0, pulse);
}

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION: Motor Stop
// ─────────────────────────────────────────────────────────────────────────────
void motorsStop() {
    digitalWrite(LEFT_IN1,  LOW);
    digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(LEFT_PWM,  LOW);
    digitalWrite(RIGHT_PWM, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// FUNCTION: HC-SR04 — manual polling with stuck-HIGH detection
// Returns: distance(m) | -1.0 = no echo (timeout) | -2.0 = ECHO stuck HIGH
// ─────────────────────────────────────────────────────────────────────────────
float measureDistance() {
    // Guard: if ECHO already HIGH before we even trigger → wiring issue
    if (digitalRead(ECHO_PIN) == HIGH) {
        return -2.0f;
    }

    // Send TRIG pulse: LOW 4µs → HIGH 10µs → LOW
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(4);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait up to 30ms for ECHO to go HIGH (sensor fires echo)
    unsigned long startWait = micros();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (micros() - startWait > 30000UL) {
            return -1.0f;  // TRIG sent but no echo returned
        }
    }

    // Measure echo pulse duration
    unsigned long echoStart = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (micros() - echoStart > 30000UL) break;
    }
    unsigned long duration = micros() - echoStart;
    return (float)duration * 0.0001715f;
}




// ─────────────────────────────────────────────────────────────────────────────
// LCD helpers
// ─────────────────────────────────────────────────────────────────────────────
void lcdClearRow(uint8_t row) {
    if (!lcd_found || lcd == nullptr) return;
    lcd->setCursor(0, row);
    lcd->print("                ");
    lcd->setCursor(0, row);
}

void lcdPrint(uint8_t col, uint8_t row, const char* text) {
    if (!lcd_found || lcd == nullptr) return;
    lcd->setCursor(col, row);
    lcd->print(text);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("============================================");
    Serial.println(" BUILDScan Rover — Hardware Diagnostic v2");
    Serial.println(" PCA9685 Servo Control");
    Serial.println("============================================");

    // ── Motors STOPPED first ─────────────────────────────────────────────────
    pinMode(LEFT_PWM,  OUTPUT); pinMode(LEFT_IN1,  OUTPUT); pinMode(LEFT_IN2,  OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
    motorsStop();
    Serial.println("[MOTORS] STOPPED ✓");

    // ── I2C Init + Full Scan ──────────────────────────────────────────────────
    Serial.println("--------------------------------------------");
    Wire.begin(LCD_SDA, LCD_SCL);
    delay(100);
    runI2CScan();  // Shows ALL devices on the bus

    // ── LCD Init ─────────────────────────────────────────────────────────────
    Serial.println("--------------------------------------------");
    // Try 0x27 then 0x3F
    const uint8_t lcd_candidates[] = {0x27, 0x3F};
    for (uint8_t i = 0; i < 2; i++) {
        Wire.beginTransmission(lcd_candidates[i]);
        if (Wire.endTransmission() == 0) {
            lcd_addr  = lcd_candidates[i];
            lcd_found = true;
            break;
        }
    }

    if (lcd_found) {
        lcd = new LiquidCrystal_I2C(lcd_addr, 16, 2);
        lcd->init();
        lcd->backlight();
        Serial.print("[LCD] Found at 0x"); Serial.println(lcd_addr, HEX);
        lcd->setCursor(0, 0); lcd->print("BUILDScan v2    ");
        lcd->setCursor(0, 1); lcd->print("PCA9685 Mode    ");
        delay(1500);
        lcd->clear();
    } else {
        Serial.println("[LCD] NOT FOUND at 0x27 or 0x3F");
    }

    // ── PCA9685 Init ─────────────────────────────────────────────────────────
    Serial.println("--------------------------------------------");
    // Check if PCA9685 responds on I2C before initializing
    Wire.beginTransmission(PCA9685_ADDR);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
        pca_found = true;
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);  // calibrated oscillator
        pwm.setPWMFreq(SERVO_FREQ_HZ);
        delay(10);
        Serial.print("[PCA9685] Found at 0x"); Serial.print(PCA9685_ADDR, HEX);
        Serial.print(" — Servo on CH"); Serial.println(SERVO_CHANNEL);
        Serial.print("[PCA9685] Oscillator: 27MHz | PWM freq: ");
        Serial.print(SERVO_FREQ_HZ); Serial.println("Hz");

        // Move to neutral immediately
        servoWrite(90);
        Serial.println("[SERVO] Moved to 90° (neutral) via PCA9685 ✓");

        if (lcd_found) {
            lcdPrint(0, 0, "PCA9685 OK!     ");
            lcdPrint(0, 1, "Servo: 90deg    ");
        }
        delay(1000);
    } else {
        Serial.print("[PCA9685] NOT FOUND at 0x"); Serial.println(PCA9685_ADDR, HEX);
        Serial.println("[PCA9685] Check: VCC=3.3V or 5V, GND, SDA=GPIO21, SCL=GPIO22");
        Serial.println("[PCA9685] Servo will NOT move without PCA9685");
        if (lcd_found) {
            lcdPrint(0, 0, "PCA9685 MISSING ");
            lcdPrint(0, 1, "Check wiring!   ");
        }
    }

    // ── HC-SR04 Init ─────────────────────────────────────────────────────────
    Serial.println("--------------------------------------------");
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    // Test 1: INPUT (no pullup/pulldown) — baseline
    pinMode(ECHO_PIN, INPUT);
    delay(10);
    int echo_plain = digitalRead(ECHO_PIN);

    // Test 2: INPUT_PULLDOWN — if still HIGH, something is ACTIVELY driving it HIGH
    pinMode(ECHO_PIN, INPUT_PULLDOWN);
    delay(10);
    int echo_pulldown = digitalRead(ECHO_PIN);

    // Test 3: INPUT_PULLUP — if LOW with pullup, the line is being actively pulled LOW
    pinMode(ECHO_PIN, INPUT_PULLUP);
    delay(10);
    int echo_pullup = digitalRead(ECHO_PIN);

    // Reset to INPUT_PULLDOWN for operation (safer than floating)
    pinMode(ECHO_PIN, INPUT_PULLDOWN);

    Serial.print("[HC-SR04] GPIO18 plain INPUT   = "); Serial.println(echo_plain);
    Serial.print("[HC-SR04] GPIO18 INPUT_PULLDOWN= "); Serial.println(echo_pulldown);
    Serial.print("[HC-SR04] GPIO18 INPUT_PULLUP  = "); Serial.println(echo_pullup);

    if (echo_pulldown == 1) {
        Serial.println("[HC-SR04] *** DIAGNOSIS: GPIO18 ACTIVELY DRIVEN HIGH ***");
        Serial.println("[HC-SR04] Something is forcing 5V or 3.3V onto ECHO line.");
        Serial.println("[HC-SR04] Check: ECHO wire not touching VCC, level shifter HV ref=5V");
    } else {
        Serial.println("[HC-SR04] GPIO18 OK at rest (LOW with pulldown) — level shifter may need HV=5V ref");
    }
    Serial.println("[HC-SR04] Ready. TRIG=GPIO5 ECHO=GPIO18");

    Serial.println("============================================");
    Serial.println(" LIVE TERMINAL ACTIVE — 115200 baud");
    Serial.println(" Row 0 (LCD): Distance");
    Serial.println(" Row 1 (LCD): Servo angle");
    Serial.println(" Motors: ALWAYS STOPPED");
    Serial.println("============================================");

    if (lcd_found) {
        lcd->clear();
        lcd->setCursor(0, 0); lcd->print("Dist: ----      ");
        lcd->setCursor(0, 1); lcd->print("Servo:  90 deg  ");
    }

    lastUltrasonicMs = millis();
    lastServoMs      = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // ── Ultrasonic ──────────────────────────────────────────────────────
    if (now - lastUltrasonicMs >= 300) {
        lastUltrasonicMs = now;
        float dist = measureDistance();

        if (dist == -2.0f) {
            Serial.println("ECHO_STUCK_HIGH");
            if (lcd_found) { lcdClearRow(0); lcdPrint(0, 0, "ECHO STUCK HIGH "); }
        } else if (dist < 0.0f) {
            Serial.println("NO ECHO");
            if (lcd_found) { lcdClearRow(0); lcdPrint(0, 0, "Dist: NO ECHO   "); }
        } else {
            Serial.println(dist, 3);
            if (lcd_found) {
                char buf[17];
                snprintf(buf, sizeof(buf), "Dist: %.3f m    ", dist);
                lcdClearRow(0); lcdPrint(0, 0, buf);
            }
        }
    }

    // ── Servo sweep via PCA9685 (every 900ms) ────────────────────────────────
    if (now - lastServoMs >= SERVO_STEP_DELAY_MS) {
        lastServoMs = now;
        int angle = SERVO_POSITIONS[servoIndex];
        servoWrite(angle);
        if (lcd_found) {
            char buf[17]; snprintf(buf, sizeof(buf), "Srv:%3d deg     ", angle);
            lcdClearRow(1); lcdPrint(0, 1, buf);
        }
        servoIndex = (servoIndex + 1) % SERVO_POS_COUNT;
    }

    // ── Motors confirmed stopped every 5s ────────────────────────────────────
    static unsigned long lastMotorCheck = 0;
    if (now - lastMotorCheck >= 5000) {
        lastMotorCheck = now;
        motorsStop();
    }
}


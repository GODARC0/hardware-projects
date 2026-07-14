// ============================================================
//  GODARC BOT-V2 — DIRECTION TEST BUILD
//  Purpose: Verify motor direction polarity (Issue #1)
//  Hardware: ESP32 + MPU6500 + DRV8825 + NEMA17 Steppers
//  Library:  MPU6500_WE (install via Arduino Library Manager)
//  Author:   GODARC (NIT Calicut)
//
//  HOW TO USE:
//   1. Upload this file as-is.
//   2. Open Serial Monitor at 115200 baud.
//   3. Prop the bot upright by hand (motors will auto-enable).
//   4. Slowly tilt the WHOLE bot forward by hand and hold it.
//      - Watch the printed angle value.
//      - Watch which way the wheels spin.
//   5. Correct behavior: wheels spin in the SAME direction you
//      tilted (chasing the fall), not away from it.
//   6. If wheels spin the WRONG way, find the FLIP SWITCH below
//      (clearly marked) and change forward ? HIGH:LOW <-> LOW:HIGH
//      on BOTH lines, then re-upload and re-test.
//   7. Once confirmed correct, go back to the full PID build —
//      it uses the exact same setMotorSpeeds() convention, so
//      fixing it here fixes the real controller too.
// ============================================================

#include <Wire.h>
#include <MPU6500_WE.h>

// ============================================================
//  SECTION 1: MPU6500 SETUP
// ============================================================
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// ============================================================
//  SECTION 2: PIN DEFINITIONS
// ============================================================
// Left motor
#define LEFT_STEP_PIN    18
#define LEFT_DIR_PIN     19
#define LEFT_ENABLE_PIN  21

// Right motor
#define RIGHT_STEP_PIN   22
#define RIGHT_DIR_PIN    23
#define RIGHT_ENABLE_PIN 27

// ============================================================
//  SECTION 3: TEST PARAMETERS
// ============================================================
#define TEST_TILT_THRESHOLD_DEG  3.0   // ignore tiny noise around 0
#define TEST_STEP_DELAY_US       2000  // fixed slow step rate for the test

// ============================================================
//  SECTION 4: STATE
// ============================================================
unsigned long lastTime = 0;

// ============================================================
//  SECTION 5: SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=============================");
    Serial.println("  DIRECTION TEST — Starting...");
    Serial.println("=============================");

    Wire.begin(25, 26);   // SDA, SCL
    Wire.setClock(400000);

    pinMode(LEFT_STEP_PIN,    OUTPUT);
    pinMode(LEFT_DIR_PIN,     OUTPUT);
    pinMode(LEFT_ENABLE_PIN,  OUTPUT);
    pinMode(RIGHT_STEP_PIN,   OUTPUT);
    pinMode(RIGHT_DIR_PIN,    OUTPUT);
    pinMode(RIGHT_ENABLE_PIN, OUTPUT);

    disableMotors();

    if (!myMPU6500.init()) {
        Serial.println("ERROR: MPU6500 not found! Check wiring.");
        Serial.println("  - SDA -> GPIO 25");
        Serial.println("  - SCL -> GPIO 26");
        Serial.println("  - VCC -> 3.3V");
        Serial.println("  - AD0 -> GND");
        while (true) { delay(500); }
    }
    Serial.println("MPU6500 connected!");

    Serial.println("Keep the robot FLAT and STILL for calibration...");
    delay(2000);
    myMPU6500.autoOffsets();
    Serial.println("Calibration done!");

    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);

    lastTime = micros();

    Serial.println("Enabling motors...");
    enableMotors();
    Serial.println("Tilt the bot forward/back by hand now.");
    Serial.println("angle_deg");
}

// ============================================================
//  SECTION 6: MAIN LOOP — DIRECTION TEST ONLY
//  (No PID, no complementary filter — pure accel angle,
//   so this test is independent of Issue #2.)
// ============================================================
void loop() {
    xyzFloat accel = myMPU6500.getGValues();
    float testPitch = atan2(accel.x, accel.z) * 180.0 / PI;

    Serial.println(testPitch, 2);

    if (abs(testPitch) > TEST_TILT_THRESHOLD_DEG) {
        bool forward = (testPitch > 0);

        // ------------------------------------------------------
        // >>> FLIP SWITCH <<<
        // If wheels spin AWAY from the tilt direction, swap
        // HIGH/LOW on BOTH lines below (not just one).
        // ------------------------------------------------------
        digitalWrite(LEFT_DIR_PIN,  forward ? HIGH : LOW);
        digitalWrite(RIGHT_DIR_PIN, forward ? LOW  : HIGH);  // mirrored mount

        stepOnce(LEFT_STEP_PIN);
        stepOnce(RIGHT_STEP_PIN);
    }

    delayMicroseconds(TEST_STEP_DELAY_US);
}

// ============================================================
//  SECTION 7: SINGLE STEP PULSE
// ============================================================
inline void stepOnce(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(4);
    digitalWrite(stepPin, LOW);
}

// ============================================================
//  SECTION 8: MOTOR ENABLE / DISABLE
// ============================================================
void enableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  LOW);
    digitalWrite(RIGHT_ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  HIGH);
    digitalWrite(RIGHT_ENABLE_PIN, HIGH);
}

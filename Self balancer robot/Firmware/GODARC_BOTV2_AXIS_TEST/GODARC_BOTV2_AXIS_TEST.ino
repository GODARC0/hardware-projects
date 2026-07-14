// ============================================================
//  GODARC BOT-V2 — GYRO/ACCEL AXIS TEST BUILD
//  Purpose: Verify gyro and accel agree on pitch direction (Issue #2)
//  Hardware: ESP32 + MPU6500 + DRV8825 + NEMA17 Steppers
//  Library:  MPU6500_WE (install via Arduino Library Manager)
//  Author:   GODARC (NIT Calicut)
//
//  HOW TO USE:
//   1. Upload this file. Open Serial PLOTTER (not Monitor).
//      Tools -> Serial Plotter, 115200 baud.
//      You will see TWO lines:
//        - accelPitch  (blue)  — absolute angle from accelerometer
//        - gyroOnlyPitch (red) — integrated gyro angle (will drift)
//
//   2. Keep the bot still for ~5 seconds after "Calibration done!"
//      Both lines should sit near 0. (gyroOnlyPitch may drift slowly
//      over time even when still — that is normal and expected.)
//
//   3. Slowly tilt the WHOLE bot FORWARD by hand and HOLD it.
//      Watch the two lines:
//        GOOD: both lines move in the SAME direction (both go up,
//              or both go down). Short-term direction must agree.
//        BAD:  lines move in OPPOSITE directions when you tilt.
//
//   4. Then tilt BACKWARD and hold. Same check.
//
//   RESULT:
//   - If GOOD on both directions -> Issue #2 is fine, no fix needed.
//   - If BAD (opposite directions) -> change GYRO_SIGN below from
//     +1.0 to -1.0, re-upload, re-test.
//   - If one trace barely moves while the other moves a lot ->
//     wrong gyro axis. Change GYRO_AXIS below from gyro.y to
//     gyro.x or gyro.z, re-upload, re-test.
//
//  NOTE: Motors are DISABLED in this test — we are only checking
//  the sensor, not driving anything.
// ============================================================

#include <Wire.h>
#include <MPU6500_WE.h>

// ============================================================
//  SECTION 1: MPU6500 SETUP
// ============================================================
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// ============================================================
//  SECTION 2: PIN DEFINITIONS (motors kept disabled)
// ============================================================
#define LEFT_ENABLE_PIN  21
#define RIGHT_ENABLE_PIN 27

// ============================================================
//  SECTION 3: AXIS CONFIGURATION — EDIT HERE IF NEEDED
//
//  GYRO_SIGN: +1.0 (default) or -1.0 (if gyro opposes accel)
//  GYRO_AXIS: which gyro axis matches your pitch rotation.
//             Start with gyro.y. If wrong, try gyro.x.
// ============================================================
#define GYRO_SIGN   -1.0     // change to -1.0 if lines go opposite

// We can't do a #define for struct member, so use this flag:
// 0 = gyro.y (default), 1 = gyro.x, 2 = gyro.z
#define GYRO_AXIS_SELECT  0

// ============================================================
//  SECTION 4: STATE
// ============================================================
float gyroOnlyPitch = 0.0;
unsigned long lastTime = 0;

// ============================================================
//  SECTION 5: SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=============================");
    Serial.println("  AXIS TEST — Starting...");
    Serial.println("=============================");

    Wire.begin(25, 26);
    Wire.setClock(400000);

    // Keep motors disabled for this test
    pinMode(LEFT_ENABLE_PIN,  OUTPUT);
    pinMode(RIGHT_ENABLE_PIN, OUTPUT);
    digitalWrite(LEFT_ENABLE_PIN,  HIGH);
    digitalWrite(RIGHT_ENABLE_PIN, HIGH);

    if (!myMPU6500.init()) {
        Serial.println("ERROR: MPU6500 not found! Check wiring.");
        while (true) { delay(500); }
    }
    Serial.println("MPU6500 connected!");

    Serial.println("Keep the robot FLAT and STILL for calibration...");
    delay(2000);
    myMPU6500.autoOffsets();
    Serial.println("Calibration done! Open Serial PLOTTER now.");

    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);

    // Reset gyro integration
    gyroOnlyPitch = 0.0;
    lastTime = micros();

    // Header for Serial Plotter — labels the two traces
    Serial.println("accelPitch,gyroOnlyPitch");
}

// ============================================================
//  SECTION 6: MAIN LOOP — AXIS TEST ONLY
// ============================================================
void loop() {
    unsigned long now = micros();
    double dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt <= 0 || dt > 0.05) dt = 0.01;

    xyzFloat accel = myMPU6500.getGValues();
    xyzFloat gyro  = myMPU6500.getGyrValues();

    // Accelerometer pitch (absolute, noisy)
    float accelPitch = atan2(accel.x, accel.z) * 180.0 / PI;

    // Select gyro axis based on GYRO_AXIS_SELECT
    float gyroRate = 0.0;
    #if GYRO_AXIS_SELECT == 0
        gyroRate = gyro.y;
    #elif GYRO_AXIS_SELECT == 1
        gyroRate = gyro.x;
    #elif GYRO_AXIS_SELECT == 2
        gyroRate = gyro.z;
    #endif

    // Integrate gyro (will drift — that is normal and expected)
    gyroOnlyPitch += GYRO_SIGN * gyroRate * dt;

    // Print both for Serial Plotter
    Serial.print(accelPitch, 2);
    Serial.print(",");
    Serial.println(gyroOnlyPitch, 2);
}

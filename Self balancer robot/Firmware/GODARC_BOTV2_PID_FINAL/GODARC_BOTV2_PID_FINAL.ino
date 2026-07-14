 // ============================================================
//  GODARC BOT-V2 — Full PID Build 
//  Hardware: ESP32 + MPU6500 + DRV8825 + NEMA17 Steppers
//  Library:  MPU6500_WE (install via Arduino Library Manager)
//  Author:   GODARC (NIT Calicut)
// ============================================================

#include <Wire.h>
#include <MPU6500_WE.h>

// ============================================================
//  SECTION 1: MPU6500 SETUP
//  SDA = GPIO 25, SCL = GPIO 26
//  AD0 tied to GND → I2C address 0x68
// ============================================================
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

// ============================================================
//  SECTION 2: PIN DEFINITIONS
//  DRV8825: STEP/DIR per motor. ENABLE is active-LOW.
//  SLP + RST must be bridged together and pulled to 3.3V.
//  M0=3.3V, M1=3.3V, M2=GND → 1/8 microstepping on both drivers
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
//  SECTION 3: PID CONSTANTS — TUNE THESE
//
//  Tuning procedure:
//    1. Set Ki=0, Kd=0. Increase Kp until bot oscillates
//       but doesn't fall. (Try Kp=20 first.)
//    2. Increase Kd slowly to damp the oscillation.
//    3. Add small Ki (0.1–1.0) to remove steady lean.
//    4. Trim setpoint by ±1–2° if bot drifts forward/back
//       at rest even after tuning.
// ============================================================
double Kp = 120.0;
double Ki = 0.2;
double Kd = 1.5;

// Setpoint = angle at which bot is perfectly vertical.
// Trim this if bot has a steady lean even with Ki.
double setpoint = 0.0;

// ============================================================
//  SECTION 4: STEPPER SPEED PARAMETERS
//  1/8 microstepping → 1600 steps/rev on NEMA17.
//  MIN_STEP_INTERVAL = 500µs → 2000 steps/s → ~1.25 rev/s
//  MAX_STEP_INTERVAL = 8000µs → near stopped
// ============================================================
#define MAX_STEP_INTERVAL  8000   // µs — near stopped
#define MIN_STEP_INTERVAL  500    // µs — full speed

// ============================================================
//  SECTION 5: COMPLEMENTARY FILTER
//  [FIX #2] Gyro sign is NEGATIVE — confirmed on hardware.
//  alpha = 0.98: trust gyro 98%, accel 2% per sample.
// ============================================================
#define COMPLEMENTARY_ALPHA  0.98

float pitch = 0.0;

// ============================================================
//  SECTION 6: PID STATE VARIABLES
// ============================================================
double pidOutput  = 0.0;
double errorSum   = 0.0;
double lastError  = 0.0;
unsigned long lastTime = 0;

// ============================================================
//  SECTION 7: STEP TIMING
//  [FIX #3] Step timers are checked independently of the
//  main loop — I2C reads no longer block step generation.
// ============================================================
unsigned long leftLastStep  = 0;
unsigned long rightLastStep = 0;
unsigned long stepInterval  = MAX_STEP_INTERVAL;
bool motorForward = true;

// ============================================================
//  SECTION 8: FALL RECOVERY STATE
//  [FIX #7] Non-blocking fall recovery — no delay(2000).
//  When fallen, motors are disabled and we wait
//  FALL_RECOVERY_MS before re-enabling. Bot can be picked
//  up and re-balanced during this window.
// ============================================================
#define FALL_ANGLE_DEG      45.0
#define FALL_RECOVERY_MS    2000

bool fallen = false;
unsigned long fallTime = 0;

// ============================================================
//  SECTION 9: SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=============================");
    Serial.println("  GODARC BOT-V2 Starting...");
    Serial.println("=============================");

    Wire.begin(25, 26);
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

    // Accelerometer: ±2g (most sensitive, good for balancing)
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

    // Gyroscope: ±250°/s
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);

    // Low-pass filters to reduce noise
    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);

    // Initialise pitch from accelerometer
    xyzFloat accel = myMPU6500.getGValues();
    pitch = atan2(accel.x, accel.z) * 180.0 / PI;

    lastTime = micros();

    Serial.println("Enabling motors...");
    enableMotors();
    Serial.println("Running! Open Serial Plotter to see angle.");
    Serial.println("angle,setpoint");
}

// ============================================================
//  SECTION 10: MAIN LOOP
//
//  Structure (FIX #3 explained):
//  The loop does three things every iteration:
//    A) tryStep()  — fires step pulses if interval has elapsed.
//                    Called FIRST so it is as timely as possible.
//    B) Sensor read + filter + PID — updates stepInterval and
//                    motorForward for tryStep() to use.
//    C) tryStep()  — called AGAIN after PID so the new interval
//                    takes effect immediately this iteration.
//
//  Why this works: tryStep() is non-blocking (it just checks
//  micros() and exits immediately if the interval hasn't
//  elapsed). The I2C reads (A) take ~1–2ms, which is fine
//  because tryStep() will fire on the very next call after
//  the interval expires — worst-case latency is one loop
//  iteration (~2ms), well within the 500µs–8ms step window.
// ============================================================
void loop() {

    // --- A: Step pulse attempt (before sensor read) ---
    tryStep();

    // --- B: Sensor read + filter + PID ---
    unsigned long now = micros();
    double dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    if (dt <= 0 || dt > 0.05) dt = 0.01;

    xyzFloat accel = myMPU6500.getGValues();
    xyzFloat gyro  = myMPU6500.getGyrValues();

    // Accelerometer absolute angle
    float accelPitch = atan2(accel.x, accel.z) * 180.0 / PI;

    // [FIX #2] Gyro sign is NEGATIVE — confirmed on hardware
    pitch = COMPLEMENTARY_ALPHA * (pitch - gyro.y * dt)
          + (1.0 - COMPLEMENTARY_ALPHA) * accelPitch;

    // --- PID ---
    double error      = setpoint - pitch;
    errorSum          = constrain(errorSum + error * dt, -80.0, 80.0);
    double derivative = (error - lastError) / dt;
    lastError         = error;

    pidOutput = (Kp * error) + (Ki * errorSum) + (Kd * derivative);
    pidOutput = constrain(pidOutput, -100.0, 100.0);

    // Update step interval and direction for tryStep()
    updateStepParams(pidOutput);

    // --- C: Step pulse attempt (after PID update) ---
    tryStep();

    // --- [FIX #7] Non-blocking fall recovery ---
    if (!fallen && abs(pitch) > FALL_ANGLE_DEG) {
        // Just fell
        fallen   = true;
        fallTime = millis();
        disableMotors();
        errorSum  = 0;
        lastError = 0;
        Serial.println("FALLEN — motors disabled, waiting to recover...");
    }

    if (fallen && (millis() - fallTime >= FALL_RECOVERY_MS)) {
        // Recovery window elapsed — re-init and re-enable
        xyzFloat accelRecov = myMPU6500.getGValues();
        pitch = atan2(accelRecov.x, accelRecov.z) * 180.0 / PI;
        if (abs(pitch) < FALL_ANGLE_DEG) {
            // Only re-enable if bot has been picked upright
            fallen = false;
            enableMotors();
            Serial.println("Recovered — motors re-enabled.");
        } else {
            // Still fallen — reset the timer and wait again
            fallTime = millis();
            Serial.println("Still fallen — waiting...");
        }
    }

    // --- Serial output every 50ms ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
        lastPrint = millis();
        Serial.print(pitch, 2);
        Serial.print(",");
        Serial.println(setpoint, 2);
    }
}

// ============================================================
//  SECTION 11: UPDATE STEP PARAMETERS FROM PID OUTPUT
//  Maps pidOutput magnitude → stepInterval, sets direction.
//  Called every loop; tryStep() uses these globals.
// ============================================================
void updateStepParams(double output) {
    double magnitude = abs(output);

    if (magnitude < 3.0) {
        // Dead zone — stop stepping
        stepInterval = MAX_STEP_INTERVAL + 1;  // effectively stopped
        return;
    }

    stepInterval = (unsigned long)map(
        (long)magnitude,
        3L, 100L,
        (long)MAX_STEP_INTERVAL,
        (long)MIN_STEP_INTERVAL
    );
    stepInterval = constrain(stepInterval, MIN_STEP_INTERVAL, MAX_STEP_INTERVAL);

    motorForward = (output < 0);

    // [OK #1] Direction polarity confirmed correct on hardware
    digitalWrite(LEFT_DIR_PIN,  motorForward ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, motorForward ? LOW  : HIGH);  // mirrored mount
}

// ============================================================
//  SECTION 12: NON-BLOCKING STEP PULSE (FIX #3)
//  Checks elapsed time per motor independently.
//  Returns immediately if interval hasn't elapsed yet.
//  Called twice per loop — before and after the PID update.
// ============================================================
void tryStep() {
    if (stepInterval > MAX_STEP_INTERVAL) return;  // dead zone

    unsigned long now = micros();

    if (now - leftLastStep >= stepInterval) {
        stepOnce(LEFT_STEP_PIN);
        leftLastStep = now;
    }
    if (now - rightLastStep >= stepInterval) {
        stepOnce(RIGHT_STEP_PIN);
        rightLastStep = now;
    }
}

// ============================================================
//  SECTION 13: SINGLE STEP PULSE
//  DRV8825 min pulse width: 1.9µs. We use 4µs safely.
// ============================================================
inline void stepOnce(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(4);
    digitalWrite(stepPin, LOW);
}

// ============================================================
//  SECTION 14: MOTOR ENABLE / DISABLE
//  DRV8825 EN is active-LOW.
// ============================================================
void enableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  LOW);
    digitalWrite(RIGHT_ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  HIGH);
    digitalWrite(RIGHT_ENABLE_PIN, HIGH);
}

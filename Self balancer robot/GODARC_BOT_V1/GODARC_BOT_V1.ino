// ============================================================
//  GODARC BOT-V1 — Self-Balancing Robot Firmware
//  Hardware: ESP32 + MPU6500 + DRV8825 + NEMA17 Steppers
//  Author: GODARC (NIT Calicut)
// ============================================================

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ============================================================
//  SECTION 1: PIN DEFINITIONS
//  DRV8825 uses STEP + DIR per motor. No PWM needed.
//  MS1/MS2/MS3 set microstepping — wire to 3.3V or GND on PCB.
//  ENABLE pin is active-LOW: LOW = motors energised.
// ============================================================
// Left motor (Motor A)
#define LEFT_STEP_PIN    18
#define LEFT_DIR_PIN     19
#define LEFT_ENABLE_PIN  21

// Right motor (Motor B)
#define RIGHT_STEP_PIN   22
#define RIGHT_DIR_PIN    23
#define RIGHT_ENABLE_PIN 27

// ============================================================
//  SECTION 2: MPU6500 SETUP
//  MPU6050 library works with MPU6500 — same DMP firmware.
//  SDA = GPIO 25, SCL = GPIO 26 on ESP32 .
// ============================================================
MPU6050 mpu;

bool     dmpReady   = false;
uint8_t  mpuIntStatus;
uint8_t  devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];          // yaw, pitch, roll in radians

// DMP interrupt flag (set by ISR, consumed in loop)
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }
#define MPU_INT_PIN 34          // ESP32 GPIO 34 → MPU6500 INT

// ============================================================
//  SECTION 3: PID CONSTANTS — TUNE THESE
//
//  Start tuning procedure:
//    1. Set Ki=0, Kd=0. Increase Kp until the robot
//       oscillates around vertical but doesn't fall.
//    2. Increase Kd to damp the oscillation.
//    3. Add small Ki (0.1–1.0) to eliminate steady-state lean.
//
//  Typical starting values for a 300mm tall bot:
//    Kp = 25–40, Ki = 0.5–2.0, Kd = 1.0–2.5
// ============================================================
double Kp       = 30.0;
double Ki       = 1.0;
double Kd       = 1.5;

double setpoint = 0.0;   // Target tilt angle in degrees
                         // Trim this if the bot leans forward/back at rest

// ============================================================
//  SECTION 4: STEPPER SPEED CONTROL PARAMETERS
//
//  MAX_STEP_INTERVAL = slowest pulse period (µs) → lowest speed
//  MIN_STEP_INTERVAL = fastest pulse period (µs) → max speed
//
//  DRV8825 default: 1/32 microstepping
//  NEMA17: 200 full steps/rev → 6400 microsteps/rev at 1/32
//  100mm wheel diameter → 314mm circumference
//  At MIN_STEP_INTERVAL=200µs: ~780 steps/s → ~0.12 rev/s → ~37mm/s
//
//  Adjust MIN_STEP_INTERVAL lower to increase max speed,
//  but don't go below ~150µs or DRV8825 may miss steps.
// ============================================================
#define MAX_STEP_INTERVAL  10000   // µs — stopped
#define MIN_STEP_INTERVAL  200     // µs — full speed

// ============================================================
//  SECTION 5: PID STATE VARIABLES
// ============================================================
double   pidInput;
double   pidOutput;
double   errorSum      = 0.0;
double   lastError     = 0.0;
uint32_t lastPIDTime   = 0;

// Step timing accumulators (one per motor)
uint32_t leftLastStep  = 0;
uint32_t rightLastStep = 0;

// Motor direction state
bool leftForward  = true;
bool rightForward = true;

// ============================================================
//  SECTION 6: BATTERY MONITOR (optional)
//  2200mAh 2S LiPo nominal 7.4V, cutoff ~7.0V (3.5V/cell)
//  Divider: 10kΩ + 3.3kΩ → ratio = 3.3/(10+3.3) = 0.248
//  Adjust R1, R2 to your actual resistor values.
// ============================================================
#define BATTERY_PIN      35     // ADC1 channel, GPIO35
#define BATT_R1          10000  // Top resistor (ohms)
#define BATT_R2          3300   // Bottom resistor (ohms)
#define BATT_LOW_V       7.0    // Volts — warn below this
#define BATT_CHECK_MS    5000   // Check every 5 seconds

uint32_t lastBattCheck = 0;

// ============================================================
//  SECTION 7: SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println(F("GODARC BOT-V1 starting..."));

    // --- I2C on custom pins (avoiding conflicts with motor pins) ---
    Wire.begin(25, 26);          // SDA=25, SCL=26
    Wire.setClock(400000);       // 400kHz fast mode

    // --- Motor driver pins ---
    pinMode(LEFT_STEP_PIN,    OUTPUT);
    pinMode(LEFT_DIR_PIN,     OUTPUT);
    pinMode(LEFT_ENABLE_PIN,  OUTPUT);
    pinMode(RIGHT_STEP_PIN,   OUTPUT);
    pinMode(RIGHT_DIR_PIN,    OUTPUT);
    pinMode(RIGHT_ENABLE_PIN, OUTPUT);

    // Disable motors until IMU is ready (prevents runaway)
    disableMotors();

    // --- MPU6500 init ---
    // --- MPU6500 init ---
    mpu.initialize();

    uint8_t deviceId = mpu.getDeviceID();
    // 0x34 = MPU6050, 0x38/0x70/etc = MPU6500 variants
    if (deviceId != 0x34 && deviceId != 0x38 && deviceId != 0x3C) { 
        Serial.print(F("ERROR: Unknown IMU Device ID: 0x"));
        Serial.println(deviceId, HEX);
        while (true) { delay(1000); }   // Halt
    }
    Serial.println(F("MPU6500 recognized and connected."));
    // ---- GYRO OFFSETS — calibrate per your unit ----
    // Run the MPU6050_calibration sketch to get your values.
    // These are placeholder values — REPLACE THEM.
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        // Attach interrupt on falling edge (MPU6500 INT is active-low)
        pinMode(MPU_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN),
                        dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();
        dmpReady     = true;
        packetSize   = mpu.dmpGetFIFOPacketSize();

        Serial.println(F("DMP ready. Enabling motors..."));
        enableMotors();
    } else {
        Serial.print(F("DMP init failed, code: "));
        Serial.println(devStatus);
        while (true) { delay(1000); }   // Halt
    }

    lastPIDTime = micros();
}

// ============================================================
//  SECTION 8: MAIN LOOP
// ============================================================
void loop() {
    if (!dmpReady) return;

    // Wait for MPU interrupt or fresh FIFO packet
    if (!mpuInterrupt && fifoCount < packetSize) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount    = mpu.getFIFOCount();

    // Handle FIFO overflow (shouldn't happen, but safety net)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return;
    }

    // Read all available packets, keep only the latest
    if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Compute tilt angle from quaternion + gravity vector
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // ypr[1] = pitch in radians → convert to degrees
        // Adjust sign (+/-) if the bot drives the wrong way
        pidInput = ypr[1] * (180.0 / M_PI);

        // Compute PID and drive motors
        computePIDAndStep();
    }

    // Periodic battery check
    checkBattery();
}

// ============================================================
//  SECTION 9: PID COMPUTATION
//
//  This function runs every time we get a fresh IMU packet.
//  dt (delta time) makes the integrator and derivative
//  time-accurate regardless of loop jitter.
// ============================================================
void computePIDAndStep() {
    uint32_t now = micros();
    double dt = (now - lastPIDTime) / 1000000.0;  // seconds
    if (dt <= 0 || dt > 0.1) {                    // Sanity clamp
        lastPIDTime = now;
        return;
    }
    lastPIDTime = now;

    double error = setpoint - pidInput;

    // Anti-windup: clamp integrator
    errorSum = constrain(errorSum + (error * dt), -100.0, 100.0);

    double derivative = (error - lastError) / dt;
    lastError = error;

    pidOutput = (Kp * error) + (Ki * errorSum) + (Kd * derivative);

    // Clamp output to ±100 (we'll map this to step interval)
    pidOutput = constrain(pidOutput, -100.0, 100.0);

    // Convert PID output to step interval and direction
    setMotorSpeeds(pidOutput);
}

// ============================================================
//  SECTION 10: STEPPER SPEED MAPPING
//
//  pidOutput > 0  → lean forward  → drive forward
//  pidOutput < 0  → lean backward → drive backward
//  |pidOutput| maps to step interval: higher output = faster steps
//
//  Both motors run at the same speed (straight balancing).
//  For turning in Phase 2, offset left/right speeds here.
// ============================================================
void setMotorSpeeds(double output) {
    double magnitude = abs(output);

    if (magnitude < 2.0) {
        // Dead zone — don't jitter motors for tiny errors
        leftLastStep  = micros();
        rightLastStep = micros();
        return;
    }

    // Map magnitude (2–100) to interval (MAX→MIN µs)
    uint32_t stepInterval = (uint32_t) map(
        (long)(magnitude * 100),        // scale to integer
        200L, 10000L,                   // input range × 100
        (long)MIN_STEP_INTERVAL,
        (long)MAX_STEP_INTERVAL
    );
    stepInterval = constrain(stepInterval, MIN_STEP_INTERVAL, MAX_STEP_INTERVAL);

    // Direction: positive output = forward, negative = backward
    // NOTE: one motor may need its direction inverted depending
    // on how it's mounted. Flip RIGHT_DIR logic if needed.
    bool forward = (output > 0);
    digitalWrite(LEFT_DIR_PIN,  forward ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, forward ? HIGH : LOW);  // Adjust if mirrored

    // Issue STEP pulses if enough time has elapsed
    uint32_t now = micros();

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
//  SECTION 11: SINGLE STEP PULSE
//  DRV8825 minimum STEP pulse width: 1.9µs
//  We use 5µs for safety margin.
// ============================================================
inline void stepOnce(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
}

// ============================================================
//  SECTION 12: MOTOR ENABLE / DISABLE
//  DRV8825 ENABLE is active-LOW:
//    LOW  → driver active, motor holds torque
//    HIGH → driver off, motor freewheels
// ============================================================
void enableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  LOW);
    digitalWrite(RIGHT_ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(LEFT_ENABLE_PIN,  HIGH);
    digitalWrite(RIGHT_ENABLE_PIN, HIGH);
}

// ============================================================
//  SECTION 13: BATTERY MONITOR
//  ADC on ESP32: 12-bit (0–4095) → 0–3.3V
//  Battery voltage = ADC_voltage × (R1+R2)/R2
//  If bot falls over (|pitch| > 45°), disable motors to
//  prevent motor stalling and excess current draw.
// ============================================================
void checkBattery() {
    uint32_t now = millis();
    if (now - lastBattCheck < BATT_CHECK_MS) return;
    lastBattCheck = now;

    int   raw    = analogRead(BATTERY_PIN);
    float adcV   = raw * (3.3f / 4095.0f);
    float battV  = adcV * ((BATT_R1 + BATT_R2) / (float)BATT_R2);

    Serial.print(F("Battery: "));
    Serial.print(battV, 2);
    Serial.print(F("V  |  Pitch: "));
    Serial.print(pidInput, 2);
    Serial.println(F("°"));

    if (battV < BATT_LOW_V && battV > 1.0) {   // >1V to ignore disconnected state
        Serial.println(F("WARNING: Low battery!"));
    }

    // Safety cutoff: if bot has fallen, disable motors
    if (abs(pidInput) > 45.0) {
        Serial.println(F("FALLEN — disabling motors."));
        disableMotors();
        errorSum = 0;   // Reset integrator so it doesn't wind up
        delay(2000);
        enableMotors();
    }
}

// ============================================================
//  END OF FIRMWARE
//
//  WIRING QUICK-REFERENCE
//  ─────────────────────────────────────────────────────────
//  MPU6500        →  ESP32
//    VCC          →  3.3V
//    GND          →  GND
//    SDA          →  GPIO 25
//    SCL          →  GPIO 26
//    INT          →  GPIO 34
//
//  DRV8825 (Left) →  ESP32
//    STEP         →  GPIO 18
//    DIR          →  GPIO 19
//    EN           →  GPIO 21
//    VMOT         →  LiPo+ (7.4V)
//    GND (both)   →  GND
//    MS1/2/3      →  Set for 1/16 or 1/32 microstepping
//    A1/A2/B1/B2  →  NEMA17 coil pairs
//
//  DRV8825 (Right)→  ESP32
//    STEP         →  GPIO 22
//    DIR          →  GPIO 23
//    EN           →  GPIO 27
//    (rest same as left)
//
//  LiPo Battery
//    + (7.4V)     →  VMOT pins (both drivers)
//    -            →  GND (common ground with ESP32)
//    Battery voltage divider → GPIO 35 (ADC)
//      LiPo+ → 10kΩ → GPIO35 → 3.3kΩ → GND
//
//  DRV8825 CURRENT LIMIT (important!)
//    Set Vref = I_max × 0.5  (for NEMA17 rated ~1.5A)
//    Target Vref ≈ 0.75V
//    Measure at potentiometer wiper relative to GND.
// ============================================================

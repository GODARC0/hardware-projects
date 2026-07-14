/*
  LQR_TwoWheelSBR.ino
  --------------------------------------------------------------------------
  LQR balance controller for the "Two Wheel SBR" (ESP32 + MPU6500 + 2x DRV8825
  + 2x NEMA17, 1/8 microstepping hardwired on M0/M1/M2).

  KEY DESIGN CHOICE vs.  earlier LQR attempt:
  Your NEMA17/DRV8825 steppers are VELOCITY-actuated (open-loop step pulses),
  not FORCE-actuated. The LQR model (see lqr_sbr_design.m) computes u as a
  horizontal force. So instead of trying to apply u directly as some kind of
  "motor power," we:
      1) convert u -> commanded wheel acceleration (u / M_effective)
      2) integrate acceleration over dt -> a velocity target
      3) convert that velocity target -> step frequency
  This also means the "dx" state the controller needs is exactly the velocity
  you just commanded in step (2) -- no accelerometer double-integration, no
  drift, no extra sensor.

  Gains below (Kdx, Ktheta, Kdtheta) were recomputed from lqr_sbr_design.m with
  the MEASURED masses m = 0.900 kg (body) and M = 1.100 kg (drive/cart
  assembly), L = 0.16 m, Ts = 0.005 s. They match what was already loaded here
  -- verified by re-running dlqr() with these exact parameters.
  --------------------------------------------------------------------------
*/

#include <Wire.h>
#include <MPU6500_WE.h>

#define MPU6500_ADDR 0x68
MPU6500_WE myMPU = MPU6500_WE(MPU6500_ADDR);

// ---------------- I2C pins (remapped per report -- default 21/22 collide
// with Left Motor EN / Right Motor STEP) --------------------------------
const int SDA_PIN = 25;
const int SCL_PIN = 26;

// ---------------- Pin map (matches Tables: Left/Right DRV8825 connections) -
const int STEP_L = 18;
const int DIR_L  = 19;
const int EN_L   = 21;   // active-LOW: LOW = driver enabled

const int STEP_R = 22;
const int DIR_R  = 23;
const int EN_R   = 27;   // active-LOW: LOW = driver enabled

// RST/SLP on both DRV8825 are hardwired to 3.3V (per your earlier fix) --
// not controlled from the ESP32.

// ---------------- Loop timing (MUST match Ts used in MATLAB dlqr) ---------
const unsigned long LOOP_US = 5000;   // 5 ms -> 200 Hz. Keep this in sync with
                                       // Ts in lqr_sbr_design.m or the gains
                                       // below will be wrong for this sample rate.
const float DT = LOOP_US / 1.0e6f;    // seconds

// ---------------- LQR gains: from MATLAB, m=0.9 kg, M=1.1 kg, L=0.16 m ----
// Order: [Kdx, Ktheta, Kdtheta]
float K_dx     = -3.5210;
float K_theta  = 70.4607;
float K_dtheta = 35.8783;

// M must match the "M" (effective translating/cart mass) used in MATLAB, so
// that u/M below produces the acceleration the model actually assumes.
const float M_EFFECTIVE = 1.100f;  // kg -- matches m=0.9/M=1.1 measured split

// ---------------- Complementary filter state -------------------------------
float theta = 0.0f;          // rad, 0 = upright, matches model convention directly
float thetaDot = 0.0f;       // rad/s
const float ALPHA = 0.98f;   // complementary filter weight (same as your PID build)

// ---------------- Wheel / drivetrain geometry -------------------------------
const float WHEEL_RADIUS_M   = 0.05f;      // 100 mm wheel diameter per chassis spec
const float WHEEL_CIRC_M     = 2.0f * PI * WHEEL_RADIUS_M;
const float STEPS_PER_REV    = 200.0f * 8; // 200 full steps x 1/8 microstepping
const float MAX_WHEEL_SPEED  = 0.8f;       // m/s, clamp for safety/traction
const float MAX_STEP_FREQ    = MAX_WHEEL_SPEED / WHEEL_CIRC_M * STEPS_PER_REV;

// ---------------- Controller / actuation state -------------------------------
float wheelVel = 0.0f;        // m/s, this doubles as the "dx" state fed back to LQR
                               // -- it is EXACT because it's the value we just commanded,
                               // not an estimate.
float stepFreqL = 0.0f, stepFreqR = 0.0f;   // steps/s, magnitude
unsigned long lastStepTimeL = 0, lastStepTimeR = 0;

// ---------------- Fall detection -------------------------------
const float FALL_ANGLE_RAD = 35.0f * PI / 180.0f;

unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);   // remapped I2C pins

  if (!myMPU.init()) {
    Serial.println("MPU6500 not responding -- check wiring.");
    while (1) delay(100);
  }
  Serial.println("Calibrating MPU6500, keep the robot still and upright...");
  delay(1000);
  myMPU.autoOffsets();
  myMPU.setSampleRateDivider(0);
  myMPU.setAccRange(MPU6500_ACC_RANGE_4G);
  myMPU.setGyrRange(MPU6500_GYRO_RANGE_500);

  pinMode(STEP_L, OUTPUT); pinMode(DIR_L, OUTPUT); pinMode(EN_L, OUTPUT);
  pinMode(STEP_R, OUTPUT); pinMode(DIR_R, OUTPUT); pinMode(EN_R, OUTPUT);

  digitalWrite(EN_L, LOW);   // enable left driver (active-LOW)
  digitalWrite(EN_R, LOW);   // enable right driver (active-LOW)

  // seed theta from accelerometer only, before the filter has gyro history
  xyzFloat acc = myMPU.getGValues();
  theta = atan2(acc.x, acc.z);   // adjust axis mapping to match your mounting

  lastLoopTime = micros();
  lastStepTimeL = lastStepTimeR = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoopTime < LOOP_US) {
    // still generate step pulses between control updates -- non-blocking, same
    // pattern as your PID firmware's tryStep(), called twice per loop.
    tryStep(STEP_L, stepFreqL, lastStepTimeL);
    tryStep(STEP_R, stepFreqR, lastStepTimeR);
    return;
  }
  lastLoopTime = now;

  // ---- 1) Read IMU, update complementary filter ----
  xyzFloat gyr = myMPU.getGyrValues();
  xyzFloat acc = myMPU.getGValues();

  float accAngle = atan2(acc.x, acc.z);       // match your existing axis mapping
  float gyroRate = -gyr.y * PI / 180.0f;      // sign-flipped per your earlier fix (Y-axis inverted), deg/s -> rad/s

  theta = ALPHA * (theta + gyroRate * DT) + (1.0f - ALPHA) * accAngle;
  thetaDot = gyroRate;

  // ---- 2) Fall check ----
  if (fabs(theta) > FALL_ANGLE_RAD) {
    stepFreqL = stepFreqR = 0.0f;
    wheelVel = 0.0f;
    Serial.println("FALLEN -- motors stopped. Right the robot to resume.");
    return;
  }
  // ---- 3) LQR control law ----
  // u is a horizontal force in the model's units (see lqr_sbr_design.m).
  float u = -(K_dx * wheelVel + K_theta * theta + K_dtheta * thetaDot);

  // ---- 4) Convert force -> acceleration -> velocity target ----
  float accelCmd = u / M_EFFECTIVE;           // m/s^2
  wheelVel += accelCmd * DT;                  // integrate to a velocity command
  wheelVel = constrain(wheelVel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

  // ---- 5) Convert velocity -> step frequency, apply to both wheels ----
  float stepFreq = fabs(wheelVel) / WHEEL_CIRC_M * STEPS_PER_REV;
  stepFreq = constrain(stepFreq, 0.0f, MAX_STEP_FREQ);

  bool motorForward = (wheelVel < 0);   // matches the sign convention you already
                                         // validated on the PID build -- flip if
                                         // the robot drives away from falling.
  digitalWrite(DIR_L, motorForward ? HIGH : LOW);
  digitalWrite(DIR_R, motorForward ? LOW  : HIGH);   // wheels mounted opposite-facing

  stepFreqL = stepFreqR = stepFreq;

  // ---- 6) Logging for settling-time capture (matches your PuTTY/serialport workflow) ----
  if (now - lastPrintTime > 20000) {  // 50 Hz print, plenty for settling-time plots
    lastPrintTime = now;
    Serial.println("Controller: LQR");
    Serial.print(now / 1000.0f, 1); Serial.print(",");
    Serial.print(theta * 180.0f / PI, 2); Serial.print(",");
    Serial.print(thetaDot * 180.0f / PI, 2); Serial.print(",");
    Serial.println(wheelVel, 3);
  }
}

// Non-blocking step pulse generator -- call twice per loop, same pattern as your
// PID firmware.
void tryStep(int stepPin, float freqHz, unsigned long &lastStepTime) {
  if (freqHz <= 1.0f) return;
  unsigned long periodUs = (unsigned long)(1.0e6f / freqHz);
  unsigned long now = micros();
  if (now - lastStepTime >= periodUs) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(stepPin, LOW);
    lastStepTime = now;
  }
}

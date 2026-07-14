/*
  LQR_TwoWheelSBR.ino
  --------------------------------------------------------------------------
 
  --------------------------------------------------------------------------
*/

#include <Wire.h>
#include <MPU6500_WE.h>

#define MPU6500_ADDR 0x68
MPU6500_WE myMPU = MPU6500_WE(MPU6500_ADDR);

// ---------------- Pin map (MATCH THESE TO YOUR PID SKETCH) ----------------
const int STEP_L = 26;
const int DIR_L  = 27;
const int STEP_R = 32;
const int DIR_R  = 33;
// RST/SLP on both DRV8825 must be hardwired to 3.3V (per your earlier fix).

// ---------------- Loop timing (MUST match Ts used in MATLAB dlqr) ---------
const unsigned long LOOP_US = 5000;   // 5 ms -> 200 Hz. Keep this in sync with
                                       // Ts in lqr_sbr_design.m or your gains
                                       // will be wrong for this sample rate.
const float DT = LOOP_US / 1.0e6f;    // seconds

// ---------------- LQR gains: PASTE Kd3 FROM MATLAB HERE -------------------
// Order: [Kdx, Ktheta, Kdtheta]
float K_dx     = -3.5210;     
float K_theta  = 70.4607;    
float K_dtheta = 35.8783;    

// M must match the "M" (effective translating mass) used in MATLAB, so that
// u/M below produces the acceleration the model actually assumes.
const float M_EFFECTIVE = 0.15f;  // kg -- keep identical to lqr_sbr_design.m

// ---------------- Complementary filter state -------------------------------
float theta = 0.0f;          // rad, 0 = upright, matches model convention directly
float thetaDot = 0.0f;       // rad/s
const float ALPHA = 0.98f;   // complementary filter weight (same as your PID build)

// ---------------- Wheel / drivetrain geometry -------------------------------
const float WHEEL_RADIUS_M   = 0.034f;     // measure your actual wheel
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
  Wire.begin();

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

  pinMode(STEP_L, OUTPUT); pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT); pinMode(DIR_R, OUTPUT);

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
  float gyroRate = -gyr.y * PI / 180.0f;      // sign-flipped per your earlier fix, deg/s -> rad/s

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

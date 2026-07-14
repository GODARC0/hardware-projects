# Two-Wheeled Self-Balancing Robot (SBR) — PID vs LQR Control

A hardware implementation and comparative analysis of **PID** and **LQR** control strategies for a two-wheeled self-balancing robot. The robot balances itself using an IMU for tilt sensing and two NEMA17 stepper motors for actuation, with both control strategies implemented and tested on the same physical hardware for direct comparison.

---

## Table of Contents

- [Overview](#overview)
- [Hardware, Wiring & Sensor Fusion](#hardware-wiring--sensor-fusion)
- [Control Approach](#control-approach)
- [Repository Structure](#repository-structure)

---

## Overview

This project designs, builds, and evaluates a self-balancing two-wheeled robot, comparing classical **PID** control against a model-based **LQR (Linear Quadratic Regulator)** controller. Both controllers are deployed on identical hardware, allowing a direct comparison of stability, settling time, and disturbance rejection. The robot's dynamics are modeled as an inverted pendulum on a cart, linearized about the upright equilibrium, and used to design the LQR gains in MATLAB before deployment to the ESP32 firmware.

## Hardware, Wiring & Sensor Fusion

**Components:** ESP32 microcontroller, MPU6500 IMU, 2× NEMA17 stepper motors, 2× DRV8825 drivers, 2200 mAh 3S LiPo battery. Wheel diameter 100 mm; chassis height (base to top plate) 300 mm; 1/8 microstepping (1600 steps/rev).

**Mass distribution** used in modeling: drive assembly (motors + wheels + brackets) M = 1.100 kg, body/pendulum m = 0.900 kg.

**Wiring:**
- IMU (I2C): SDA → GPIO 25, SCL → GPIO 26
- Left motor: STEP → GPIO 18, DIR → GPIO 19, EN → GPIO 21
- Right motor: STEP → GPIO 22, DIR → GPIO 23, EN → GPIO 27
- DRV8825: M0/M1 → 3.3V, M2 → GND (1/8 microstepping); RST and SLP bridged to 3.3V; EN active-LOW, driven LOW at boot
- LiPo connects directly to VMOT; ESP32 GND common with motor driver GND
- ESP32 lacks auto-reset — hold BOOT during the `Connecting...` phase of upload

**Sensor fusion:** Tilt angle is estimated with a complementary filter (98% gyroscope, 2% accelerometer) using the MPU6500_WE library, polled over I2C at 200 Hz (`Ts = 0.005 s`). Fall-detection safety cutoff is set at ±35°.

## Control Approach

**PID:** Classical proportional-integral-derivative control on pitch angle, commanding stepper velocity. Error convention is `setpoint − pitch`, with motor direction set as `motorForward = (output < 0)`.

**LQR:** Full-state feedback designed from a linearized inverted-pendulum model, reduced to a 3-state form:

```
K_dx      (velocity feedback)
K_θ       (pitch angle feedback)
K_dθ      (pitch rate feedback)
```

Cost weighting: `Q = diag([1, 1, 10, 100])`, `R = 0.0001`. Since the NEMA17 steppers are velocity-actuated (open-loop step pulses) rather than force-actuated, the LQR's force output is converted through force → acceleration → velocity → step frequency, with the commanded step rate fed back directly as the `dx` state.

## Repository Structure

```
.
├── firmware/
│   ├── pid_controller/       # ESP32 PID control firmware
│   └── lqr_controller/       # ESP32 LQR control firmware
├── matlab/
│   ├── cartpend.m               # Nonlinear cart-pendulum dynamics
│   ├── linearize_cartpend.m     # Linearization about upright equilibrium
│   ├── lqr_cartpend.m           # LQR gain design and closed-loop simulation
│   ├── poleplace_cartpend.m     # Pole-placement controller design
│   ├── obsv_cartpend.m          # Observability analysis
│   ├── kf_cartpend.m            # Kalman filter estimator design
│   ├── sim_cartpend.m           # Open-loop simulation
│   └── drawcartpend.m / drawcartpend_bw.m   # Visualization of cart-pendulum state
└── docs/                     # Photos, wiring diagrams, figures
```


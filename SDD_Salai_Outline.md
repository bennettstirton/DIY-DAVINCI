# System Design Document — Project Salai
## DIY Surgical Robot
**Status:** Outline / Living Document
**Last updated:** 2026-06-01
**Author:** Bennett Stirton

---

> **How to read this document**
> Each section is marked with one of two flags:
> - ✅ **Can fill now** — sufficient detail exists in the handoff documents to write this section.
> - ⚠️ **Open design question** — the decision is unresolved, the hardware is unbuilt, or the information is missing and must be resolved before this section can be completed.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Overview](#2-system-overview)
3. [Architecture Overview](#3-architecture-overview)
4. [Subsystem Design — Coarse Arm](#4-subsystem-design--coarse-arm)
5. [Subsystem Design — Fine Instrument](#5-subsystem-design--fine-instrument)
6. [Subsystem Design — Operator Input](#6-subsystem-design--operator-input)
7. [Subsystem Design — Raspberry Pi Compute Layer](#7-subsystem-design--raspberry-pi-compute-layer)
8. [Communication Interfaces & Protocols](#8-communication-interfaces--protocols)
9. [Kinematics & Control Design](#9-kinematics--control-design)
10. [Signal Processing](#10-signal-processing)
11. [Safety & Error Handling](#11-safety--error-handling)
12. [Configuration & Tuning Parameters](#12-configuration--tuning-parameters)
13. [Testing & Verification](#13-testing--verification)
14. [Known Limitations & Future Work](#14-known-limitations--future-work)
15. [Appendices](#15-appendices)

---

## 1. Introduction

### 1.1 Purpose
*What this document is for, who it is for, and how it should be maintained.*

✅ **Can fill now** — document this as a living engineering reference for DIY development; audience is the developer (you) and any future contributors or collaborators.

### 1.2 Project Background & Motivation
*Why this robot exists. Inspiration (Da Vinci Si). DIY/hobbyist context. Resume/conference goals.*

✅ **Can fill now** — handoff documents clearly state the goal: a 2-arm (expanding to 4-arm) surgical robot inspired by the Intuitive Da Vinci Si, built for fun and resume/conference presentation purposes.

### 1.3 Scope
*What this document covers. Which subsystems are in scope for the current MVP. What is explicitly out of scope (sterile field, clinical use, etc.).*

✅ **Can fill now** (partially) — define scope around the current MVP: coarse arm (working), fine instrument (working single arm), Pi integration (not started). Explicitly note this is a non-clinical prototype.

### 1.4 Definitions & Abbreviations

✅ **Can fill now** — populate from handoff docs: DOF, EMA, PID, IK, NC, ESP32, BNO055, ADS1115, STS3215, ST bus, NEMA, DM556T, TB6600, etc.

---

## 2. System Overview

### 2.1 High-Level System Description
*One-paragraph prose description of what the system does and how it works at the top level.*

✅ **Can fill now**

### 2.2 System Goals & Design Constraints

✅ **Can fill now** (partially) — known constraints include:
- ESP32 GPIO fully committed (no expansion without I2C GPIO expander)
- I2C addressing conflicts requiring separate buses for dual AS5600 encoders
- Cable-driven EndoWrist IK couples all 4 fine instrument axes
- Real-time motor control must remain on ESP32; compute-heavy tasks on Pi

⚠️ **Open question** — formal latency budget (maximum acceptable end-to-end delay from operator input to instrument motion) has not been defined.

### 2.3 System Block Diagram
*Top-level diagram showing all major hardware nodes and the signal paths between them.*

⚠️ **Open question** — no diagram exists yet. Create one showing: Operator → Input devices → Pi → ESP32 (coarse) / ESP32 (fine) → Motors.

### 2.4 Operating Modes
*Normal operation, homing, e-stop, clutch/freeze, boot sequence.*

✅ **Can fill now** (partially) — homing and normal jogging modes documented. 

⚠️ **Open question** — clutch/freeze and foot pedal modes are planned but not implemented. Full re-capture (mid-session home reset for IMU) is architecturally possible but not yet built.

---

## 3. Architecture Overview

### 3.1 Split-Responsibility Model
*Narrative explanation of the ESP32 / Pi division of responsibility.*

✅ **Can fill now** — this is the core architectural decision and is well documented: ESP32s are "dumb" real-time motor controllers; Pi handles IK, input processing, and orchestration.

### 3.2 Compute Nodes Summary

| Node | Hardware | Role | Status |
|------|----------|------|--------|
| Coarse ESP32 | ESP32 | Stepper PWM, encoder reads, PID | Working |
| Fine instrument ESP32 | ESP32 | STS3215 servo commands, IK | Working |
| Raspberry Pi 5 | RPi 5 | Input aggregation, IK, serial bridge | Not started |

✅ **Can fill now**

### 3.3 Scalability Path
*How the architecture scales from 1-arm MVP to 2-arm and eventually 4-arm system.*

⚠️ **Open question** — the handoff notes the 4-arm goal but does not specify the scaling architecture. Questions to resolve: Does each arm get its own ESP32 pair? Is there one Pi per arm pair or one central Pi? How does the serial bus topology change?

---

## 4. Subsystem Design — Coarse Arm

*Covers the 3-DOF stepper-based positioning stage. Per arm; design is identical across arms.*

### 4.1 Mechanical Configuration
*3-DOF layout: Pitch (base rotation), Roll (arm rotation via 10:1 gearbox), Linear (cable-driven carriage, 220mm travel, 20mm spool diameter).*

✅ **Can fill now**

### 4.2 Actuator Hardware

| Axis | Motor | Driver | Feedback |
|------|-------|--------|----------|
| Pitch | NEMA 23 | DM556T | AS5600 encoder (I2C bus 2, GPIO 4/5) |
| Roll | NEMA 17 + 10:1 gearbox | DM542TE | AS5600 encoder (I2C bus 1, GPIO 21/22) |
| Linear | NEMA 17 (size TBD) | TB6600 | None (open-loop) + NC limit switch GPIO 23 |

✅ **Can fill now** — note: linear motor size listed as "NEMA ??" in primary handoff; confirm and update.

⚠️ **Open question** — NEMA size for linear axis motor is not documented.

### 4.3 GPIO Pin Allocation
*Full 38-pin ESP32 assignment table. All pins committed; any changes require full audit.*

✅ **Can fill now** — full table in primary handoff.

### 4.4 Firmware Architecture (`main.py`)
*Main loop (40ms tick), routing logic, per-axis decel flags, PWM generation.*

✅ **Can fill now** — thoroughly documented in primary handoff including key architectural decisions and bug history.

#### 4.4.1 Main Loop Routing
#### 4.4.2 Per-Axis Deceleration Flags
#### 4.4.3 PWM Motor Drive (STEP/DIR)

### 4.5 PID Control — Pitch & Roll Axes
*Target angle calculation from joystick, EMA smoothing, deadband, PID gains, integral clamp.*

✅ **Can fill now**

### 4.6 Open-Loop Control — Linear Axis
*Step counter position tracking, soft limits, NC limit switch hard stop, cable spool geometry.*

✅ **Can fill now** — include known accuracy caveat (cable layer buildup causes drift near full extension).

### 4.7 Homing Sequence
*Boot window, drive-to-switch, backoff, PWM deinit/reinit.*

✅ **Can fill now**

### 4.8 I2C Bus Configuration
*Two SoftI2C buses at 100kHz, address conflict rationale, cable run guidelines.*

✅ **Can fill now** — include guidance on pull-up resistors, minimum frequency, cable type recommendations.

### 4.9 Known Firmware Constraints & Bug History
*Summary of resolved bugs; architectural decisions that must not be reversed.*

✅ **Can fill now** — full bug table in primary handoff.

---

## 5. Subsystem Design — Fine Instrument

*Covers the 4-DOF cable-driven EndoWrist instrument control stage.*

### 5.1 Target Instrument
*Intuitive Surgical EndoWrist (used). Cable-driven architecture overview.*

✅ **Can fill now** — brief description; note this is a real surgical instrument repurposed for this project.

### 5.2 Actuator Hardware
*STS3215-C047 smart servos × 4. 1:345 metal gearbox, 12V, ST bus, magnetic encoder.*

✅ **Can fill now**

### 5.3 Servo ID Assignment

| Servo ID | Axis |
|----------|------|
| 4 | Pitch |
| 5 | Roll |
| 6 | Yaw / Grip 1 |
| 7 | Yaw / Grip 2 |

✅ **Can fill now**

### 5.4 Waveshare Adapter Wiring
*UART mode, TX→TX/RX→RX (non-crossover), jumper cap position, 12V servo power, 5V logic.*

✅ **Can fill now** — include the counter-intuitive TX→TX wiring explanation.

### 5.5 Feetech ST Bus Protocol
*Half-duplex serial, 1Mbps, packet structure, key commands used.*

✅ **Can fill now** (partially) — handoff notes the protocol was implemented from scratch in ~50 lines. Detail the key registers used: `write_pos_timed`, `write_pos_ex`, `read_register`, `read_moving`, `wait_until_stopped`.

⚠️ **Open question** — full protocol register map is not documented in the handoff. Expand from code or Feetech datasheet.

### 5.6 Firmware Architecture (`instrumentcontrol.py` — stable baseline)
*Two async tasks: `joystick_task` and `motor_task`. 50Hz update rate.*

✅ **Can fill now**

### 5.7 Firmware Architecture (`imu_instrumentcontrol.py` — next to test)
*Branched from joystick version. BNO055 replaces joystick ADC. `imu_task` and `motor_task`.*

✅ **Can fill now** — written but not yet tested; flag accordingly.

### 5.8 Boot Sequence
*All motors drive to center (2048 counts) before handing off to main loop. No homing required (absolute encoder retention across power cycles).*

✅ **Can fill now**

### 5.9 Actuator Iteration History
*Documents the design path: 28BYJ-48 unipolar → 28BYJ-48 bipolar TMC2209 → STS3215.*

✅ **Can fill now** — full iteration history preserved in secondary handoff. This is valuable institutional knowledge worth retaining in the SDD.

---

## 6. Subsystem Design — Operator Input

### 6.1 Input Device Overview

| Input | Used for | Connected to | Status |
|-------|----------|--------------|--------|
| 2-axis analog joystick | Coarse arm pitch + roll | Coarse ESP32 GPIO 34/35 | Working |
| 3× rocker switches | Coarse arm axis jogging | Coarse ESP32 | Working |
| Hall-effect joystick | Fine instrument pitch + yaw | Fine ESP32 GPIO 34/35 | Working |
| BNO055 IMU (wrist-worn) | Fine instrument control | Fine ESP32 I2C (GPIO 21/22) | Written, not tested |
| Grip button | Jaw open/close | Fine ESP32 GPIO 25 | Not yet wired |
| ADS1115 + joystick | Coarse arm (Pi path) | Pi I2C | Not started |
| BNO055 × 2 (Pi path) | Both arms (Pi path) | Pi I2C | Not started |
| Foot pedal / clutch | Motion freeze | TBD | Not started |

✅ **Can fill now** (status table)

### 6.2 Joystick Signal Chain (Current)
*ADC read → EMA smoothing → deadband → normalized output → axis command.*

✅ **Can fill now**

### 6.3 BNO055 IMU Signal Chain
*NDOF fusion mode, Euler angle read → delta-from-home calculation → deadband → EMA → scale → IK.*

✅ **Can fill now** — document calibration requirement (rotate for 20-30s on first power-up in new location).

### 6.4 Grip Button
*GPIO 25, momentary NO, internal pull-up. Hold = jaw open (25°), release = jaw closed (0°).*

✅ **Can fill now** (spec is defined, not yet physically wired)

### 6.5 Foot Pedal / Clutch
*Plan: normally-open switch, freezes motion commands. Hardware path to driver ENA pins TBD.*

⚠️ **Open question** — not yet designed. Key decision: software-only freeze (Pi reads pin, stops sending commands) vs. hardware path directly to stepper driver ENA pins. Hardware path is safer but requires physical wiring to each driver.

### 6.6 Future Input Options
*Vision-based hand tracking: Pi + USB webcam + MediaPipe Hands. Architecture already supports swapping input source.*

⚠️ **Open question** — not yet pursued. Note that the Pi is the natural location for this; no architectural changes needed beyond input source swap.

---

## 7. Subsystem Design — Raspberry Pi Compute Layer

*The Pi 5 is the central compute hub for the multi-arm system. Not yet implemented.*

### 7.1 Responsibilities
*Read BNO055 IMUs and ADS1115 joysticks via I2C; run IK; send position commands to ESP32s over USB serial; receive encoder feedback; clutch/freeze logic.*

✅ **Can fill now** (requirements defined)

### 7.2 I2C Device Map (Pi Bus 1)

| Device | I2C Address | Notes |
|--------|-------------|-------|
| BNO055 arm 1 | 0x28 | ADDR pin to GND |
| BNO055 arm 2 | 0x29 | ADDR pin to VCC |
| ADS1115 arm 1 | 0x48 | |
| ADS1115 arm 2 | 0x49 | ADDR pin to VCC |

✅ **Can fill now**

### 7.3 USB Serial Interface to ESP32s
*Command and feedback protocol over USB serial. Format, baud rate, packet structure.*

⚠️ **Open question** — serial protocol between Pi and ESP32s is not yet designed. Decisions needed: packet framing, command format (joint angles? raw steps? normalized floats?), feedback format (encoder values? error state?), baud rate, error/timeout handling.

### 7.4 Software Architecture (Pi)
*Python. Libraries: `smbus2` or `board`/`busio`, `pyserial`, `adafruit-circuitpython-bno055`, `adafruit-circuitpython-ads1x15`.*

⚠️ **Open question** — Pi software is not yet written. The handoff identifies required libraries and responsibilities but nothing has been implemented.

### 7.5 Timing & Latency Budget

⚠️ **Open question** — no latency budget defined. Measure round-trip time once Pi serial integration is built. Consider: BNO055 read → IK compute → serial transmit → ESP32 receive → motor command → mechanical response.

---

## 8. Communication Interfaces & Protocols

### 8.1 I2C — AS5600 Magnetic Encoders (Coarse ESP32)
*Two SoftI2C buses, 100kHz, address 0x36 on each bus. Pull-up guidance. Minimum frequency floor.*

✅ **Can fill now**

### 8.2 I2C — BNO055 IMU (Fine ESP32, standalone mode)
*Single I2C bus, NDOF fusion mode. Calibration procedure. Register access (Euler angles).*

✅ **Can fill now** (from `imu_instrumentcontrol.py` inline driver)

### 8.3 I2C — Pi Device Bus
*BNO055 × 2 + ADS1115 × 2 on Pi bus 1. Address assignment table.*

✅ **Can fill now** (defined, not yet implemented)

### 8.4 UART — Feetech ST Bus (Fine ESP32 ↔ Waveshare ↔ STS3215 Servos)
*1Mbps, half-duplex. TX→TX/RX→RX wiring. Packet structure. Key registers.*

✅ **Can fill now** (partially) — expand with full packet format from Feetech datasheet.

### 8.5 USB Serial — Pi ↔ ESP32 (Coarse Arm)
*Command format, feedback format, baud rate, framing, error handling.*

⚠️ **Open question** — protocol not yet designed. This is a prerequisite for Pi integration.

### 8.6 USB Serial — Pi ↔ ESP32 (Fine Instrument, future)
*Currently not used — fine instrument ESP32 is standalone. If Pi takes over input processing, this interface will be needed.*

⚠️ **Open question** — may not be required if fine instrument stays ESP32-standalone.

---

## 9. Kinematics & Control Design

### 9.1 Coarse Arm — Joint Coordinate System
*Define the coordinate frame and positive direction for each of the 3 coarse axes.*

⚠️ **Open question** — no formal coordinate system or joint limits defined in the handoffs. Soft limits exist in firmware but are documented as config parameters, not in terms of physical joint angles.

### 9.2 Coarse Arm — PID Position Control
*Error definition, gain structure, EMA pre-filtering, deadband, integral clamp, output mapping to PWM frequency.*

✅ **Can fill now**

### 9.3 Coarse Arm — Joystick-to-Target Mapping
*Joystick position × MAX_DEGREES + home_deg = target angle. EMA applied before multiplication.*

✅ **Can fill now**

### 9.4 Fine Instrument — Inverse Kinematics (Cable Coupling)
*The 4-cable coupling matrix. Pitch, roll, yaw, grip mapped to Motor 4–7.*

```
Motor 4 = pitch
Motor 5 = roll
Motor 6 = 0.5·pitch + yaw + 0.5·grip
Motor 7 = 0.5·pitch + yaw − 0.5·grip
```

✅ **Can fill now** — note physical rationale: cables are shared across axes, requiring coordinated motion to maintain tension and geometry.

⚠️ **Open question** — the IK matrix is empirically derived for this specific EndoWrist design. Cable stretch, play, and wear will affect accuracy over time. No calibration or compensation procedure is defined.

### 9.5 Fine Instrument — Motor Synchronisation via `goal_time`
*All 4 servos commanded with identical `goal_time` so they arrive simultaneously regardless of distance.*

✅ **Can fill now** — explain why this is critical: preserves cable-coupling ratios at every intermediate point, not just arrival.

### 9.6 BNO055 — Delta-from-Home Control Mode
*Orientation at boot = neutral. Deviations drive the instrument. Scale factors per axis.*

✅ **Can fill now** — include rationale for delta mode vs. absolute mode (more Da Vinci-like, works better with clutch).

### 9.7 Clutch / Re-capture (Future)
*Planned: second button re-captures home mid-session. Architecture already supports this.*

⚠️ **Open question** — not yet implemented.

### 9.8 Forward Kinematics / Workspace Analysis

⚠️ **Open question** — no FK model or workspace envelope defined. For a DIY project this may be out of scope, but a rough workspace diagram would be useful for conference presentation.

---

## 10. Signal Processing

### 10.1 Exponential Moving Average (EMA) Filter

*Applied to all analog inputs before control logic. Formula: `ema = alpha * raw + (1 - alpha) * ema_prev`*

| Application | EMA_ALPHA | Effective time constant |
|-------------|-----------|------------------------|
| Coarse arm joystick | 0.2 | — |
| Fine instrument joystick | 0.2 | ~90ms at 50Hz |
| Fine instrument IMU | 0.2 | ~90ms at 50Hz |

✅ **Can fill now**

### 10.2 Deadband

| Application | Deadband | Units |
|-------------|----------|-------|
| Coarse arm (pitch/roll) | POSITION_DEADBAND_DEG | Degrees of position error |
| Fine instrument (joystick) | 0.04 | Normalized 0–1 |
| Fine instrument (IMU) | 1.5° | Degrees of wrist angle |

✅ **Can fill now**

### 10.3 Joystick ADC Characteristics
*Hall-effect joystick: low center noise, persistent high-frequency noise at full deflection. Carbon film pot: wiper noise profile differs. Current deadband and EMA settings tuned empirically for hall-effect.*

✅ **Can fill now**

### 10.4 BNO055 NDOF Fusion Mode
*Sensor fusion of accelerometer, gyro, magnetometer. Output: Euler angles (heading, pitch, roll). Magnetometer calibration requirement. Output rate 100Hz; firmware reads at 50Hz.*

✅ **Can fill now**

---

## 11. Safety & Error Handling

### 11.1 Hardware Safety — Linear Axis Limit Switch
*Normally-closed switch. Broken/disconnected wire reads as triggered (safe fail). Immediate hard stop on trigger; position counter zeroed.*

✅ **Can fill now** — document the safety rationale for NC wiring explicitly.

### 11.2 Soft Limits
*Linear axis: 0 steps (home) and LINEAR_MAX_STEPS (full extension). Enforced every tick in `handle_linear()`.*

⚠️ **Open question** — soft limits for Pitch and Roll axes are not defined in the handoffs. Joint angle limits should be specified to prevent mechanical over-travel.

### 11.3 Motor Enable Pin (ENA) — Not Connected
*Stepper driver ENA pins are not wired to GPIO. Motors cannot be de-energised under software control.*

⚠️ **Open question** — this is a known gap. Wiring ENA pins to GPIO would allow de-energising motors on fault, Ctrl+C, or e-stop. A `try/finally` in `main()` partially mitigates the Ctrl+C case. Hardware e-stop path to ENA pins is recommended before expanding to full 4-arm system.

### 11.4 Emergency Stop (Current)
*Software only. From Thonny REPL: `for p in [pitch_pwm, roll_pwm, linear_pwm]: p.duty(0)`. Not suitable for unattended operation.*

⚠️ **Open question** — no physical e-stop button exists. Recommend adding a normally-closed e-stop in series with motor driver power or ENA lines before public demonstration.

### 11.5 Foot Pedal / Clutch (Planned)
*Freeze all motion commands. Requires deciding between software-only path (Pi stops sending) and hardware path (pedal → ENA pins).*

⚠️ **Open question** — hardware path is strongly preferred for safety. Not yet implemented.

### 11.6 PWM Timer Safety (ESP32)
*`set_motor()` only calls `pwm.freq()` when frequency has changed, to prevent PWM timer resource leak. `linear_pwm.deinit()` required after homing to prevent spontaneous motor restart.*

✅ **Can fill now** — document these as firmware invariants that must not be changed.

### 11.7 I2C Fault Handling
*Bus scan on startup. If encoder scan returns empty, motor commands are blocked.*

⚠️ **Open question** — runtime I2C fault handling during normal operation is not documented. What happens if an encoder read fails mid-operation?

### 11.8 Stepper Missed Step / Position Drift
*Open-loop linear axis can accumulate drift, especially at full extension due to cable layer buildup. Periodic re-homing corrects this. No missed-step detection exists.*

✅ **Can fill now** — document as a known limitation with re-homing as the mitigation.

---

## 12. Configuration & Tuning Parameters

### 12.1 Coarse Arm Configuration Table

| Parameter | Current Value | Description |
|-----------|--------------|-------------|
| EMA_ALPHA | 0.2 | Joystick smoothing |
| PITCH/ROLL_KP | (tuned) | PID proportional gain |
| PITCH/ROLL_KI | (tuned) | PID integral gain |
| PITCH/ROLL_KD | (tuned) | PID derivative gain |
| PITCH/ROLL_POSITION_DEADBAND_DEG | (tuned) | PID skip zone |
| PITCH/ROLL_ENCODER_OFFSET_DEG | (tuned) | Physical home offset |
| LINEAR_JOG_RPS | 1.5 | Linear top jog speed |
| LINEAR_JOG_ACCEL_RPS2 | (tuned) | Accel ramp rate |
| LINEAR_JOG_DECEL_RPS2 | (tuned) | Decel ramp rate |
| LINEAR_HOMING_RPS | 0.5 | Homing approach speed |
| PITCH_MICROSTEPS | 16x | Must match DIP switches |
| ROLL_MICROSTEPS | 4x | Must match DIP switches |
| LINEAR_MICROSTEPS | 4x | Must match DIP switches |
| CONTROL_LOOP_MS | 40 | Main loop period (25Hz) |
| *_INVERT_DIR | False | Flip motor direction |

✅ **Can fill now** — populate actual KP/KI/KD/deadband values from firmware source.

### 12.2 Fine Instrument Configuration Table

| Parameter | Current Value | Description |
|-----------|--------------|-------------|
| DEADBAND | 0.04 | Joystick dead zone (normalized) |
| DEADBAND_DEG | 1.5° | IMU dead zone (degrees) |
| EMA_ALPHA | 0.2 | Input smoothing |
| JOYSTICK_HZ / IMU_HZ | 50 | Update rate |
| JOYSTICK_GOAL_TIME_MS | 20 | Servo move time budget |
| JOYSTICK_ACC | 0 | Servo acceleration ramp |
| SCALE_PITCH | 0.8 | IMU sensitivity (pitch) |
| SCALE_ROLL | 0.8 | IMU sensitivity (roll) |
| SCALE_YAW | 0.8 | IMU sensitivity (yaw) |
| SCALE_ROLL (joystick) | 0.0 | Disabled — pot disconnected |

✅ **Can fill now**

### 12.3 Tuning Procedures
*PID tuning order (EMA → KP → KD → KI → re-check EMA). Linear axis tuning (RPS and ramp rates only). IMU scale and deadband tuning.*

✅ **Can fill now** — well documented in primary handoff Section 6.

---

## 13. Testing & Verification

### 13.1 Subsystem Test Checklist — Coarse Arm
*I2C bus scan at startup, homing routine validation, joystick PID response, rocker jog with decel, limit switch function.*

✅ **Can fill now** — derive from working system behavior.

### 13.2 Subsystem Test Checklist — Fine Instrument (Joystick)
*Boot centering routine, joystick-to-servo response, IK coupling verification (pitch input should move motors 4, 6, and 7), deadband validation.*

✅ **Can fill now**

### 13.3 Subsystem Test Checklist — Fine Instrument (IMU)
*BNO055 calibration verification, delta-from-home baseline, axis scale and deadband tuning, grip button function, behavior under cable tension load.*

⚠️ **Open question** — `imu_instrumentcontrol.py` is written but not yet tested. This test plan must be executed before section can be marked complete.

### 13.4 Integration Test — Pi Serial Bridge
*End-to-end test: operator input → Pi IK → ESP32 motor command → encoder feedback.*

⚠️ **Open question** — Pi software not yet written. Test plan to be developed during implementation.

### 13.5 Load Testing / Cable Tension Testing
*Verify instrument IK accuracy under realistic cable tension load. Assess servo torque margins.*

⚠️ **Open question** — not yet performed. Critical before demo or expanded operation.

---

## 14. Known Limitations & Future Work

### 14.1 Current Limitations

| Item | Description | Mitigation |
|------|-------------|-----------|
| Linear position drift | Open-loop step counting; cable layer buildup at full extension | Periodic re-homing |
| GPIO fully committed (ESP32) | No pins available for expansion | I2C GPIO expander |
| No motor ENA pin control | Cannot de-energise motors in software | Wire ENA; add try/finally in main() |
| No physical e-stop | Software REPL command only | Add NC e-stop button |
| NC limit switch hard stop | No decel ramp before stop at high speed | Add decel-on-limit at higher speeds |
| I2C cable run sensitivity | Long runs cause read failures | Shielded cable, 100kHz, 2.2kΩ pull-ups |
| Roll pot disconnected | Roll axis disabled in joystick fine instrument firmware | Replace pot or use IMU |
| BNO055 IMU version not tested | `imu_instrumentcontrol.py` is written, not validated | Test per Section 13.3 |

✅ **Can fill now**

### 14.2 Planned Next Steps (Near-Term)
1. Wire BNO055 to fine instrument ESP32 (GPIO 21/22) and test `imu_instrumentcontrol.py`
2. Add grip button to GPIO 25
3. Tune SCALE and DEADBAND_DEG for feel under cable load
4. Mechanically couple instrument to servo discs; test under load
5. Begin Pi serial integration for coarse arm

✅ **Can fill now**

### 14.3 Future Work (Medium-Term)

| Item | Status |
|------|--------|
| Pi serial integration (coarse arm) | Not started |
| Pi-based BNO055 input system | Not started |
| ADS1115 joystick to Pi | Not started |
| Foot pedal / clutch | Not started |
| Second arm (clone of first) | Not started |
| 4-arm expansion | Not started |
| Vision-based hand tracking (MediaPipe) | Architecture ready, not pursued |

✅ **Can fill now** (status table)

⚠️ **Open question** — multi-arm coordination strategy (how two arms interact, whether there are cross-arm safety interlocks) is not defined.

---

## 15. Appendices

### Appendix A — GPIO Pin Assignment Table (Coarse Arm ESP32)
*Full 38-pin table from primary handoff.*

✅ **Can fill now**

### Appendix B — Wiring Diagrams
*Limit switch NC wiring, I2C bus topology, UART/ST bus wiring, motor driver connections.*

⚠️ **Open question** — no wiring diagrams exist yet. These should be created; Fritzing or hand-drawn schematics are sufficient for a DIY project.

### Appendix C — Actuator Iteration History
*Documents 28BYJ-48 unipolar → 28BYJ-48 bipolar TMC2209 → STS3215 design path. Preserved for conference presentation.*

✅ **Can fill now** — content from secondary handoff, Section "Design Iteration."

### Appendix D — Feetech ST Bus Protocol Reference
*Packet structure, key register addresses, `write_pos_timed` / `write_pos_ex` command formats.*

⚠️ **Open question** — compile from Feetech STS3215 datasheet + code inspection.

### Appendix E — Bill of Materials (BOM)
*Per-arm BOM for coarse and fine subsystems.*

⚠️ **Open question** — BOM does not exist in the handoffs. Should be created.

### Appendix F — Software File Index
*Maps each source file to the subsystem it controls.*

| File | Subsystem | Status |
|------|-----------|--------|
| `main.py` | Coarse arm ESP32 | Working |
| `instrumentcontrol.py` | Fine instrument (joystick) | Working |
| `imu_instrumentcontrol.py` | Fine instrument (IMU) | Written, not tested |
| Pi scripts | Coarse arm Pi integration | Not started |
| `deprecated/iteration1_*/` | Archived — 28BYJ-48 unipolar | Deprecated |
| `deprecated/iteration2_*/` | Archived — 28BYJ-48 bipolar TMC2209 | Deprecated |
| `deprecated/iteration3_*/` | Archived — STS3215 early tests | Deprecated |

✅ **Can fill now**

---

*End of SDD Outline — Project Salai*

# DIY Surgical Robot — Claude Code Handoff Document

## Project Overview

A DIY 2-arm surgical robot inspired by the Intuitive Da Vinci Si, built by a mechanical engineer for fun and resume purposes. Each arm has 7 DOF. The project is in early MVP stage — coarse arm control firmware is working, fine instrument control is functional for a single arm.

Future goal: expand to 4 arms.

---

## System Architecture

### The split-responsibility model

- **ESP32s** handle all real-time motor control. They are "dumb" motor controllers: receive position commands, execute them, return state. They do not run inverse kinematics or input processing.
- **Raspberry Pi 5** sits in the middle for the coarse arm — reads operator inputs (BNO055 IMUs, joysticks via ADS1115), runs inverse kinematics, sends position commands to ESP32s over USB serial, receives encoder feedback.

> **Note on fine instrument control:** The current instrument firmware (`instrumentcontrol.py`) runs fully standalone on an ESP32 — joystick is wired directly to the ESP32 ADC, and the Pi is not involved. If the Pi is later added for input (e.g. BNO055 orientation control), the IK and servo command logic will migrate there, and the ESP32 will become a thin serial-to-servo bridge.

### Data flow (coarse arm)

```
Operator inputs → Pi (I2C) → IK math → serial → ESP32 → motors
                                                ESP32 → encoders → serial → Pi
```

### Data flow (fine instrument — joystick version, stable baseline)

```
Joystick (ADC) → ESP32 → IK math → UART → Waveshare adapter → STS3215 servos
```

### Data flow (fine instrument — BNO055 version, next to test)

```
BNO055 (I2C) → ESP32 → delta-from-home → IK math → UART → Waveshare adapter → STS3215 servos
Grip button (GPIO) ──────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware — Per Arm

### Coarse positioning (3 DOF) — WORKING

- **Pitch axis**: NEMA 23 stepper + DM556T driver. Closed-loop PID. AS5600 magnetic encoder on dedicated I2C bus (GPIO 4/5 on ESP32).
- **Roll axis**: NEMA 17 stepper + DM542TE driver + 10:1 gearbox. Closed-loop PID. AS5600 encoder on second I2C bus (GPIO 21/22 on ESP32).
- **Linear axis**: NEMA 17 stepper + TB6600 driver. Open-loop (no encoder). Normally-closed limit switch at home/retracted position (GPIO 23). Cable-driven, 20mm spool diameter, ~220mm travel.

Full GPIO assignments and architecture details are in `primary_robot_arm_handoff.md`.

### Fine positioning / instrument control (4 DOF) — WORKING

- **Target instrument**: Intuitive Surgical EndoWrist (used) — cable-driven EndoWrist architecture
- **Motors**: STS3215-C047 smart servos × 4 (Feetech ST bus, 12V, 1:345 metal gearbox)
- **Adapter**: Waveshare Bus Servo Adapter (A) in UART mode
- **Controller file**: `instrumentcontrol.py`
- **Axes**: Pitch (Motor 4), Roll (Motor 5), Yaw/Grip 1 (Motor 6), Yaw/Grip 2 (Motor 7)

#### Wiring

| ESP32 pin | Waveshare adapter |
|---|---|
| GPIO 17 (UART2 TX) | TX |
| GPIO 16 (UART2 RX) | RX |
| GND | GND |

> **Important:** Wiring is TX→TX, RX→RX (not the usual crossover). The adapter handles half-duplex direction switching internally, presenting a standard full-duplex UART interface to the ESP32. Jumper cap must be on the **A** position.

Servo bus power: 12V to the adapter's servo terminals. Adapter logic: 5V via USB or 5V pin.

#### Servo IDs (assigned via Waveshare setup tool)

| ID | Axis |
|---|---|
| 4 | Pitch |
| 5 | Roll |
| 6 | Yaw / Grip 1 |
| 7 | Yaw / Grip 2 |

#### Inverse kinematics

The instrument uses a cable-driven EndoWrist architecture. Cables are shared across axes, requiring coordinated motor motion. For a (pitch, yaw, roll, grip) command:

```
Motor 4 = pitch
Motor 5 = roll
Motor 6 = 0.5·pitch + yaw + 0.5·grip
Motor 7 = 0.5·pitch + yaw − 0.5·grip
```

#### Motor synchronisation

All motors receive a `goal_time` command (not a speed command) so that each servo moves from its current position to its target in exactly the same duration, regardless of distance. This preserves the cable-coupling ratios at every intermediate point during a move — not just at arrival. `goal_time` is set to match the joystick update period (20ms).

#### Known performance

- Unloaded max output shaft speed at 12V: ~31 RPM (≈ 186°/sec)
- Servos retain absolute encoder position across power cycles — no homing routine required
- On boot, all motors drive to center (2048 counts) before handing off to the joystick loop

---

## Hardware — Operator Input

### Current (fine instrument — joystick version, `instrumentcontrol.py`)

- 3-axis hall-effect analog joystick wired directly into the fine instrument ESP32
  - Pitch: GPIO 34
  - Yaw: GPIO 35
  - Roll: GPIO 36 *(pot disconnected — SCALE_ROLL = 0.0 until replaced)*
- Roll axis disabled in firmware until a working pot/sensor is connected

### Next (fine instrument — BNO055 version, `imu_instrumentcontrol.py`)

- Adafruit BNO055 breakout connected via I2C
  - SDA: GPIO 21
  - SCL: GPIO 22
  - I2C address: 0x28 (ADDR pin to GND); use 0x29 if ADDR tied high
- Grip button: momentary NO push-button, GPIO 25 → GND (internal pull-up; hold to open jaw)
- DOF mapping:
  - BNO055 Pitch (forward/back tilt) → Instrument Pitch
  - BNO055 Roll (side tilt) → Instrument Roll
  - BNO055 Heading (wrist rotation, "tilt") → Instrument Yaw
  - Button hold → Grip open (25°); release → Grip closed (0°)
- Control mode: delta-from-home — orientation at boot is neutral; deviations from that drive the instrument

### Current (coarse arm)

- 2-axis analog joystick wired directly into coarse ESP32 (GPIO 34/35)
- 4 momentary trim switches on ESP32 to adjust home position

### Planned (Pi-based, both arms)

- **BNO055 IMU** × 2 (one per arm) — connected to Pi via I2C. Will be used to command fine instrument control.
- **Analog joystick** × 2 — connected to Pi via ADS1115 ADC breakout (I2C). Pi has no analog inputs so ADS1115 is required.

#### I2C layout on Pi

```
Pi I2C bus 1 (default, enabled):
  - BNO055 arm 1  (address 0x28)
  - ADS1115 arm 1 (address 0x48)
  - BNO055 arm 2  (address 0x29)  ← ADDR pin tied high
  - ADS1115 arm 2 (address 0x49)  ← ADDR pin tied to VCC
```

### Future input option (not yet pursued)

- Vision-based hand tracking via USB webcam + MediaPipe Hands (Python, runs on Pi). Architecture already supports this — swapping input source only requires changing Pi code.

### Foot pedal / clutch

- Not yet implemented
- Plan: normally-open switch read by Pi (or ESP32), freezes all motion commands when pressed
- E-stop should have hardware path to driver ENA pins as well as software path

---

## Signal Processing — Fine Instrument Joystick (`instrumentcontrol.py`)

| Parameter | Value | Notes |
|---|---|---|
| `DEADBAND` | 0.04 | 4% dead zone around center (normalized 0–1). Tuned empirically for this hall-effect joystick. |
| `EMA_ALPHA` | 0.2 | Exponential moving average smoothing. Lower = smoother/laggier, higher = more responsive/noisier. 0.2 gives ~90ms time constant at 50Hz. |
| `JOYSTICK_HZ` | 50 | Joystick read and servo command rate. |
| `JOYSTICK_GOAL_TIME_MS` | 20 | Time budget per servo move. Matched to update period for maximum responsiveness. |
| `JOYSTICK_ACC` | 0 | Servo acceleration ramp. 0 = instantaneous (appropriate given negligible instrument inertia). |

> The hall-effect joystick has very low noise at center but measurable high-frequency noise at full deflection. The double-deadband approach from the original stepper firmware was simplified to a single post-EMA deadband. If center drift appears, raise `DEADBAND` to 0.05–0.06. If full-deflection jitter returns, lower `EMA_ALPHA` toward 0.15.

## Signal Processing — Fine Instrument IMU (`imu_instrumentcontrol.py`)

| Parameter | Value | Notes |
|---|---|---|
| `DEADBAND_DEG` | 1.5° | Dead zone in degrees (not normalized). BNO055 NDOF mode has low drift at rest; if steady-state creep appears, raise to 2.0–3.0°. |
| `EMA_ALPHA` | 0.2 | Same smoothing as joystick version. May need to go lower (0.1–0.15) if BNO055 high-frequency noise causes jitter. |
| `SCALE_PITCH/ROLL/YAW` | 0.8 | Sensitivity: degrees of instrument motion per degree of wrist motion. 1.0 = full range BNO055 input maps to full instrument limit. Start at 0.8 and tune up/down for feel. |
| `IMU_HZ` | 50 | BNO055 read and servo command rate. BNO055 NDOF output rate is 100Hz; 50Hz read rate is fine. |
| `GOAL_TIME_MS` | 20 | Same as joystick version — matched to update period. |

> BNO055 in NDOF mode requires calibration (especially the magnetometer). On first power-up in a new location, rotate the sensor through several orientations for 20–30 seconds until `cal_sys` reads 3 in the serial log. Calibration is stored internally and survives power cycles. If the heading drifts slowly at rest, mag calibration is the likely cause — re-run the rotation procedure.

---

## Software Architecture

### Fine instrument firmware — joystick version (`instrumentcontrol.py`) — STABLE BASELINE

- MicroPython on ESP32, two async tasks: `joystick_task` (reads ADC, computes IK, updates targets) and `motor_task` (pushes targets to servos at 50Hz)
- Implements the Feetech ST bus protocol from scratch (~50 lines) — `write_pos_timed` for joystick commands, `write_pos_ex` for boot centering, `read_register`/`read_moving`/`wait_until_stopped` for encoder readback during boot
- STS3215 servos handle all internal position control using their own encoders — the firmware only sends target positions and time budgets

### Fine instrument firmware — BNO055 IMU version (`imu_instrumentcontrol.py`) — WRITTEN, NOT YET TESTED

- Branched from `instrumentcontrol.py` (2026-05-07). All servo, UART, and IK logic is identical.
- Joystick replaced by Adafruit BNO055 breakout (I2C, NDOF fusion mode). Minimal register-level BNO055 driver included in the file — no external library required.
- Two async tasks: `imu_task` (reads BNO055 Euler angles, computes delta-from-home, applies deadband + EMA + scale, runs IK, updates motor targets) and `motor_task` (unchanged from joystick version).
- Grip button (GPIO 25, PULL_UP) provides jaw open/close — hold to open, release to close.
- Key tuning knobs: `SCALE_PITCH/ROLL/YAW` (sensitivity), `DEADBAND_DEG` (dead zone in degrees; note this is different from the joystick version's normalized 0–1 deadband), `EMA_ALPHA` (smoothing).

### Coarse arm firmware (existing)

- Written in MicroPython (see `primary_robot_arm_handoff.md` for full architecture, bug history, and tribal knowledge)
- Main loop runs every 40ms
- Handles: joystick with EMA smoothing, closed-loop PID for pitch and roll, open-loop step counting for linear, homing routine, limit switch, rocker jog switches with accel/decel ramps

### Raspberry Pi (not yet written)

- Language: Python
- Responsibilities: read BNO055 IMUs (I2C), read joysticks via ADS1115 (I2C), run IK, send position commands to ESP32s over USB serial, receive encoder feedback, clutch/freeze logic
- Libraries likely needed: `smbus2` or `board`/`busio`, `pyserial`, `adafruit-circuitpython-bno055`, `adafruit-circuitpython-ads1x15`

---

## Current Status

| Component | Status |
|---|---|
| Coarse arm firmware (pitch, roll, linear) | Working |
| Fine instrument firmware (STS3215 servos, joystick) | Working — `instrumentcontrol.py` |
| Fine instrument firmware (STS3215 servos, BNO055 IMU) | Written, not yet tested — `imu_instrumentcontrol.py` |
| Raspberry Pi integration | Not started |
| BNO055 input system (Pi-based) | Not started |
| Joystick → ADS1115 → Pi | Not started |
| Foot pedal / clutch | Not started |
| Second arm (clone of first) | Not started |

---

## Immediate Next Steps

1. Wire up BNO055 breakout to fine instrument ESP32 (GPIO 21/22 for I2C) and test `imu_instrumentcontrol.py`
2. Add a grip button to GPIO 25 (momentary NO switch to GND; internal pull-up enabled)
3. Tune `SCALE_PITCH / SCALE_ROLL / SCALE_YAW` and `DEADBAND_DEG` in `imu_instrumentcontrol.py` to feel right under load
4. Mechanically attach instrument to servo coupling discs and test under cable tension load
5. Begin Pi serial integration for coarse arm

---

## Key Decisions Still Open

- **BNO055 control mode**: absolute orientation matching vs. delta/rate mode. Delta mode is more Da Vinci-like and works better with clutch. Recommend delta mode.
- **Fine instrument input migration**: joystick version (`instrumentcontrol.py`) is the stable baseline. BNO055 version (`imu_instrumentcontrol.py`) is the next path; it runs standalone on the same ESP32, no Pi required. Pi+BNO055 integration is a separate future step if latency or compute requirements demand it.
- **BNO055 control mode**: implemented as delta-from-home (as recommended). The orientation at boot becomes the neutral position. A full clutch (re-capture home mid-session via button) is not yet implemented but the architecture supports it — add a second button, capture home again in the imu_task loop when pressed.
- **Foot pedal wiring**: decide whether pedal signal goes to Pi (software only) or also has hardware path to driver ENA pins (safer).
- **Grip control**: `imu_instrumentcontrol.py` implements grip as a momentary push-button on GPIO 25 (hold to open). A squeeze sensor, hall-effect trigger, or EMG sensor could replace this for a more natural feel.

---
---

# Design Iteration — Instrument Actuator Selection

*This section documents the full development journey toward the STS3215 servo solution. It is preserved for conference presentation purposes. Each approach is presented as an experiment: what was tried, what was observed, and what drove the transition to the next iteration.*

---

## Iteration 1 — 28BYJ-48 Unipolar Steppers with ULN2003 Boards

### What we tried

The 28BYJ-48 is a small, cheap, highly-geared (1:64) unipolar stepper widely used in hobby robotics. Initial plan used four of these driven by ULN2003 Darlington array boards, the standard pairing for this motor. The firmware was to run on an ESP32 via MicroPython.

### What we observed

- The 28BYJ-48 in unipolar mode produces very low torque — insufficient to tension the EndoWrist cables under any meaningful load
- The ULN2003 board limits current per phase, compounding the torque problem
- The 1:64 gear ratio (producing ~4096 steps/rev in half-step mode) provides good angular resolution but the output torque was still inadequate
- The high gear ratio also means the motor moves slowly — top angular velocity at the output shaft was not competitive with the Da Vinci instrument motion

### Why we moved on

Torque and speed were both insufficient for cable-driven surgical instrument control. The ULN2003 boards provided no path to improvement — there is no meaningful way to increase torque from a unipolar 28BYJ-48 within its rated operating conditions.

*Code preserved at `deprecated/iteration1_ULN2003_28BYJ48/ULN2003instrumentcontrol.py`.*

---

## Iteration 2 — 28BYJ-48 Bipolar Conversion with TMC2209 UART Drivers

### What we tried

A known modification to the 28BYJ-48 cuts the red wire at the motor's PCB, converting it from unipolar (5-wire) to bipolar (4-wire) operation. In bipolar mode, the full coil is energised rather than half, approximately doubling holding torque. The motors were paired with TMC2209 stepper driver ICs, which offer UART configuration of microstepping (up to 1/256) and run current, and use a single shared PDN_UART line in a star topology.

This approach is fully documented in `deprecated/iteration2_TMC2209_28BYJ48_bipolar/TMC2209_instrumentcontrol.py`, which represents the final, working form of the 28BYJ-48 bipolar firmware. Supporting test scripts in the same folder: `bus_test.py` (per-driver UART validation), `uart_test.py` (interpolation comparison), `uart_loopback.py` (UART hardware check), `speed_test.py` (motor speed ramp), `steppertest.py` (basic back-and-forth). Architecture highlights:

- All four TMC2209 drivers share a single UART TX line (GPIO 17) using driver address assignment via MS1/MS2 pins
- Half-step + interpolation mode: 4096 steps/rev effective resolution
- Coordinated motion via Bresenham error accumulation — all four motors stepped proportionally to preserve cable-coupling ratios during moves
- Joystick input with EMA smoothing, home capture, and soft limits
- Alignment routine for seating coupling discs into the instrument shaft at startup

### What we observed

- Bipolar operation meaningfully improved holding torque over the ULN2003 approach
- The TMC2209 UART configuration worked reliably on the shared bus with address-based routing
- The Bresenham-based coordination kept the four motors proportionally synchronised throughout moves
- However, torque under real cable tension loads was still marginal — the 28BYJ-48 stall torque even in bipolar mode is low
- Motor heat was a concern at any sustained run current sufficient to hold cable tension
- The stepper approach inherently has no position feedback — missed steps accumulate silently, and the instrument drifts from commanded position without any indication in firmware

### Why we moved on

The 28BYJ-48 bipolar conversion represented a meaningful improvement over iteration 1 and produced a clean, functional firmware architecture, but the fundamental constraint — motor torque — remained a ceiling that could not be engineered around within the 28BYJ-48 platform. The lack of position feedback also meant any lost steps went undetected. A switch to a motor with closed-loop position control was warranted.

---

## Iteration 3 — STS3215 Smart Servos with Waveshare Bus Servo Adapter (A) — **Current Plan of Record**

### What we tried

The STS3215-C047 is a Feetech smart servo with a 1:345 metal gearbox, 12V rated operation, and a built-in magnetic encoder on the output shaft. Servos communicate over a shared half-duplex serial bus (Feetech ST protocol) and retain absolute position across power cycles. Four servos (IDs 4–7) are controlled via a Waveshare Bus Servo Adapter (A) connected to the ESP32 via UART at 1Mbps, with TX→TX, RX→RX wiring (the adapter handles direction switching internally).

### What we observed

- **Unloaded max output shaft speed at 12V: ~31 RPM (≈ 186°/sec)** — exceeding the original stepper target of ~175°/sec
- Absolute encoder retention across power cycles eliminated the need for a homing or alignment routine
- The ST bus protocol was implemented from scratch in MicroPython in ~50 lines, proving the protocol is tractable without an SDK
- Motor synchronisation is achieved via the servo's `goal_time` register: all four servos are commanded with the same time budget, so they execute proportional trajectories and arrive simultaneously regardless of individual travel distance — the internal encoder loop makes this timing accurate
- The firmware is substantially simpler than the stepper version: no Bresenham algorithm, no step/dir pulse generation, no TMC2209 UART driver configuration
- Hall-effect joystick noise at full deflection required EMA smoothing comparable to the original carbon film potentiometer approach — the noise profile is different (no wiper noise, but persistent high-frequency noise at full deflection) and a deadband of 4% with EMA_ALPHA = 0.2 was empirically tuned as a stable starting point

### Current status

Working. See `instrumentcontrol.py` and the plan-of-record sections above for full details.

*Early exploratory scripts preserved at `deprecated/iteration3_STS3215_early_tests/` (`main.py`, `sanitycheck.py`, `AprilSTS3215Test/sts3215.py`).*

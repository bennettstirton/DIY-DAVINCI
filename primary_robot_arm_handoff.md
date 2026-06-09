# 3-Axis Robot Arm Controller
## Developer Handoff Document
### ESP32 / MicroPython — armcontrol.py

---

# 1. Project Overview

This project is a DIY 3-axis robot arm controlled by an ESP32 microcontroller running MicroPython. It was developed iteratively through a single extended conversation, so this document captures architectural decisions, bug history, and tribal knowledge that is not present in the code comments.

The three axes are:

- **Pitch (Base):** NEMA 23 stepper + DM556T driver. Rotary joint. Closed-loop PID with AS5600 magnetic encoder via TCA9548A I2C multiplexer (channel 1).
- **Roll (Arm):** NEMA 17 stepper + TB6600 driver + 10:1 gearbox. Rotary joint. Closed-loop PID with AS5600 magnetic encoder via TCA9548A I2C multiplexer (channel 0).
- **Linear (End Effector):** NEMA stepper + TB6600 driver. Cable-driven spool (20mm diameter, 175mm travel). Position-controlled via optical quadrature encoder input (GPIO 16/17). Normally-closed limit switch at home end (GPIO 23).

Control inputs:

- **MPU6050 IMU (main input):** controls Pitch and Roll target angle by tilting the handheld controller. Shares the I2C bus (GPIO 21/22) alongside the TCA9548A/AS5600 encoders. Replaced the original 2-axis analog joystick (GPIO 34/35), which was removed due to reliability issues. See §5.3 for IMU wiring and calibration notes.
- **Trim 2-axis analog joystick:** jogs Pitch and Roll home position via ADC on GPIO 32/33. Speed scales with deflection. Jogging updates the rotary home positions, changing where the arm returns when the IMU is held at neutral.
- **Optical quadrature encoder:** drives the linear axis. Rotating the encoder extends/retracts the end effector. Input on GPIO 16 (A) and 17 (B).

---

# 2. GPIO Pin Assignments

All 38 ESP32 pins are now fully committed. Do not reassign without a full audit. Pins 6–11 are internal flash and must never be used.

| GPIO Pin | Purpose |
|----------|---------|
| 14 | Pitch STEP (PWM output to DM556T) |
| 27 | Pitch DIR (direction output to DM556T) |
| 26 | Roll STEP (PWM output to TB6600) |
| 25 | Roll DIR (direction output to TB6600) |
| 13 | Linear STEP (PWM output to TB6600) |
| 15 | Linear DIR (direction output to TB6600) |
| 35 | **Unused** (was main joystick Pitch ADC — joystick replaced by MPU6050 IMU) |
| 34 | **Unused** (was main joystick Roll ADC — joystick replaced by MPU6050 IMU) |
| 32 | Trim joystick — Pitch trim ADC |
| 33 | Trim joystick — Roll trim ADC |
| 16 | Optical encoder channel A (INPUT_PULLUP) |
| 17 | Optical encoder channel B (INPUT_PULLUP) |
| 18 | Unused (was Pitch rocker FWD) |
| 19 | Unused (was Pitch rocker BWD) |
| 23 | Linear limit switch (INPUT_PULLUP, normally closed — reads 0 at rest, 1 when triggered) |
| 21 | I2C SDA — shared bus: TCA9548A (→ both AS5600s) + MPU6050 IMU |
| 22 | I2C SCL — shared bus: TCA9548A (→ both AS5600s) + MPU6050 IMU |
| 4  | E-stop button (NC, INPUT_PULLUP — open = triggered, fail-safe wiring) |
| 5  | Re-arm button (NO, INPUT_PULLUP — press after releasing e-stop to resume) |
| 2  | Onboard blue LED (OUTPUT — blinks 1Hz while armcontrol.py main loop is running) |
| 0  | BOOT button — used by boot.py for standalone launch; also skips linear homing in armcontrol.py |

---

# 3. Code Architecture

## 3.1 Main Loop Routing

The main loop runs every 40ms (CONTROL_LOOP_MS). Each tick it evaluates three routing decisions:

- **handle_jogging(trim_p, trim_r):** Called when the trim joystick is deflected on either axis, OR when either of those axes is still in its post-release decel ramp. Speed scales with trim deflection magnitude. On decel end, home is updated to `actual - joy_now * MAX_DEGREES` so the arm stays put when the main joystick is centred.
- **handle_main_input(dt_ms):** Called when no rotary jog is active and no rotary decel is in progress. Reads the IMU, applies EMA smoothing, and runs PID for both Pitch and Roll. Does nothing with the linear axis.
- **handle_linear():** Called EVERY TICK, unconditionally, after the jogging/joystick decision. This is critical — linear is fully independent of the rotary routing. It handles its own ramp, soft limits, and limit switch check.

> ⚠ The linear axis was originally inside handle_jogging() but was extracted to handle_linear() to fix a bug where pressing the linear rocker during joystick mode caused continuous uncontrolled motion. Never merge it back into handle_jogging().

## 3.2 Per-Axis Decel Flags

Each axis has a dedicated boolean flag (pitch_decelerating, roll_decelerating, linear_decelerating). When a rocker is released, the flag is set to True and the ramp-down continues in the handler until the frequency reaches MIN_FREQ, at which point the flag is cleared. This allows each axis to decelerate independently without blocking joystick control on the other axes.

> ⚠ This was a deliberate fix from the original code, which used a single any_jog_active check based on current_freq > 0. That caused joystick control to be blocked on all axes until every motor had fully stopped.

## 3.3 PID Control (Pitch and Roll)

Both rotary axes use a standard PID loop. The IMU tilt (normalised to [-1, +1] over IMU_PITCH/ROLL_MAX_TILT_DEG) is multiplied by MAX_DEGREES and added to home_deg to produce a target angle. The encoder reads the actual angle. The error drives the PID, whose output is mapped to a PWM frequency and direction.

- **EMA smoothing is applied to the IMU read BEFORE the target angle is calculated.** This is essential — without it, accelerometer noise gets multiplied by MAX_DEGREES and causes constant motor buzzing at rest. EMA_ALPHA = 0.2 is the current setting (lower = smoother/more sluggish, higher = more responsive/noisier).
- **Deadband:** If the error is within the deadband, the motor stops and the PID is skipped entirely (using an else block). This is important — early versions fell through the deadband check and ran the PID anyway, causing integral accumulation at rest.
- **Integral clamp:** KI_CLAMP prevents integral windup. The integral is also reset to zero when the deadband is entered or when jogging ends.

## 3.4 Encoder-Driven Linear Axis

The linear axis is driven by an optical quadrature encoder (handle_encoder_linear()). The encoder acts as the input device — rotating it commands the linear stepper to extend or retract. This is position-based, not velocity-based.

- **encoder_position_steps** tracks the absolute input device position in arm-equivalent steps. It is never clamped — it follows the encoder freely past the arm's physical limits.
- **commanded_steps** = encoder_position_steps clamped to [0, LINEAR_MAX_STEPS]. The arm only moves to reach commanded_steps.
- **Overshoot behaviour:** If the encoder is driven past the arm's limit, the arm stops at the limit but encoder_position_steps keeps accumulating. When the encoder reverses, the arm does not move until it has wound back through the overshoot. This ties arm position to a real encoder position.
- **ENCODER_SCALE:** Gear ratio between encoder and stepper. 1.0 = 1 encoder rev → 1 stepper rev. Start at 0.5–0.75 and increase for coarser/faster response.
- **Soft limits:** Motion is blocked at 0 steps (home) and LINEAR_MAX_STEPS (full extension).
- **Hard limit (normally closed switch):** Triggers when value() == 1 (circuit opens). Stops the motor immediately, re-zeros both position counters.
- **Cable spool note:** The spool has a 20mm diameter giving ~62.83mm/rev circumference. Steps/mm drifts slightly near full extension as cable layers accumulate on the spool. This is a known physical limitation of cable-drive.

## 3.5 Homing Sequence

Homing runs automatically at startup. There is a 3-second skip window at the start:

- **With laptop connected:** press any key + Enter in the MicroPico terminal to skip.
- **Standalone (no laptop):** hold the BOOT button during the 3-second countdown to skip. Two LED flashes confirm the skip was registered.

If not skipped, the homing sequence is:

1. Check if limit switch is already triggered — if so, skip drive phase.
2. Drive in retract direction at LINEAR_HOMING_FREQ until limit switch triggers.
3. Stop motor, zero position counter.
4. Back off 0.5 revolutions in extend direction using step-by-step timing (not PWM) to guarantee exact distance.
5. Call linear_pwm.deinit() and reinitialise the PWM object clean. Set linear_current_freq = 0.

> ⚠ The deinit/reinit step is critical. Without it, the PWM timer retains its last configured frequency after stop_motor() sets duty=0. This caused the motor to spontaneously resume movement when the main loop started. linear_pwm must be declared global inside home_linear_axis() for this to work.

---

# 4. Bug History (All Fixed)

These bugs were present in the original code and have been resolved. Do not reintroduce them.

| Bug | Description / Fix |
|-----|------------------|
| **EMA disabled** | ema_joy_pitch and ema_joy_roll were never updated — joystick did nothing. EMA reads are now applied every tick before target angle calculation. |
| Deadband fall-through | The deadband if-block stopped the motor but didn't prevent the PID from running. Now uses if/else so PID is fully skipped inside the deadband. |
| **global declarations missing** | pitch_pid_last_error and ema_joy_pitch were assigned in handle_jogging() without being declared global, silently creating local variables. Both are now in the global declaration. |
| Decel blocking joystick | Original code used current_freq > 0 to decide whether to call handle_jogging(). This blocked joystick control during the decel ramp. Now uses per-axis decel flags. |
| **Linear inside handle_jogging()** | Linear rocker presses during joystick mode caused continuous uncontrolled motion. Linear was extracted to its own handle_linear() function called every tick. |
| PWM timer not cleared after homing backoff | After homing, residual PWM frequency caused motor to restart unexpectedly. Fixed with linear_pwm.deinit() and reinit. |
| **NC limit switch logic** | Original code had value()==0 for trigger (normally open logic). Corrected to value()==0 meaning not triggered for NC wiring — limit_switch_triggered() returns True when value()==0. |
| Dead config variable | JOG_FREQ was defined but never used. Removed. |

---

# 5. Hardware Notes

## 5.1 Limit Switch Wiring (Normally Closed)

The linear limit switch is wired normally closed (NC): one terminal to GPIO 23, other terminal to GND, with the ESP32 internal pull-up enabled. At rest (switch not triggered), the circuit is complete and the pin reads LOW (0). When the carriage presses the actuator, the circuit opens and the pin reads HIGH (1) due to the pull-up.

In the code: limit_switch_triggered() returns True when linear_limit.value() == 0 (circuit open = carriage pressing switch).

> ⚠ NC is safer than NO — a broken or disconnected wire reads as triggered (value stays HIGH via pull-up), which causes the code to stop motion rather than ignore the fault.

## 5.2 I2C Bus Configuration

Both AS5600 encoders share a single SoftI2C bus (GPIO 21/22) via a **TCA9548A I2C multiplexer** (address 0x70). The TCA9548A selects which AS5600 is active before each read — Roll on channel 0, Pitch on channel 1. This replaced the earlier two-bus approach (GPIO 4/5 and 21/22) to free up GPIO 4/5 for the e-stop and re-arm buttons.

The bus is configured at 100kHz.

- **Why 100kHz:** The encoder signals run over extended cable runs. At 400kHz, high cable capacitance rounds the signal edges and causes read failures. 100kHz gives the edges more time to settle. If reads fail again after a cable change, try dropping to 50kHz before investigating other causes.
- **Pull-up resistors:** The AS5600 breakout boards likely have 10kΩ pull-ups. For long cable runs, adding 2.2kΩ or 1kΩ pull-ups from SDA/SCL to 3.3V at the ESP32 end can significantly improve reliability. Try this before reducing frequency further.
- **Minimum safe I2C frequency:** ~50kHz. Below this, AS5600 internal timeout behaviour becomes unpredictable. Do not go below 10kHz under any circumstance.
- **Ethernet cable warning:** Ethernet cable is not suitable for I2C over distance. Its high inter-conductor capacitance rounds signal edges. Use shielded twisted pair (STP) cable, or consider I2C bus extender chips (PCA9600, LTC4311) for runs longer than ~0.5m.
- **Intermittent read failures:** Three Roll AS5600 read failures were observed in a single test session, likely vibration-induced contact issues on the cable to the TCA9548A. Each failure stops the roll motor for one tick. Check all connectors under the multiplexer if failures recur.

## 5.3 MPU6050 IMU (Main Input)

The MPU6050 (GY-521 breakout) replaced the analog joystick as the pitch/roll input on 2026-06-08. The original joystick was removed due to reliability and noise issues; the IMU provides gravity-referenced, non-drifting tilt angles.

**Wiring:** VCC → 3.3V, GND → GND, SDA → GPIO 21, SCL → GPIO 22, AD0 → unconnected (defaults to address 0x68). XDA/XCL/INT unused.

**I2C bus sharing:** The MPU6050 sits directly on the same bus as the TCA9548A (0x70). No mux needed — its address (0x68) doesn't collide with anything on the bus. The physical wires that previously carried the joystick analog signals (through the control station ethernet cable) now carry this I2C bus to the IMU.

**Pull-up resistors:** 2.2kΩ–1kΩ pull-ups on SDA and SCL at the ESP32 end are recommended for the long cable run from control station to ESP32. The GY-521 has onboard 10kΩ pull-ups, but these are too weak over extended cable. Standard 100kHz bus speed.

**How angle derivation works:** The code uses only the accelerometer — not the gyroscope. It computes pitch and roll from the gravity vector direction using `atan2`. This gives absolute, gravity-referenced angles that do not drift over time. The gyroscope is not used. During fast motion, accelerometer noise increases; the EMA filter (EMA_ALPHA) mitigates this.

**Calibration:** A one-time calibration offset is baked into CONFIG (`IMU_PITCH_OFFSET_DEG`, `IMU_ROLL_OFFSET_DEG`). To re-measure: run armcontrolsetup.py Test D with zeroing skipped, note the raw readings at working-neutral mount position, and enter the corrections needed to bring them to 0. Current values: pitch 0.0°, roll -90.0° (the IMU's physical mount orientation produces ~+90° raw roll at neutral).

**Control mapping:** IMU tilt is normalised to [-1, +1] over `IMU_PITCH/ROLL_MAX_TILT_DEG` (currently 30°), with a `IMU_DEADBAND_PITCH/ROLL_DEG` (currently 1.5°) deadband. The normalised value is then multiplied by `PITCH/ROLL_MAX_DEGREES` to produce the arm target angle — identical to how the old joystick worked.

## 5.4 PWM and Stepper Drivers

Motors are driven by toggling a PWM signal on the STEP pin. The duty cycle is fixed at 512/1023 (50%). Only the frequency changes to control motor speed. Direction is set via the DIR pin.

- **ESP32 PWM timer leak:** Calling pwm.freq() every tick even when the frequency hasn't changed causes a PWM timer resource leak on the ESP32 and will eventually crash the firmware. The set_motor() function only calls pwm.freq() when the frequency has actually changed (freq != prev_freq).
- **Microstep DIP switches:** PITCH_MICROSTEPS and ROLL_MICROSTEPS/LINEAR_MICROSTEPS in CONFIG must match the physical DIP switch settings on each driver. Mismatch causes wrong speeds and positions. Current settings: Pitch=16x, Roll=4x, Linear=4x.
- **Stopping motors from REPL:** After Ctrl+C in VSCode (MicroPico extension), PWM timers keep running. Stop all motors from the REPL with: `for p in [pitch_pwm, roll_pwm, linear_pwm]: p.duty(0)`
- **Direction inversion:** PITCH_INVERT_DIR, ROLL_INVERT_DIR, LINEAR_INVERT_DIR in CONFIG. Flip to True if a motor runs the wrong way. Do not rewire.

---

# 6. Tuning Guide

## 6.1 PID Tuning Order (Rotary Axes)

Always tune in this order. Each step depends on the previous one being stable.

1. **Step 1 — EMA first:** Tune EMA_ALPHA before touching PID. Get the signal clean before the PID sees it. Increase toward 1.0 for more responsiveness, decrease toward 0.0 for more smoothing. Target: motor sits quietly at rest, responds crisply to deliberate stick movement.
2. **Step 2 — KP:** Set KI=0, KD=0. Raise KP until the axis oscillates, then back off ~20%.
3. **Step 3 — KD:** Add damping to kill overshoot and jerkiness. Raise until motion is smooth and the axis settles cleanly. Too much KD causes sluggishness and high-frequency buzz.
4. **Step 4 — KI (only if needed):** Only add if the axis consistently stops just short of target. Small values only. Too much causes slow lazy oscillation.
5. **Step 5 — Re-check EMA:** A well-tuned KD absorbs some noise, so a slightly higher EMA_ALPHA may now be tolerable.

## 6.2 Linear Axis Tuning

The linear axis is open-loop and rocker-only. The only parameters to tune are:

- **LINEAR_JOG_RPS:** Top speed while rocker is held. Currently 1.5 rev/sec.
- **LINEAR_JOG_ACCEL_RPS2 / LINEAR_JOG_DECEL_RPS2:** Acceleration and deceleration rates in rev/sec². Edit only these — the derived HZ values are calculated automatically. Higher = snappier, lower = smoother.
- **LINEAR_HOMING_RPS:** Speed during homing approach. Keep conservative — the carriage hits the switch at this speed. Currently 0.5 rev/sec.

---

# 7. Current Status

## 7.1 Working Well

- Pitch and Roll closed-loop PID with encoder feedback
- MPU6050 IMU tilt control of Pitch and Roll with EMA smoothing
- Trim joystick jogging for Pitch and Roll with accel/decel ramps and home update
- Optical encoder input driving linear axis position control
- Linear homing routine with backoff and position zeroing; optional skip via BOOT button or keypress
- Linear soft limits and NC limit switch hard stop
- Per-axis independent decel so axes don't block each other
- Debug printout showing target/actual/error for all axes at 10Hz
- Onboard blue LED blinks 1Hz while main loop is running (visual heartbeat)
- E-stop (GPIO 4, NC) with ISR — kills all PWM immediately; re-arm button (GPIO 5) resumes
- boot.py: standalone launch via BOOT button press within 5s of power-on

## 7.2 Known Limitations / Future Work

- **PID tuning incomplete:** ROLL_KP reduced from 200 → 50 to fix sign-flip bouncing. Further tuning needed. See Section 6.1.
- **Intermittent Roll AS5600 read failures:** Seen in testing, likely vibration-induced connector issue. Each failure causes a 1-tick motor stop.
- **No hard stop protection at BWD high speed:** The limit switch causes an immediate hard stop rather than a decel ramp. At current speeds this is fine; consider decel-on-limit if speed is increased.
- **I2C cable runs:** Ethernet cable was tried and failed. Currently on shorter runs at 100kHz. Proper shielded cable and/or I2C bus extenders recommended for any future cable lengthening.
- **No motor enable pin control:** Driver ENA pins are not wired to GPIO. A hard reset leaves motors running until Ctrl+C or REPL cleanup.

---

# 8. Quick Reference

## 8.1 Key Config Parameters

| Parameter | Notes |
|-----------|-------|
| EMA_ALPHA | IMU input smoothing. 0.2 currently. Lower = smoother, higher = more responsive. |
| IMU_PITCH/ROLL_OFFSET_DEG | One-time calibration offsets. Measured via armcontrolsetup.py Test D (run with zeroing skipped, record raw reading at working-neutral, enter correction). |
| IMU_PITCH/ROLL_MAX_TILT_DEG | Physical tilt (degrees) that maps to full ±1.0 command — equivalent to joystick full deflection. Currently 30.0. |
| IMU_DEADBAND_PITCH/ROLL_DEG | Tilt within this many degrees of neutral reads as zero. Currently 1.5. |
| PITCH/ROLL_MAX_DEGREES | IMU full deflection → this many degrees of arm target angle change. Currently 15.0. |
| PITCH/ROLL_KP/KI/KD | PID gains. ROLL_KP=50, PITCH_KP=50. See tuning guide. |
| PITCH/ROLL_POSITION_DEADBAND_DEG | Dead zone around target angle. Motor stops and PID skips if error is within this band. |
| ENCODER_SCALE | Gear ratio between optical encoder and linear stepper. 0.75 currently. |
| LINEAR_HOMING_RPS | Homing approach speed. Keep conservative — carriage hits switch at this speed. |
| PITCH/ROLL_INVERT_DIR, LINEAR_INVERT_DIR | Flip motor direction without rewiring. |
| LINEAR_ENCODER_INVERT, LINEAR_POSITION_INVERT | Independently flip encoder input direction and position counter direction. |
| ROLL/PITCH_ENCODER_OFFSET_DEG | Physical home offset. Jog to desired zero, note printed angle, enter here. |
| CONTROL_LOOP_MS | Main loop period in ms. Currently 40ms (25Hz). Decreasing improves PID response but increases I2C read load. |

## 8.2 Emergency Stop from VSCode REPL (MicroPico)

```python
for p in [pitch_pwm, roll_pwm, linear_pwm]: p.duty(0)
```

## 8.3 I2C Bus Scan (run at startup automatically)

```python
# Scan the main bus — should return [0x68, 0x70] (MPU6050 + TCA9548A)
i2c.scan()
# Scan each AS5600 through the mux
tca_select(0); i2c.scan()   # Roll channel — should return [0x36]
tca_select(1); i2c.scan()   # Pitch channel — should return [0x36]
```

If 0x68 (MPU6050) is missing: check VCC→3.3V, GND→GND, SDA/SCL to GPIO 21/22, pull-up resistors on long cable run.
If 0x70 (TCA9548A) is missing: check its SDA/SCL connections and VCC.
If AS5600 missing on a channel: check TCA9548A connector for vibration-induced contact issues.

## 8.4 Standalone Operation (boot.py)

`boot.py` lives on the ESP32 root. On power-on it waits up to 5 seconds for the BOOT button press. If pressed: LED flashes once, armcontrol.py starts. If not pressed: drops to REPL (MicroPico can then connect normally).

When armcontrol starts, it runs a 3-second homing skip window: hold BOOT (standalone) or press any key + Enter (laptop) to skip linear homing. Two LED flashes confirm the skip.

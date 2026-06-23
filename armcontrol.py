# =============================================================================
# 3-Axis Robot Arm Controller
# Target: ESP32 running MicroPython
#
# Hardware:
#   - Pitch Axis (Base):    NEMA 23 + DM556T driver  (STEP/DIR)
#   - Roll Axis  (Arm):     NEMA 17 + TB6600 driver  (STEP/DIR)
#                           + AS5600 magnetic encoder (I2C, absolute 12-bit)
#   - Linear Axis (Extend): NEMA ?? + TB6600 driver  (STEP/DIR)
#                           Open-loop, cable-driven spool (20mm dia, 175mm travel)
#                           + limit switch at home end (GPIO23, normally closed)
#   - Main input: MPU6050 IMU — tilt the controller to command pitch/roll.
#                 Shares the AS5600 I2C bus (GPIO 21/22). Replaced the original
#                 2-axis analog joystick (GPIO 34/35), which proved unreliable.
#   - Trim joystick: 2-axis analog (pitch + roll home jog, via ADC, GPIO 32/33)
#   - Optical quadrature encoder for linear axis
#
# Control logic:
#   - Pitch + Roll: Closed-loop PID using AS5600 encoders.
#                   IMU tilt (relative to a calibrated neutral) commands target
#                   angle, same as the joystick did. Trim joystick jogs and
#                   updates home position (speed scales with deflection magnitude).
#   - Linear:       Open-loop, encoder-commanded. Rotating the optical encoder
#                   drives the linear stepper at a scaled rate.
#                   ENCODER_SCALE sets the gear ratio (encoder revs → stepper revs).
#                   Soft travel limits prevent motion beyond [0, MAX_STEPS].
#                   Limit switch (normally closed) stops retract immediately.
#   - Homing:       Linear axis homes automatically at startup — drives toward
#                   limit switch, then backs off. Wait for "Ready." before use.
#   - All tunable parameters are in the CONFIG section below.
# =============================================================================

from machine import Pin, ADC, PWM, SoftI2C, disable_irq, enable_irq
import math
import time
import sys
import select
from config import *
from mcp23017 import MCP23017

BOLD  = "\033[1m"
RESET = "\033[0m"

# =============================================================================
# HARDWARE INITIALISATION
# =============================================================================

_status_led = Pin(2, Pin.OUT)   # onboard blue LED — blinks while armcontrol is running

pitch_dir  = Pin(PITCH_DIR_PIN,  Pin.OUT, value=0)
roll_dir   = Pin(ROLL_DIR_PIN,   Pin.OUT, value=0)
linear_dir = Pin(LINEAR_DIR_PIN, Pin.OUT, value=0)

pitch_pwm  = PWM(Pin(PITCH_STEP_PIN),  freq=1000, duty=0)
roll_pwm   = PWM(Pin(ROLL_STEP_PIN),   freq=1000, duty=0)
linear_pwm = PWM(Pin(LINEAR_STEP_PIN), freq=1000, duty=0)

trim_joy_x = ADC(Pin(TRIM_JOY_X_PIN))
trim_joy_y = ADC(Pin(TRIM_JOY_Y_PIN))
trim_joy_x.atten(ADC.ATTN_11DB)
trim_joy_y.atten(ADC.ATTN_11DB)

# Optical encoder — inputs only, no pull-up (encoder drives the line actively)
encoder_a = Pin(ENCODER_A_PIN, Pin.IN, Pin.PULL_UP)
encoder_b = Pin(ENCODER_B_PIN, Pin.IN, Pin.PULL_UP)

# E-stop: normally closed button pulls pin LOW at rest; opening (press or wire break)
# lets pull-up take it HIGH, triggering the ISR.
estop_pin = Pin(ESTOP_PIN, Pin.IN, Pin.PULL_UP)
rearm_pin = Pin(REARM_PIN, Pin.IN, Pin.PULL_UP)

# Single I2C bus → TCA9548A → both AS5600s, plus the MPU6050 IMU (GPIO 21/22).
# The IMU sits directly on the bus (not behind the mux) — its address (0x68)
# doesn't collide with the TCA9548A (0x70) or the AS5600s (0x36, mux-selected).
i2c = SoftI2C(sda=Pin(AS5600_SDA_PIN), scl=Pin(AS5600_SCL_PIN),
              freq=AS5600_I2C_FREQ)

# Limit/crash switches — MCP23017 GPIO expander on the same I2C bus (see config.py
# for the PAx bit map). INTA/INTB are left unconnected: read_limit_switches()
# polls mcp.read_gpio() once every 40ms tick regardless, which is already fast
# enough for mechanical limit switches, so there's no wiring or ISR to add.
mcp = MCP23017(i2c, MCP23017_ADDR)

# =============================================================================
# OPTICAL ENCODER STATE + INTERRUPT HANDLER
# =============================================================================
#
# How quadrature decoding works:
#   The encoder produces two pulse trains (A and B) that are 90° out of phase.
#   By looking at the combined state of A and B every time either pin changes,
#   and comparing to the previous state, we can determine direction.
#   The lookup table below maps (prev_state << 2 | current_state) → +1, -1, or 0.
#   This is called "4x decoding" because it counts every edge on both channels,
#   giving 4 * PPR counts per revolution.
#
# The ISR (interrupt service routine) updates encoder_count.
# The main loop reads and resets encoder_count each tick.

_ENC_TABLE = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
_enc_state  = 0
encoder_count = 0   # accumulated counts since last main-loop read; modified by ISR

def _encoder_irq(pin):
    """Interrupt handler — fires on every edge of encoder A or B."""
    global _enc_state, encoder_count
    a = encoder_a.value()
    b = encoder_b.value()
    _enc_state    = ((_enc_state << 2) | (a << 1) | b) & 0x0F
    encoder_count += _ENC_TABLE[_enc_state]

# Attach the same handler to both channels, triggering on rising AND falling edges.
encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)
encoder_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)

# =============================================================================
# E-STOP
# =============================================================================

estop_active  = False   # set by ISR; main loop checks this every tick
estop_handled = False   # True once the main loop has run the full stop sequence

def _estop_isr(_):
    global estop_active
    # Unconditional stop — the 100nF cap on GPIO4 already filters sub-millisecond
    # EMI glitches at the hardware level, so any rising edge reaching this ISR is real.
    # Do NOT check _pin.value() here: the RC rise time (~5 ms) means the pin may still
    # read LOW when the ISR fires, which would silently discard a genuine e-stop press.
    estop_active = True
    pitch_pwm.duty(0)
    roll_pwm.duty(0)
    linear_pwm.duty(0)

estop_pin.irq(trigger=Pin.IRQ_RISING, handler=_estop_isr)

# =============================================================================
# AXIS LIMIT / CRASH SWITCHES (MCP23017)
# =============================================================================
#
# PA0 (linear home) is read every tick but is NOT a latched fault — it's the
# existing homing reference / retract backstop, which already auto-resyncs
# position in handle_encoder_linear() and home_linear_axis(). PA1-PA5 are new
# crash detectors: tripping one latches axis_fault[...] = True, which halts
# only that axis (PID/jog/encoder-drive) until RE-ARM clears it.

axis_fault = {"pitch": False, "roll": False, "linear": False}


def read_limit_switches():
    """Poll the MCP23017 once this tick and latch any new crash-switch trips.
    Call once per main-loop iteration, before the drive functions run.

    While a fault is latched, this also forces that axis's current_freq/
    decelerating state to zero every tick — not just on the initial trip —
    so the jog/PID decel ramps in handle_jogging()/_run_pid() can't re-energize
    the PWM at a decaying frequency after stop_motor() already silenced it."""
    global pitch_current_freq, roll_current_freq, linear_current_freq
    global pitch_decelerating, roll_decelerating, linear_decelerating
    _mcp_bits = mcp.read_gpio()

    if (_mcp_bits >> MCP_LINEAR_EXTEND_BIT) & 1 and not axis_fault["linear"]:
        axis_fault["linear"] = True
        print("!!! LINEAR EXTEND LIMIT TRIPPED — linear axis halted. Press RE-ARM to clear. !!!")
    if (_mcp_bits >> MCP_PITCH_MIN_BIT) & 1 and not axis_fault["pitch"]:
        axis_fault["pitch"] = True
        print("!!! PITCH MIN LIMIT TRIPPED — pitch axis halted. Press RE-ARM to clear. !!!")
    if (_mcp_bits >> MCP_PITCH_MAX_BIT) & 1 and not axis_fault["pitch"]:
        axis_fault["pitch"] = True
        print("!!! PITCH MAX LIMIT TRIPPED — pitch axis halted. Press RE-ARM to clear. !!!")
    if (_mcp_bits >> MCP_ROLL_MIN_BIT) & 1 and not axis_fault["roll"]:
        axis_fault["roll"] = True
        print("!!! ROLL MIN LIMIT TRIPPED — roll axis halted. Press RE-ARM to clear. !!!")
    if (_mcp_bits >> MCP_ROLL_MAX_BIT) & 1 and not axis_fault["roll"]:
        axis_fault["roll"] = True
        print("!!! ROLL MAX LIMIT TRIPPED — roll axis halted. Press RE-ARM to clear. !!!")

    if axis_fault["pitch"]:
        stop_motor(pitch_pwm)
        pitch_current_freq = 0
        pitch_decelerating = False
    if axis_fault["roll"]:
        stop_motor(roll_pwm)
        roll_current_freq = 0
        roll_decelerating = False
    if axis_fault["linear"]:
        stop_linear_motor()
        linear_current_freq = 0
        linear_decelerating = False


def clear_axis_faults():
    """Clear latched crash-switch faults and resync PID/jog state for the
    axes that were faulted, same pattern as the e-stop RE-ARM resync."""
    global pitch_target_deg, pitch_home_deg, ema_joy_pitch
    global pitch_pid_integral, pitch_pid_last_error, pitch_pid_last_actual, pitch_current_freq, pitch_decelerating
    global roll_target_deg, roll_home_deg, ema_joy_roll
    global roll_pid_integral, roll_pid_last_error, roll_pid_last_actual, roll_current_freq, roll_decelerating
    global linear_current_freq, linear_decelerating

    if axis_fault["pitch"]:
        p_angle = read_pitch_angle_deg()
        _p, _ = read_imu_commands()
        if p_angle is not None:
            ema_joy_pitch         = _p
            pitch_home_deg        = p_angle - (_p * PITCH_MAX_DEGREES)
            pitch_target_deg      = p_angle
            pitch_pid_last_error  = 0.0
            pitch_pid_last_actual = None
        pitch_pid_integral = 0.0
        pitch_current_freq = 0
        pitch_decelerating = False
        axis_fault["pitch"] = False

    if axis_fault["roll"]:
        r_angle = read_roll_angle_deg()
        _, _r = read_imu_commands()
        if r_angle is not None:
            ema_joy_roll          = _r
            roll_home_deg         = r_angle - (_r * ROLL_MAX_DEGREES)
            roll_target_deg       = r_angle
            roll_pid_last_error   = 0.0
            roll_pid_last_actual  = None
        roll_pid_integral = 0.0
        roll_current_freq = 0
        roll_decelerating = False
        axis_fault["roll"] = False

    if axis_fault["linear"]:
        linear_current_freq = 0
        linear_decelerating = False
        axis_fault["linear"] = False

    print("RE-ARM — axis fault(s) cleared. Resuming.")


# =============================================================================
# STATE
# =============================================================================

pitch_home_deg       = 0.0
pitch_target_deg     = 0.0
pitch_pid_integral    = 0.0
pitch_pid_last_error  = 0.0
pitch_pid_last_actual = None   # None = skip D-term on first call / after reset

roll_home_deg        = 0.0
roll_target_deg      = 0.0
roll_pid_integral    = 0.0
roll_pid_last_error  = 0.0
roll_pid_last_actual = None    # None = skip D-term on first call / after reset

ema_joy_pitch = 0.0
ema_joy_roll  = 0.0

pitch_current_freq  = 0
roll_current_freq   = 0
linear_current_freq = 0

pitch_last_forward  = True
roll_last_forward   = True
linear_last_forward = True

pitch_decelerating  = False
roll_decelerating   = False
linear_decelerating = False

# Last trim joystick readings — updated by handle_demo(); zero otherwise.
# Included in debug_print so the visualizer can display joystick drift.
_last_trim_p = 0.0
_last_trim_r = 0.0

# Open-loop position counter for linear axis (in steps from home).
# Positive = extended away from home.
linear_position_steps  = 0
encoder_position_steps = 0   # absolute input device position, in arm-equivalent steps
                              # (unclamped — allowed to wander outside arm's valid range)
linear_is_homed        = False

# AS5600 multi-turn tracking for linear axis.
# The AS5600 only knows its angle within one revolution (0-4095 counts = 0-360°).
# We accumulate delta counts each tick to track total travel across multiple revolutions.
_linear_as5600_raw_prev  = None   # raw reading from last tick; None until first read
_linear_as5600_accum_raw = 0      # accumulated raw counts from home (signed)
_linear_as5600_fail_count = 0     # consecutive I2C read failures; resets prev on ≥3
_linear_actual_mm        = None   # updated every tick by update_linear_encoder_mm()

debug_last_print_ms = 0
input_debug_last_ms = 0
last_tick_ms = time.ticks_ms()

# =============================================================================
# AS5600 ENCODER FUNCTIONS
# =============================================================================

def tca_select(channel):
    """Activate one TCA9548A channel. Must be called before reading an AS5600."""
    i2c.writeto(TCA9548A_ADDR, bytes([1 << channel]))


def _read_as5600_raw(channel):
    """Read the 12-bit raw angle (0-4095) from the AS5600 on the given TCA channel. Returns None on failure."""
    tca_select(channel)
    for _ in range(3):
        try:
            data = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            return ((data[0] & 0x0F) << 8) | data[1]
        except OSError:
            time.sleep_us(500)
    return None


def read_roll_angle_deg():
    """Read roll joint angle in degrees, wrapped to [-180, +180]. Returns None on failure."""
    raw = _read_as5600_raw(ROLL_TCA_CHANNEL)
    if raw is None:
        return None
    angle = (raw / 4096.0) * 360.0 - ROLL_ENCODER_OFFSET_DEG
    if angle > 180.0:
        angle -= 360.0
    elif angle < -180.0:
        angle += 360.0
    return -angle if ROLL_ENCODER_INVERT else angle


def read_pitch_angle_deg():
    """Read pitch joint angle in degrees, wrapped to [-180, +180]. Returns None on failure."""
    raw = _read_as5600_raw(PITCH_TCA_CHANNEL)
    if raw is None:
        return None
    angle = (raw / 4096.0) * 360.0 - PITCH_ENCODER_OFFSET_DEG
    if angle > 180.0:
        angle -= 360.0
    elif angle < -180.0:
        angle += 360.0
    return -angle if PITCH_ENCODER_INVERT else angle


def update_linear_encoder_mm():
    """
    Read the linear axis AS5600, accumulate multi-turn position, return mm from home.

    Must be called every control loop tick. Detects wrap-arounds by checking if the
    raw delta exceeds 2048 counts (half a revolution) — any real motion in one 40ms
    tick is far smaller than that, so a large delta unambiguously means a wrap.
    Returns None on I2C failure.
    """
    global _linear_as5600_raw_prev, _linear_as5600_accum_raw, _linear_as5600_fail_count

    raw = _read_as5600_raw(LINEAR_TCA_CHANNEL)
    if raw is None:
        _linear_as5600_fail_count += 1
        # After 3 consecutive failures, invalidate prev so the next successful read
        # initialises fresh rather than computing a stale delta that could look like
        # a large position jump (up to ±62 mm if several ticks are skipped).
        if _linear_as5600_fail_count >= 3:
            _linear_as5600_raw_prev = None
        return None

    _linear_as5600_fail_count = 0

    if _linear_as5600_raw_prev is None:
        _linear_as5600_raw_prev = raw
        return 0.0

    delta = raw - _linear_as5600_raw_prev
    if delta > 2048:
        delta -= 4096   # encoder wrapped backward (e.g. 4095 → 0)
    elif delta < -2048:
        delta += 4096   # encoder wrapped forward  (e.g. 0 → 4095)

    # Sanity check: discard deltas that are physically impossible in one 40 ms tick.
    # At LINEAR_MAX_RPS the encoder moves at most ~330 counts/tick; 500 gives margin.
    if abs(delta) > 500:
        print("WARNING: Linear encoder delta {} out of range — discarding.".format(delta))
        _linear_as5600_raw_prev = raw
        return None

    _linear_as5600_accum_raw += delta
    _linear_as5600_raw_prev   = raw
    mm = (_linear_as5600_accum_raw / 4096.0) * LINEAR_SPOOL_CIRC_MM
    return -mm if LINEAR_ENCODER_INVERT else mm


def zero_linear_encoder():
    """Reset the multi-turn accumulator to zero. Call after homing completes."""
    global _linear_as5600_raw_prev, _linear_as5600_accum_raw
    _linear_as5600_raw_prev  = None
    _linear_as5600_accum_raw = 0


# =============================================================================
# MPU6050 IMU — MAIN INPUT (replaces analog joystick)
# =============================================================================
#
# The MPU6050 has no onboard sensor fusion — we derive pitch/roll directly
# from the accelerometer's gravity vector (atan2 of the gravity components).
# This is gravity-referenced (absolute, non-drifting) but noisy under fast
# motion; the existing EMA smoothing in handle_main_input() handles that,
# the same way it smoothed the joystick's ADC noise.
#
# Calibration offsets (IMU_PITCH_OFFSET_DEG / IMU_ROLL_OFFSET_DEG) correct for
# the chip's mounting tilt so the working-neutral position reads ~0,0.
# Measured using armcontrolsetup.py Test D — see CONFIG comments for procedure.

_MPU_PWR_MGMT_1  = 0x6B
_MPU_WHO_AM_I    = 0x75
_MPU_ACCEL_OUT   = 0x3B   # 6 bytes: XH XL YH YL ZH ZL
_MPU_ACCEL_SCALE = 16384.0  # counts per g at +/-2g (chip default range)


def imu_init():
    """Wake the MPU6050 from sleep. Returns True if found and initialised."""
    try:
        who = i2c.readfrom_mem(IMU_ADDR, _MPU_WHO_AM_I, 1)[0]
        if who not in (0x68, 0x72):
            return False
        i2c.writeto_mem(IMU_ADDR, _MPU_PWR_MGMT_1, bytes([0x00]))
        time.sleep_ms(100)
        return True
    except OSError:
        return False


def read_imu_angles():
    """
    Returns (pitch_deg, roll_deg), gravity-referenced and calibration-offset
    so the working-neutral mount position reads ~(0, 0). Returns (None, None)
    on read failure.
    """
    try:
        data = i2c.readfrom_mem(IMU_ADDR, _MPU_ACCEL_OUT, 6)
    except OSError:
        return None, None

    def s16(h, l):
        v = (h << 8) | l
        return v - 65536 if v >= 32768 else v

    ax = s16(data[0], data[1]) / _MPU_ACCEL_SCALE
    ay = s16(data[2], data[3]) / _MPU_ACCEL_SCALE
    az = s16(data[4], data[5]) / _MPU_ACCEL_SCALE

    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 57.2958 + IMU_PITCH_OFFSET_DEG
    roll  = math.atan2( ay, az) * 57.2958 + IMU_ROLL_OFFSET_DEG
    return pitch, roll


def read_imu_commands():
    """
    Read both IMU axes in a single I2C transaction.
    Returns (pitch_cmd, roll_cmd), each deadbanded and normalised to [-1.0, +1.0].
    Returns (0.0, 0.0) on read failure.
    """
    pitch, roll = read_imu_angles()
    if pitch is None:
        return 0.0, 0.0
    p = 0.0 if abs(pitch) < IMU_DEADBAND_PITCH_DEG else max(-1.0, min(1.0, pitch / IMU_PITCH_MAX_TILT_DEG))
    r = 0.0 if abs(roll)  < IMU_DEADBAND_ROLL_DEG  else max(-1.0, min(1.0, roll  / IMU_ROLL_MAX_TILT_DEG))
    return p, r


# =============================================================================
# LIMIT SWITCH HELPER
# =============================================================================

def limit_switch_triggered():
    """Return True if the linear home/retract limit switch is triggered.
    Does its own live I2C read (rather than using the read_limit_switches() cache)
    since home_linear_axis() polls this from a tight blocking loop outside the
    normal 40ms tick cadence."""
    return (mcp.read_porta() >> MCP_LINEAR_HOME_BIT) & 1 == 1


# =============================================================================
# PWM HELPER FUNCTIONS
# =============================================================================

def set_motor(pwm, dir_pin, invert, freq, forward, prev_freq):
    """
    Set motor direction and PWM frequency.
    freq <= MIN_FREQ stops the motor.
    prev_freq: frequency from the previous tick — avoids redundant freq() calls
    that cause PWM timer leaks on the ESP32.
    Returns the actual frequency set.
    """
    if freq <= MIN_FREQ:
        if pwm.duty() > 0:
            pwm.duty(0)
        return 0

    dir_pin.value(1 if (forward ^ invert) else 0)

    if freq != prev_freq:
        # Use init() instead of freq() — freq() leaks a hardware timer slot on ESP32
        # MicroPython each call; init() properly releases the old slot first.
        pwm.init(freq=freq, duty=512)
    elif pwm.duty() == 0:
        pwm.duty(512)

    return freq


def stop_motor(pwm):
    """Immediately stop a motor."""
    pwm.duty(0)


# =============================================================================
# LINEAR MOTOR STOP
# =============================================================================

def stop_linear_motor():
    """Stop the linear axis PWM. duty(0) outputs constant LOW — no rising edges, no steps."""
    linear_pwm.duty(0)


# =============================================================================
# POSITION WRAP HELPER
# =============================================================================

def wrap_angle_error(error):
    """Wrap an angle error to the range [-180, +180]."""
    while error > 180.0:
        error -= 360.0
    while error < -180.0:
        error += 360.0
    return error


# =============================================================================
# LINEAR HOMING
# =============================================================================

def home_linear_axis():
    """
    Drive the linear axis toward the limit switch at homing speed.
    Blocks until the switch triggers, then backs off and resets position to zero.
    Called automatically at startup.
    Returns True on success.
    """
    global linear_position_steps, encoder_position_steps, linear_is_homed, linear_current_freq

    print("Homing linear axis... driving toward limit switch.")

    # Worst-case travel time at homing speed, with 50% margin.
    HOMING_TIMEOUT_MS = int((LINEAR_MAX_MM / (LINEAR_HOMING_RPS * LINEAR_SPOOL_CIRC_MM)) * 1500)

    def _backoff():
        """Drive 0.5 rev in the extend direction to clear the limit switch. Returns False if e-stop fires."""
        backoff_steps = LINEAR_STEPS_PER_REV // 2
        backoff_delay = 1.0 / (2 * LINEAR_HOMING_FREQ)
        linear_dir.value(0 if not LINEAR_INVERT_DIR else 1)
        linear_pwm.init(freq=LINEAR_HOMING_FREQ, duty=0)
        for _ in range(backoff_steps):
            if estop_active:
                linear_pwm.duty(0)
                return False
            linear_pwm.duty(512)
            time.sleep(backoff_delay)
        linear_pwm.duty(0)
        return True

    # If the switch is already triggered, skip the drive phase but still back off.
    # Without backoff the switch stays physically pressed, which blocks all
    # retract motion and leaves encoder_position_steps unsynced.
    if limit_switch_triggered():
        print("Limit switch already triggered — skipping drive phase, backing off...")
        linear_position_steps  = 0
        linear_is_homed        = True
        stop_motor(linear_pwm)
        linear_current_freq    = 0

        if not _backoff():
            linear_is_homed = False
            print("Homing aborted during backoff — E-STOP triggered. Re-home before using linear axis.")
            return False

        linear_current_freq    = 0
        linear_position_steps  = LINEAR_STEPS_PER_REV // 2
        encoder_position_steps = LINEAR_STEPS_PER_REV // 2
        zero_linear_encoder()
        print("Backoff complete. Position zeroed at limit switch, now {:.2f}mm extended.".format(
            (LINEAR_STEPS_PER_REV // 2) / LINEAR_STEPS_PER_MM))
        return True

    # Drive in the retract direction at homing speed.
    # Passing LINEAR_INVERT_DIR as the forward argument means set_motor's internal
    # XOR always cancels to the same physical pin state regardless of invert setting.
    set_motor(linear_pwm, linear_dir, LINEAR_INVERT_DIR, LINEAR_HOMING_FREQ, LINEAR_INVERT_DIR, 0)

    homing_start = time.ticks_ms()
    while not limit_switch_triggered() and not estop_active:
        if time.ticks_diff(time.ticks_ms(), homing_start) > HOMING_TIMEOUT_MS:
            stop_motor(linear_pwm)
            linear_current_freq = 0
            linear_is_homed     = False
            print("ERROR: Homing timeout — limit switch not reached. Check wiring/alignment.")
            return False
        time.sleep_ms(5)

    stop_motor(linear_pwm)
    linear_current_freq = 0

    if estop_active:
        linear_is_homed = False
        print("Homing aborted — E-STOP triggered. Re-home before using linear axis.")
        return False

    linear_position_steps = 0
    linear_is_homed       = True
    print("Linear axis homed. Backing off limit switch...")

    if not _backoff():
        linear_is_homed = False
        print("Homing aborted during backoff — E-STOP triggered. Re-home before using linear axis.")
        return False

    linear_current_freq    = 0
    linear_position_steps  = LINEAR_STEPS_PER_REV // 2
    encoder_position_steps = LINEAR_STEPS_PER_REV // 2
    zero_linear_encoder()
    print("Backoff complete. Position zeroed at limit switch, now {:.2f}mm extended.".format(
        (LINEAR_STEPS_PER_REV // 2) / LINEAR_STEPS_PER_MM))
    return True


# =============================================================================
# JOGGING (trim joystick — pitch and roll only)
# =============================================================================

def _read_trim(adc, invert):
    """Return normalised trim joystick axis value in [-1.0, 1.0], 0.0 in deadband."""
    raw = sum(adc.read() for _ in range(4)) // 4
    if abs(raw - TRIM_JOY_CENTRE) < TRIM_JOY_DEADBAND:
        return 0.0
    if raw > TRIM_JOY_CENTRE:
        val = min(1.0, (raw - TRIM_JOY_CENTRE) / (TRIM_JOY_MAX - TRIM_JOY_CENTRE))
    else:
        val = max(-1.0, (raw - TRIM_JOY_CENTRE) / (TRIM_JOY_CENTRE - TRIM_JOY_MIN))
    return -val if invert else val


def handle_jogging(trim_p, trim_r):
    """
    Jog pitch and roll axes using the trim joystick.
    trim_p, trim_r: normalised [-1, 1] values from _read_trim().
    Positive = forward, negative = backward, 0 = decelerate and re-capture home.
    Speed scales with deflection magnitude; accel/decel ramps preserved.
    """
    global pitch_home_deg, roll_home_deg, roll_target_deg, pitch_target_deg
    global pitch_current_freq, roll_current_freq
    global pitch_last_forward, roll_last_forward
    global roll_pid_integral, roll_pid_last_error, roll_pid_last_actual
    global pitch_pid_integral, pitch_pid_last_error, pitch_pid_last_actual
    global ema_joy_roll, ema_joy_pitch
    global pitch_decelerating, roll_decelerating

    # -------------------------------------------------------------------------
    # PITCH
    # -------------------------------------------------------------------------
    if axis_fault["pitch"]:
        trim_p = 0.0
    # Clamp to MIN_FREQ+1 so the motor actually runs at low deflections.
    # (PITCH_JOG_FREQ = 32 Hz; without this, deflections <62% fall below MIN_FREQ and stop.)
    pitch_target_freq = max(MIN_FREQ + 1, int(abs(trim_p) * PITCH_JOG_FREQ)) if abs(trim_p) > 0 else 0
    trim_p_fwd = trim_p > 0

    if abs(trim_p) > 0 and (pitch_current_freq == 0 or trim_p_fwd == pitch_last_forward):
        pitch_last_forward = trim_p_fwd
        prev = pitch_current_freq
        if pitch_current_freq < pitch_target_freq:
            pitch_current_freq = min(pitch_current_freq + PITCH_JOG_ACCEL_HZ, pitch_target_freq)
        else:
            pitch_current_freq = max(pitch_target_freq, pitch_current_freq - PITCH_JOG_DECEL_HZ)
        pitch_current_freq = set_motor(pitch_pwm, pitch_dir, PITCH_INVERT_DIR,
                                       pitch_current_freq, trim_p_fwd, prev)
        pitch_decelerating = True

    elif pitch_current_freq > 0 or pitch_decelerating:
        pitch_decelerating = True
        prev = pitch_current_freq
        pitch_current_freq = max(0, pitch_current_freq - PITCH_JOG_DECEL_HZ)
        if pitch_current_freq <= MIN_FREQ:
            pitch_current_freq = 0
            pitch_decelerating = False
            stop_motor(pitch_pwm)
            pitch_pid_integral    = 0.0
            pitch_pid_last_error  = 0.0
            pitch_pid_last_actual = None
            angle = read_pitch_angle_deg()
            if angle is not None:
                joy_now, _       = read_imu_commands()
                ema_joy_pitch    = joy_now
                pitch_home_deg   = angle - (joy_now * PITCH_MAX_DEGREES)
                pitch_target_deg = angle
        else:
            pitch_current_freq = set_motor(pitch_pwm, pitch_dir, PITCH_INVERT_DIR,
                                           pitch_current_freq, pitch_last_forward, prev)

    # -------------------------------------------------------------------------
    # ROLL
    # -------------------------------------------------------------------------
    if axis_fault["roll"]:
        trim_r = 0.0
    roll_target_freq = max(MIN_FREQ + 1, int(abs(trim_r) * ROLL_JOG_FREQ)) if abs(trim_r) > 0 else 0
    trim_r_fwd = trim_r > 0

    if abs(trim_r) > 0 and (roll_current_freq == 0 or trim_r_fwd == roll_last_forward):
        roll_last_forward = trim_r_fwd
        prev = roll_current_freq
        if roll_current_freq < roll_target_freq:
            roll_current_freq = min(roll_current_freq + ROLL_JOG_ACCEL_HZ, roll_target_freq)
        else:
            roll_current_freq = max(roll_target_freq, roll_current_freq - ROLL_JOG_DECEL_HZ)
        roll_current_freq = set_motor(roll_pwm, roll_dir, ROLL_INVERT_DIR,
                                      roll_current_freq, trim_r_fwd, prev)
        roll_decelerating = True

    elif roll_current_freq > 0 or roll_decelerating:
        roll_decelerating = True
        prev = roll_current_freq
        roll_current_freq = max(0, roll_current_freq - ROLL_JOG_DECEL_HZ)
        if roll_current_freq <= MIN_FREQ:
            roll_current_freq = 0
            roll_decelerating = False
            stop_motor(roll_pwm)
            roll_pid_integral    = 0.0
            roll_pid_last_error  = 0.0
            roll_pid_last_actual = None
            angle = read_roll_angle_deg()
            if angle is not None:
                _, joy_now      = read_imu_commands()
                ema_joy_roll    = joy_now
                roll_home_deg   = angle - (joy_now * ROLL_MAX_DEGREES)
                roll_target_deg = angle
        else:
            roll_current_freq = set_motor(roll_pwm, roll_dir, ROLL_INVERT_DIR,
                                          roll_current_freq, roll_last_forward, prev)



# =============================================================================
# DEBUG / PRINT
# =============================================================================

def debug_print(pitch_target, pitch_actual, roll_target, roll_actual, linear_actual_mm):
    global debug_last_print_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, debug_last_print_ms) < 100:
        return
    debug_last_print_ms = now
    linear_cmd_mm = linear_position_steps / LINEAR_STEPS_PER_MM
    lin_act_str = "{:5.1f}mm".format(linear_actual_mm) if linear_actual_mm is not None else " ?.?mm"
    print("| {}PITCH:{} Tgt:{:5.1f} Act:{:5.1f} Err:{:4.1f}"
          " | {}ROLL:{} Tgt:{:5.1f} Act:{:5.1f} Err:{:4.1f}"
          " | {}LINEAR:{} Cmd:{:5.1f}mm Act:{} / {:.0f}mm"
          " | TRIM: P:{:5.2f} R:{:5.2f}"
          " | FREQ: P:{} R:{} L:{}".format(
              BOLD, RESET, pitch_target, pitch_actual, pitch_target - pitch_actual,
              BOLD, RESET, roll_target,  roll_actual,  roll_target  - roll_actual,
              BOLD, RESET, linear_cmd_mm, lin_act_str, LINEAR_MAX_MM,
              _last_trim_p, _last_trim_r,
              pitch_current_freq, roll_current_freq, linear_current_freq))


def print_input_debug():
    """
    Diagnostic: print exactly what the ESP32 reads from its input devices,
    independent of any control logic. Throttled to 10 Hz.

    rawX/rawY are the unprocessed 12-bit ADC readings (0-4095) from the trim
    joystick pins. If these don't change when you move the stick, the problem
    is wiring (pin, VCC, or GND) — not code. sclX/sclY are the deadbanded,
    normalised [-1,1] values the jog logic actually uses. IMU p/r are the
    raw tilt angles in degrees.
    """
    global input_debug_last_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, input_debug_last_ms) < 100:
        return
    input_debug_last_ms = now
    raw_x = sum(trim_joy_x.read() for _ in range(4)) // 4
    raw_y = sum(trim_joy_y.read() for _ in range(4)) // 4
    scl_x = _read_trim(trim_joy_x, TRIM_JOY_INVERT_X)
    scl_y = _read_trim(trim_joy_y, TRIM_JOY_INVERT_Y)
    imu_p, imu_r = read_imu_angles()
    imu_p = imu_p if imu_p is not None else 0.0
    imu_r = imu_r if imu_r is not None else 0.0
    print("INPUT | TRIM rawX:{:4d} rawY:{:4d} sclX:{:+.2f} sclY:{:+.2f}"
          " | IMU p:{:+6.1f} r:{:+6.1f}".format(
              raw_x, raw_y, scl_x, scl_y, imu_p, imu_r))


# =============================================================================
# LINEAR ENCODER CONTROL
# =============================================================================

def handle_encoder_linear():
    """
    Drive the linear axis based on optical encoder input.

    This function is POSITION-based, not velocity-based.

    Key idea:
      - encoder_position_steps tracks the absolute position of the input device
        in arm-equivalent steps. It is NEVER clamped — it freely follows the
        input device wherever it goes, even past the arm's physical limits.
      - commanded_steps is encoder_position_steps clamped to [0, LINEAR_MAX_STEPS].
        This is what the arm actually tries to reach.
      - The arm only moves when commanded_steps != linear_position_steps.

    This means:
      - If the user drives the input past the arm's limit, the arm stops at the
        limit but encoder_position_steps keeps accumulating the overshoot.
      - When the user reverses, the arm does NOT move until the input device
        has traveled back through the overshoot and re-entered the valid range.
        The arm's position therefore stays tied to a real position on the input.

    Direction conventions:
      - forward: the logical signal passed to set_motor(). set_motor() XORs this
        with LINEAR_INVERT_DIR internally to produce the physical pin state.
        forward=True means "increase position" in logical space, but the actual
        physical direction depends on LINEAR_INVERT_DIR.
      - counting_up: derived from forward XOR LINEAR_POSITION_INVERT. Separates
        the position counter direction from the motor direction entirely, so
        LINEAR_INVERT_DIR and LINEAR_POSITION_INVERT can be set independently.
      - Limit switch checks use forward directly — with LINEAR_INVERT_DIR=True,
        forward=True corresponds to physical retraction (toward the switch).
    """
    global linear_current_freq, linear_last_forward, linear_decelerating
    global linear_position_steps, encoder_position_steps, encoder_count

    # ------------------------------------------------------------------
    # 1. Snapshot and reset the encoder counter.
    # ------------------------------------------------------------------
    irq_state = disable_irq()
    delta = encoder_count
    encoder_count = 0
    enable_irq(irq_state)

    if axis_fault["linear"]:
        # Extend-limit crash fault is latched — hold position, drop any encoder
        # input accumulated while faulted (drained above, so it can't spike the
        # freq calc on the tick after RE-ARM clears it).
        stop_linear_motor()
        linear_current_freq = 0
        linear_decelerating = False
        return

    # Normalise so positive delta always means "extend" intent.
    if LINEAR_ENCODER_INVERT:
        delta = -delta

    # ------------------------------------------------------------------
    # 2. Advance the absolute input position (UNCLAMPED).
    #    This is the only place encoder_position_steps is updated.
    # ------------------------------------------------------------------
    encoder_position_steps += int(delta * STEPS_PER_ENCODER_COUNT)

    # ------------------------------------------------------------------
    # 3. Commanded position = input position clamped to arm's valid range.
    # ------------------------------------------------------------------
    commanded_steps = max(0, min(LINEAR_MAX_STEPS, encoder_position_steps))

    # How far does the arm still need to travel?
    error = commanded_steps - linear_position_steps

    # Steps the motor will take this tick at the current frequency
    # (used for the position estimate below).
    steps_this_tick = int(linear_current_freq * CONTROL_LOOP_SEC)

    if abs(delta) >= LINEAR_ENC_MIN_DELTA and error != 0:
        # ------------------------------------------------------------------
        # CASE A: Encoder is turning AND the arm has somewhere to go.
        # ------------------------------------------------------------------
        forward = (error > 0) ^ LINEAR_INVERT_DIR
        counting_up = forward ^ LINEAR_POSITION_INVERT

        # Limit switch safety — stop immediately if triggered while retracting.
        # With LINEAR_INVERT_DIR=True, forward=True is the retract direction.
        if forward and limit_switch_triggered():
            stop_linear_motor()
            linear_current_freq    = 0
            linear_position_steps  = 0
            encoder_position_steps = 0   # re-sync input to arm
            linear_decelerating    = False
            return

        # Motor speed is proportional to how fast the encoder is turning.
        desired_steps = abs(delta) * STEPS_PER_ENCODER_COUNT
        desired_freq  = min(int(desired_steps / CONTROL_LOOP_SEC), LINEAR_MAX_FREQ)
        desired_freq  = max(desired_freq, MIN_FREQ + 1)

        linear_last_forward = forward
        linear_decelerating = True   # arm is moving; decelerate next tick if encoder stops
        prev = linear_current_freq
        linear_current_freq = set_motor(linear_pwm, linear_dir, LINEAR_INVERT_DIR, desired_freq, forward, prev)

        # Update position estimate.
        if counting_up:
            linear_position_steps = min(LINEAR_MAX_STEPS, linear_position_steps + steps_this_tick)
        else:
            linear_position_steps = max(0, linear_position_steps - steps_this_tick)

    else:
        # ------------------------------------------------------------------
        # CASE B: Encoder is stationary, OR the arm is already at the
        #         commanded position (input is in the out-of-bounds zone).
        #         Either way: decelerate to a stop.
        # ------------------------------------------------------------------
        if linear_current_freq > 0 or linear_decelerating:
            linear_decelerating = True
            prev = linear_current_freq
            linear_current_freq = max(0, linear_current_freq - LINEAR_ENC_DECEL_HZ)
            counting_up = linear_last_forward ^ LINEAR_POSITION_INVERT

            if linear_current_freq <= MIN_FREQ:
                linear_current_freq = 0
                linear_decelerating = False
                stop_linear_motor()
            else:
                # Update position while coasting.
                if counting_up:
                    linear_position_steps = min(LINEAR_MAX_STEPS, linear_position_steps + steps_this_tick)
                else:
                    linear_position_steps = max(0, linear_position_steps - steps_this_tick)

                linear_current_freq = set_motor(linear_pwm, linear_dir, LINEAR_INVERT_DIR, linear_current_freq, linear_last_forward, prev)

            # Safety: limit switch triggers during coast.
            # With LINEAR_INVERT_DIR=True, linear_last_forward=True is the retract direction.
            if linear_last_forward and limit_switch_triggered():
                stop_linear_motor()
                linear_current_freq    = 0
                linear_position_steps  = 0
                encoder_position_steps = 0   # re-sync input to arm
                linear_decelerating    = False


# =============================================================================
# PID EXECUTION (shared by normal and demo input paths)
# =============================================================================

def _run_pid(dt_ms):
    """Drive pitch and roll toward pitch_target_deg / roll_target_deg via PID."""
    global pitch_current_freq, roll_current_freq
    global pitch_pid_integral, pitch_pid_last_error, pitch_pid_last_actual
    global roll_pid_integral, roll_pid_last_error, roll_pid_last_actual

    dt_sec = max(dt_ms / 1000.0, 0.001)

    # -------------------------------------------------------------------------
    # PITCH
    # -------------------------------------------------------------------------
    actual_pitch = read_pitch_angle_deg()
    if actual_pitch is None:
        stop_motor(pitch_pwm)
        stop_motor(roll_pwm)
        pitch_current_freq = 0
        roll_current_freq  = 0
        print("WARNING: Pitch AS5600 read failed. Check wiring.")
        return

    if axis_fault["pitch"]:
        # Min/max limit crash fault is latched — hold position, skip PID
        # entirely until RE-ARM clears it (see clear_axis_faults()).
        stop_motor(pitch_pwm)
        pitch_current_freq = 0
    else:
        error_pitch = wrap_angle_error(pitch_target_deg - actual_pitch)

        if abs(error_pitch) <= PITCH_POSITION_DEADBAND_DEG:
            stop_motor(pitch_pwm)
            pitch_current_freq    = 0
            pitch_pid_integral    = 0.0
            pitch_pid_last_error  = error_pitch
            pitch_pid_last_actual = actual_pitch
        else:
            p_term = PITCH_KP * error_pitch
            pitch_pid_integral   = max(-PITCH_KI_CLAMP,
                                       min(PITCH_KI_CLAMP, pitch_pid_integral + error_pitch * dt_sec))
            i_term = PITCH_KI * pitch_pid_integral
            if pitch_pid_last_actual is None:
                d_term = 0.0
            else:
                d_term = -PITCH_KD * (actual_pitch - pitch_pid_last_actual) / dt_sec
            pitch_pid_last_error  = error_pitch
            pitch_pid_last_actual = actual_pitch

            pid_output         = p_term + i_term + d_term
            desired_freq_pitch = min(int(abs(pid_output)), PITCH_MAX_FREQ)
            forward_pitch      = pid_output > 0
            prev_pitch         = pitch_current_freq

            if desired_freq_pitch <= MIN_FREQ:
                stop_motor(pitch_pwm)
                pitch_current_freq = 0
            else:
                pitch_current_freq = set_motor(pitch_pwm, pitch_dir, PITCH_INVERT_DIR,
                                               desired_freq_pitch, forward_pitch, prev_pitch)

    # -------------------------------------------------------------------------
    # ROLL
    # -------------------------------------------------------------------------
    actual_roll = read_roll_angle_deg()
    if actual_roll is None:
        stop_motor(pitch_pwm)
        stop_motor(roll_pwm)
        pitch_current_freq = 0
        roll_current_freq  = 0
        print("WARNING: Roll AS5600 read failed. Check wiring.")
        return

    if axis_fault["roll"]:
        # Min/max limit crash fault is latched — hold position, skip PID
        # entirely until RE-ARM clears it (see clear_axis_faults()).
        stop_motor(roll_pwm)
        roll_current_freq = 0
    else:
        error_roll = wrap_angle_error(roll_target_deg - actual_roll)

        if abs(error_roll) <= ROLL_POSITION_DEADBAND:
            stop_motor(roll_pwm)
            roll_current_freq    = 0
            roll_pid_integral    = 0.0
            roll_pid_last_error  = error_roll
            roll_pid_last_actual = actual_roll
        else:
            p_term = ROLL_KP * error_roll
            roll_pid_integral   = max(-ROLL_KI_CLAMP,
                                      min(ROLL_KI_CLAMP, roll_pid_integral + error_roll * dt_sec))
            i_term = ROLL_KI * roll_pid_integral
            if roll_pid_last_actual is None:
                d_term = 0.0
            else:
                d_term = -ROLL_KD * (actual_roll - roll_pid_last_actual) / dt_sec
            roll_pid_last_error  = error_roll
            roll_pid_last_actual = actual_roll

            pid_output        = p_term + i_term + d_term
            desired_freq_roll = min(int(abs(pid_output)), ROLL_MAX_FREQ)
            forward_roll      = pid_output > 0
            prev_roll         = roll_current_freq

            if desired_freq_roll <= MIN_FREQ:
                stop_motor(roll_pwm)
                roll_current_freq = 0
            else:
                roll_current_freq = set_motor(roll_pwm, roll_dir, ROLL_INVERT_DIR,
                                              desired_freq_roll, forward_roll, prev_roll)

    debug_print(pitch_target_deg, actual_pitch, roll_target_deg, actual_roll, _linear_actual_mm)


# =============================================================================
# NORMAL INPUT PATH (IMU → targets → PID)
# =============================================================================

def handle_main_input(dt_ms):
    global ema_joy_pitch, ema_joy_roll
    global pitch_target_deg, roll_target_deg

    _p, _r        = read_imu_commands()
    ema_joy_pitch = EMA_ALPHA * _p + (1.0 - EMA_ALPHA) * ema_joy_pitch
    ema_joy_roll  = EMA_ALPHA * _r + (1.0 - EMA_ALPHA) * ema_joy_roll

    pitch_target_deg = pitch_home_deg + (ema_joy_pitch * PITCH_MAX_DEGREES)
    roll_target_deg  = roll_home_deg  + (ema_joy_roll  * ROLL_MAX_DEGREES)

    _run_pid(dt_ms)


# =============================================================================
# DEMO MODE — preprogrammed motion sequence
# =============================================================================
#
# Sequence: brief hold at center → continuous circular orbit (forever).
# The reference point is pitch_home_deg / roll_home_deg, set at startup from
# wherever the arm physically sits (jog to position before pressing RE-ARM).
#
# Orbit math: a parametric circle in pitch/roll space.
#   pitch = home + r_eff * cos(ω * t)
#   roll  = home + r_eff * sin(ω * t)
# The orbit phase is TERMINAL — once entered it loops on itself indefinitely.
# A sinusoid is inherently seamless, so there is no visible "restart." The
# radius r_eff ramps from 0 → DEMO_ORBIT_RADIUS_DEG over the first revolution
# (a one-time spiral-out), so entry starts exactly at the center with no step.

_demo_phase          = 0
_demo_phase_start_ms = 0

# Each entry is (pitch_offset_deg, roll_offset_deg) from home, or None for orbit.
_DEMO_PHASES = [
    (0, 0), # 0: hold at center — one-time dwell; trim stick shifts center
    None,   # 1: circular orbit — terminal, loops continuously and seamlessly
]


def handle_demo(dt_ms):
    global pitch_target_deg, roll_target_deg, pitch_home_deg, roll_home_deg
    global _demo_phase, _demo_phase_start_ms
    global _last_trim_p, _last_trim_r

    now     = time.ticks_ms()
    elapsed = time.ticks_diff(now, _demo_phase_start_ms)
    offsets = _DEMO_PHASES[_demo_phase]

    # Advance ONLY from the initial hold into the orbit, once. The orbit phase
    # is terminal: it never transitions back, so the demo loops seamlessly with
    # no visible restart.
    if offsets is not None and elapsed >= DEMO_HOLD_MS:
        _demo_phase          = 1
        _demo_phase_start_ms = now
        elapsed              = 0
        offsets              = _DEMO_PHASES[1]

    trim_p = _read_trim(trim_joy_x, TRIM_JOY_INVERT_X)
    trim_r = _read_trim(trim_joy_y, TRIM_JOY_INVERT_Y)
    _last_trim_p = trim_p
    _last_trim_r = trim_r

    if offsets is not None:
        # Hold phase: trim shifts the orbit center.
        if trim_p != 0.0:
            pitch_home_deg += trim_p * DEMO_TRIM_PITCH_RPS * 360.0 * dt_ms / 1000.0
        if trim_r != 0.0:
            roll_home_deg  += trim_r * DEMO_TRIM_ROLL_RPS  * 360.0 * dt_ms / 1000.0
        pitch_target_deg = pitch_home_deg + offsets[0]
        roll_target_deg  = roll_home_deg  + offsets[1]
    else:
        # Orbit phase: continuous. Ramp radius 0→R over the first revolution so
        # entry starts at the center (no step), then full radius forever.
        # elapsed never resets here, so the ramp happens exactly once.
        t      = elapsed / 1000.0
        omega  = DEMO_ORBIT_RPS * 2.0 * math.pi
        ramp_s = 1.0 / DEMO_ORBIT_RPS                       # one orbit period
        r_eff  = DEMO_ORBIT_RADIUS_DEG * min(1.0, t / ramp_s)
        pitch_target_deg = pitch_home_deg + r_eff * math.cos(omega * t)
        roll_target_deg  = roll_home_deg  + r_eff * math.sin(omega * t)

    _run_pid(dt_ms)


# =============================================================================
# MAIN
# =============================================================================

def main():
    global last_tick_ms, roll_home_deg, roll_target_deg, ema_joy_roll
    global pitch_home_deg, pitch_target_deg, ema_joy_pitch
    global roll_pid_integral, roll_pid_last_error, roll_pid_last_actual
    global pitch_pid_integral, pitch_pid_last_error, pitch_pid_last_actual
    global linear_is_homed
    global estop_active, estop_handled
    global pitch_current_freq, roll_current_freq, linear_current_freq
    global pitch_decelerating, roll_decelerating
    global _demo_phase, _demo_phase_start_ms
    global _linear_actual_mm

    print("Scanning I2C bus...")
    devices = i2c.scan()
    print("  Bus (expect 0x70 for TCA9548A):  ", [hex(d) for d in devices])
    tca_select(ROLL_TCA_CHANNEL)
    devices = i2c.scan()
    print("  Roll  channel (expect 0x36):     ", [hex(d) for d in devices])
    tca_select(PITCH_TCA_CHANNEL)
    devices = i2c.scan()
    print("  Pitch channel (expect 0x36):     ", [hex(d) for d in devices])
    tca_select(LINEAR_TCA_CHANNEL)
    devices = i2c.scan()
    print("  Linear channel (expect 0x36):    ", [hex(d) for d in devices])

    # Read-based health check: a scan only proves the address ACKs. Actually read
    # the raw angle register from each channel to confirm the register read works
    # (different I2C transaction). FAIL here but ACK above = wiring/intermittent.
    print("I2C read check (raw angle register, expect 0-4095):")
    for _name, _ch in (("Roll  ", ROLL_TCA_CHANNEL),
                       ("Pitch ", PITCH_TCA_CHANNEL),
                       ("Linear", LINEAR_TCA_CHANNEL)):
        _raw = _read_as5600_raw(_ch)
        print("  {} ch{}: {}".format(_name, _ch,
              "FAIL (register read returned None)" if _raw is None else _raw))

    try:
        _mcp_boot_bits = mcp.read_gpio()
        print("MCP23017 (0x{:02X}) PA0-5: {} (1 = triggered; expect 000000 at rest)".format(
            MCP23017_ADDR, "".join(str((_mcp_boot_bits >> b) & 1) for b in range(5, -1, -1))))
    except OSError:
        print("WARNING: MCP23017 not responding at 0x{:02X}. Check wiring — limit switches disabled.".format(
            MCP23017_ADDR))

    print("Robot arm controller starting.")
    print("--- Derived config ---")
    print("Pitch:   {}x microsteps, {} steps/rev, max {} Hz ({} rev/sec)".format(
        PITCH_MICROSTEPS,  PITCH_STEPS_PER_REV,  PITCH_MAX_FREQ,  PITCH_MAX_RPS))
    print("Roll:    {}x microsteps, {} steps/rev, max {} Hz ({} rev/sec)".format(
        ROLL_MICROSTEPS,   ROLL_STEPS_PER_REV,   ROLL_MAX_FREQ,   ROLL_MAX_RPS))
    print("Linear:  {}x microsteps, {} steps/rev, max {} Hz ({} rev/sec)".format(
        LINEAR_MICROSTEPS, LINEAR_STEPS_PER_REV, LINEAR_MAX_FREQ, LINEAR_MAX_RPS))
    print("         {:.2f} steps/mm, max travel {:.0f}mm ({} steps)".format(
        LINEAR_STEPS_PER_MM, LINEAR_MAX_MM, LINEAR_MAX_STEPS))
    print("Encoder: {} PPR, {}x quadrature = {} counts/rev, scale = {}x".format(
        ENCODER_PPR, 4, ENCODER_COUNTS_PER_REV, ENCODER_SCALE))
    print("         {:.4f} stepper steps per encoder count".format(STEPS_PER_ENCODER_COUNT))
    print("----------------------")

    # --- Encoder startup ---
    print("Reading roll AS5600 encoder...")
    time.sleep_ms(1000)
    initial_angle = read_roll_angle_deg()
    if initial_angle is None:
        print("ERROR: Could not read roll AS5600.")
        print("Check: VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22.")
        print("Confirm magnet is mounted within ~3mm of sensor face.")
        return

    print("Reading pitch AS5600 encoder...")
    initial_pitch_angle = read_pitch_angle_deg()
    if initial_pitch_angle is None:
        print("ERROR: Could not read pitch AS5600.")
        print("Check: VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22.")
        print("Confirm magnet is within ~3mm of sensor face.")
        return

    # --- IMU init ---
    print("Initialising MPU6050 IMU...")
    if not imu_init():
        print("ERROR: MPU6050 not found at 0x{:02X}. Check SDA->GPIO21, SCL->GPIO22, VCC->3.3V.".format(IMU_ADDR))
        return
    print("  IMU ready. Pitch offset: {:.1f}deg  Roll offset: {:.1f}deg".format(
        IMU_PITCH_OFFSET_DEG, IMU_ROLL_OFFSET_DEG))

    # --- Initialise rotary axis state ---
    # Back-calculate home so that the IMU's current reading produces
    # zero error on the first tick — prevents the arm from lurching on startup
    # if the joystick is not perfectly centred.
    initial_joy_pitch, initial_joy_roll = read_imu_commands()

    roll_home_deg        = initial_angle - (initial_joy_roll  * ROLL_MAX_DEGREES)
    roll_target_deg      = initial_angle
    ema_joy_roll         = initial_joy_roll
    roll_pid_integral    = 0.0
    roll_pid_last_error  = 0.0
    roll_pid_last_actual = None
    print("Roll encoder OK. Roll home: {:.2f} deg".format(initial_angle))

    pitch_home_deg        = initial_pitch_angle - (initial_joy_pitch * PITCH_MAX_DEGREES)
    pitch_target_deg      = initial_pitch_angle
    ema_joy_pitch         = initial_joy_pitch
    pitch_pid_integral    = 0.0
    pitch_pid_last_error  = 0.0
    pitch_pid_last_actual = None
    print("Pitch encoder OK. Pitch home: {:.2f} deg".format(initial_pitch_angle))

    # --- Linear homing (automatic) ---
    # Skipped entirely in demo mode: the demo orbit only moves pitch/roll, and
    # handle_encoder_linear() is disabled in demo (stepper EMI). Homing would
    # just drive the carriage at homing speed toward the limit switch for no
    # reason — and if the switch isn't wired/triggering, it drives to end-of-ROM.
    if DEMO_MODE:
        print("Demo mode — skipping linear homing (linear axis is not used in the demo).")
    else:
        if estop_pin.value() == 1:
            estop_active  = True
            estop_handled = False
            print("WARNING: E-stop is active at startup. Release e-stop and press RE-ARM before homing.")

        print("")
        _boot_btn = Pin(0, Pin.IN, Pin.PULL_UP)
        print("Homing linear axis in 3s — press BOOT button or any key + Enter to skip...")
        skip_homing = False
        for _ in range(30):
            if _boot_btn.value() == 0:
                skip_homing = True
                break
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
                skip_homing = True
                break
            time.sleep_ms(100)

        if skip_homing:
            for _ in range(2):
                _status_led.value(1)
                time.sleep_ms(150)
                _status_led.value(0)
                time.sleep_ms(150)
            print("Homing skipped. Position tracking starts from 0.")
        elif estop_active:
            print("Homing skipped — E-STOP active. Release e-stop and re-arm before using linear axis.")
        else:
            print("Homing linear axis automatically...")
            home_linear_axis()

    print("")
    if DEMO_MODE:
        print("{}DEMO MODE ACTIVE{}\n".format(BOLD, RESET))
        # Seed from raw encoder position so orbit center starts at arm's actual position.
        pitch_home_deg   = initial_pitch_angle
        roll_home_deg    = initial_angle
        pitch_target_deg = initial_pitch_angle
        roll_target_deg  = initial_angle

        # Pre-demo jog: hold here until user positions arm and presses RE-ARM.
        print("  Jog arm to desired orbit center with trim joystick.")
        print("  Press RE-ARM button (GPIO5) when ready.\n")
        _pre_tick_ms = time.ticks_ms()
        while True:
            _loop_start  = time.ticks_ms()
            _dt          = time.ticks_diff(_loop_start, _pre_tick_ms)
            _pre_tick_ms = _loop_start

            if not rearm_pin.value():
                break
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
                break

            _linear_actual_mm = update_linear_encoder_mm()  # keep linear live before re-arm
            print_input_debug()   # so the visualizer shows live joystick state here
            read_limit_switches()  # pitch/roll crash switches are live even during pre-demo jog

            if not estop_active:
                trim_p = _read_trim(trim_joy_x, TRIM_JOY_INVERT_X)
                trim_r = _read_trim(trim_joy_y, TRIM_JOY_INVERT_Y)
                any_jog = (trim_p != 0.0 or trim_r != 0.0
                           or pitch_decelerating or roll_decelerating)
                if any_jog:
                    handle_jogging(trim_p, trim_r)
                else:
                    _run_pid(_dt)
            else:
                stop_motor(pitch_pwm)
                stop_motor(roll_pwm)

            _elapsed = time.ticks_diff(time.ticks_ms(), _loop_start)
            _sleep   = CONTROL_LOOP_MS - _elapsed
            if _sleep > 0:
                time.sleep_ms(_sleep)

        # Latch actual encoder position as orbit center; clear all jog momentum.
        _p = read_pitch_angle_deg()
        _r = read_roll_angle_deg()
        if _p is not None:
            pitch_home_deg = _p
        if _r is not None:
            roll_home_deg  = _r
        pitch_target_deg      = pitch_home_deg
        roll_target_deg       = roll_home_deg
        pitch_pid_integral    = 0.0
        pitch_pid_last_error  = 0.0
        pitch_pid_last_actual = None
        roll_pid_integral     = 0.0
        roll_pid_last_error   = 0.0
        roll_pid_last_actual  = None
        pitch_current_freq    = 0
        roll_current_freq     = 0
        pitch_decelerating    = False
        roll_decelerating     = False
        stop_motor(pitch_pwm)
        stop_motor(roll_pwm)

        print("\nRE-ARM pressed — demo starting. Orbit center: pitch={:.1f}°  roll={:.1f}°".format(
            pitch_home_deg, roll_home_deg))
        _demo_phase          = 0
        _demo_phase_start_ms = time.ticks_ms()
    else:
        print("Ready. Tilt IMU: pitch/roll. Trim joystick: jog home. Encoder: linear.\n")

    last_tick_ms = time.ticks_ms()   # reset after blocking init so first dt_ms is sane

    # --- Main control loop ---
    while True:
        loop_start   = time.ticks_ms()
        dt_ms        = time.ticks_diff(loop_start, last_tick_ms)
        last_tick_ms = loop_start

        # --- E-stop gate ---
        if estop_pin.value() == 1:   # belt-and-suspenders poll in case ISR edge was missed
            estop_active = True
        if estop_active:
            if not estop_handled:
                stop_motor(pitch_pwm)
                stop_motor(roll_pwm)
                stop_linear_motor()
                pitch_current_freq  = 0
                roll_current_freq   = 0
                linear_current_freq = 0
                estop_handled = True
                print("\n!!! E-STOP ACTIVE !!! Release e-stop, then press RE-ARM to resume.")

            if not rearm_pin.value():
                if estop_pin.value() == 1:
                    print("Release e-stop button before re-arming!   ")
                    time.sleep_ms(1000)
                else:
                    # Re-arm: snap PID state to current position so arm holds in place.
                    # Read live IMU to initialize ema_joy so the first PID tick has
                    # accurate ema values — stale pre-estop ema causes a D-term spike.
                    p_angle = read_pitch_angle_deg()
                    r_angle = read_roll_angle_deg()
                    _p, _r = read_imu_commands()
                    if p_angle is not None:
                        ema_joy_pitch         = _p
                        pitch_home_deg        = p_angle - (_p * PITCH_MAX_DEGREES)
                        pitch_target_deg      = p_angle
                        pitch_pid_last_error  = 0.0
                        pitch_pid_last_actual = None
                    if r_angle is not None:
                        ema_joy_roll          = _r
                        roll_home_deg         = r_angle - (_r * ROLL_MAX_DEGREES)
                        roll_target_deg       = r_angle
                        roll_pid_last_error   = 0.0
                        roll_pid_last_actual  = None
                    pitch_pid_integral   = 0.0
                    roll_pid_integral    = 0.0
                    pitch_decelerating   = False
                    roll_decelerating    = False
                    estop_active         = False
                    estop_handled        = False
                    print("Re-armed. Resuming.")

            # Rapid LED flash (4 Hz) while e-stop is active — covers both
            # "button still pressed" and "triggered but not yet re-armed".
            _status_led.value((time.ticks_ms() // 125) % 2)

            elapsed  = time.ticks_diff(time.ticks_ms(), loop_start)
            sleep_ms = CONTROL_LOOP_MS - elapsed
            if sleep_ms > 0:
                time.sleep_ms(sleep_ms)
            continue

        # --- Axis crash/limit switches ---
        read_limit_switches()
        if any(axis_fault.values()) and not rearm_pin.value():
            clear_axis_faults()

        if DEMO_MODE:
            handle_demo(dt_ms)
        else:
            trim_p = _read_trim(trim_joy_x, TRIM_JOY_INVERT_X)
            trim_r = _read_trim(trim_joy_y, TRIM_JOY_INVERT_Y)

            any_jog_active = (trim_p != 0.0 or trim_r != 0.0
                              or pitch_decelerating or roll_decelerating)

            if any_jog_active:
                handle_jogging(trim_p, trim_r)
            else:
                handle_main_input(dt_ms)

        if not DEMO_MODE:
            handle_encoder_linear()   # skip in demo — stepper EMI drives phantom counts
        _linear_actual_mm = update_linear_encoder_mm()
        print_input_debug()   # raw + scaled joystick / IMU for the visualizer input panel
        _status_led.value((time.ticks_ms() // 1000) % 2)

        elapsed  = time.ticks_diff(time.ticks_ms(), loop_start)
        sleep_ms = CONTROL_LOOP_MS - elapsed
        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

try:
    main()
finally:
    pitch_pwm.deinit()
    roll_pwm.deinit()
    linear_pwm.deinit()
    print("\nMotors stopped.")
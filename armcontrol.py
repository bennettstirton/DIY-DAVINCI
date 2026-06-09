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

from machine import Pin, ADC, PWM, SoftI2C
import math
import time
import sys
import select


# =============================================================================
# CONFIG — tune all parameters here
# =============================================================================

# --- Loop timing ---
CONTROL_LOOP_MS  = 40
CONTROL_LOOP_SEC = CONTROL_LOOP_MS / 1000.0

# --- Microstep settings ---
# IMPORTANT: These values must match your physical driver DIP switch settings.
#   DM556T  (Pitch):  valid values 1, 2, 4, 8, 16, 32
#   TB6600  (Roll):   valid values 1, 2, 4, 8, 16, 32
#   TB6600  (Linear): valid values 1, 2, 4, 8, 16, 32
PITCH_MICROSTEPS  = 16
ROLL_MICROSTEPS   = 4
LINEAR_MICROSTEPS = 4    # set TB6600 DIP switches to match

# --- Derived: steps per revolution ---
PITCH_STEPS_PER_REV  = 200 * PITCH_MICROSTEPS    # 16x → 3200 steps/rev
ROLL_STEPS_PER_REV   = 200 * ROLL_MICROSTEPS     #  4x →  800 steps/rev
LINEAR_STEPS_PER_REV = 200 * LINEAR_MICROSTEPS   #  4x →  800 steps/rev

# --- Linear axis geometry ---
# Cable spool diameter: 20mm  →  circumference = 20 * pi ≈ 62.83 mm/rev
# Total travel: LINEAR_MAX_MM (175mm)  →  175 / 62.83 ≈ 2.8 revolutions max
#
# NOTE: cable spool effective diameter grows slightly as cable layers accumulate.
# Over ~3 revolutions this is a small effect but means steps/mm drifts
# slightly near full extension. Acceptable for most applications.
LINEAR_SPOOL_DIAMETER_MM = 20.0
LINEAR_SPOOL_CIRC_MM     = math.pi * LINEAR_SPOOL_DIAMETER_MM   # ~62.83 mm/rev
LINEAR_STEPS_PER_MM      = LINEAR_STEPS_PER_REV / LINEAR_SPOOL_CIRC_MM
LINEAR_MAX_MM            = 175.0
LINEAR_MAX_STEPS         = int(LINEAR_MAX_MM * LINEAR_STEPS_PER_MM)

# --- Optical encoder (linear axis input) ---
#
# !! FILL THIS IN BEFORE RUNNING !!
# PPR (Pulses Per Revolution) is printed on the encoder body or its datasheet.
# Common values: 100, 200, 360, 600. Using 4x quadrature decoding, the actual
# resolution the code sees is PPR * 4 counts per revolution.
ENCODER_PPR = 600   # <-- UPDATE THIS to match your encoder

# Gear ratio between optical encoder and linear stepper.
# ENCODER_SCALE = 1.0 means: 1 full turn of the encoder → 1 full rev of the stepper.
# ENCODER_SCALE = 2.0 means: 0.5 turns of the encoder → 1 full rev of the stepper (faster/coarser).
# ENCODER_SCALE = 0.5 means: 2 full turns of the encoder → 1 full rev of the stepper (slower/finer).
# Start conservatively (0.5 or 1.0) and increase once motion feels right.
ENCODER_SCALE = 0.75

# Derived: how many stepper steps result from one encoder count
# (ENCODER_PPR * 4 because we decode all 4 edges per cycle = 4x resolution)
_ENCODER_COUNTS_PER_REV  = ENCODER_PPR * 4
_STEPS_PER_ENCODER_COUNT = (LINEAR_STEPS_PER_REV * ENCODER_SCALE) / _ENCODER_COUNTS_PER_REV

# --- GPIO pin assignments ---
PITCH_STEP_PIN  = 14
PITCH_DIR_PIN   = 27
ROLL_STEP_PIN   = 26
ROLL_DIR_PIN    = 25
LINEAR_STEP_PIN = 13
LINEAR_DIR_PIN  = 15

# Main input: MPU6050 IMU (replaces the 2-axis analog joystick formerly on
# GPIO 34/35 — those wires now carry the IMU's SDA/SCL to the I2C bus below).
# The IMU shares the existing AS5600/TCA9548A I2C bus (GPIO 21/22) — its
# address (0x68) doesn't collide with the TCA9548A (0x70) or AS5600s (0x36,
# selected behind the mux).
IMU_ADDR = 0x68

# One-time calibration offsets — measured with armcontrolsetup.py Test D
# (run with zeroing skipped, note the raw reading at the working-neutral
# mount position, enter the correction needed to bring it to 0).
# calibrated_angle = raw_accel_angle + offset
IMU_PITCH_OFFSET_DEG = 0.0
IMU_ROLL_OFFSET_DEG  = -90.0

# Physical tilt range (degrees from calibrated neutral) that maps to full
# +/-1.0 command — i.e. the IMU equivalent of full joystick deflection.
IMU_PITCH_MAX_TILT_DEG = 30.0
IMU_ROLL_MAX_TILT_DEG  = 30.0

# Tilt within this many degrees of neutral reads as zero (prevents jitter
# at rest from being interpreted as a command).
IMU_DEADBAND_PITCH_DEG = 1.5
IMU_DEADBAND_ROLL_DEG  = 1.5

# Trim joystick (replaces 4 rocker switches, freeing GPIO 18 and 19)
# Wire X axis to GPIO 32, Y axis to GPIO 33 (both ADC1, safe for use).
TRIM_JOY_X_PIN = 32   # controls Pitch trim
TRIM_JOY_Y_PIN = 33   # controls Roll trim

# Optical encoder pins (previously linear rocker FWD/BWD)
# Wire via voltage divider if encoder runs on 5V (see wiring notes at top of file).
#   Channel A → R1(1kΩ) → junction → R2(2kΩ) → GND  ; junction → GPIO 16
#   Channel B → R1(1kΩ) → junction → R2(2kΩ) → GND  ; junction → GPIO 17
ENCODER_A_PIN = 16
ENCODER_B_PIN = 17

# Limit switch — wired between GPIO23 and GND (pull-up used, normally closed)
# Triggered when the carriage reaches the home (fully retracted) end.
LINEAR_LIMIT_PIN = 23

# E-stop — NC (normally closed) button between GPIO4 and GND (pull-up used).
# Normal: button closed → pin LOW.  E-stop pressed (or wire break): pin HIGH.
# Fail-safe: a broken wire triggers the stop, same as pressing the button.
ESTOP_PIN = 4

# Re-arm — NO (normally open) momentary button between GPIO5 and GND (pull-up used).
# Press after releasing the e-stop mushroom to resume without a full ESP32 reset.
REARM_PIN = 5

# --- I2C bus + TCA9548A multiplexer ---
# Both AS5600 encoders share one I2C bus via the TCA9548A multiplexer.
# TCA9548A wiring: VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22, A0/A1/A2→GND (addr 0x70).
# Each AS5600 connects to its own TCA channel — no address collision.
AS5600_SDA_PIN       = 21
AS5600_SCL_PIN       = 22
AS5600_I2C_FREQ      = 100000
AS5600_ADDR          = 0x36
AS5600_RAW_ANGLE_REG = 0x0C
TCA9548A_ADDR        = 0x70
ROLL_TCA_CHANNEL     = 0    # roll  AS5600 on TCA channel 0
PITCH_TCA_CHANNEL    = 1    # pitch AS5600 on TCA channel 1

# Flip to True if encoder reads increasing angles in the wrong direction.
ROLL_ENCODER_INVERT   = False
PITCH_ENCODER_INVERT  = False
LINEAR_ENCODER_INVERT = False  # flip to True if extending input decreases encoder_position_steps

# Flip to True if the position counter moves in the wrong direction after
# LINEAR_INVERT_DIR is already set correctly for motor direction.
# These two flags are intentionally independent — motor direction and counter
# direction can be set separately without affecting each other or homing.
LINEAR_POSITION_INVERT = True

# Physical home offset. Jog to desired zero, note the printed angle, enter it here.
ROLL_ENCODER_OFFSET_DEG  = 0.0
PITCH_ENCODER_OFFSET_DEG = 0.0

# --- Position control scaling ---
ROLL_MAX_DEGREES  = 15.0 # was 45.0
PITCH_MAX_DEGREES = 15.0 # was 45.0

# --- PWM motor speed ---
PITCH_MAX_RPS  = 1.0    # rev/sec — NEMA 23 / DM556T
ROLL_MAX_RPS   = 10.0   # rev/sec — NEMA 17 + 10:1 gearbox
LINEAR_MAX_RPS = 2.0    # rev/sec — tune to taste; start conservatively

# --- Derived: frequency limits ---
PITCH_MAX_FREQ  = int(PITCH_MAX_RPS  * PITCH_STEPS_PER_REV)
ROLL_MAX_FREQ   = int(ROLL_MAX_RPS   * ROLL_STEPS_PER_REV)
LINEAR_MAX_FREQ = int(LINEAR_MAX_RPS * LINEAR_STEPS_PER_REV)

# Minimum PWM frequency — below this the motor stops entirely.
MIN_FREQ = 20

# --- PID gains (rotary axes only — linear is open-loop) ---
ROLL_KP       = 50.0
ROLL_KI       = 5.0
ROLL_KD       = 1.0
ROLL_KI_CLAMP = 500.0

PITCH_KP       = 50.0
PITCH_KI       = 5.0
PITCH_KD       = 3.0
PITCH_KI_CLAMP = 500.0

# --- Position deadbands ---
PITCH_POSITION_DEADBAND_DEG = 1.0
ROLL_POSITION_DEADBAND      = 1.0

# --- Trim joystick ADC config (must match physical joystick) ---
TRIM_JOY_MIN      = 200
TRIM_JOY_MAX      = 3895
TRIM_JOY_CENTRE   = 2048
TRIM_JOY_DEADBAND = 300
TRIM_JOY_INVERT_X = False
TRIM_JOY_INVERT_Y = False

# --- Jogging (trim joystick) ---
PITCH_JOG_RPS = 0.1 # Open loop. Does not factor in capstan reduction.
ROLL_JOG_RPS  = 1.0 # Open loop. Does not factor in capstan/gearbox reduction.

PITCH_JOG_FREQ = int(PITCH_JOG_RPS * PITCH_STEPS_PER_REV)
ROLL_JOG_FREQ  = int(ROLL_JOG_RPS  * ROLL_STEPS_PER_REV)

# --- Jog ramp rates ---
ROLL_JOG_ACCEL_RPS2 = 3.0
ROLL_JOG_DECEL_RPS2 = 3.0
ROLL_JOG_ACCEL_HZ   = max(1, int(ROLL_JOG_ACCEL_RPS2  * ROLL_STEPS_PER_REV  * CONTROL_LOOP_SEC))
ROLL_JOG_DECEL_HZ   = max(1, int(ROLL_JOG_DECEL_RPS2  * ROLL_STEPS_PER_REV  * CONTROL_LOOP_SEC))

PITCH_JOG_ACCEL_RPS2 = 1.0
PITCH_JOG_DECEL_RPS2 = 1.0
PITCH_JOG_ACCEL_HZ   = max(1, int(PITCH_JOG_ACCEL_RPS2 * PITCH_STEPS_PER_REV * CONTROL_LOOP_SEC))
PITCH_JOG_DECEL_HZ   = max(1, int(PITCH_JOG_DECEL_RPS2 * PITCH_STEPS_PER_REV * CONTROL_LOOP_SEC))

# Deceleration rate when optical encoder stops turning.
# Higher value = stops more abruptly. Lower = coasts to a stop.
LINEAR_ENC_DECEL_RPS2 = 4.0
LINEAR_ENC_DECEL_HZ   = max(1, int(LINEAR_ENC_DECEL_RPS2 * LINEAR_STEPS_PER_REV * CONTROL_LOOP_SEC))

# --- Step direction polarity ---
PITCH_INVERT_DIR  = False
ROLL_INVERT_DIR   = False
LINEAR_INVERT_DIR = True   # flip if extend/retract are physically backwards

# --- Homing speed ---
# Keep this conservative — the carriage hits the switch at this speed.
LINEAR_HOMING_RPS  = 0.1
LINEAR_HOMING_FREQ = int(LINEAR_HOMING_RPS * LINEAR_STEPS_PER_REV)

# --- IMU input EMA smoothing ---
# Lower = smoother but more sluggish. Higher = more responsive but noisier.
EMA_ALPHA = 0.2

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

# Limit switch: normally closed (reads 1 at rest, drops to 0 when carriage opens it)
linear_limit = Pin(LINEAR_LIMIT_PIN, Pin.IN, Pin.PULL_UP)

# E-stop: normally closed button pulls pin LOW at rest; opening (press or wire break)
# lets pull-up take it HIGH, triggering the ISR.
estop_pin = Pin(ESTOP_PIN, Pin.IN, Pin.PULL_UP)
rearm_pin = Pin(REARM_PIN, Pin.IN, Pin.PULL_UP)

# Single I2C bus → TCA9548A → both AS5600s, plus the MPU6050 IMU (GPIO 21/22).
# The IMU sits directly on the bus (not behind the mux) — its address (0x68)
# doesn't collide with the TCA9548A (0x70) or the AS5600s (0x36, mux-selected).
i2c = SoftI2C(sda=Pin(AS5600_SDA_PIN), scl=Pin(AS5600_SCL_PIN),
              freq=AS5600_I2C_FREQ)

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

def _estop_isr(_pin):
    global estop_active
    estop_active = True
    # Kill PWM immediately — duty(0) is a safe hardware register write from an ISR.
    # linear_pwm.deinit() cannot be called here; main loop handles it on the next tick.
    pitch_pwm.duty(0)
    roll_pwm.duty(0)
    linear_pwm.duty(0)

estop_pin.irq(trigger=Pin.IRQ_RISING, handler=_estop_isr)

# =============================================================================
# STATE
# =============================================================================

pitch_home_deg       = 0.0
pitch_target_deg     = 0.0
pitch_pid_integral   = 0.0
pitch_pid_last_error = 0.0

roll_home_deg       = 0.0
roll_target_deg     = 0.0
roll_pid_integral   = 0.0
roll_pid_last_error = 0.0

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

# Open-loop position counter for linear axis (in steps from home).
# Positive = extended away from home.
linear_position_steps  = 0
encoder_position_steps = 0   # absolute input device position, in arm-equivalent steps
                              # (unclamped — allowed to wander outside arm's valid range)
linear_is_homed        = False

debug_last_print_ms = 0
last_tick_ms = time.ticks_ms()

# =============================================================================
# AS5600 ENCODER FUNCTIONS
# =============================================================================

def tca_select(channel):
    """Activate one TCA9548A channel. Must be called before reading an AS5600."""
    i2c.writeto(TCA9548A_ADDR, bytes([1 << channel]))


def read_as5600_raw():
    """Read the 12-bit raw angle (0-4095) from the roll AS5600. Returns None on failure."""
    tca_select(ROLL_TCA_CHANNEL)
    for _ in range(3):
        try:
            data = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            return ((data[0] & 0x0F) << 8) | data[1]
        except OSError:
            time.sleep_us(500)
    return None


def read_roll_angle_deg():
    """
    Read the roll joint angle in degrees, applying offset and invert.
    Returns angle relative to ROLL_ENCODER_OFFSET_DEG, wrapped to [-180, +180].
    Returns None on read failure.
    """
    raw = read_as5600_raw()
    if raw is None:
        return None
    angle = (raw / 4096.0) * 360.0 - ROLL_ENCODER_OFFSET_DEG
    if angle > 180.0:
        angle -= 360.0
    elif angle < -180.0:
        angle += 360.0
    return -angle if ROLL_ENCODER_INVERT else angle


def read_pitch_as5600_raw():
    """Read the 12-bit raw angle (0-4095) from the pitch AS5600. Returns None on failure."""
    tca_select(PITCH_TCA_CHANNEL)
    for _ in range(3):
        try:
            data = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            return ((data[0] & 0x0F) << 8) | data[1]
        except OSError:
            time.sleep_us(500)
    return None


def read_pitch_angle_deg():
    """Read the pitch joint angle in degrees, wrapped to [-180, +180]. Returns None on failure."""
    raw = read_pitch_as5600_raw()
    if raw is None:
        return None
    angle = (raw / 4096.0) * 360.0 - PITCH_ENCODER_OFFSET_DEG
    if angle > 180.0:
        angle -= 360.0
    elif angle < -180.0:
        angle += 360.0
    return -angle if PITCH_ENCODER_INVERT else angle


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


def read_pitch_imu():
    """
    Read IMU pitch tilt with deadband applied, normalised to [-1.0, +1.0]
    over IMU_PITCH_MAX_TILT_DEG. Same contract as the old read_pitch_joystick():
    callers (handle_main_input, handle_jogging, main) need no changes.
    """
    pitch, _ = read_imu_angles()
    if pitch is None:
        return 0.0
    if abs(pitch) < IMU_DEADBAND_PITCH_DEG:
        return 0.0
    return max(-1.0, min(1.0, pitch / IMU_PITCH_MAX_TILT_DEG))


def read_roll_imu():
    """
    Read IMU roll tilt with deadband applied, normalised to [-1.0, +1.0]
    over IMU_ROLL_MAX_TILT_DEG. Same contract as the old read_roll_joystick().
    """
    _, roll = read_imu_angles()
    if roll is None:
        return 0.0
    if abs(roll) < IMU_DEADBAND_ROLL_DEG:
        return 0.0
    return max(-1.0, min(1.0, roll / IMU_ROLL_MAX_TILT_DEG))


# =============================================================================
# LIMIT SWITCH HELPER
# =============================================================================

def limit_switch_triggered():
    """Return True if the normally-closed limit switch is triggered (circuit opened = reads 1)."""
    return linear_limit.value() == 1


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
        pwm.freq(freq)
        pwm.duty(512)
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
    """
    Reliably stop the linear axis PWM on ESP32.
    pwm.duty(0) alone is not sufficient — the LEDC timer keeps running and
    may continue outputting pulses. Deinit + reinit fully resets the hardware.
    """
    global linear_pwm
    linear_pwm.deinit()
    linear_pwm = PWM(Pin(LINEAR_STEP_PIN), freq=1000, duty=0)


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
    global linear_position_steps, encoder_position_steps, linear_is_homed, linear_current_freq, linear_pwm

    print("Homing linear axis... driving toward limit switch.")

    # If the switch is already triggered, skip the drive phase but still back off.
    # Without backoff the switch stays physically pressed, which blocks all
    # retract motion and leaves encoder_position_steps unsynced.
    if limit_switch_triggered():
        print("Limit switch already triggered — skipping drive phase, backing off...")
        linear_position_steps  = 0
        linear_is_homed        = True
        stop_motor(linear_pwm)
        linear_current_freq    = 0

        backoff_steps = LINEAR_STEPS_PER_REV // 2
        backoff_delay = 1.0 / (2 * LINEAR_HOMING_FREQ)
        linear_dir.value(0 if not LINEAR_INVERT_DIR else 1)
        for _ in range(backoff_steps):
            linear_pwm.freq(LINEAR_HOMING_FREQ)
            linear_pwm.duty(512)
            time.sleep(backoff_delay)

        linear_pwm.deinit()
        linear_pwm             = PWM(Pin(LINEAR_STEP_PIN), freq=1000, duty=0)
        linear_current_freq    = 0
        linear_position_steps  = backoff_steps
        encoder_position_steps = backoff_steps   # sync input to arm
        print("Backoff complete. Position zeroed at limit switch, now {:.2f}mm extended.".format(
            backoff_steps / LINEAR_STEPS_PER_MM))
        return True

    # Drive in the retract direction at homing speed.
    # Passing LINEAR_INVERT_DIR as the forward argument means set_motor's internal
    # XOR always cancels to the same physical pin state regardless of invert setting.
    set_motor(linear_pwm, linear_dir, LINEAR_INVERT_DIR, LINEAR_HOMING_FREQ, LINEAR_INVERT_DIR, 0)

    while not limit_switch_triggered() and not estop_active:
        time.sleep_ms(5)   # poll at 200 Hz during homing

    stop_motor(linear_pwm)
    linear_current_freq   = 0
    if estop_active:
        print("Homing aborted — E-STOP triggered.")
        return
    linear_position_steps = 0
    linear_is_homed       = True
    print("Linear axis homed. Backing off limit switch...")

    # Move 0.5 revolutions in the extend direction to clear the limit switch.
    backoff_steps = LINEAR_STEPS_PER_REV // 2
    backoff_delay = 1.0 / (2 * LINEAR_HOMING_FREQ)
    linear_dir.value(0 if not LINEAR_INVERT_DIR else 1)
    for _ in range(backoff_steps):
        linear_pwm.freq(LINEAR_HOMING_FREQ)
        linear_pwm.duty(512)
        time.sleep(backoff_delay)

    linear_pwm.deinit()
    linear_pwm = PWM(Pin(LINEAR_STEP_PIN), freq=1000, duty=0)

    linear_current_freq    = 0
    linear_position_steps  = backoff_steps
    encoder_position_steps = backoff_steps   # keep input in sync with arm after homing
    print("Backoff complete. Position zeroed at limit switch, now {:.2f}mm extended.".format(
        backoff_steps / LINEAR_STEPS_PER_MM))
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
    global roll_pid_integral, roll_pid_last_error
    global pitch_pid_integral, pitch_pid_last_error
    global ema_joy_roll, ema_joy_pitch
    global pitch_decelerating, roll_decelerating

    # -------------------------------------------------------------------------
    # PITCH
    # -------------------------------------------------------------------------
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
            pitch_pid_integral   = 0.0
            pitch_pid_last_error = 0.0
            angle = read_pitch_angle_deg()
            if angle is not None:
                joy_now          = read_pitch_imu()
                ema_joy_pitch    = joy_now
                pitch_home_deg   = angle - (joy_now * PITCH_MAX_DEGREES)
                pitch_target_deg = angle
        else:
            pitch_current_freq = set_motor(pitch_pwm, pitch_dir, PITCH_INVERT_DIR,
                                           pitch_current_freq, pitch_last_forward, prev)

    # -------------------------------------------------------------------------
    # ROLL
    # -------------------------------------------------------------------------
    roll_target_freq = int(abs(trim_r) * ROLL_JOG_FREQ)
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
            roll_pid_integral   = 0.0
            roll_pid_last_error = 0.0
            angle = read_roll_angle_deg()
            if angle is not None:
                joy_now             = read_roll_imu()
                ema_joy_roll        = joy_now
                roll_home_deg       = angle - (joy_now * ROLL_MAX_DEGREES)
                roll_target_deg     = angle
                roll_pid_last_error = 0.0
        else:
            roll_current_freq = set_motor(roll_pwm, roll_dir, ROLL_INVERT_DIR,
                                          roll_current_freq, roll_last_forward, prev)

    # -------------------------------------------------------------------------
    # Home tracking (rotary axes only)
    # -------------------------------------------------------------------------
    #angle = read_pitch_angle_deg()
    #if angle is not None:
    #    pitch_home_deg   = angle
    #    pitch_pid_last_error = pitch_target_deg - angle
#
    #angle = read_roll_angle_deg()
    #if angle is not None:
     #   roll_home_deg       = angle
     #   roll_target_deg     = angle
    #  roll_pid_last_error = roll_target_deg - angle


# =============================================================================
# DEBUG / PRINT
# =============================================================================

def debug_print(pitch_target, pitch_actual, roll_target, roll_actual):
    global debug_last_print_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, debug_last_print_ms) < 100:
        return
    debug_last_print_ms = now
    linear_mm = linear_position_steps / LINEAR_STEPS_PER_MM
    print("| {}PITCH:{} Tgt:{:5.1f} Act:{:5.1f} Err:{:4.1f}"
          " | {}ROLL:{} Tgt:{:5.1f} Act:{:5.1f} Err:{:4.1f}"
          " | {}LINEAR:{} {:5.1f}mm / {:.0f}mm".format(
              BOLD, RESET, pitch_target, pitch_actual, pitch_target - pitch_actual,
              BOLD, RESET, roll_target,  roll_actual,  roll_target  - roll_actual,
              BOLD, RESET, linear_mm, LINEAR_MAX_MM))


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
    encoder_a.irq(handler=None)
    encoder_b.irq(handler=None)
    delta = encoder_count
    encoder_count = 0
    encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)
    encoder_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)

    # Normalise so positive delta always means "extend" intent.
    if LINEAR_ENCODER_INVERT:
        delta = -delta

    # ------------------------------------------------------------------
    # 2. Advance the absolute input position (UNCLAMPED).
    #    This is the only place encoder_position_steps is updated.
    # ------------------------------------------------------------------
    encoder_position_steps += int(delta * _STEPS_PER_ENCODER_COUNT)

    # ------------------------------------------------------------------
    # 3. Commanded position = input position clamped to arm's valid range.
    # ------------------------------------------------------------------
    commanded_steps = max(0, min(LINEAR_MAX_STEPS, encoder_position_steps))

    # How far does the arm still need to travel?
    error = commanded_steps - linear_position_steps

    # Steps the motor will take this tick at the current frequency
    # (used for the position estimate below).
    steps_this_tick = int(linear_current_freq * CONTROL_LOOP_SEC)

    if delta != 0 and error != 0:
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
        desired_steps = abs(delta) * _STEPS_PER_ENCODER_COUNT
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
# JOYSTICK POSITION CONTROL
# =============================================================================

def handle_main_input(dt_ms):
    """
    Drive pitch and roll toward joystick-commanded targets using PID.
    Linear axis is encoder-only — nothing happens here for that axis.
    """
    global ema_joy_pitch, ema_joy_roll
    global pitch_current_freq, roll_current_freq
    global pitch_target_deg, pitch_home_deg, pitch_pid_integral, pitch_pid_last_error
    global roll_target_deg, roll_home_deg, roll_pid_integral, roll_pid_last_error

    dt_sec = max(dt_ms / 1000.0, 0.001)

    ema_joy_pitch = EMA_ALPHA * read_pitch_imu() + (1.0 - EMA_ALPHA) * ema_joy_pitch
    ema_joy_roll  = EMA_ALPHA * read_roll_imu()  + (1.0 - EMA_ALPHA) * ema_joy_roll

    # -------------------------------------------------------------------------
    # PITCH — closed-loop PID
    # -------------------------------------------------------------------------
    pitch_target_deg = pitch_home_deg + (ema_joy_pitch * PITCH_MAX_DEGREES)

    actual_pitch = read_pitch_angle_deg()
    if actual_pitch is None:
        stop_motor(pitch_pwm)
        pitch_current_freq = 0
        print("WARNING: Pitch AS5600 read failed. Check wiring.")
        return

    error_pitch = wrap_angle_error(pitch_target_deg - actual_pitch)

    if abs(error_pitch) <= PITCH_POSITION_DEADBAND_DEG:
        stop_motor(pitch_pwm)
        pitch_current_freq   = 0
        pitch_pid_integral   = 0.0
        pitch_pid_last_error = error_pitch
    else:
        p_term = PITCH_KP * error_pitch
        pitch_pid_integral   = max(-PITCH_KI_CLAMP,
                                   min(PITCH_KI_CLAMP, pitch_pid_integral + error_pitch * dt_sec))
        i_term = PITCH_KI * pitch_pid_integral
        d_term = PITCH_KD * (error_pitch - pitch_pid_last_error) / dt_sec
        pitch_pid_last_error = error_pitch

        pid_output         = p_term + i_term + d_term
        desired_freq_pitch = min(int(abs(pid_output)), PITCH_MAX_FREQ)
        forward_pitch      = pid_output > 0
        prev_pitch         = pitch_current_freq

        if desired_freq_pitch <= MIN_FREQ:
            stop_motor(pitch_pwm)
            pitch_current_freq = 0
        else:
            pitch_current_freq = set_motor(pitch_pwm, pitch_dir, PITCH_INVERT_DIR, desired_freq_pitch, forward_pitch, prev_pitch)

    # -------------------------------------------------------------------------
    # ROLL — closed-loop PID
    # -------------------------------------------------------------------------
    roll_target_deg = roll_home_deg + (ema_joy_roll * ROLL_MAX_DEGREES)

    actual_roll = read_roll_angle_deg()
    if actual_roll is None:
        stop_motor(roll_pwm)
        roll_current_freq = 0
        print("WARNING: Roll AS5600 read failed. Check wiring.")
        return

    error_roll = wrap_angle_error(roll_target_deg - actual_roll)

    if abs(error_roll) <= ROLL_POSITION_DEADBAND:
        stop_motor(roll_pwm)
        roll_current_freq   = 0
        roll_pid_integral   = 0.0
        roll_pid_last_error = error_roll
    else:
        p_term = ROLL_KP * error_roll
        roll_pid_integral   = max(-ROLL_KI_CLAMP,
                                  min(ROLL_KI_CLAMP, roll_pid_integral + error_roll * dt_sec))
        i_term = ROLL_KI * roll_pid_integral
        d_term = ROLL_KD * (error_roll - roll_pid_last_error) / dt_sec
        roll_pid_last_error = error_roll

        pid_output        = p_term + i_term + d_term
        desired_freq_roll = min(int(abs(pid_output)), ROLL_MAX_FREQ)
        forward_roll      = pid_output > 0
        prev_roll         = roll_current_freq

        if desired_freq_roll <= MIN_FREQ:
            stop_motor(roll_pwm)
            roll_current_freq = 0
        else:
            roll_current_freq = set_motor(roll_pwm, roll_dir, ROLL_INVERT_DIR, desired_freq_roll, forward_roll, prev_roll)

    debug_print(pitch_target_deg, actual_pitch, roll_target_deg, actual_roll)


# =============================================================================
# MAIN
# =============================================================================

def main():
    global last_tick_ms, roll_home_deg, roll_target_deg, ema_joy_roll
    global pitch_home_deg, pitch_target_deg, ema_joy_pitch
    global roll_pid_integral, roll_pid_last_error
    global pitch_pid_integral, pitch_pid_last_error
    global linear_is_homed
    global estop_active, estop_handled
    global pitch_current_freq, roll_current_freq, linear_current_freq
    global pitch_decelerating, roll_decelerating

    print("Scanning I2C bus...")
    devices = i2c.scan()
    print("  Bus (expect 0x70 for TCA9548A):  ", [hex(d) for d in devices])
    tca_select(ROLL_TCA_CHANNEL)
    devices = i2c.scan()
    print("  Roll  channel (expect 0x36):     ", [hex(d) for d in devices])
    tca_select(PITCH_TCA_CHANNEL)
    devices = i2c.scan()
    print("  Pitch channel (expect 0x36):     ", [hex(d) for d in devices])

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
        ENCODER_PPR, 4, _ENCODER_COUNTS_PER_REV, ENCODER_SCALE))
    print("         {:.4f} stepper steps per encoder count".format(_STEPS_PER_ENCODER_COUNT))
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
        print("Check: VCC->3.3V, GND->GND, SDA->GPIO4, SCL->GPIO5.")
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
    initial_joy_roll  = read_roll_imu()
    initial_joy_pitch = read_pitch_imu()

    roll_home_deg        = initial_angle - (initial_joy_roll  * ROLL_MAX_DEGREES)
    roll_target_deg      = initial_angle
    ema_joy_roll         = initial_joy_roll
    roll_pid_integral    = 0.0
    roll_pid_last_error  = 0.0
    print("Roll encoder OK. Roll home: {:.2f} deg".format(initial_angle))

    pitch_home_deg       = initial_pitch_angle - (initial_joy_pitch * PITCH_MAX_DEGREES)
    pitch_target_deg     = initial_pitch_angle
    ema_joy_pitch        = initial_joy_pitch
    pitch_pid_integral   = 0.0
    pitch_pid_last_error = 0.0
    print("Pitch encoder OK. Pitch home: {:.2f} deg".format(initial_pitch_angle))

    # --- Linear homing (automatic) ---
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
    else:
        print("Homing linear axis automatically...")
        home_linear_axis()

    print("")
    print("Ready. Tilt IMU: pitch/roll. Trim joystick: jog home. Encoder: linear.\n")

    # --- Main control loop ---
    while True:
        loop_start   = time.ticks_ms()
        dt_ms        = time.ticks_diff(loop_start, last_tick_ms)
        last_tick_ms = loop_start

        # --- E-stop gate ---
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
                    # Re-arm: snap PID state to current position so arm holds in place
                    p_angle = read_pitch_angle_deg()
                    r_angle = read_roll_angle_deg()
                    if p_angle is not None:
                        joy_now          = read_pitch_imu()
                        ema_joy_pitch    = joy_now
                        pitch_home_deg   = p_angle - (joy_now * PITCH_MAX_DEGREES)
                        pitch_target_deg = p_angle
                    if r_angle is not None:
                        joy_now          = read_roll_imu()
                        ema_joy_roll     = joy_now
                        roll_home_deg    = r_angle - (joy_now * ROLL_MAX_DEGREES)
                        roll_target_deg  = r_angle
                    pitch_pid_integral   = 0.0
                    pitch_pid_last_error = 0.0
                    roll_pid_integral    = 0.0
                    roll_pid_last_error  = 0.0
                    pitch_decelerating   = False
                    roll_decelerating    = False
                    estop_active         = False
                    estop_handled        = False
                    print("Re-armed. Resuming.")

            elapsed  = time.ticks_diff(time.ticks_ms(), loop_start)
            sleep_ms = CONTROL_LOOP_MS - elapsed
            if sleep_ms > 0:
                time.sleep_ms(sleep_ms)
            continue

        trim_p = _read_trim(trim_joy_x, TRIM_JOY_INVERT_X)
        trim_r = _read_trim(trim_joy_y, TRIM_JOY_INVERT_Y)

        any_jog_active = (trim_p != 0.0 or trim_r != 0.0
                          or pitch_decelerating or roll_decelerating)

        if any_jog_active:
            handle_jogging(trim_p, trim_r)
        else:
            handle_main_input(dt_ms)

        handle_encoder_linear()   # always runs, independent of jog state
        _status_led.value((time.ticks_ms() // 1000) % 2)

        elapsed  = time.ticks_diff(time.ticks_ms(), loop_start)
        sleep_ms = CONTROL_LOOP_MS - elapsed
        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

main()
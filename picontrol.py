# =============================================================================
# Surgical Robot — Raspberry Pi Control
# Target: Raspberry Pi 5, Python 3
#
# Reads operator input and translates it to instrument position commands
# sent to the instrument ESP32 over USB serial.
#
# Input modes (set INPUT_MODE in CONFIG):
#   "joystick" — 2-axis analog joystick via ADS1115 ADC (I2C).
#                X axis → pitch, Y axis → roll. Yaw stays at home.
#                Use this while the BNO055 is in transit.
#   "bno055"   — BNO055 9-DOF IMU (I2C). Full pitch/roll/yaw from wrist motion.
#                Flip to this once the sensor arrives.
#
# Control mode: DELTA (Da Vinci-style)
# ──────────────────────────────────────
#   Both input modes feed into the same delta accumulator. The instrument moves
#   only when the operator's hand/wrist moves (or when the joystick is off center).
#
#   - Clutch RELEASED: deltas accumulate into instrument position each tick.
#   - Clutch PRESSED:  instrument position is frozen. Input keeps reading so
#                      there is no position jump when the clutch is released.
#
#   Clutch input: normally-open switch on Pi GPIO (CLUTCH_GPIO_PIN).
#   Set CLUTCH_GPIO_PIN = None to disable (instrument always moves — bench test).
#
# Serial protocol (matches instrumentcontrol.py):
#   Pi → ESP32:  {"p": <steps>, "r": <steps>, "y": <steps>}\n
#   Pi → ESP32:  {"cmd": "home"}\n   or   {"cmd": "stop"}\n
#   ESP32 → Pi:  {"p": <pos>, "r": <pos>, "y": <pos>}\n  (logged, not acted on)
#
# Installation (run once on Pi):
#   pip install adafruit-blinka pyserial RPi.GPIO
#   pip install adafruit-circuitpython-bno055        # only needed for bno055 mode
#   pip install adafruit-circuitpython-ads1x15       # only needed for joystick mode
#
# I2C must be enabled: raspi-config → Interface Options → I2C
# =============================================================================

import time
import json
import board
import busio
import serial

# =============================================================================
# CONFIG — tune all parameters here
# =============================================================================

# --- Input source ---
# "joystick" : ADS1115 + analog joystick (use now, while BNO055 is in mail)
# "bno055"   : BNO055 IMU (swap to this once sensor arrives)
INPUT_MODE = "joystick"

# --- Serial port to instrument ESP32 ---
# Run `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` to find the correct port.
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

# --- Scale factors: "degrees" of input motion → stepper steps ---
# Baseline (no EndoWrist gearing): 2048 steps/rev ÷ 360°/rev = 5.69 steps/deg.
# Tune after mechanical assembly: rotate/push 45° of input, measure instrument
# travel, adjust until response feels 1:1 (or whatever ratio feels natural).
# Sign controls direction: negate any axis to invert it.
STEPS_PER_DEG_PITCH =  5.69
STEPS_PER_DEG_ROLL  =  5.69
STEPS_PER_DEG_YAW   =  5.69

# --- Soft limits (steps from home) ---
# ESP32 also enforces these — this is a redundant belt-and-suspenders guard.
PITCH_MAX_STEPS = 1024
ROLL_MAX_STEPS  = 1024
YAW_MAX_STEPS   = 1024

# --- Clutch GPIO ---
# BCM pin number for foot-pedal switch (normally open, active LOW, internal pull-up).
# Set to None to disable (no hardware clutch yet).
CLUTCH_GPIO_PIN = None   # e.g. 17 once foot pedal is wired

# --- Control loop ---
LOOP_HZ     = 25
LOOP_PERIOD = 1.0 / LOOP_HZ

# --- Debug printing ---
DEBUG_HZ = 2   # Lines printed per second. Set 0 to disable.

# ── Joystick / ADS1115 config (only used when INPUT_MODE = "joystick") ────────
# ADS1115 I2C address: 0x48 default, 0x49 if ADDR tied to VCC.
ADS1115_ADDRESS   = 0x48
JOY_PITCH_CHANNEL = 0    # ADS1115 channel for joystick X → pitch
JOY_ROLL_CHANNEL  = 1    # ADS1115 channel for joystick Y → roll
# Wiring: potentiometer wiper → ADS1115 Ax, one end → 3.3V, other end → GND.

# Dead zone: normalized deflection [-1, +1] below which movement is suppressed.
# Prevents instrument creep from joystick mechanical center offset.
JOY_DEADBAND = 0.05

# Maximum instrument speed at full joystick deflection, expressed in
# "degrees per second". Multiplied by LOOP_PERIOD each tick to get the delta.
# Tune this to set how fast the instrument moves at full stick — start low.
JOY_MAX_DEG_SEC = 90.0   # → at full deflection: 90°/s × (1/25 s) = 3.6°/tick

# ── BNO055 config (only used when INPUT_MODE = "bno055") ──────────────────────
BNO055_ADDRESS = 0x28   # 0x28 default, 0x29 if ADDR tied HIGH

# BNO055 euler output order: (heading, roll, pitch)
# Adjust indices if sensor is mounted in a non-standard orientation.
# To find the right mapping: move one wrist axis at a time, watch which euler
# index changes, assign it to the desired instrument DOF.
BNO_YAW_IDX   = 0   # euler[0] = heading (0–360°)     → instrument yaw
BNO_ROLL_IDX  = 1   # euler[1] = roll    (±90°)        → instrument roll
BNO_PITCH_IDX = 2   # euler[2] = pitch   (±180°)       → instrument pitch

# =============================================================================
# Utilities
# =============================================================================

def angle_diff(a, b):
    """
    Shortest signed difference from angle b to angle a, in degrees [-180, +180].
    Handles all wraparound cases (e.g. 359° → 1° = +2°, not -358°).
    """
    d = a - b
    while d >  180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d

def _apply_deadband(value, band):
    """Zero out values within ±band, then rescale the remainder to [-1, +1]."""
    if abs(value) < band:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - band) / (1.0 - band)


# =============================================================================
# Input mode: joystick via ADS1115
# =============================================================================

def init_joystick():
    """
    Initialise ADS1115 and auto-calibrate joystick center.

    The center voltage is sampled at startup (assumes joystick is at rest).
    This corrects for mechanical center offset without needing to hardcode a
    voltage. Hold the joystick still for the first second after starting.

    Returns a state dict consumed by read_joystick_deltas().
    """
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=ADS1115_ADDRESS)
    ads.gain = 1    # ±4.096V range — plenty of headroom for a 3.3V joystick signal

    chan_pitch = AnalogIn(ads, JOY_PITCH_CHANNEL)
    chan_roll  = AnalogIn(ads, JOY_ROLL_CHANNEL)

    # Auto-calibrate center: average 20 readings at rest
    print("Calibrating joystick center (hold still)...", end="", flush=True)
    samples = 20
    center_pitch = sum(chan_pitch.voltage for _ in range(samples)) / samples
    center_roll  = sum(chan_roll.voltage  for _ in range(samples)) / samples
    # Use the larger of the two half-ranges as the scale denominator so
    # deflection in either direction maps to ±1.0.
    range_pitch = max(center_pitch, ads.gain_volts - center_pitch)
    range_roll  = max(center_roll,  ads.gain_volts - center_roll)
    print(f" done.\n  Pitch center: {center_pitch:.3f}V  Roll center: {center_roll:.3f}V")

    return {
        "chan_pitch":    chan_pitch,
        "chan_roll":     chan_roll,
        "center_pitch":  center_pitch,
        "center_roll":   center_roll,
        "range_pitch":   range_pitch,
        "range_roll":    range_roll,
    }

def read_joystick_deltas(state):
    """
    Read the joystick and return (d_pitch, d_roll, d_yaw) in degree-equivalents.

    Joystick deflection is normalized to [-1, +1], deadbanded, then multiplied
    by JOY_MAX_DEG_SEC × LOOP_PERIOD to produce a per-tick delta in degrees.
    This keeps the STEPS_PER_DEG_* scale factors meaningful and consistent with
    BNO055 mode — only JOY_MAX_DEG_SEC needs tuning when switching.

    Yaw is always 0.0 (a 2-axis joystick has no third axis).
    """
    vp = state["chan_pitch"].voltage
    vr = state["chan_roll"].voltage

    norm_pitch = (vp - state["center_pitch"]) / state["range_pitch"]
    norm_roll  = (vr - state["center_roll"])  / state["range_roll"]

    norm_pitch = max(-1.0, min(1.0, norm_pitch))
    norm_roll  = max(-1.0, min(1.0, norm_roll))

    dp = _apply_deadband(norm_pitch, JOY_DEADBAND) * JOY_MAX_DEG_SEC * LOOP_PERIOD
    dr = _apply_deadband(norm_roll,  JOY_DEADBAND) * JOY_MAX_DEG_SEC * LOOP_PERIOD

    return dp, dr, 0.0, {"norm_pitch": norm_pitch, "norm_roll": norm_roll}


# =============================================================================
# Input mode: BNO055 IMU
# =============================================================================

def init_bno055():
    """
    Initialise the BNO055 on Pi I2C bus 1. Waits for the first valid reading.

    Calibration status: (sys, gyro, accel, mag) — each 0–3 (3 = fully cal).
    For delta mode, gyro calibration matters most (prevents heading drift).
    Hold sensor still for a few seconds after power-on to let gyro calibrate.
    """
    import adafruit_bno055

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=BNO055_ADDRESS)

    print("Waiting for BNO055 first valid reading...", end="", flush=True)
    while True:
        try:
            euler = sensor.euler
            if euler is not None and euler[0] is not None:
                break
        except Exception:
            pass
        time.sleep(0.1)
        print(".", end="", flush=True)

    cal = sensor.calibration_status
    print(f"\nBNO055 ready.  Euler: {euler}")
    print(f"  Calibration: sys={cal[0]} gyro={cal[1]} accel={cal[2]} mag={cal[3]}")
    if cal[1] < 2:
        print("  NOTE: Gyro cal is low — hold sensor still. Heading may drift until cal ≥ 2.")

    return {"sensor": sensor, "prev_euler": euler}

def read_bno055_deltas(state):
    """
    Read BNO055 and return (d_pitch, d_roll, d_yaw, debug_info).
    Returns None for the delta tuple if the sensor is not ready this tick.
    """
    euler = state["sensor"].euler
    if euler is None or euler[0] is None:
        return None, None, None, {}

    prev = state["prev_euler"]
    d_yaw   = angle_diff(euler[BNO_YAW_IDX],   prev[BNO_YAW_IDX])
    d_roll  = angle_diff(euler[BNO_ROLL_IDX],  prev[BNO_ROLL_IDX])
    d_pitch = angle_diff(euler[BNO_PITCH_IDX], prev[BNO_PITCH_IDX])
    state["prev_euler"] = euler

    cal = state["sensor"].calibration_status
    return d_pitch, d_roll, d_yaw, {"euler": euler, "cal": cal}


# =============================================================================
# Input dispatcher — called by main(), mode-agnostic
# =============================================================================

def init_input():
    if INPUT_MODE == "joystick":
        print("Input mode: joystick via ADS1115")
        return ("joystick", init_joystick())
    elif INPUT_MODE == "bno055":
        print("Input mode: BNO055 IMU")
        return ("bno055", init_bno055())
    else:
        raise ValueError(f"Unknown INPUT_MODE: {INPUT_MODE!r}. Use 'joystick' or 'bno055'.")

def read_deltas(mode_state):
    """
    Returns (d_pitch, d_roll, d_yaw, debug_info) or (None, None, None, {}) if
    the sensor is not ready this tick (BNO055 only; joystick always returns data).
    """
    mode, state = mode_state
    if mode == "joystick":
        return read_joystick_deltas(state)
    else:
        return read_bno055_deltas(state)


# =============================================================================
# Serial connection to instrument ESP32
# =============================================================================

def init_serial():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
    time.sleep(0.5)
    ser.reset_input_buffer()
    print(f"Serial open: {SERIAL_PORT} @ {SERIAL_BAUD} baud")
    return ser

def send_move(ser, p, r, y):
    ser.write((json.dumps({"p": int(p), "r": int(r), "y": int(y)}) + "\n").encode())

def send_command(ser, cmd_str):
    ser.write((json.dumps({"cmd": cmd_str}) + "\n").encode())

def drain_serial(ser):
    while ser.in_waiting:
        ser.readline()   # discard; uncomment below to log:
        # print(f"  ESP32: {ser.readline().decode(errors='replace').strip()}")


# =============================================================================
# Clutch GPIO (optional)
# =============================================================================

_clutch_pin = None

def init_clutch():
    global _clutch_pin
    if CLUTCH_GPIO_PIN is None:
        print("Clutch: disabled (instrument always moves).")
        return
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CLUTCH_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        _clutch_pin = CLUTCH_GPIO_PIN
        print(f"Clutch: GPIO {CLUTCH_GPIO_PIN} (active LOW, normally open switch).")
    except ImportError:
        print("WARNING: RPi.GPIO not installed — clutch disabled.")

def clutch_pressed():
    if _clutch_pin is None:
        return False
    import RPi.GPIO as GPIO
    return GPIO.input(_clutch_pin) == GPIO.LOW


# =============================================================================
# Main control loop
# =============================================================================

def main():
    mode_state = init_input()
    ser        = init_serial()
    init_clutch()

    instr_pitch = 0.0
    instr_roll  = 0.0
    instr_yaw   = 0.0

    debug_period = (1.0 / DEBUG_HZ) if DEBUG_HZ > 0 else float("inf")
    last_debug   = time.monotonic()

    print(f"\nRunning at {LOOP_HZ} Hz. Press Ctrl+C to stop.\n")

    try:
        while True:
            loop_start = time.monotonic()

            # ── Read input ───────────────────────────────────────────────────
            d_pitch, d_roll, d_yaw, dbg = read_deltas(mode_state)
            if d_pitch is None:
                # BNO055 not ready this tick — skip without advancing position
                time.sleep(LOOP_PERIOD)
                continue

            # ── Apply deltas (unless clutched out) ───────────────────────────
            clutched = clutch_pressed()
            if not clutched:
                instr_pitch += d_pitch * STEPS_PER_DEG_PITCH
                instr_roll  += d_roll  * STEPS_PER_DEG_ROLL
                instr_yaw   += d_yaw   * STEPS_PER_DEG_YAW

                instr_pitch = max(-PITCH_MAX_STEPS, min(PITCH_MAX_STEPS, instr_pitch))
                instr_roll  = max(-ROLL_MAX_STEPS,  min(ROLL_MAX_STEPS,  instr_roll))
                instr_yaw   = max(-YAW_MAX_STEPS,   min(YAW_MAX_STEPS,   instr_yaw))

                send_move(ser, instr_pitch, instr_roll, instr_yaw)

            # ── Drain ESP32 status ───────────────────────────────────────────
            drain_serial(ser)

            # ── Debug print ──────────────────────────────────────────────────
            now = time.monotonic()
            if now - last_debug >= debug_period:
                mode, _ = mode_state
                if mode == "joystick":
                    src = f"joy: p={dbg['norm_pitch']:+.2f} r={dbg['norm_roll']:+.2f}"
                else:
                    e = dbg["euler"]
                    src = f"euler: h={e[0]:6.1f} r={e[1]:6.1f} p={e[2]:6.1f}  cal={dbg['cal']}"
                print(
                    f"{src}"
                    f"  |  instr: p={instr_pitch:+7.0f} r={instr_roll:+7.0f} y={instr_yaw:+7.0f}"
                    f"  clutch={clutched}"
                )
                last_debug = now

            # ── Loop timing ──────────────────────────────────────────────────
            elapsed = time.monotonic() - loop_start
            remainder = LOOP_PERIOD - elapsed
            if remainder > 0:
                time.sleep(remainder)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        send_command(ser, "stop")
        ser.flush()
        ser.close()
        if _clutch_pin is not None:
            import RPi.GPIO as GPIO
            GPIO.cleanup()


if __name__ == "__main__":
    main()

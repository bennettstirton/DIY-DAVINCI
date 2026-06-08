# =============================================================================
# Arm Controller Setup / Hardware Verification
# Target: ESP32 running MicroPython
#
# Run this after making hardware changes to the leader/input assembly.
# Tests run one at a time; each asks Y/N before starting.
#
# Tests:
#   A) Joystick     — verify pitch/roll ADC axes map to correct angular range
#   B) Encoder      — verify linear axis encoder maps correctly to insertion mm
#   C) Trim switches — jog the virtual home position via AS5600 readings
#   D) MPU6050 IMU  — identify pitch/roll axes and confirm invert flags
#
# Usage: upload and run in Thonny (or ampy). Ctrl+C ends each running test.
# =============================================================================

from machine import Pin, ADC, SoftI2C
import math
import time

# =============================================================================
# CONFIG — must match armcontrol.py
# =============================================================================

# Joystick
JOY_X_PIN          = 35   # physically the joystick Y wire
JOY_Y_PIN          = 34   # physically the joystick X wire
JOY_MIN            = 200
JOY_MAX            = 3895
JOY_CENTRE         = 2048
JOY_DEADBAND_PITCH = 250
JOY_DEADBAND_ROLL  = 120
PITCH_MAX_DEGREES  = 45.0
ROLL_MAX_DEGREES   = 45.0

# Optical encoder
ENCODER_A_PIN         = 16
ENCODER_B_PIN         = 17
ENCODER_PPR           = 600
ENCODER_SCALE         = 0.75
LINEAR_ENCODER_INVERT = False

# Linear axis geometry
LINEAR_MICROSTEPS        = 4
LINEAR_STEPS_PER_REV     = 200 * LINEAR_MICROSTEPS
LINEAR_SPOOL_DIAMETER_MM = 20.0
LINEAR_SPOOL_CIRC_MM     = math.pi * LINEAR_SPOOL_DIAMETER_MM
LINEAR_STEPS_PER_MM      = LINEAR_STEPS_PER_REV / LINEAR_SPOOL_CIRC_MM
LINEAR_MAX_MM            = 175.0
LINEAR_MAX_STEPS         = int(LINEAR_MAX_MM * LINEAR_STEPS_PER_MM)

_ENCODER_COUNTS_PER_REV  = ENCODER_PPR * 4
_STEPS_PER_ENCODER_COUNT = (LINEAR_STEPS_PER_REV * ENCODER_SCALE) / _ENCODER_COUNTS_PER_REV

# Trim joystick (replaces 4 rocker switches)
TRIM_JOY_X_PIN    = 32   # controls Pitch trim
TRIM_JOY_Y_PIN    = 33   # controls Roll trim
TRIM_JOY_MIN      = 200
TRIM_JOY_MAX      = 3895
TRIM_JOY_CENTRE   = 2048
TRIM_JOY_DEADBAND = 300
TRIM_JOY_INVERT_X = False
TRIM_JOY_INVERT_Y = False

# MPU6050 IMU
MPU6050_ADDR    = 0x68   # AD0 low; change to 0x69 if AD0 high
MPU6050_SDA     = 21     # shared bus with TCA9548A
MPU6050_SCL     = 22
MPU6050_I2C_FREQ = 100000
IMU_MAX_DEGREES = 45.0   # display range for the live bars

# One-time calibration offsets — measured by running Test D with zeroing
# skipped, noting the raw reading at the IMU's working-neutral mount position,
# and entering the correction needed to bring that reading to 0.
# calibrated_angle = raw_angle + offset
IMU_PITCH_OFFSET_DEG = 0.0
IMU_ROLL_OFFSET_DEG  = -90.0

# AS5600 encoders via TCA9548A multiplexer
AS5600_ADDR          = 0x36
AS5600_RAW_ANGLE_REG = 0x0C
AS5600_STATUS_REG    = 0x0B
AS5600_SDA           = 21
AS5600_SCL           = 22
AS5600_I2C_FREQ      = 100000
TCA9548A_ADDR        = 0x70
ROLL_TCA_CHANNEL     = 0
PITCH_TCA_CHANNEL    = 1
ROLL_ENCODER_INVERT  = False
PITCH_ENCODER_INVERT = False

TRIM_JOG_DEG_PER_SEC = 8.0

# =============================================================================
# ANSI
# =============================================================================

BOLD  = "\033[1m"
GREEN = "\033[32m"
RED   = "\033[31m"
DIM   = "\033[2m"
RESET = "\033[0m"

# =============================================================================
# HELPERS
# =============================================================================

def ask(prompt):
    while True:
        resp = input(prompt + " [y/n]: ").strip().lower()
        if resp in ("y", "yes"):
            return True
        if resp in ("n", "no"):
            return False
        print("  Enter y or n.")


def normalise(raw, centre, lo, hi, deadband):
    if abs(raw - centre) < deadband:
        return 0.0
    if raw > centre:
        return min(1.0,  (raw - centre) / (hi - centre))
    else:
        return max(-1.0, (raw - centre) / (centre - lo))


def deg_bar(value_deg, max_deg, width=22):
    """
    Centre-origin bar. | = home, > or < = cursor, # = fill.
    width is the number of cells inside the brackets.
    """
    half  = width // 2
    norm  = max(-1.0, min(1.0, value_deg / max_deg))
    pos   = int(norm * half)

    cells = [" "] * width
    cells[half] = "|"

    if pos > 0:
        for i in range(1, pos + 1):
            if half + i < width:
                cells[half + i] = "#"
        if half + pos < width:
            cells[half + pos] = ">"
    elif pos < 0:
        for i in range(pos, 0):
            if half + i >= 0:
                cells[half + i] = "#"
        if half + pos >= 0:
            cells[half + pos] = "<"

    return "[" + "".join(cells) + "]"


def mm_bar(value_mm, max_mm, width=30):
    """Left-origin bar, 0 → max_mm."""
    norm   = max(0.0, min(1.0, value_mm / max_mm))
    filled = int(norm * (width - 1))
    cells  = ["="] * filled + [">"] + [" "] * (width - filled - 1)
    return "[" + "".join(cells[:width]) + "]"


# =============================================================================
# AS5600 HELPER
# =============================================================================

def tca_select(i2c, channel):
    """Activate one TCA9548A channel before reading an AS5600."""
    i2c.writeto(TCA9548A_ADDR, bytes([1 << channel]))


def read_as5600_status(i2c, channel):
    """
    Returns (angle_deg, status_str) or (None, "NOT FOUND").
    status_str is one of: "OK", "NO MAGNET", "MAGNET TOO WEAK", "MAGNET TOO STRONG".
    """
    tca_select(i2c, channel)
    for _ in range(3):
        try:
            status = i2c.readfrom_mem(AS5600_ADDR, AS5600_STATUS_REG, 1)[0]
            md = (status >> 3) & 1  # magnet detected
            ml = (status >> 4) & 1  # magnet too weak
            mh = (status >> 5) & 1  # magnet too strong
            if mh:
                mag_str = "MAGNET TOO STRONG"
            elif ml:
                mag_str = "MAGNET TOO WEAK"
            elif md:
                mag_str = "OK"
            else:
                mag_str = "NO MAGNET"
            data  = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            raw   = ((data[0] & 0x0F) << 8) | data[1]
            angle = (raw / 4096.0) * 360.0
            if angle >  180.0: angle -= 360.0
            if angle < -180.0: angle += 360.0
            return angle, mag_str
        except OSError:
            time.sleep_us(500)
    return None, "NOT FOUND"


def read_as5600(i2c, channel, invert=False):
    tca_select(i2c, channel)
    for _ in range(3):
        try:
            data  = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            raw   = ((data[0] & 0x0F) << 8) | data[1]
            angle = (raw / 4096.0) * 360.0
            if angle >  180.0: angle -= 360.0
            if angle < -180.0: angle += 360.0
            return -angle if invert else angle
        except OSError:
            time.sleep_us(500)
    return None


# =============================================================================
# TEST A: JOYSTICK
# =============================================================================

def test_joystick():
    print()
    print(BOLD + "=== TEST A: Joystick ===" + RESET)
    print("Bar shows expected arm displacement from home.")
    print("Full deflection = +/-{:.0f}deg pitch, +/-{:.0f}deg roll.".format(
        PITCH_MAX_DEGREES, ROLL_MAX_DEGREES))
    print("Ctrl+C to end.")
    print()

    joy_x = ADC(Pin(JOY_X_PIN))
    joy_y = ADC(Pin(JOY_Y_PIN))
    joy_x.atten(ADC.ATTN_11DB)
    joy_y.atten(ADC.ATTN_11DB)

    print("Leave joystick centred — measuring resting centre...")
    time.sleep_ms(600)
    cx = sum(joy_x.read() for _ in range(50)) // 50
    cy = sum(joy_y.read() for _ in range(50)) // 50
    print("  Pitch centre: {}  Roll centre: {}".format(cx, cy))
    if abs(cx - JOY_CENTRE) < 400 and abs(cy - JOY_CENTRE) < 400:
        print(GREEN + "  Centres look good." + RESET)
    else:
        print(RED + "  WARNING: centre far from expected {}. Check wiring.".format(JOY_CENTRE) + RESET)
    print()

    try:
        while True:
            raw_x = sum(joy_x.read() for _ in range(4)) // 4
            raw_y = sum(joy_y.read() for _ in range(4)) // 4

            dp = normalise(raw_x, cx, JOY_MIN, JOY_MAX, JOY_DEADBAND_PITCH) * PITCH_MAX_DEGREES
            dr = normalise(raw_y, cy, JOY_MIN, JOY_MAX, JOY_DEADBAND_ROLL)  * ROLL_MAX_DEGREES

            line = "Pitch -{m:.0f}deg{bar}{m:.0f}deg {dp:+6.1f}deg   Roll -{m:.0f}deg{bar2}{m:.0f}deg {dr:+6.1f}deg".format(
                m=PITCH_MAX_DEGREES,
                bar=deg_bar(dp, PITCH_MAX_DEGREES),
                bar2=deg_bar(dr, ROLL_MAX_DEGREES),
                dp=dp, dr=dr)

            print(line + "   ", end="\r")
            time.sleep_ms(80)

    except KeyboardInterrupt:
        print()
        print("Joystick test ended.")


# =============================================================================
# TEST B: OPTICAL ENCODER → INSERTION AXIS (mm)
# =============================================================================

_ENC_TABLE = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
_enc_state = 0
_enc_count = 0


def _make_encoder_irq(enc_a_pin, enc_b_pin):
    def _irq(pin):
        global _enc_state, _enc_count
        a = enc_a_pin.value()
        b = enc_b_pin.value()
        _enc_state  = ((_enc_state << 2) | (a << 1) | b) & 0x0F
        _enc_count += _ENC_TABLE[_enc_state]
    return _irq


def test_encoder():
    global _enc_state, _enc_count

    print()
    print(BOLD + "=== TEST B: Encoder -> Insertion Axis ===" + RESET)
    print("Bar shows virtual insertion depth: 0mm (retracted) to {:.0f}mm (extended).".format(LINEAR_MAX_MM))
    print()
    print("IMPORTANT: move the insertion axis to the FULLY RETRACTED position.")
    print("You have 5 seconds before the encoder zeroes and tracking begins.")
    print()
    for i in range(5, 0, -1):
        print("  Starting in {}...   ".format(i), end="\r")
        time.sleep(1)
    print("  Go! Encoder zeroed at retracted position.   ")
    print()
    print("Extending should move the bar RIGHT.")
    print("If it moves left, flip LINEAR_ENCODER_INVERT in armcontrol.py.")
    print("Ctrl+C to end.")
    print()

    enc_a = Pin(ENCODER_A_PIN, Pin.IN, Pin.PULL_UP)
    enc_b = Pin(ENCODER_B_PIN, Pin.IN, Pin.PULL_UP)
    _enc_state = 0
    _enc_count = 0

    handler = _make_encoder_irq(enc_a, enc_b)
    enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handler)
    enc_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handler)

    position_steps = 0
    last_raw_count = 0

    try:
        while True:
            raw = _enc_count
            delta = raw - last_raw_count
            last_raw_count = raw

            if LINEAR_ENCODER_INVERT:
                delta = -delta
            position_steps += int(delta * _STEPS_PER_ENCODER_COUNT)
            pos_mm = position_steps / LINEAR_STEPS_PER_MM

            display_mm = max(0.0, min(LINEAR_MAX_MM, pos_mm))

            if pos_mm < -2.0:
                flag = " " + RED + "PAST HOME" + RESET
            elif pos_mm > LINEAR_MAX_MM + 2.0:
                flag = " " + RED + "PAST LIMIT" + RESET
            else:
                flag = ""

            line = "0mm {bar} {max:.0f}mm  {pos:6.1f}mm{flag}   ".format(
                bar=mm_bar(display_mm, LINEAR_MAX_MM),
                max=LINEAR_MAX_MM,
                pos=pos_mm,
                flag=flag)

            print(line, end="\r")
            time.sleep_ms(50)

    except KeyboardInterrupt:
        enc_a.irq(handler=None)
        enc_b.irq(handler=None)
        pos_mm = position_steps / LINEAR_STEPS_PER_MM
        print()
        print("Encoder test ended. Final position: {:.2f}mm".format(pos_mm))


# =============================================================================
# TEST C: TRIM SWITCHES → VIRTUAL HOME POSITION
# =============================================================================

def test_trim_switches():
    print()
    print(BOLD + "=== TEST C: Trim Switches ===" + RESET)
    print("Reads AS5600 encoders for actual arm angles.")
    print("Hold a switch to jog the virtual home at {:.0f}deg/s.".format(TRIM_JOG_DEG_PER_SEC))
    print("Expected: trim joystick X+ increases pitch home, Y+ increases roll home.")
    print("Ctrl+C to end — final home values printed as ready-to-paste config.")
    print()

    i2c = SoftI2C(sda=Pin(AS5600_SDA), scl=Pin(AS5600_SCL), freq=AS5600_I2C_FREQ)

    print("Checking TCA9548A + AS5600 encoders...")
    bus_devices = i2c.scan()
    if TCA9548A_ADDR not in bus_devices:
        print(RED + "  TCA9548A NOT FOUND at 0x70 — check SDA=GPIO{} SCL=GPIO{}".format(
            AS5600_SDA, AS5600_SCL) + RESET)
        return
    print(GREEN + "  TCA9548A found." + RESET)

    t_roll,  roll_status  = read_as5600_status(i2c, ROLL_TCA_CHANNEL)
    t_pitch, pitch_status = read_as5600_status(i2c, PITCH_TCA_CHANNEL)

    roll_ok  = t_roll  is not None
    pitch_ok = t_pitch is not None

    if roll_ok:
        color = GREEN if roll_status == "OK" else RED
        print(color + "  Roll  AS5600 {} ({:.1f}deg)".format(roll_status, t_roll) + RESET)
    else:
        print(RED + "  Roll  AS5600 NOT FOUND — chip dead or no power (TCA ch {})".format(ROLL_TCA_CHANNEL) + RESET)

    if pitch_ok:
        color = GREEN if pitch_status == "OK" else RED
        print(color + "  Pitch AS5600 {} ({:.1f}deg)".format(pitch_status, t_pitch) + RESET)
    else:
        print(RED + "  Pitch AS5600 NOT FOUND — chip dead or no power (TCA ch {})".format(PITCH_TCA_CHANNEL) + RESET)

    if not (roll_ok or pitch_ok):
        print(RED + "No encoders found. Cannot run test." + RESET)
        return

    print()

    trim_joy_x = ADC(Pin(TRIM_JOY_X_PIN))
    trim_joy_y = ADC(Pin(TRIM_JOY_Y_PIN))
    trim_joy_x.atten(ADC.ATTN_11DB)
    trim_joy_y.atten(ADC.ATTN_11DB)

    pitch_home = t_pitch if pitch_ok else 0.0
    roll_home  = t_roll  if roll_ok  else 0.0
    last_ms    = time.ticks_ms()

    try:
        while True:
            now    = time.ticks_ms()
            dt     = time.ticks_diff(now, last_ms) / 1000.0
            last_ms = now

            p_act = read_as5600(i2c, PITCH_TCA_CHANNEL, invert=PITCH_ENCODER_INVERT) if pitch_ok else None
            r_act = read_as5600(i2c, ROLL_TCA_CHANNEL,  invert=ROLL_ENCODER_INVERT)  if roll_ok  else None

            raw_p = trim_joy_x.read()
            raw_r = trim_joy_y.read()

            if abs(raw_p - TRIM_JOY_CENTRE) < TRIM_JOY_DEADBAND:
                trim_p = 0.0
            elif raw_p > TRIM_JOY_CENTRE:
                trim_p = min(1.0, (raw_p - TRIM_JOY_CENTRE) / (TRIM_JOY_MAX - TRIM_JOY_CENTRE))
            else:
                trim_p = max(-1.0, (raw_p - TRIM_JOY_CENTRE) / (TRIM_JOY_CENTRE - TRIM_JOY_MIN))
            if TRIM_JOY_INVERT_X: trim_p = -trim_p

            if abs(raw_r - TRIM_JOY_CENTRE) < TRIM_JOY_DEADBAND:
                trim_r = 0.0
            elif raw_r > TRIM_JOY_CENTRE:
                trim_r = min(1.0, (raw_r - TRIM_JOY_CENTRE) / (TRIM_JOY_MAX - TRIM_JOY_CENTRE))
            else:
                trim_r = max(-1.0, (raw_r - TRIM_JOY_CENTRE) / (TRIM_JOY_CENTRE - TRIM_JOY_MIN))
            if TRIM_JOY_INVERT_Y: trim_r = -trim_r

            pitch_home += trim_p * TRIM_JOG_DEG_PER_SEC * dt
            roll_home  += trim_r * TRIM_JOG_DEG_PER_SEC * dt

            p_act_str = "{:+7.2f}deg".format(p_act) if p_act is not None else "   N/A   "
            r_act_str = "{:+7.2f}deg".format(r_act) if r_act is not None else "   N/A   "

            line = ("P: act={pa}  home={ph:+7.2f}deg  trim={pt:+4.0f}%    "
                    "R: act={ra}  home={rh:+7.2f}deg  trim={rt:+4.0f}%   ").format(
                pa=p_act_str, ph=pitch_home, pt=trim_p * 100,
                ra=r_act_str, rh=roll_home,  rt=trim_r * 100)

            print(line, end="\r")
            time.sleep_ms(50)

    except KeyboardInterrupt:
        print()
        print()
        print("Trim switch test ended.")
        print()
        print("Final home values — paste into armcontrol.py CONFIG if desired:")
        print("  PITCH_ENCODER_OFFSET_DEG = {:.2f}".format(pitch_home))
        print("  ROLL_ENCODER_OFFSET_DEG  = {:.2f}".format(roll_home))


# =============================================================================
# MPU6050 HELPERS
# =============================================================================

_MPU_PWR_MGMT_1  = 0x6B
_MPU_WHO_AM_I    = 0x75
_MPU_ACCEL_OUT   = 0x3B   # 6 bytes: XH XL YH YL ZH ZL
_MPU_ACCEL_SCALE = 16384.0  # counts per g at ±2g default


def _mpu_init(i2c):
    """Wake MPU6050 from sleep. Returns True if found."""
    try:
        who = i2c.readfrom_mem(MPU6050_ADDR, _MPU_WHO_AM_I, 1)[0]
        if who not in (0x68, 0x72):
            return False
        i2c.writeto_mem(MPU6050_ADDR, _MPU_PWR_MGMT_1, bytes([0x00]))
        time.sleep_ms(100)
        return True
    except OSError:
        return False


def _mpu_read_accel(i2c):
    """Returns (ax, ay, az) in g, or None on error."""
    try:
        data = i2c.readfrom_mem(MPU6050_ADDR, _MPU_ACCEL_OUT, 6)
        def s16(h, l):
            v = (h << 8) | l
            return v - 65536 if v >= 32768 else v
        ax = s16(data[0], data[1]) / _MPU_ACCEL_SCALE
        ay = s16(data[2], data[3]) / _MPU_ACCEL_SCALE
        az = s16(data[4], data[5]) / _MPU_ACCEL_SCALE
        return ax, ay, az
    except OSError:
        return None


def _accel_to_angles(ax, ay, az):
    """
    Returns (pitch_deg, roll_deg) from gravity vector, with the one-time
    calibration offsets applied so the working-neutral mount position reads ~0.
    Pitch: rotation around Y axis (nose up/down).
    Roll:  rotation around X axis (side tilt).
    """
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 57.2958 + IMU_PITCH_OFFSET_DEG
    roll  = math.atan2( ay, az) * 57.2958 + IMU_ROLL_OFFSET_DEG
    return pitch, roll


# =============================================================================
# TEST D: MPU6050 AXIS IDENTIFICATION
# =============================================================================

IMU_TEST_DURATION_MS = 10000

def test_mpu6050():
    print()
    print(BOLD + "=== TEST D: MPU6050 IMU — Axis Identification ===" + RESET)
    print("Tilt the IMU through its full range of motion in each direction.")
    print("Runs for {:.0f} seconds, then reports the measured range of motion (ROM).".format(
        IMU_TEST_DURATION_MS / 1000.0))
    print("Use the live bars to confirm which computed angle responds to which tilt.")
    print("Ctrl+C ends early.")
    print()

    i2c = SoftI2C(sda=Pin(MPU6050_SDA), scl=Pin(MPU6050_SCL), freq=MPU6050_I2C_FREQ)

    print("Scanning I2C bus for MPU6050...")
    devices = i2c.scan()
    addrs   = [hex(d) for d in devices]
    print("  Found: {}".format(addrs if addrs else "nothing"))

    if MPU6050_ADDR not in devices:
        alt = 0x69
        if alt in devices:
            print(RED + "  MPU6050 not at 0x68 — found 0x69. Change MPU6050_ADDR to 0x69 at top of this file." + RESET)
        else:
            print(RED + "  MPU6050 not found. Check VCC/GND/SDA/SCL wiring." + RESET)
        return

    if not _mpu_init(i2c):
        print(RED + "  MPU6050 found but failed to initialise." + RESET)
        return

    print(GREEN + "  MPU6050 ready at 0x{:02X}.".format(MPU6050_ADDR) + RESET)
    print()

    if ask("Capture a home/zero orientation before starting? (n = use raw absolute angles, zero = chip's natural flat reading)"):
        print("Hold the IMU flat (as it will be mounted). Waiting 2s for it to settle...")
        time.sleep_ms(2000)

        samples = [_mpu_read_accel(i2c) for _ in range(20)]
        samples = [s for s in samples if s is not None]
        if not samples:
            print(RED + "Failed to read accel data." + RESET)
            return
        ax0 = sum(s[0] for s in samples) / len(samples)
        ay0 = sum(s[1] for s in samples) / len(samples)
        az0 = sum(s[2] for s in samples) / len(samples)
        p0, r0 = _accel_to_angles(ax0, ay0, az0)
        print("  Home orientation captured: Pitch={:.1f}deg  Roll={:.1f}deg".format(p0, r0))
    else:
        p0, r0 = 0.0, 0.0
        print("  Skipping zeroing — angles shown are raw absolute values from the accelerometer.")
    print()
    print("GO — tilt the IMU through its full range of motion now!")
    print()
    print(DIM + "  ax=accel X  ay=accel Y  az=accel Z  (g = ~1.0 when that axis points down)" + RESET)
    print()

    dp_min = dp_max = 0.0
    dr_min = dr_max = 0.0

    deadline = time.ticks_add(time.ticks_ms(), IMU_TEST_DURATION_MS)

    try:
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            result = _mpu_read_accel(i2c)
            if result is None:
                print("  READ ERROR   ", end="\r")
                time.sleep_ms(100)
                continue

            ax, ay, az = result
            pitch, roll = _accel_to_angles(ax, ay, az)
            dp = pitch - p0
            dr = roll  - r0

            dp_min = min(dp_min, dp)
            dp_max = max(dp_max, dp)
            dr_min = min(dr_min, dr)
            dr_max = max(dr_max, dr)

            secs_left = time.ticks_diff(deadline, time.ticks_ms()) / 1000.0
            line = ("[{t:4.1f}s] ax={ax:+5.2f}g  ay={ay:+5.2f}g  az={az:+5.2f}g    "
                    "Pitch{pb}{dp:+6.1f}deg   Roll{rb}{dr:+6.1f}deg   ").format(
                t=secs_left,
                ax=ax, ay=ay, az=az,
                pb=deg_bar(dp, IMU_MAX_DEGREES),
                rb=deg_bar(dr, IMU_MAX_DEGREES),
                dp=dp, dr=dr)

            print(line, end="\r")
            time.sleep_ms(80)

    except KeyboardInterrupt:
        pass

    print()
    angle_basis = "delta from home" if (p0 != 0.0 or r0 != 0.0) else "raw absolute angle"
    print()
    print(BOLD + "IMU axis test ended. Measured range of motion ({}):".format(angle_basis) + RESET)
    print("  Pitch: {:+6.1f}deg  to  {:+6.1f}deg   (span: {:.1f}deg)".format(
        dp_min, dp_max, dp_max - dp_min))
    print("  Roll:  {:+6.1f}deg  to  {:+6.1f}deg   (span: {:.1f}deg)".format(
        dr_min, dr_max, dr_max - dr_min))
    print()
    print("Reminder — set these in armcontrol.py once axes are confirmed:")
    print("  IMU_PITCH_INVERT = False   # set True if pitch direction is backwards")
    print("  IMU_ROLL_INVERT  = False   # set True if roll  direction is backwards")


# =============================================================================
# MAIN
# =============================================================================

def main():
    print()
    print(BOLD + "Arm Controller Hardware Setup" + RESET)
    print("Tests run one at a time. Ctrl+C ends a running test and moves to the next.")
    print()

    if ask("Run Test A — Joystick (pitch/roll angular displacement)?"):
        test_joystick()
        print()

    if ask("Run Test B — Optical encoder (insertion axis mm)?"):
        test_encoder()
        print()

    if ask("Run Test C — Trim switches (virtual home position)?"):
        test_trim_switches()
        print()

    if ask("Run Test D — MPU6050 IMU axis identification?"):
        test_mpu6050()
        print()

    print(BOLD + "Done." + RESET)


main()

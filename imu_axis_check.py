# =============================================================================
# IMU Axis Check — BNO055 + AS5600 guided diagnostic
# Target: ESP32 running MicroPython
#
# Runs a guided axis-by-axis validation:
#   1. Calibration check + home capture + jaw calibration
#   2. Isolation tests — move one axis at a time, measure bleed into others
#   3. Direction checks — confirm positive direction for each axis
#
# Run this standalone before using imu_instrumentcontrol.py.
# No motors need to be connected.
# =============================================================================

import sys
import math
import time
from machine import I2C, Pin

# =============================================================================
# CONFIGURATION — mirror values from imu_instrumentcontrol.py
# =============================================================================

I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
I2C_FREQ    = 400_000
BNO055_ADDR = 0x28
AS5600_ADDR = 0x36

SCALE_PITCH    = 0.8
SCALE_ROLL     = 0.8
SCALE_YAW      = 0.8
GRIP_INVERT    = False

LIMIT_PITCH    = 90.0
LIMIT_YAW      = 80.0
LIMIT_ROLL     = 175.0
LIMIT_GRIP_MAX = 25.0
DEADBAND_DEG   = 1.5
EMA_ALPHA      = 0.2

JAW_MIN_RANGE_COUNTS = 50

# Set False if ANSI escape codes show as garbage in your terminal
DISPLAY_ANSI = True

TEST_DURATION_MS = 5000   # how long each isolation test records
DIR_DURATION_MS  = 3000   # how long each direction check records

# =============================================================================
# BNO055 DRIVER
# =============================================================================

_BNO_PAGE_ID     = 0x07
_BNO_CHIP_ID     = 0x00
_BNO_OPR_MODE    = 0x3D
_BNO_PWR_MODE    = 0x3E
_BNO_SYS_TRIGGER = 0x3F
_BNO_CALIB_STAT  = 0x35
_BNO_EULER_H_LSB = 0x1A
_BNO_QUAT_W_LSB  = 0x20
_NDOF_MODE       = 0x0C


def _s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


class BNO055:
    def __init__(self, i2c, addr=BNO055_ADDR):
        self.i2c  = i2c
        self.addr = addr
        self._write(_BNO_PAGE_ID, 0x00)
        time.sleep_ms(10)
        chip_id = self._read(_BNO_CHIP_ID, 1)[0]
        if chip_id != 0xA0:
            raise RuntimeError(f"BNO055 not found (chip_id=0x{chip_id:02X})")
        self._write(_BNO_SYS_TRIGGER, 0x20)
        time.sleep_ms(650)
        self._write(_BNO_PWR_MODE, 0x00)
        time.sleep_ms(10)
        self._write(_BNO_OPR_MODE, _NDOF_MODE)
        time.sleep_ms(20)
        print("[BNO055] OK")

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def calibration(self):
        s = self._read(_BNO_CALIB_STAT, 1)[0]
        return (s >> 6) & 3, (s >> 4) & 3, (s >> 2) & 3, s & 3

    def quaternion(self):
        d = self._read(_BNO_QUAT_W_LSB, 8)
        return (_s16(d[0],d[1])/16384.0, _s16(d[2],d[3])/16384.0,
                _s16(d[4],d[5])/16384.0, _s16(d[6],d[7])/16384.0)


# =============================================================================
# AS5600 DRIVER
# =============================================================================

class AS5600:
    _ANGLE_REG  = 0x0E
    _STATUS_REG = 0x1A

    def __init__(self, i2c, addr=AS5600_ADDR):
        self.i2c  = i2c
        self.addr = addr
        if not (i2c.readfrom_mem(addr, self._STATUS_REG, 1)[0] & 0x08):
            raise RuntimeError("AS5600: no magnet detected")
        print("[AS5600] OK")

    def angle(self):
        d = self.i2c.readfrom_mem(self.addr, self._ANGLE_REG, 2)
        return ((d[0] & 0x0F) << 8) | d[1]


# =============================================================================
# QUATERNION MATH
# =============================================================================

def _quat_delta(hw, hx, hy, hz, w, x, y, z):
    dw =  hw*w + hx*x + hy*y + hz*z
    dx =  hw*x - hx*w - hy*z + hz*y
    dy =  hw*y + hx*z - hy*w - hz*x
    dz =  hw*z - hx*y + hy*x - hz*w
    return dw, dx, dy, dz


def _quat_to_euler(dw, dx, dy, dz):
    roll  = math.degrees(math.atan2(2*(dw*dx + dy*dz), 1 - 2*(dx*dx + dy*dy)))
    sinp  = max(-1.0, min(1.0, 2*(dw*dy - dz*dx)))
    pitch = math.degrees(math.asin(sinp))
    yaw   = math.degrees(math.atan2(2*(dw*dz + dx*dy), 1 - 2*(dy*dy + dz*dz)))
    return roll, pitch, yaw


# =============================================================================
# DISPLAY HELPERS
# =============================================================================

_BAR_W  = 20
DIV     = '═' * 60


def _bar_bipolar(value, limit):
    clipped = abs(value) >= limit * 0.99
    pos = round((value + limit) / (2.0 * limit) * _BAR_W)
    pos = max(0, min(_BAR_W, pos))
    chars = ['-'] * (_BAR_W + 1)
    chars[pos] = '!' if clipped else 'X'
    return '[' + ''.join(chars) + ']', clipped


def _bar_grip(value, limit):
    fill = round(value / limit * _BAR_W)
    fill = max(0, min(_BAR_W, fill))
    return '[' + '█' * fill + '░' * (_BAR_W - fill) + ']'


def _bleed_label(pct):
    if pct < 5:  return 'OK'
    if pct < 15: return 'MODERATE'
    return 'HIGH !'


def _countdown(seconds, prompt='Starting in:'):
    print(prompt)
    for i in range(seconds, 0, -1):
        print(f'  {i}...')
        time.sleep_ms(1000)


# =============================================================================
# SETUP ROUTINES
# =============================================================================

def _wait_calibration(imu):
    print('Checking calibration...')
    sys_c = imu.calibration()[0]
    if sys_c >= 2:
        print(f'  Already calibrated (sys:{sys_c}) — continuing')
        return
    print('  Calibration needed — move the sensor naturally until sys >= 2')
    while True:
        sys_c, g, a, m = imu.calibration()
        print(f'  sys:{sys_c}  gyro:{g}  accel:{a}  mag:{m}')
        if sys_c >= 2:
            print(f'  Calibration OK (sys:{sys_c})')
            return
        time.sleep_ms(1000)


def _capture_home(imu):
    N = 16
    hw, hx, hy, hz = imu.quaternion()
    sw, sx, sy, sz  = hw, hx, hy, hz
    for _ in range(N - 1):
        w, x, y, z = imu.quaternion()
        if w*sw + x*sx + y*sy + z*sz < 0:
            w, x, y, z = -w, -x, -y, -z
        sw += w; sx += x; sy += y; sz += z
        time.sleep_ms(10)
    hw = sw/N; hx = sx/N; hy = sy/N; hz = sz/N
    mag = (hw**2 + hx**2 + hy**2 + hz**2) ** 0.5
    return hw/mag, hx/mag, hy/mag, hz/mag


def _calibrate_jaw(jaw):
    print('Jaw calibration — curl finger through FULL range for 5 seconds...')
    jaw_min, jaw_max = 4095, 0
    for remaining in range(5, 0, -1):
        print(f'  {remaining}s  current:{jaw.angle()}  range so far:{jaw_max - jaw_min} counts')
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 1000:
            c = jaw.angle()
            if c < jaw_min: jaw_min = c
            if c > jaw_max: jaw_max = c
            time.sleep_ms(20)
    span = jaw_max - jaw_min
    if span < JAW_MIN_RANGE_COUNTS:
        raise RuntimeError(f'Jaw cal failed — range {span} counts (need {JAW_MIN_RANGE_COUNTS})')
    print(f'  OK — min:{jaw_min}  max:{jaw_max}  range:{span} counts')
    return jaw_min, jaw_max


def _read_grip_deg(jaw, jaw_min, jaw_max):
    t = (jaw.angle() - jaw_min) / (jaw_max - jaw_min)
    t = max(0.0, min(1.0, t))
    if GRIP_INVERT: t = 1.0 - t
    return t * LIMIT_GRIP_MAX


def _read_all(imu, jaw, home, jaw_min, jaw_max, ema):
    hw, hx, hy, hz = home
    w, x, y, z = imu.quaternion()
    if w*hw + x*hx + y*hy + z*hz < 0:
        w, x, y, z = -w, -x, -y, -z
    dw, dx, dy, dz = _quat_delta(hw, hx, hy, hz, w, x, y, z)
    qroll, qpitch, qyaw = _quat_to_euler(dw, dx, dy, dz)

    ema[0] = EMA_ALPHA * qroll  + (1 - EMA_ALPHA) * ema[0]   # → pitch
    ema[1] = EMA_ALPHA * qpitch + (1 - EMA_ALPHA) * ema[1]   # → roll
    ema[2] = EMA_ALPHA * qyaw   + (1 - EMA_ALPHA) * ema[2]   # → yaw

    pitch = max(-LIMIT_PITCH, min(LIMIT_PITCH, ema[0] * SCALE_PITCH))
    roll  = max(-LIMIT_ROLL,  min(LIMIT_ROLL,  ema[1] * SCALE_ROLL))
    yaw   = max(-LIMIT_YAW,   min(LIMIT_YAW,   ema[2] * SCALE_YAW))
    grip  = _read_grip_deg(jaw, jaw_min, jaw_max)
    return pitch, roll, yaw, grip


# =============================================================================
# TEST PHASES
# =============================================================================

def _isolation_test(num, total, axis_key, label, instruction,
                    imu, jaw, home, jaw_min, jaw_max):
    print(DIV)
    print(f'ISOLATION TEST {num}/{total} — {label}')
    print(f'  {instruction}')
    _countdown(3)
    print(f'GO — moving for {TEST_DURATION_MS // 1000} seconds...')

    ema = [0.0, 0.0, 0.0]
    mins = dict(pitch=0.0, roll=0.0, yaw=0.0, grip=0.0)
    maxs = dict(pitch=0.0, roll=0.0, yaw=0.0, grip=0.0)
    first = True

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < TEST_DURATION_MS:
        pitch, roll, yaw, grip = _read_all(imu, jaw, home, jaw_min, jaw_max, ema)

        for k, v in (('pitch', pitch), ('roll', roll), ('yaw', yaw), ('grip', grip)):
            if v < mins[k]: mins[k] = v
            if v > maxs[k]: maxs[k] = v

        if DISPLAY_ANSI and not first:
            sys.stdout.write('\033[4A')
        first = False

        pb, _ = _bar_bipolar(pitch, LIMIT_PITCH)
        rb, _ = _bar_bipolar(roll,  LIMIT_ROLL)
        yb, _ = _bar_bipolar(yaw,   LIMIT_YAW)
        gb    = _bar_grip(grip,     LIMIT_GRIP_MAX)
        t     = lambda k: '  ← TARGET' if k == axis_key else ''
        print(f'  Pitch: {pb} {pitch:+7.1f}°{t("pitch")}')
        print(f'  Roll : {rb} {roll:+7.1f}°{t("roll")}')
        print(f'  Yaw  : {yb} {yaw:+7.1f}°{t("yaw")}')
        print(f'  Grip : {gb} {grip:6.1f}°{t("grip")}')

        time.sleep_ms(100)

    print()   # leave last display state visible
    ranges = {k: maxs[k] - mins[k] for k in ('pitch', 'roll', 'yaw', 'grip')}
    target_range = ranges[axis_key]

    print(f'\nRESULTS — {label} isolation:')
    for k in ('pitch', 'roll', 'yaw', 'grip'):
        r = ranges[k]
        if k == axis_key:
            print(f'  {k.upper():5}: {r:6.1f}°  ← target')
        else:
            pct = (r / target_range * 100) if target_range > 0.5 else 0.0
            print(f'  {k.upper():5}: {r:6.1f}°  ({pct:4.1f}% bleed — {_bleed_label(pct)})')
    print()


def _direction_test(num, total, axis_key, label, pos_description, config_hint,
                    imu, jaw, home, jaw_min, jaw_max):
    print(DIV)
    print(f'DIRECTION CHECK {num}/{total} — {label}')
    print(f'  Move in the POSITIVE direction: {pos_description}')
    _countdown(3)
    print(f'Hold it... ({DIR_DURATION_MS // 1000} seconds)')

    ema = [0.0, 0.0, 0.0]
    peak = 0.0

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < DIR_DURATION_MS:
        vals = _read_all(imu, jaw, home, jaw_min, jaw_max, ema)
        v = dict(pitch=vals[0], roll=vals[1], yaw=vals[2], grip=vals[3])[axis_key]
        if abs(v) > abs(peak): peak = v
        time.sleep_ms(100)

    print(f'\n  Peak {label}: {peak:+.1f}°')
    if abs(peak) < DEADBAND_DEG:
        print(f'  Result: too small to read — try moving further from home')
    elif peak > 0:
        print(f'  Result: POSITIVE ✓  no change needed')
    else:
        print(f'  Result: NEGATIVE  → flip sign: {config_hint}')
    print()


# =============================================================================
# MAIN
# =============================================================================

def main():
    print(DIV)
    print('IMU AXIS CHECK — BNO055 + AS5600 diagnostic')
    print(DIV)

    i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ)
    imu = BNO055(i2c)
    jaw = AS5600(i2c)

    _wait_calibration(imu)

    print('\nPlace controller at NEUTRAL position — home captures in:')
    _countdown(3)
    home = _capture_home(imu)
    print('Home captured.\n')

    jaw_min, jaw_max = _calibrate_jaw(jaw)
    print()

    # ---- isolation tests ----
    isolation_tests = [
        ('roll',  'ROLL',  'Tilt side-to-side. Keep pitch and yaw as still as possible.'),
        ('pitch', 'PITCH', 'Tip forward and back. Keep roll and yaw as still as possible.'),
        ('yaw',   'YAW',   'Rotate wrist (like turning a screwdriver). Keep roll and pitch still.'),
        ('grip',  'GRIP',  'Curl and uncurl your finger through full range. Hold IMU completely still.'),
    ]

    for i, (key, label, instruction) in enumerate(isolation_tests, 1):
        _isolation_test(i, len(isolation_tests), key, label, instruction,
                        imu, jaw, home, jaw_min, jaw_max)
        time.sleep_ms(1500)

    # ---- direction checks ----
    print(DIV)
    print('DIRECTION CHECKS')
    print('Move each axis in whichever direction intuitively feels positive.')
    print('If the result is NEGATIVE, the fix is shown — update imu_instrumentcontrol.py.')
    time.sleep_ms(2000)

    direction_tests = [
        ('roll',  'ROLL',  'tilt right (or whichever side feels positive)',
         'change SCALE_ROLL = -0.8 in imu_instrumentcontrol.py'),
        ('pitch', 'PITCH', 'tip forward (or whichever direction feels positive)',
         'change SCALE_PITCH = -0.8 in imu_instrumentcontrol.py'),
        ('yaw',   'YAW',   'rotate wrist clockwise (or whichever feels positive)',
         'change SCALE_YAW = -0.8 in imu_instrumentcontrol.py'),
        ('grip',  'GRIP',  'curl finger — grip is always 0 → 25°, should always be positive',
         'set GRIP_INVERT = True in imu_instrumentcontrol.py'),
    ]

    for i, (key, label, desc, hint) in enumerate(direction_tests, 1):
        _direction_test(i, len(direction_tests), key, label, desc, hint,
                        imu, jaw, home, jaw_min, jaw_max)
        time.sleep_ms(1000)

    print(DIV)
    print('AXIS CHECK COMPLETE')
    print('  Bleed < 5%:  OK — normal for a hand-mounted controller')
    print('  Bleed 5-15%: MODERATE — acceptable, may be physical coupling')
    print('  Bleed > 15%: HIGH — worth investigating (mounting, gimbal lock territory)')
    print()
    print('  If any DIRECTION was NEGATIVE, flip the sign of the relevant SCALE_*')
    print('  constant in imu_instrumentcontrol.py as shown above.')
    print(DIV)


main()

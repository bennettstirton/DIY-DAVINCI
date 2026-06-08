# =============================================================================
# Fine Instrument Controller — BNO055 IMU Edition
# Branched from: instrumentcontrol.py (joystick version, working as of 2026-05-07)
# Target: ESP32 running MicroPython
#
# Hardware
# --------
# Servos: STS3215-C047 (Feetech ST bus, 12V, 1:345 metal gearbox)
#   Encoder measures output shaft. Full range = 0–4095 counts = 360°.
#   Center/neutral = 2048 counts.
#
# Adapter: Waveshare Bus Servo Adapter (A) in UART mode
#   Jumper cap must be on the A position.
#   Wiring is TX-TX, RX-RX (not crossover) — adapter handles half-duplex
#   direction switching internally.
#
# IMU: Adafruit BNO055 breakout (I2C, address 0x28)
#   Provides fused Euler angles (heading/tilt, roll, pitch) in NDOF mode.
#   Mounted on operator's wrist/hand. Control is delta-from-home:
#   the orientation at boot is captured as the neutral position,
#   and deviations from that home drive the instrument axes.
#
# Jaw encoder: AS5600 magnetic rotary encoder on the pointer finger (I2C, address 0x36).
#   Shares the same I2C bus as the BNO055 — no address conflict.
#   The AS5600 reads absolute angle (0–4095 counts = 0–360°).
#   A jaw calibration sweep at boot captures the min/max counts for the finger's
#   physical ROM (~30–45°) and maps that range linearly to instrument grip (0–25°).
#
# Wiring — Servos
# ---------------
#   ESP32 GPIO 17 (UART2 TX) → Waveshare TX
#   ESP32 GPIO 16 (UART2 RX) → Waveshare RX
#   ESP32 GND                → Waveshare GND
#   Waveshare USB or 5V pin  → 5V adapter logic power
#   Waveshare servo terminals → 12V servo bus power
#
# Wiring — BNO055
# ---------------
#   ESP32 GPIO 21 (I2C SDA)  → BNO055 SDA  (shared bus with AS5600)
#   ESP32 GPIO 22 (I2C SCL)  → BNO055 SCL  (shared bus with AS5600)
#   ESP32 3.3V               → BNO055 VIN
#   ESP32 GND                → BNO055 GND
#   BNO055 ADR               → GND (I2C address 0x28; tie HIGH for 0x29)
#
# Wiring — AS5600 jaw encoder
# ---------------------------
#   ESP32 GPIO 21 (I2C SDA)  → AS5600 SDA  (shared bus with BNO055)
#   ESP32 GPIO 22 (I2C SCL)  → AS5600 SCL  (shared bus with BNO055)
#   ESP32 3.3V               → AS5600 VDD
#   ESP32 GND                → AS5600 GND
#   AS5600 address is fixed at 0x36 — no ADDR pin
#
# Servo IDs
# ---------
#   Motor 4 – Pitch
#   Motor 5 – Roll
#   Motor 6 – Yaw/Grip 1
#   Motor 7 – Yaw/Grip 2
#
# DOF Mapping (BNO055 → Instrument)
# -----------------------------------
#   BNO055 Pitch   (X-axis tilt, forward/back)      → Instrument Pitch  (Motor 4)
#   BNO055 Roll    (Y-axis tilt, side-to-side)       → Instrument Roll   (Motor 5)
#   BNO055 Heading (Z-axis rotation, wrist "tilt")   → Instrument Yaw    (Motors 6 & 7)
#   AS5600 finger encoder (pointer finger curl)      → Instrument Grip   (Motors 6 & 7 differential)
#
# Control mode: delta from home (Da Vinci-style)
#   On boot, the BNO055's orientation is captured as the neutral/home position.
#   All motion is computed as the angular delta from that home orientation.
#   This lets the operator hold their hand in any comfortable resting posture —
#   wherever the hand is at boot becomes "instrument centered."
#   A clutch button (not yet wired) can re-capture home mid-session.
#
# Motion Limits
# -------------
#   Roll:  ±175°  (from center)
#   Pitch:  ±90°  (from center)
#   Yaw:    ±80°  (soft limit; physical max ±95°, reduced to preserve grip range)
#   Grip:    0–25° (0 = fully closed; 25 = fully open)
#
# Axis Coupling / Inverse Kinematics
# ------------------------------------
# Cable-driven EndoWrist architecture — same IK as the joystick version.
#
#   Roll  (Motor 5 only — fully independent):
#     Motor 5 = roll_angle
#
#   Pitch (Motors 4, 6, 7 — cables 6 and 7 route through the pitch joint):
#     Motor 4 = pitch_angle
#     Motor 6 += pitch_angle × 0.5
#     Motor 7 += pitch_angle × 0.5
#
#   Yaw   (Motors 6, 7 — both cables move same direction):
#     Motor 6 += yaw_angle
#     Motor 7 += yaw_angle
#
#   Grip  (Motors 6, 7 — differential):
#     Motor 6 += grip_angle × 0.5
#     Motor 7 -= grip_angle × 0.5
#
# Combined motor targets for a (pitch, yaw, roll, grip) command:
#   Motor 4 = pitch
#   Motor 5 = roll
#   Motor 6 = 0.5·pitch + yaw + 0.5·grip
#   Motor 7 = 0.5·pitch + yaw − 0.5·grip
# =============================================================================

print("[boot] imports starting...")
import sys
import math
import time
import uasyncio as asyncio  # type: ignore — MicroPython only
from machine import I2C, Pin, UART
print("[boot] imports OK")

# =============================================================================
# CONFIGURATION
# =============================================================================

# Set True to skip all servo/UART init and just print live BNO055 readings.
# Use this to verify axis mapping before connecting the servo bus adapter.
# Set False for normal operation.
DRY_RUN = False

# UART (unchanged from joystick version)
UART_TX_PIN = 17
UART_RX_PIN = 16
UART_BAUD   = 1_000_000

# Servo position conversion
COUNTS_PER_DEG = 4096 / 360.0
SERVO_CENTER   = 2048

# I2C bus (shared by BNO055 and AS5600)
I2C_SDA_PIN  = 21
I2C_SCL_PIN  = 22
I2C_FREQ     = 400_000

# BNO055
BNO055_ADDR  = 0x28    # 0x29 if ADDR pin tied high

# AS5600 jaw encoder
AS5600_ADDR  = 0x36    # fixed, no ADDR pin
# Set True if closing the finger decreases the encoder count instead of increasing it.
# Flip this if the jaw moves backwards relative to your finger.
GRIP_INVERT  = True

# Jaw calibration: minimum encoder range (counts) required for a valid calibration.
# If the sweep produces less than this, the sensor is likely not moving.
JAW_MIN_RANGE_COUNTS = 50

# Scale factors: how many degrees of instrument motion per degree of wrist motion.
# Set to 1.0 for 1:1 (full range of BNO055 input = full instrument limit).
# Reduce to make control feel less twitchy (e.g. 0.5 = 50% sensitivity).
SCALE_PITCH   = 0.8
SCALE_ROLL    = 0.8
SCALE_YAW     = 0.8

# Axis soft limits (degrees from center)
LIMIT_PITCH    = 90.0
LIMIT_YAW      = 80.0
LIMIT_ROLL     = 175.0
LIMIT_GRIP_MAX = 25.0

# Servo position count limits — worst-case combined angle per motor
_clim = lambda d: round(d * COUNTS_PER_DEG)
COUNT_LIMIT_4  = _clim(LIMIT_PITCH)
COUNT_LIMIT_5  = _clim(LIMIT_ROLL)
COUNT_LIMIT_67 = _clim(0.5 * LIMIT_PITCH + LIMIT_YAW + 0.5 * LIMIT_GRIP_MAX)

# IMU update rate
IMU_HZ = 50
IMU_MS = 1000 // IMU_HZ

# EMA smoothing (0.0–1.0); lower = smoother/laggier
EMA_ALPHA = 0.2

# Servo motion parameters
GOAL_TIME_MS = 20   # ms — matched to IMU update period; see joystick version for rationale
GOAL_ACC     = 0    # 0 = max acceleration

# Deadband (degrees): ignore BNO055 delta smaller than this to suppress drift at rest
DEADBAND_DEG = 1.5

# DRY_RUN display: set False if ANSI escape codes show as garbage in your terminal.
# When True, the 5-line display updates in-place. When False, it scrolls.
DISPLAY_ANSI = True

# =============================================================================
# ST BUS PROTOCOL
# =============================================================================

INST_WRITE = 0x03
INST_READ  = 0x02

REG_ACC           = 41
REG_GOAL_POSITION = 42
REG_GOAL_TIME     = 44
REG_GOAL_SPEED    = 46
REG_PRESENT_POS   = 56
REG_MOVING        = 66


def _checksum(body):
    return (~sum(body)) & 0xFF


def _build_packet(scs_id, instruction, params):
    length = len(params) + 2
    body   = [scs_id, length, instruction] + list(params)
    return bytes([0xFF, 0xFF] + body + [_checksum(body)])


def write_pos_ex(uart, scs_id, position, speed, acc):
    params = [
        REG_ACC,
        acc,
        position & 0xFF, (position >> 8) & 0xFF,
        0, 0,
        speed & 0xFF, (speed >> 8) & 0xFF,
    ]
    uart.write(_build_packet(scs_id, INST_WRITE, params))
    time.sleep_ms(2)
    uart.read()


def write_pos_timed(uart, scs_id, position, goal_time_ms, acc):
    params = [
        REG_ACC,
        acc,
        position & 0xFF, (position >> 8) & 0xFF,
        goal_time_ms & 0xFF, (goal_time_ms >> 8) & 0xFF,
        0, 0,
    ]
    uart.write(_build_packet(scs_id, INST_WRITE, params))
    time.sleep_ms(2)
    uart.read()


def read_register(uart, scs_id, address, length):
    uart.write(_build_packet(scs_id, INST_READ, [address, length]))
    expected = 6 + length
    deadline = time.ticks_add(time.ticks_ms(), 50)
    buf = b''
    while len(buf) < expected:
        chunk = uart.read(expected - len(buf))
        if chunk:
            buf += chunk
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            return None
    idx = buf.find(b'\xff\xff')
    if idx == -1 or len(buf) < idx + expected:
        return None
    return buf[idx + 5 : idx + 5 + length]


def read_moving(uart, scs_id):
    raw = read_register(uart, scs_id, REG_MOVING, 1)
    return raw is not None and raw[0] != 0


def wait_until_stopped(uart, scs_id, timeout_ms=10000):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if not read_moving(uart, scs_id):
            return time.ticks_diff(time.ticks_ms(), t0)
        time.sleep_ms(10)
    return timeout_ms


# =============================================================================
# SERVO
# =============================================================================

class Servo:
    def __init__(self, scs_id, count_limit, center_counts=SERVO_CENTER):
        self.scs_id        = scs_id
        self.count_limit   = count_limit
        self.center_counts = center_counts
        self.target_deg    = 0.0

    def set_target_degrees(self, degrees):
        self.target_deg = max(-self.count_limit / COUNTS_PER_DEG,
                              min( self.count_limit / COUNTS_PER_DEG, degrees))

    def _deg_to_counts(self, degrees):
        raw = self.center_counts + round(degrees * COUNTS_PER_DEG)
        return max(0, min(4095, raw))

    def send_target(self, uart, goal_time_ms=GOAL_TIME_MS, acc=GOAL_ACC):
        counts = self._deg_to_counts(self.target_deg)
        write_pos_timed(uart, self.scs_id, counts, goal_time_ms, acc)


# =============================================================================
# BNO055 DRIVER (minimal, register-level, no external library)
# =============================================================================

# Register addresses
_BNO_PAGE_ID      = 0x07
_BNO_CHIP_ID      = 0x00
_BNO_OPR_MODE     = 0x3D
_BNO_PWR_MODE     = 0x3E
_BNO_SYS_TRIGGER  = 0x3F
_BNO_CALIB_STAT   = 0x35
_BNO_EULER_H_LSB  = 0x1A   # 6 bytes: H_LSB, H_MSB, R_LSB, R_MSB, P_LSB, P_MSB
_BNO_QUAT_W_LSB   = 0x20   # 8 bytes: W, X, Y, Z each as signed 16-bit little-endian, scale 1/16384

_CONFIGMODE = 0x00
_NDOF_MODE  = 0x0C   # full fusion: accel + gyro + mag
_IMU_MODE   = 0x08   # accel + gyro only — use if AS5600 magnet interferes with heading


def _s16(lo, hi):
    """Signed 16-bit from two bytes, little-endian."""
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


class BNO055:
    """
    Minimal BNO055 driver for MicroPython.
    Reads quaternion output from NDOF fusion mode.
    Euler angle reading retained for calibration display only.
    """

    def __init__(self, i2c, addr=BNO055_ADDR):
        self.i2c  = i2c
        self.addr = addr
        self._init()

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _init(self):
        # Select page 0
        self._write(_BNO_PAGE_ID, 0x00)
        time.sleep_ms(10)

        chip_id = self._read(_BNO_CHIP_ID, 1)[0]
        if chip_id != 0xA0:
            raise RuntimeError(f"BNO055 not found (chip_id=0x{chip_id:02X}, expected 0xA0)")

        # Reset
        self._write(_BNO_SYS_TRIGGER, 0x20)
        time.sleep_ms(650)   # datasheet: 650ms after reset before accepting commands

        # Normal power, NDOF fusion mode
        self._write(_BNO_PWR_MODE, 0x00)
        time.sleep_ms(10)
        self._write(_BNO_OPR_MODE, _NDOF_MODE)
        time.sleep_ms(20)
        print("[BNO055] init OK — NDOF fusion mode")

    def calibration(self):
        """Returns (sys, gyro, accel, mag) calibration scores, each 0–3. 3 = fully calibrated."""
        stat = self._read(_BNO_CALIB_STAT, 1)[0]
        return (stat >> 6) & 3, (stat >> 4) & 3, (stat >> 2) & 3, stat & 3

    def euler(self):
        """Returns (heading, roll, pitch) in degrees. Used for calibration display only."""
        data = self._read(_BNO_EULER_H_LSB, 6)
        h = _s16(data[0], data[1]) / 16.0
        r = _s16(data[2], data[3]) / 16.0
        p = _s16(data[4], data[5]) / 16.0
        return h, r, p

    def quaternion(self):
        """Returns (w, x, y, z) unit quaternion representing current orientation."""
        data = self._read(_BNO_QUAT_W_LSB, 8)
        w = _s16(data[0], data[1]) / 16384.0
        x = _s16(data[2], data[3]) / 16384.0
        y = _s16(data[4], data[5]) / 16384.0
        z = _s16(data[6], data[7]) / 16384.0
        return w, x, y, z


# =============================================================================
# AS5600 DRIVER (minimal, register-level)
# =============================================================================

class AS5600:
    """
    Minimal AS5600 driver. Reads the filtered 12-bit angle (0–4095 = 0–360°).
    I2C address is fixed at 0x36.
    """
    _ANGLE_REG  = 0x0E   # 2 bytes, big-endian, 12-bit (bits 11:0)
    _STATUS_REG = 0x1A   # bit 3 = MD (magnet detected)

    def __init__(self, i2c, addr=AS5600_ADDR):
        self.i2c  = i2c
        self.addr = addr
        status = i2c.readfrom_mem(addr, self._STATUS_REG, 1)[0]
        if not (status & 0x08):
            raise RuntimeError("AS5600: no magnet detected — check placement and power")
        print("[AS5600] init OK — magnet detected")

    def angle(self):
        """Returns raw 12-bit angle count (0–4095)."""
        data = self.i2c.readfrom_mem(self.addr, self._ANGLE_REG, 2)
        return ((data[0] & 0x0F) << 8) | data[1]


def _read_grip_deg(jaw, jaw_min, jaw_max):
    """Map current AS5600 count to grip degrees using calibrated min/max."""
    t = (jaw.angle() - jaw_min) / (jaw_max - jaw_min)
    t = max(0.0, min(1.0, t))
    if GRIP_INVERT:
        t = 1.0 - t
    return t * LIMIT_GRIP_MAX


def _quat_delta(hw, hx, hy, hz, w, x, y, z):
    """
    Compute the delta quaternion from home to current: q_delta = q_home_conj * q_curr.
    Returns (dw, dx, dy, dz) representing the rotation away from home,
    expressed in the home frame — so extracted angles are home-relative.
    """
    dw =  hw*w + hx*x + hy*y + hz*z
    dx =  hw*x - hx*w - hy*z + hz*y
    dy =  hw*y + hx*z - hy*w - hz*x
    dz =  hw*z - hx*y + hy*x - hz*w
    return dw, dx, dy, dz


def _quat_to_euler(dw, dx, dy, dz):
    """
    Convert a delta quaternion to (roll, pitch, yaw) in degrees using ZYX convention.
    roll  = rotation about X (maps to instrument pitch after axis swap)
    pitch = rotation about Y (maps to instrument roll after axis swap)
    yaw   = rotation about Z (maps to instrument yaw)
    No singularities for the axes within our servo limits.
    """
    roll  = math.degrees(math.atan2(2*(dw*dx + dy*dz), 1 - 2*(dx*dx + dy*dy)))
    sinp  = max(-1.0, min(1.0, 2*(dw*dy - dz*dx)))
    pitch = math.degrees(math.asin(sinp))
    yaw   = math.degrees(math.atan2(2*(dw*dz + dx*dy), 1 - 2*(dy*dy + dz*dz)))
    return roll, pitch, yaw


def _deadband(val, threshold):
    return val if abs(val) >= threshold else 0.0


# =============================================================================
# ASCII SENSOR DISPLAY (DRY_RUN)
# =============================================================================

_BAR_W = 20          # internal characters in each bar (21 positions for ± axes)
_display_first = True


def _bar_bipolar(value, limit):
    """Position marker on a ±limit scale. Returns (bar_str, is_clipped)."""
    clipped = abs(value) >= limit * 0.99
    pos = round((value + limit) / (2.0 * limit) * _BAR_W)
    pos = max(0, min(_BAR_W, pos))
    chars = ['-'] * (_BAR_W + 1)
    chars[pos] = '!' if clipped else 'X'
    return '[' + ''.join(chars) + ']', clipped


def _bar_grip(value, limit):
    """Fill bar for a 0–limit axis."""
    fill = round(value / limit * _BAR_W)
    fill = max(0, min(_BAR_W, fill))
    return '[' + '█' * fill + '░' * (_BAR_W - fill) + ']'


def _arrow(value):
    if value >  DEADBAND_DEG: return '→'
    if value < -DEADBAND_DEG: return '←'
    return '·'


def _print_sensor_display(pitch_deg, yaw_deg, roll_deg, grip_deg,
                           raw_pitch, raw_yaw, raw_roll, sys_cal):
    """
    Print a 5-line in-place sensor display. 'raw' is the EMA-smoothed value
    before deadband and clamping — useful for seeing what the deadband is eating.
    '!' on the bar means the axis is at its limit.
    """
    global _display_first
    if DISPLAY_ANSI and not _display_first:
        sys.stdout.write('\033[5A')
    _display_first = False

    rb,  rc  = _bar_bipolar(roll_deg,  LIMIT_ROLL)
    pb,  pc  = _bar_bipolar(pitch_deg, LIMIT_PITCH)
    yb,  yc  = _bar_bipolar(yaw_deg,   LIMIT_YAW)
    gb       = _bar_grip(grip_deg,     LIMIT_GRIP_MAX)

    clip = lambda c: ' !' if c else '  '
    print(f"Roll : {rb}  {roll_deg:+7.1f}°  {_arrow(roll_deg)}  raw:{raw_roll:+7.1f}°{clip(rc)}")
    print(f"Pitch: {pb}  {pitch_deg:+7.1f}°  {_arrow(pitch_deg)}  raw:{raw_pitch:+7.1f}°{clip(pc)}")
    print(f"Yaw  : {yb}  {yaw_deg:+7.1f}°  {_arrow(yaw_deg)}  raw:{raw_yaw:+7.1f}°{clip(yc)}")
    print(f"Grip : {gb}  {grip_deg:4.1f}° / {LIMIT_GRIP_MAX:.0f}°")
    print(f"{'─' * 58}  cal_sys:{sys_cal}")


async def _calibrate_jaw(jaw, prefix="[jaw]"):
    """
    Sweep the finger through its full ROM for 5 seconds while recording
    the min and max AS5600 counts. Returns (jaw_min, jaw_max).
    Raises if the range is too small (magnet not moving / not detected).
    """
    print(f"{prefix} jaw calibration — curl finger through FULL range for 5 seconds...")
    jaw_min = 4095
    jaw_max = 0
    for remaining in range(5, 0, -1):
        print(f"{prefix}   {remaining}s — current count: {jaw.angle()}  range so far: {jaw_max - jaw_min}")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 1000:
            c = jaw.angle()
            if c < jaw_min: jaw_min = c
            if c > jaw_max: jaw_max = c
            await asyncio.sleep_ms(20)
    span = jaw_max - jaw_min
    if span < JAW_MIN_RANGE_COUNTS:
        raise RuntimeError(f"jaw calibration failed — range only {span} counts (need {JAW_MIN_RANGE_COUNTS}); check magnet")
    print(f"{prefix} jaw calibration OK — min:{jaw_min}  max:{jaw_max}  range:{span} counts")
    return jaw_min, jaw_max


async def _wait_for_calibration(imu, prefix="[cal]"):
    """
    Block until the BNO055 gyro and accel are sufficiently calibrated (score >= 2).
    Mag is intentionally excluded — it takes ~30s and contributes less in delta mode.
    Prints a live status line every second so the operator knows what's happening.
    """
    print(f"{prefix} waiting for calibration — move naturally, sys score must reach 2")
    while True:
        sys_c, gyro_c, accel_c, mag_c = imu.calibration()
        print(f"{prefix}   sys:{sys_c}  gyro:{gyro_c}  accel:{accel_c}  mag:{mag_c}")
        if sys_c >= 2:
            print(f"{prefix} calibration OK (sys:{sys_c})")
            break
        await asyncio.sleep_ms(1000)

    print(f"{prefix} return to neutral position — home captures in 3...")
    await asyncio.sleep_ms(1000)
    print(f"{prefix} 2...")
    await asyncio.sleep_ms(1000)
    print(f"{prefix} 1...")
    await asyncio.sleep_ms(1000)


# =============================================================================
# ASYNCIO TASKS
# =============================================================================

async def imu_monitor_task(imu, jaw):
    """
    DRY_RUN only. Captures home then prints live DOF values at ~5Hz so you
    can verify axis mapping without the servo bus connected.

    Move the sensor and confirm:
      Tip forward/back  →  Pitch changes, Roll/Tilt stay near 0
      Tilt side-to-side →  Roll changes,  Pitch/Tilt stay near 0
      Rotate wrist      →  Tilt changes,  Pitch/Roll stay near 0
      Curl pointer finger → Grip increases toward 25°
    """
    await asyncio.sleep_ms(200)

    await _wait_for_calibration(imu, prefix="[dry-run]")

    # Capture home
    N_HOME = 16
    home_w, home_x, home_y, home_z = imu.quaternion()
    sw, sx, sy, sz = home_w, home_x, home_y, home_z
    for _ in range(N_HOME - 1):
        w, x, y, z = imu.quaternion()
        if w*sw + x*sx + y*sy + z*sz < 0:
            w, x, y, z = -w, -x, -y, -z
        sw += w; sx += x; sy += y; sz += z
        await asyncio.sleep_ms(10)
    home_w = sw / N_HOME; home_x = sx / N_HOME
    home_y = sy / N_HOME; home_z = sz / N_HOME
    mag = (home_w**2 + home_x**2 + home_y**2 + home_z**2) ** 0.5
    home_w /= mag; home_x /= mag; home_y /= mag; home_z /= mag
    print(f"[dry-run] home captured — w:{home_w:.3f}  x:{home_x:.3f}  y:{home_y:.3f}  z:{home_z:.3f}")

    jaw_min, jaw_max = await _calibrate_jaw(jaw, prefix="[dry-run]")

    print("[dry-run] --- SENSOR DISPLAY (5Hz) ---")
    print("[dry-run]  Roll=tilt side  Pitch=tip fwd/back  Yaw=rotate wrist  Grip=curl finger")
    print("[dry-run]  X=position on scale  !=at limit  →/←=direction  ·=at rest  raw=before deadband")

    ema_pitch = 0.0
    ema_yaw   = 0.0
    ema_roll  = 0.0

    while True:
        w, x, y, z = imu.quaternion()
        if w*home_w + x*home_x + y*home_y + z*home_z < 0:
            w, x, y, z = -w, -x, -y, -z
        dw, dx, dy, dz = _quat_delta(home_w, home_x, home_y, home_z, w, x, y, z)
        qroll, qpitch, qyaw = _quat_to_euler(dw, dx, dy, dz)

        # qroll = X rotation → instrument pitch (axis swap, same as joystick version)
        # qpitch = Y rotation → instrument roll
        # qyaw = Z rotation → instrument yaw
        ema_pitch = EMA_ALPHA * qroll  + (1 - EMA_ALPHA) * ema_pitch
        ema_roll  = EMA_ALPHA * qpitch + (1 - EMA_ALPHA) * ema_roll
        ema_yaw   = EMA_ALPHA * qyaw   + (1 - EMA_ALPHA) * ema_yaw

        raw_pitch = ema_pitch * SCALE_PITCH
        raw_roll  = ema_roll  * SCALE_ROLL
        raw_yaw   = ema_yaw   * SCALE_YAW

        pitch_deg = max(-LIMIT_PITCH, min(LIMIT_PITCH, _deadband(raw_pitch, DEADBAND_DEG)))
        yaw_deg   = max(-LIMIT_YAW,   min(LIMIT_YAW,   _deadband(raw_yaw,  DEADBAND_DEG)))
        roll_deg  = max(-LIMIT_ROLL,  min(LIMIT_ROLL,  _deadband(raw_roll, DEADBAND_DEG)))
        grip_deg  = _read_grip_deg(jaw, jaw_min, jaw_max)

        sys_c = imu.calibration()[0]
        _print_sensor_display(pitch_deg, yaw_deg, roll_deg, grip_deg,
                               raw_pitch, raw_yaw, raw_roll, sys_c)

        await asyncio.sleep_ms(200)


async def motor_task(uart, motor_4, motor_5, motor_6, motor_7):
    motors = [motor_4, motor_5, motor_6, motor_7]
    while True:
        for m in motors:
            m.send_target(uart)
        await asyncio.sleep_ms(IMU_MS)


async def imu_task(imu, jaw, motor_4, motor_5, motor_6, motor_7):
    """
    Read BNO055 Euler angles, compute delta from home orientation,
    apply scale + deadband, run IK, update motor targets.
    """
    # Wait briefly for fusion to settle after boot centering moves
    await asyncio.sleep_ms(200)

    await _wait_for_calibration(imu, prefix="[imu]")

    # Capture home orientation as quaternion
    N_HOME = 16
    home_w, home_x, home_y, home_z = imu.quaternion()
    sw, sx, sy, sz = home_w, home_x, home_y, home_z
    for _ in range(N_HOME - 1):
        w, x, y, z = imu.quaternion()
        if w*sw + x*sx + y*sy + z*sz < 0:
            w, x, y, z = -w, -x, -y, -z
        sw += w; sx += x; sy += y; sz += z
        await asyncio.sleep_ms(10)
    home_w = sw / N_HOME; home_x = sx / N_HOME
    home_y = sy / N_HOME; home_z = sz / N_HOME
    mag = (home_w**2 + home_x**2 + home_y**2 + home_z**2) ** 0.5
    home_w /= mag; home_x /= mag; home_y /= mag; home_z /= mag
    print(f"[imu] home captured — w:{home_w:.3f}  x:{home_x:.3f}  y:{home_y:.3f}  z:{home_z:.3f}")

    jaw_min, jaw_max = await _calibrate_jaw(jaw, prefix="[imu]")

    ema_pitch = 0.0
    ema_yaw   = 0.0
    ema_roll  = 0.0
    print_counter = 0

    while True:
        w, x, y, z = imu.quaternion()
        if w*home_w + x*home_x + y*home_y + z*home_z < 0:
            w, x, y, z = -w, -x, -y, -z
        dw, dx, dy, dz = _quat_delta(home_w, home_x, home_y, home_z, w, x, y, z)
        qroll, qpitch, qyaw = _quat_to_euler(dw, dx, dy, dz)

        # qroll = X rotation → instrument pitch (axis swap)
        # qpitch = Y rotation → instrument roll
        # qyaw = Z rotation → instrument yaw
        ema_pitch = EMA_ALPHA * qroll  + (1 - EMA_ALPHA) * ema_pitch
        ema_roll  = EMA_ALPHA * qpitch + (1 - EMA_ALPHA) * ema_roll
        ema_yaw   = EMA_ALPHA * qyaw   + (1 - EMA_ALPHA) * ema_yaw

        # Deadband + clamp
        pitch_deg = max(-LIMIT_PITCH, min(LIMIT_PITCH, _deadband(ema_pitch * SCALE_PITCH, DEADBAND_DEG)))
        yaw_deg   = max(-LIMIT_YAW,   min(LIMIT_YAW,   _deadband(ema_yaw   * SCALE_YAW,  DEADBAND_DEG)))
        roll_deg  = max(-LIMIT_ROLL,  min(LIMIT_ROLL,  _deadband(ema_roll  * SCALE_ROLL,  DEADBAND_DEG)))
        grip_deg  = _read_grip_deg(jaw, jaw_min, jaw_max)

        # Inverse kinematics — see header for derivation
        motor_4.set_target_degrees(pitch_deg)
        motor_5.set_target_degrees(roll_deg)
        motor_6.set_target_degrees(0.5 * pitch_deg + yaw_deg + 0.5 * grip_deg)
        motor_7.set_target_degrees(0.5 * pitch_deg + yaw_deg - 0.5 * grip_deg)

        print_counter += 1
        if print_counter >= 50:
            print_counter = 0
            sys_c = imu.calibration()[0]
            print(f"pitch: {pitch_deg:+.1f}°  yaw: {yaw_deg:+.1f}°  roll: {roll_deg:+.1f}°  grip: {grip_deg:.1f}°  cal_sys:{sys_c}")

        await asyncio.sleep_ms(IMU_MS)


# =============================================================================
# ENTRY POINT
# =============================================================================

async def _control_loop(uart, imu, jaw, motor_4, motor_5, motor_6, motor_7):
    asyncio.create_task(motor_task(uart, motor_4, motor_5, motor_6, motor_7))
    asyncio.create_task(imu_task(imu, jaw, motor_4, motor_5, motor_6, motor_7))
    while True:
        await asyncio.sleep_ms(1000)


async def _dry_run_loop(imu, jaw):
    asyncio.create_task(imu_monitor_task(imu, jaw))
    while True:
        await asyncio.sleep_ms(1000)


try:
    print("[boot] initialising I2C...")
    i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ)

    print("[boot] initialising BNO055...")
    imu = BNO055(i2c)
    print("[boot] BNO055 OK")

    print("[boot] initialising AS5600 jaw encoder...")
    jaw = AS5600(i2c)
    print("[boot] AS5600 OK")

    if DRY_RUN:
        print("[boot] DRY_RUN=True — skipping UART and servos, starting IMU monitor...")
        asyncio.run(_dry_run_loop(imu, jaw))
    else:
        print("[boot] initialising UART2...")
        uart = UART(2, baudrate=UART_BAUD, tx=UART_TX_PIN, rx=UART_RX_PIN)
        print(f"[boot] UART2 ready — TX=GPIO{UART_TX_PIN}  RX=GPIO{UART_RX_PIN}  baud={UART_BAUD}")

        print("[boot] creating servos...")
        motor_4 = Servo(4, COUNT_LIMIT_4)
        motor_5 = Servo(5, COUNT_LIMIT_5)
        motor_6 = Servo(6, COUNT_LIMIT_67)
        motor_7 = Servo(7, COUNT_LIMIT_67)
        print("[boot] servos OK")

        print("[boot] centering all motors...")
        for mid, motor in ((4, motor_4), (5, motor_5), (6, motor_6), (7, motor_7)):
            write_pos_ex(uart, mid, SERVO_CENTER, 500, 50)
        for mid in (4, 5, 6, 7):
            wait_until_stopped(uart, mid)
        print("[boot] all motors centered")

        print("[boot] starting IMU control loop...")
        asyncio.run(_control_loop(uart, imu, jaw, motor_4, motor_5, motor_6, motor_7))

except Exception as e:
    print("[boot] FATAL ERROR:", e)
    raise

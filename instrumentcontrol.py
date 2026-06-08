# =============================================================================
# Fine Instrument Controller — STS3215 Servo Edition
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
# Wiring
# ------
#   ESP32 GPIO 17 (UART2 TX) → Waveshare TX
#   ESP32 GPIO 16 (UART2 RX) → Waveshare RX
#   ESP32 GND                → Waveshare GND
#   Waveshare USB or 5V pin  → 5V adapter logic power
#   Waveshare servo terminals → 12V servo bus power
#
# Servo IDs
# ---------
#   Motor 4 – Pitch
#   Motor 5 – Roll
#   Motor 6 – Yaw/Grip 1
#   Motor 7 – Yaw/Grip 2
#
# Joystick (3-axis analog, 10kΩ potentiometers):
#   Forward/Back   (Pitch) → ADC GPIO 34
#   Left/Right     (Yaw)   → ADC GPIO 35
#   Axial Rotation (Roll)  → ADC GPIO 36
#
# Motion Limits
# -------------
#   Roll:  ±175°  (from center)
#   Pitch:  ±90°  (from center)
#   Yaw:    ±80°  (soft limit; physical max ±95°, reduced to preserve grip range)
#   Grip:    0–25° (0 = fully closed; not joystick-controlled yet)
#
# Axis Coupling / Inverse Kinematics
# ------------------------------------
# Same cable-driven EndoWrist architecture as the original stepper version.
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
#   Grip  (Motors 6, 7 — differential; grip_angle must remain ≥ 0):
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
import time
import uasyncio as asyncio  # type: ignore — MicroPython only, no desktop stub
from machine import ADC, Pin, UART
print("[boot] imports OK")

# =============================================================================
# CONFIGURATION
# =============================================================================

# UART
UART_TX_PIN = 17        # Consistent with old instrumentcontrol.py
UART_RX_PIN = 16        # Consistent with old instrumentcontrol.py
UART_BAUD   = 1_000_000 # STS3215 factory default baudrate

# Servo position conversion
# 4096 counts = 360°, center at 2048
COUNTS_PER_DEG = 4096 / 360.0   # ≈ 11.378 counts/degree
SERVO_CENTER   = 2048

# Joystick ADC pins (same as old instrumentcontrol.py)
ADC_PITCH_PIN = 34
ADC_YAW_PIN   = 35
ADC_ROLL_PIN  = 36

ADC_MAX    = 65535   # read_u16() returns 0–65535
ADC_CENTER = ADC_MAX / 2.0

# Deadband: normalized joystick noise threshold near center (0.0–1.0)
DEADBAND = 0.04

# Axis scale factors
SCALE_PITCH = 1.0
SCALE_YAW   = 1.0
SCALE_ROLL  = 0.0  # disabled — roll pot disconnected; change to 1.0 when replaced

# Axis soft limits (degrees from center)
LIMIT_PITCH    = 90.0
LIMIT_YAW      = 80.0
LIMIT_ROLL     = 175.0
LIMIT_GRIP     = 0.0     # constant until a grip input is added
LIMIT_GRIP_MAX = 25.0    # physical max; used for worst-case servo limit calc

# Servo position limits (counts from center) — worst-case combined angle per motor
_clim = lambda d: round(d * COUNTS_PER_DEG)
COUNT_LIMIT_4  = _clim(LIMIT_PITCH)
COUNT_LIMIT_5  = _clim(LIMIT_ROLL)
COUNT_LIMIT_67 = _clim(0.5 * LIMIT_PITCH + LIMIT_YAW + 0.5 * LIMIT_GRIP_MAX)

# Joystick update rate
JOYSTICK_HZ = 50         # Hz — how often we read the joystick and send new targets
JOYSTICK_MS = 1000 // JOYSTICK_HZ

# EMA smoothing factor (0.0–1.0); lower = smoother but laggier
EMA_ALPHA = 0.2

# Servo motion parameters for joystick control
#
# We use goal_time rather than speed to synchronise all motors.
# When goal_time > 0 the servo ignores the speed register and instead moves
# from its current position to the target in exactly goal_time milliseconds,
# regardless of distance. Because all motors share the same time budget,
# they move proportionally and arrive simultaneously — preserving the
# cable-coupling ratios at every intermediate point.
#
# Set to slightly longer than the joystick update period (JOYSTICK_MS) so
# each move finishes just before the next command is issued.
# Raise if motion feels too aggressive; lower if it feels laggy.
JOYSTICK_GOAL_TIME_MS = 20   # ms — must be > 0 to activate timed mode; matched to
                             # JOYSTICK_MS so the servo chases each new command at full speed
JOYSTICK_ACC          = 0    # 0 = maximum acceleration (instantaneous ramp)

#=============================================================================
# ST BUS PROTOCOL
# =============================================================================
#
# Packet format:
#   0xFF  0xFF  ID  LENGTH  INSTRUCTION  PARAM0 ... PARAM_N  CHECKSUM
#
#   LENGTH   = num_params + 2  (instruction + checksum)
#   CHECKSUM = ~(ID + LENGTH + INSTRUCTION + all params) & 0xFF

INST_WRITE = 0x03
INST_READ  = 0x02

# STS3215 register map (from Feetech SMS/STS datasheet)
REG_ACC           = 41   # 1 byte  — acceleration (0 = max)
REG_GOAL_POSITION = 42   # 2 bytes — target position, little-endian
REG_GOAL_TIME     = 44   # 2 bytes — move duration override (0 = use speed)
REG_GOAL_SPEED    = 46   # 2 bytes — max speed (0 = no limit)
REG_PRESENT_POS   = 56   # 2 bytes — current position (read only)
REG_MOVING        = 66   # 1 byte  — 1 while moving, 0 when stopped


def _checksum(body):
    return (~sum(body)) & 0xFF


def _build_packet(scs_id, instruction, params):
    length = len(params) + 2
    body   = [scs_id, length, instruction] + list(params)
    return bytes([0xFF, 0xFF] + body + [_checksum(body)])


def write_pos_ex(uart, scs_id, position, speed, acc):
    """Send a position command using speed control. position is raw servo counts (0–4095)."""
    params = [
        REG_ACC,
        acc,
        position & 0xFF, (position >> 8) & 0xFF,
        0, 0,
        speed & 0xFF, (speed >> 8) & 0xFF,
    ]
    uart.write(_build_packet(scs_id, INST_WRITE, params))
    time.sleep_ms(2)
    uart.read()  # discard status packet


def write_pos_timed(uart, scs_id, position, goal_time_ms, acc):
    """
    Send a position command using time control.
    The servo moves from its current position to `position` in exactly
    `goal_time_ms` milliseconds, regardless of distance. All motors given
    the same goal_time will move proportionally and arrive simultaneously,
    preserving cable-coupling ratios throughout the move.
    position is raw servo counts (0–4095).
    """
    params = [
        REG_ACC,
        acc,
        position & 0xFF, (position >> 8) & 0xFF,
        goal_time_ms & 0xFF, (goal_time_ms >> 8) & 0xFF,
        0, 0,   # speed ignored when goal_time > 0
    ]
    uart.write(_build_packet(scs_id, INST_WRITE, params))
    time.sleep_ms(2)
    uart.read()  # discard status packet


def read_register(uart, scs_id, address, length):
    """Read `length` bytes from `address`. Returns bytes or None on timeout."""
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
    """
    Represents one STS3215 on the bus.
    Tracks a target in degrees; call send_target() to push to hardware.
    center_counts is the servo count that corresponds to the instrument's
    neutral position (set during alignment; default 2048).
    """

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

    def send_target(self, uart, goal_time_ms=JOYSTICK_GOAL_TIME_MS, acc=JOYSTICK_ACC):
        counts = self._deg_to_counts(self.target_deg)
        write_pos_timed(uart, self.scs_id, counts, goal_time_ms, acc)

# =============================================================================
# ASYNCIO TASKS
# =============================================================================

async def motor_task(uart, motor_4, motor_5, motor_6, motor_7):
    """
    Push new target positions to all four servos at JOYSTICK_HZ.
    The STS3215 handles its own motion profile internally — no Bresenham needed.
    """
    motors = [motor_4, motor_5, motor_6, motor_7]
    while True:
        for m in motors:
            m.send_target(uart)
        await asyncio.sleep_ms(JOYSTICK_MS)


async def joystick_task(motor_4, motor_5, motor_6, motor_7):
    """
    Read all three joystick axes at JOYSTICK_HZ, compute inverse kinematics,
    and update motor target positions.
    """
    pitch_adc = ADC(Pin(ADC_PITCH_PIN))
    yaw_adc   = ADC(Pin(ADC_YAW_PIN))
    roll_adc  = ADC(Pin(ADC_ROLL_PIN))

    for adc in (pitch_adc, yaw_adc, roll_adc):
        adc.init(atten=ADC.ATTN_11DB)   # full 0–3.3V input range

    def read_normalized(adc, home_raw):
        raw = adc.read_u16()
        n = (raw - home_raw) / ADC_CENTER
        return max(-1.0, min(1.0, n))

    # Warm up ADC then capture home position
    for _ in range(10):
        pitch_adc.read_u16(); yaw_adc.read_u16(); roll_adc.read_u16()
    N_HOME = 16
    home_pitch = sum(pitch_adc.read_u16() for _ in range(N_HOME)) // N_HOME
    home_yaw   = sum(yaw_adc.read_u16()   for _ in range(N_HOME)) // N_HOME
    home_roll  = sum(roll_adc.read_u16()  for _ in range(N_HOME)) // N_HOME
    print(f"[joystick] home captured — pitch: {home_pitch}  yaw: {home_yaw}  roll: {home_roll}")

    ema_pitch = 0.0
    ema_yaw   = 0.0
    ema_roll  = 0.0
    print_counter = 0

    while True:
        ema_pitch = EMA_ALPHA * read_normalized(pitch_adc, home_pitch) + (1 - EMA_ALPHA) * ema_pitch
        ema_yaw   = EMA_ALPHA * read_normalized(yaw_adc,   home_yaw  ) + (1 - EMA_ALPHA) * ema_yaw
        ema_roll  = EMA_ALPHA * read_normalized(roll_adc,  home_roll ) + (1 - EMA_ALPHA) * ema_roll

        pitch_n = ema_pitch if abs(ema_pitch) >= DEADBAND else 0.0
        yaw_n   = ema_yaw   if abs(ema_yaw)   >= DEADBAND else 0.0
        roll_n  = ema_roll  if abs(ema_roll)   >= DEADBAND else 0.0

        pitch_deg = pitch_n * LIMIT_PITCH * SCALE_PITCH
        yaw_deg   = yaw_n   * LIMIT_YAW   * SCALE_YAW
        roll_deg  = roll_n  * LIMIT_ROLL  * SCALE_ROLL
        grip_deg  = LIMIT_GRIP

        # Inverse kinematics — see header for full derivation
        motor_4.set_target_degrees(pitch_deg)
        motor_5.set_target_degrees(roll_deg)
        motor_6.set_target_degrees(0.5 * pitch_deg + yaw_deg + 0.5 * grip_deg)
        motor_7.set_target_degrees(0.5 * pitch_deg + yaw_deg - 0.5 * grip_deg)

        print_counter += 1
        if print_counter >= 50:
            print_counter = 0
            print(f"pitch: {pitch_deg:+.1f}°  yaw: {yaw_deg:+.1f}°  roll: {roll_deg:+.1f}°")

        await asyncio.sleep_ms(JOYSTICK_MS)


# =============================================================================
# ENTRY POINT
# =============================================================================

async def _control_loop(uart, motor_4, motor_5, motor_6, motor_7):
    asyncio.create_task(motor_task(uart, motor_4, motor_5, motor_6, motor_7))
    asyncio.create_task(joystick_task(motor_4, motor_5, motor_6, motor_7))
    while True:
        await asyncio.sleep_ms(1000)


try:
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

    print("[boot] starting joystick control loop...")
    asyncio.run(_control_loop(uart, motor_4, motor_5, motor_6, motor_7))

except Exception as e:
    print("[boot] FATAL ERROR:", e)
    raise

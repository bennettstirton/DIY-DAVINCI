# =============================================================================
# Leader / Follower Test — Grip → Motor G (ID 7)
# Target: ESP32 running MicroPython
#
# Architecture:
#   Leader  — Grip (STS3215 C046, ID 6, 7.4V, UART1, Mode 2)
#              Held in hand; encoder read every loop to get angle.
#   Follower — Motor G / ID 7 (STS3215 C047, ID 7, 12V, UART2, Mode 0)
#              Commanded to match grip displacement 1:1 in degrees.
#
# Phases:
#   1 — Grip setup: Mode 2 confirmed, friction comp applied, lever settles open.
#   2 — ROM calibration: open/close grip freely for CALIBRATION_SECONDS.
#   3 — Leader/follower loop: grip angle → motor G position.
#       Live display shows grip bar + motor G current readout.
#
# Axis labels (Intuitive naming → servo ID):
#   D = ID 4, E = ID 5, F = ID 6, G = ID 7
# =============================================================================

import time
from machine import UART

# =============================================================================
# CONFIGURATION
# =============================================================================

# --- Grip (leader) ---
GRIP_UART_ID = 1
GRIP_TX_PIN  = 4
GRIP_RX_PIN  = 5
GRIP_ID      = 6        # STS3215 C046, 7.4V, Mode 2

# +1 or -1 — positive = opening direction on the grip.
# With friction comp active, lever should drift toward open. Flip if it doesn't.
GRIP_DIR = 1

# Friction comp PWM — baseline opening torque when jaw is free. Valid range: 50–1000.
FRICTION_COMP_PWM = 75
# PWM applied to grip when motor G current exceeds ON threshold. Valid range: 50–1000.
FEEDBACK_PWM      = 250

# --- Motor G (follower) ---
FOLLOWER_UART_ID = 2
FOLLOWER_TX_PIN  = 17
FOLLOWER_RX_PIN  = 16
FOLLOWER_ID      = 7    # STS3215 C047, 12V, Mode 0

# +1 or -1 — flip if motor G moves the wrong way when grip closes.
FOLLOWER_DIR = 1

# Safety clamp: motor G will not travel more than this many degrees from its
# zero position (its position when the leader/follower loop starts).
FOLLOWER_LIMIT_DEG = 30.0

# --- Timing ---
UART_BAUD    = 1_000_000
LOOP_HZ      = 50
LOOP_MS      = 1000 // LOOP_HZ
DISPLAY_HZ   = 10
GOAL_TIME_MS = LOOP_MS   # motor G move budget per command — matches loop period

# --- Calibration ---
SETTLE_SECONDS       = 2
CALIBRATION_SECONDS  = 5
MIN_ROM_COUNTS       = 60    # ~5° — warn if ROM smaller than this

# --- Current smoothing + hysteresis ---
# EMA_ALPHA: 0.1 = very smooth/slow, 0.5 = faster/noisier. Start at 0.2 and tune.
EMA_ALPHA = 0.2

# Hysteresis band prevents chattering at the threshold.
# State flips to GRIP only when EMA exceeds ON; returns to free only below OFF.
# 0.10A ≈ 15 STS units,  0.065A ≈ 10 STS units
CURRENT_ON_THRESHOLD  = 10    # EMA units — enter GRIP state
CURRENT_OFF_THRESHOLD = 8    # EMA units — exit GRIP state (must be < ON)

# --- Display ---
BAR_WIDTH         = 28
CURRENT_MAX_UNITS = 50      # units at which current bar reads full (~0.33A)
                            # keep small so threshold is visible in the bar

# =============================================================================
# REGISTER MAP
# =============================================================================

REG_MODE            = 33
REG_TORQUE_ENABLE   = 40
REG_ACC             = 41   # also start of WritePosEx block
REG_GOAL_SPEED      = 46
REG_LOCK            = 55
REG_PRESENT_POS     = 56
REG_PRESENT_CURRENT = 69

COUNTS_PER_DEG = 4096 / 360.0

INST_WRITE = 0x03
INST_READ  = 0x02

# =============================================================================
# PROTOCOL
# =============================================================================

def _checksum(body):
    return (~sum(body)) & 0xFF

def _build_packet(scs_id, instruction, params):
    length = len(params) + 2
    body   = [scs_id, length, instruction] + list(params)
    return bytes([0xFF, 0xFF] + body + [_checksum(body)])

def _write_reg(uart, scs_id, address, data_bytes):
    uart.write(_build_packet(scs_id, INST_WRITE, [address] + list(data_bytes)))
    time.sleep_ms(5)
    uart.read()

def _read_reg(uart, scs_id, address, length):
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

# =============================================================================
# SERVO HELPERS
# =============================================================================

def read_mode(uart, scs_id):
    raw = _read_reg(uart, scs_id, REG_MODE, 1)
    return raw[0] if raw is not None else None

def set_mode(uart, scs_id, mode):
    _write_reg(uart, scs_id, REG_TORQUE_ENABLE, [0])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_LOCK, [0])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_MODE, [mode])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_LOCK, [1])
    time.sleep_ms(20)

def write_pwm(uart, scs_id, pwm):
    """Grip servo Mode 2: register 44, direction BIT10. Range 50–1000."""
    mag = max(50, min(abs(pwm), 1000))
    raw = (mag | 0x0400) if pwm < 0 else mag
    _write_reg(uart, scs_id, 44, [raw & 0xFF, (raw >> 8) & 0xFF])

def write_pos_timed(uart, scs_id, position, goal_time_ms, acc=0):
    """Mode 0 position command with timed arrival (same pattern as instrumentcontrol.py)."""
    position = max(0, min(4095, position))
    params = [
        REG_ACC, acc,
        position & 0xFF, (position >> 8) & 0xFF,
        goal_time_ms & 0xFF, (goal_time_ms >> 8) & 0xFF,
        0, 0,
    ]
    uart.write(_build_packet(scs_id, INST_WRITE, params))
    time.sleep_ms(5)
    uart.read()

def read_position(uart, scs_id):
    raw = _read_reg(uart, scs_id, REG_PRESENT_POS, 2)
    if raw is None:
        return None
    val = raw[0] | (raw[1] << 8)
    return -(val & 0x7FFF) if (val & 0x8000) else val

def read_current(uart, scs_id):
    """Returns current magnitude in STS units (1 unit = 6.5 mA), or None."""
    raw = _read_reg(uart, scs_id, REG_PRESENT_CURRENT, 2)
    if raw is None:
        return None
    return (raw[0] | (raw[1] << 8)) & 0x7FFF

# =============================================================================
# DISPLAY
# =============================================================================

def _grip_bar(ratio, width=None):
    w = width or BAR_WIDTH
    mid_i    = w // 2
    thresh_i = round(w * 2 / 3)
    fill_end = max(0, min(w, round(ratio * w)))
    chars = []
    for i in range(w):
        if i == thresh_i:
            chars.append('|')
        elif i == mid_i:
            chars.append('+')
        elif i < fill_end:
            chars.append('#')
        else:
            chars.append('.')
    return '[' + ''.join(chars) + ']'

def _current_bar(ema, width=16):
    """Fill bar for EMA current with a threshold marker at ON position."""
    thresh_i = min(width - 1, round(CURRENT_ON_THRESHOLD / CURRENT_MAX_UNITS * width))
    fill_end = max(0, min(width, round(ema / CURRENT_MAX_UNITS * width)))
    chars = []
    for i in range(width):
        if i == thresh_i:
            chars.append('!')   # ON threshold marker — always visible
        elif i < fill_end:
            chars.append('#')
        else:
            chars.append('.')
    return '[' + ''.join(chars) + ']'


def render(grip_ratio, grip_deg, raw_cur, ema_cur, in_grip, grip_pwm, follower_deg):
    state = 'GRIP' if in_grip else 'free'
    if raw_cur is not None:
        cur_str = f"raw={raw_cur:2d}u ema={ema_cur:4.1f}u {ema_cur * 6.5 / 1000:.3f}A"
        cur_bar = _current_bar(ema_cur)
    else:
        cur_str = "raw=--  ema=----"
        cur_bar = '[' + '.' * 16 + ']'
    line = (
        f"\r  grip {_grip_bar(grip_ratio)} {grip_deg:4.1f}°"
        f"  G:{cur_bar} {cur_str}"
        f"  {state} pwm={grip_pwm:+d} G={follower_deg:+.1f}°   "
    )
    print(line, end='')

# =============================================================================
# PHASE 1 — GRIP SETUP
# =============================================================================

def phase1_setup(grip_uart):
    print("[phase 1] checking grip servo (ID 6)...")
    mode = read_mode(grip_uart, GRIP_ID)
    print(f"          mode: {mode}")

    if mode != 2:
        print("          writing Mode 2 to EEPROM...")
        set_mode(grip_uart, GRIP_ID, 2)
        print()
        print("  *** POWER-CYCLE the grip board (7.4V), then re-run. ***")
        raise SystemExit(0)

    print("          Mode 2 confirmed.")
    _write_reg(grip_uart, GRIP_ID, REG_TORQUE_ENABLE, [1])
    pwm = GRIP_DIR * FRICTION_COMP_PWM
    write_pwm(grip_uart, GRIP_ID, pwm)
    print(f"          Friction comp ON: PWM = {pwm:+d}")
    print(f"          Settling for {SETTLE_SECONDS}s...")
    for _ in range(SETTLE_SECONDS * 4):
        time.sleep_ms(250)
        pos = read_position(grip_uart, GRIP_ID)
        print(f"\r          settling... pos={pos}    ", end='')
    print()

# =============================================================================
# PHASE 2 — ROM CALIBRATION
# =============================================================================

def phase2_calibrate(grip_uart):
    print()
    print("[phase 2] grip ROM calibration")

    open_counts = read_position(grip_uart, GRIP_ID)
    if open_counts is None:
        raise RuntimeError("Cannot read grip position — check wiring.")
    print(f"          Open position: {open_counts} counts")
    print()
    print(f"  >>>  Open and close the grip FULLY several times.")
    print(f"  >>>  Sampling for {CALIBRATION_SECONDS} seconds...")
    print()

    mn, mx = open_counts, open_counts
    last_sec = -1
    t_end = time.ticks_add(time.ticks_ms(), CALIBRATION_SECONDS * 1000)

    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        remaining = time.ticks_diff(t_end, time.ticks_ms()) // 1000
        pos = read_position(grip_uart, GRIP_ID)
        if pos is not None:
            mn = min(mn, pos)
            mx = max(mx, pos)
            if remaining != last_sec:
                print(f"\r  {remaining:2d}s  pos={pos:4d}  range={mx - mn:4d} counts    ", end='')
                last_sec = remaining
        time.sleep_ms(50)

    print()
    print()

    closed_counts = mx if abs(mx - open_counts) >= abs(mn - open_counts) else mn
    rom_counts    = closed_counts - open_counts
    rom_deg       = abs(rom_counts) * (360.0 / 4096)

    if abs(rom_counts) < MIN_ROM_COUNTS:
        print(f"  WARNING: ROM only {abs(rom_counts)} counts — try opening/closing more fully.")

    thresh_counts = round(open_counts + rom_counts * 2/3)

    print(f"  Open:      {open_counts:4d} counts  (0°)")
    print(f"  Closed:    {closed_counts:4d} counts  ({rom_deg:.1f}°)")
    print(f"  ROM:       {abs(rom_counts):4d} counts  ({rom_deg:.1f}°)")
    print(f"  2/3 mark:  {thresh_counts:4d} counts  ({rom_deg * 2/3:.1f}°)")
    print()

    return open_counts, rom_counts, rom_deg

# =============================================================================
# PHASE 3 — LEADER / FOLLOWER LOOP
# =============================================================================

def phase3_leader_follower(grip_uart, follower_uart, open_counts, rom_counts, rom_deg):
    # Capture motor G zero position
    follower_zero = read_position(follower_uart, FOLLOWER_ID)
    if follower_zero is None:
        raise RuntimeError("Cannot read motor G (ID 7) — check 12V bus and wiring.")
    print(f"[phase 3] Motor G zero: {follower_zero} counts")

    # Enable motor G torque (Mode 0 — already its default)
    _write_reg(follower_uart, FOLLOWER_ID, REG_TORQUE_ENABLE, [1])
    print(f"          Motor G torque enabled.")
    print()
    print(f"  Leader/follower + haptic feedback at {LOOP_HZ} Hz.")
    print(f"  Current bar: '!' marks the ON threshold ({CURRENT_ON_THRESHOLD}u, ~{CURRENT_ON_THRESHOLD * 6.5:.0f}mA)")
    print(f"  Grip free below {CURRENT_OFF_THRESHOLD}u, GRIP above {CURRENT_ON_THRESHOLD}u (hysteresis band)")
    print(f"  EMA alpha={EMA_ALPHA}  free pwm={GRIP_DIR * FRICTION_COMP_PWM:+d}  grip pwm={GRIP_DIR * FEEDBACK_PWM:+d}")
    print()

    limit_counts  = round(FOLLOWER_LIMIT_DEG * COUNTS_PER_DEG)
    display_tick  = 0
    display_every = max(1, LOOP_HZ // DISPLAY_HZ)
    ema_current   = 0.0
    in_grip       = False

    while True:
        t0 = time.ticks_ms()

        grip_pos   = read_position(grip_uart, GRIP_ID)
        raw_cur    = read_current(follower_uart, FOLLOWER_ID)

        # EMA smoothing
        if raw_cur is not None:
            ema_current = EMA_ALPHA * raw_cur + (1.0 - EMA_ALPHA) * ema_current

        # Hysteresis state machine
        if not in_grip and ema_current >= CURRENT_ON_THRESHOLD:
            in_grip = True
            print(f"\n  ── free → GRIP  ema={ema_current:.1f}u ({ema_current * 6.5:.0f}mA)")
        elif in_grip and ema_current < CURRENT_OFF_THRESHOLD:
            in_grip = False
            print(f"\n  ── GRIP → free  ema={ema_current:.1f}u ({ema_current * 6.5:.0f}mA)")

        # Grip servo torque
        grip_pwm = GRIP_DIR * (FEEDBACK_PWM if in_grip else FRICTION_COMP_PWM)
        write_pwm(grip_uart, GRIP_ID, grip_pwm)

        if grip_pos is not None and rom_counts != 0:
            ratio = max(0.0, min(1.0, (grip_pos - open_counts) / rom_counts))
            grip_deg = ratio * rom_deg

            offset_counts = round(grip_deg * COUNTS_PER_DEG * FOLLOWER_DIR)
            offset_counts = max(-limit_counts, min(limit_counts, offset_counts))
            target        = follower_zero + offset_counts
            follower_deg  = offset_counts / COUNTS_PER_DEG * FOLLOWER_DIR

            write_pos_timed(follower_uart, FOLLOWER_ID, target, GOAL_TIME_MS)

            display_tick += 1
            if display_tick >= display_every:
                display_tick = 0
                render(ratio, grip_deg, raw_cur, ema_current, in_grip, grip_pwm, follower_deg)

        elapsed   = time.ticks_diff(time.ticks_ms(), t0)
        remaining = LOOP_MS - elapsed
        if remaining > 0:
            time.sleep_ms(remaining)

# =============================================================================
# ENTRY POINT
# =============================================================================

grip_uart     = None
follower_uart = None

try:
    print("[boot] initialising UARTs...")
    grip_uart     = UART(GRIP_UART_ID,     baudrate=UART_BAUD, tx=GRIP_TX_PIN,     rx=GRIP_RX_PIN)
    follower_uart = UART(FOLLOWER_UART_ID, baudrate=UART_BAUD, tx=FOLLOWER_TX_PIN, rx=FOLLOWER_RX_PIN)
    print(f"[boot] UART{GRIP_UART_ID}: GPIO{GRIP_TX_PIN}/{GRIP_RX_PIN} — grip (leader)")
    print(f"[boot] UART{FOLLOWER_UART_ID}: GPIO{FOLLOWER_TX_PIN}/{FOLLOWER_RX_PIN} — motor G ID 7 (follower)")
    print()

    phase1_setup(grip_uart)
    open_c, rom_c, rom_deg = phase2_calibrate(grip_uart)
    phase3_leader_follower(grip_uart, follower_uart, open_c, rom_c, rom_deg)

except KeyboardInterrupt:
    print("\n[stop] Ctrl+C.")
    if grip_uart:
        _write_reg(grip_uart, GRIP_ID, REG_TORQUE_ENABLE, [0])
    if follower_uart:
        _write_reg(follower_uart, FOLLOWER_ID, REG_TORQUE_ENABLE, [0])
    print("[stop] Both servos released.")

except Exception as e:
    print("[FATAL]", e)
    for u, sid in [(grip_uart, GRIP_ID), (follower_uart, FOLLOWER_ID)]:
        if u:
            try:
                _write_reg(u, sid, REG_TORQUE_ENABLE, [0])
            except:
                pass
    raise

# =============================================================================
# Lever ROM Calibration + Mock Force Feedback Test
# Target: ESP32 running MicroPython — lever servo only (no jaw needed)
#
# What this does:
#   Phase 1 — applies friction comp so lever drifts to its natural open position
#   Phase 2 — samples lever encoder for CALIBRATION_SECONDS while you open/close
#              freely, then computes open counts, closed counts, and ROM
#   Phase 3 — runs a mock force feedback loop using lever position only:
#              past the 2/3-closed threshold → torque doubles (hysteresis)
#
# Display bar legend:
#   [#####+####║......................] 45.2°  FREE  pwm=+100
#        +  = ROM midpoint
#        ║  = 2/3-closed threshold (feedback trigger)
#        #  = filled (lever position)
#        .  = empty
# =============================================================================

import time
from machine import UART

# =============================================================================
# CONFIGURATION
# =============================================================================

LEVER_UART_ID = 1
LEVER_TX_PIN  = 4
LEVER_RX_PIN  = 5
LEVER_ID      = 6
UART_BAUD     = 1_000_000

# +1 or -1 — positive = opening direction
# Lever should drift toward open with friction comp. Flip if it goes the other way.
LEVER_DIR = 1

# PWM in opening direction when lever is in the free zone (before 2/3 threshold).
# Valid range: 50–1000.
FRICTION_COMP_PWM = 100

# PWM in opening direction when lever is past the 2/3 threshold (feedback zone).
# Tuned independently of FRICTION_COMP_PWM. Keep below 900 until confirmed safe.
FEEDBACK_PWM = 300

# Seconds to settle at open before calibration starts
SETTLE_SECONDS = 2

# Seconds of open/close sampling for ROM measurement
CALIBRATION_SECONDS = 5

# Minimum ROM in encoder counts to consider calibration valid (~5°)
MIN_ROM_COUNTS = 60

# Loop and display rates
LOOP_HZ    = 50
LOOP_MS    = 1000 // LOOP_HZ
DISPLAY_HZ = 10

# =============================================================================
# STS3215 REGISTER MAP
# =============================================================================

REG_MODE          = 33
REG_TORQUE_ENABLE = 40
REG_GOAL_PWM      = 44
REG_LOCK          = 55
REG_PRESENT_POS   = 56

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
    """Mode 2 PWM. Register 44, direction BIT10 (0x0400). Range 50–1000."""
    mag = max(50, min(abs(pwm), 1000))
    raw = (mag | 0x0400) if pwm < 0 else mag
    _write_reg(uart, scs_id, REG_GOAL_PWM, [raw & 0xFF, (raw >> 8) & 0xFF])

def read_position(uart, scs_id):
    raw = _read_reg(uart, scs_id, REG_PRESENT_POS, 2)
    if raw is None:
        return None
    val = raw[0] | (raw[1] << 8)
    return -(val & 0x7FFF) if (val & 0x8000) else val

# =============================================================================
# DISPLAY
# =============================================================================

BAR_WIDTH = 34

def lever_bar(ratio, in_feedback):
    """
    Fill bar from open (left) to closed (right).
    + = ROM midpoint   ║ = 2/3 threshold
    # = filled region  . = empty
    """
    mid_i    = BAR_WIDTH // 2
    thresh_i = round(BAR_WIDTH * 2 / 3)
    fill_end = max(0, min(BAR_WIDTH, round(ratio * BAR_WIDTH)))

    chars = []
    for i in range(BAR_WIDTH):
        if i == thresh_i:
            chars.append('║')
        elif i == mid_i:
            chars.append('+')
        elif i < fill_end:
            chars.append('#')
        else:
            chars.append('.')
    return '[' + ''.join(chars) + ']'


def render(ratio, deg, in_feedback, pwm):
    state = 'GRIP' if in_feedback else 'FREE'
    bar   = lever_bar(ratio, in_feedback)
    line  = (f"\r  open {bar} clsd"
             f"  {deg:5.1f}°  {state}  pwm={pwm:+d}    ")
    print(line, end='')

# =============================================================================
# PHASE 1 — SETUP
# =============================================================================

def phase1_setup(uart):
    print("[phase 1] checking servo mode...")
    mode = read_mode(uart, LEVER_ID)
    print(f"          current mode: {mode}")

    if mode != 2:
        print("          writing Mode 2 to EEPROM...")
        set_mode(uart, LEVER_ID, 2)
        print()
        print("  *** POWER-CYCLE the lever servo board, then re-run. ***")
        print()
        raise SystemExit(0)

    print("          Mode 2 confirmed.")
    _write_reg(uart, LEVER_ID, REG_TORQUE_ENABLE, [1])
    write_pwm(uart, LEVER_ID, LEVER_DIR * FRICTION_COMP_PWM)
    print(f"          Friction comp ON: PWM = {LEVER_DIR * FRICTION_COMP_PWM:+d}")
    print(f"          Letting lever settle for {SETTLE_SECONDS}s...")

    for i in range(SETTLE_SECONDS * 4):
        time.sleep_ms(250)
        pos = read_position(uart, LEVER_ID)
        print(f"\r          settling... pos={pos}    ", end='')
    print()

# =============================================================================
# PHASE 2 — ROM CALIBRATION
# =============================================================================

def phase2_calibrate(uart):
    print()
    print("[phase 2] ROM calibration")

    # Capture open position after settling
    open_counts = read_position(uart, LEVER_ID)
    if open_counts is None:
        raise RuntimeError("Cannot read lever position — check wiring.")
    print(f"          Open position captured: {open_counts} counts")
    print()
    print(f"  >>>  Open and close the lever FULLY several times.")
    print(f"  >>>  Sampling for {CALIBRATION_SECONDS} seconds...")
    print()

    mn = open_counts
    mx = open_counts
    last_sec = -1
    t_end = time.ticks_add(time.ticks_ms(), CALIBRATION_SECONDS * 1000)

    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        remaining = time.ticks_diff(t_end, time.ticks_ms()) // 1000
        pos = read_position(uart, LEVER_ID)
        if pos is not None:
            mn = min(mn, pos)
            mx = max(mx, pos)
            if remaining != last_sec:
                print(f"\r  {remaining:2d}s  pos={pos:4d}  range={mx - mn:4d} counts    ", end='')
                last_sec = remaining
        time.sleep_ms(50)   # 20 Hz sampling

    print()
    print()

    # Which extreme is closed? Furthest from open_counts.
    if abs(mx - open_counts) >= abs(mn - open_counts):
        closed_counts = mx
    else:
        closed_counts = mn

    rom_counts = closed_counts - open_counts   # signed

    if abs(rom_counts) < MIN_ROM_COUNTS:
        print(f"  WARNING: ROM only {abs(rom_counts)} counts — did you open/close fully?")
        print(f"           Continuing anyway, but results may be inaccurate.")

    rom_deg    = abs(rom_counts) * (360.0 / 4096)
    thresh_counts = round(open_counts + rom_counts * 2 / 3)
    mid_counts    = round(open_counts + rom_counts * 0.5)

    print(f"  Open:      {open_counts:4d} counts  (0°, lever at rest)")
    print(f"  Closed:    {closed_counts:4d} counts  ({rom_deg:.1f}°)")
    print(f"  ROM:       {abs(rom_counts):4d} counts  ({rom_deg:.1f}°)")
    print(f"  Midpoint:  {mid_counts:4d} counts  ({rom_deg * 0.5:.1f}°)")
    print(f"  Threshold: {thresh_counts:4d} counts  ({rom_deg * 2/3:.1f}°, 2/3 closed)")
    print()
    print(f"  FREE torque:  {LEVER_DIR * FRICTION_COMP_PWM:+d} PWM")
    print(f"  GRIP torque:  {LEVER_DIR * FEEDBACK_PWM:+d} PWM")
    print()

    return open_counts, closed_counts, rom_counts, rom_deg, thresh_counts

# =============================================================================
# PHASE 3 — MOCK FEEDBACK LOOP
# =============================================================================

def phase3_feedback(uart, open_counts, rom_counts, rom_deg, thresh_counts):

    print("[phase 3] mock force feedback running — Ctrl+C to stop")
    print(f"          lever bar:  open [{'#'*17}+{'#'*11}║{'.'*6}] clsd")
    print(f"                             ^ mid        ^ 2/3 threshold")
    print()

    in_feedback  = False
    display_tick = 0
    display_every = max(1, LOOP_HZ // DISPLAY_HZ)

    while True:
        t0 = time.ticks_ms()

        pos = read_position(uart, LEVER_ID)

        if pos is not None:
            # Clamp ratio to 0.0–1.0 (open → closed)
            if rom_counts == 0:
                ratio = 0.0
            else:
                ratio = max(0.0, min(1.0, (pos - open_counts) / rom_counts))

            deg = ratio * rom_deg

            # Hysteresis state machine at 2/3 threshold
            # Closing direction: rom_counts positive → pos increasing; negative → decreasing
            past_threshold = (pos - open_counts) / rom_counts > 2/3 if rom_counts != 0 else False

            if not in_feedback and past_threshold:
                in_feedback = True
                print(f"\n  ── FREE → GRIP  pos={pos}  {deg:.1f}°")
            elif in_feedback and not past_threshold:
                in_feedback = False
                print(f"\n  ── GRIP → FREE  pos={pos}  {deg:.1f}°")

            pwm = LEVER_DIR * (FEEDBACK_PWM if in_feedback else FRICTION_COMP_PWM)
            write_pwm(uart, LEVER_ID, pwm)

            display_tick += 1
            if display_tick >= display_every:
                display_tick = 0
                render(ratio, deg, in_feedback, pwm)

        elapsed   = time.ticks_diff(time.ticks_ms(), t0)
        remaining = LOOP_MS - elapsed
        if remaining > 0:
            time.sleep_ms(remaining)

# =============================================================================
# ENTRY POINT
# =============================================================================

uart = None

try:
    print("[boot] initialising UART1...")
    uart = UART(LEVER_UART_ID, baudrate=UART_BAUD, tx=LEVER_TX_PIN, rx=LEVER_RX_PIN)
    print(f"[boot] UART{LEVER_UART_ID}: TX=GPIO{LEVER_TX_PIN} RX=GPIO{LEVER_RX_PIN}")
    print()

    phase1_setup(uart)
    open_c, closed_c, rom_c, rom_deg, thresh_c = phase2_calibrate(uart)
    phase3_feedback(uart, open_c, rom_c, rom_deg, thresh_c)

except KeyboardInterrupt:
    print("\n[stop] Ctrl+C — disabling torque.")
    if uart is not None:
        _write_reg(uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
    print("[stop] Done.")

except Exception as e:
    print("[FATAL]", e)
    if uart is not None:
        try:
            _write_reg(uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
        except:
            pass
    raise

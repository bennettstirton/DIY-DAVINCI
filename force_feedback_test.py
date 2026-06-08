# =============================================================================
# Force Feedback Test — Tier 1 & Tier 2
# Target: ESP32 running MicroPython
#
# Two separate Waveshare Bus Servo Adapter boards, one per voltage rail:
#
#   LEVER BUS  — UART1, 7.4V rail
#     STS3215 C046 (1:147), ID 6 — input lever pivot, Mode 2 (direct PWM)
#       ESP32 GPIO 4 (UART1 TX) → Lever Waveshare TX
#       ESP32 GPIO 5 (UART1 RX) → Lever Waveshare RX
#
#   JAW BUS    — UART2, 12V rail
#     STS3215 C047 (1:345), ID 7 — jaw motor, Mode 0 (position, read-only here)
#       ESP32 GPIO 17 (UART2 TX) → Jaw Waveshare TX
#       ESP32 GPIO 16 (UART2 RX) → Jaw Waveshare RX
#
# Mode 2 (direct PWM) implementation notes:
#   - PWM command register: 44 (0x2C) — NOT 46 as the Feetech SDK assumes
#   - Direction bit: BIT10 (0x0400) — NOT BIT15 (0x8000) as used in Mode 1
#   - Valid PWM magnitude range: 50–1000 (0 is not a valid drive value)
#   - Torque must be DISABLED before writing mode to EEPROM
#
# First-run checklist:
#   1. Power up lever board (7.4V) only — jaw board off.
#   2. Run script. If mode ≠ 2, it writes mode 2 and asks you to power-cycle.
#   3. After power-cycle, re-run. Confirm "Lever in Mode 2".
#   4. With FRICTION_COMP_PWM = 100, lever should drift slowly toward open.
#      If it drifts toward closed, set LEVER_DIR = -1.
#   5. Tune FRICTION_COMP_PWM until lever feels neutral/free at rest.
#   6. Power up jaw board (12V). Grip something. Tune thresholds.
# =============================================================================

import time
from machine import UART

# =============================================================================
# CONFIGURATION
# =============================================================================

# --- Control tier ---
# 1 = binary end-stop (jaw free → friction comp only, jaw stalls → LEVER_TORQUE_HOLD)
# 2 = proportional    (feedback torque scales linearly with jaw current)
TIER = 2

# --- Hardware ---
LEVER_UART_ID = 1
LEVER_TX_PIN  = 4
LEVER_RX_PIN  = 5
LEVER_ID      = 6       # STS3215 C046, 7.4V

JAW_UART_ID   = 2
JAW_TX_PIN    = 17
JAW_RX_PIN    = 16
JAW_ID        = 7       # STS3215 C047, 12V

UART_BAUD     = 1_000_000

# --- Direction ---
# LEVER_DIR: +1 or -1. Positive = opening direction.
# With FRICTION_COMP_ENABLED, lever should drift toward open.
# If it drifts toward closed, flip to -1.
LEVER_DIR = 1

# --- Friction compensation ---
# True  = small opening-direction PWM when jaw is free (motor handles return)
# False = torque off when free (use a flexure for return)
FRICTION_COMP_ENABLED = True

# Opening-direction PWM to counteract gear drag. Valid range: 50–1000.
# Also pre-loads the gear train, which reduces stick-slip (stiction).
FRICTION_COMP_PWM = 100

# --- Tier 1 parameters ---
JAW_CURRENT_THRESHOLD = 300   # STS units (300 × 6.5mA ≈ 1.95A)
LEVER_TORQUE_HOLD     = 600   # PWM added on top of friction comp when jaw stalls

# --- Tier 2 parameters ---
CURRENT_THRESHOLD = 150    # STS units where feedback starts ramping
CURRENT_MAX       = 500    # STS units where feedback reaches maximum
TORQUE_MIN        = 0      # additional PWM at CURRENT_THRESHOLD
TORQUE_MAX        = 700    # additional PWM at CURRENT_MAX (on top of friction comp)

# --- Dithering ---
# Alternates PWM ±DITHER_AMPLITUDE each loop iteration to keep gear teeth
# in kinetic (sliding) contact, reducing stiction / stick-slip feel.
DITHER_ENABLED   = True
DITHER_AMPLITUDE = 15      # PWM units (keep small; try 10–25)

# --- Display ---
LEVER_ANGLE_RANGE = 45.0   # degrees shown either side of centre in lever bar
JAW_DISPLAY_MAX   = 600    # STS units at which jaw bar reads full
DISPLAY_HZ        = 10     # how often the live readout updates (≤ LOOP_HZ)

# --- Loop rate ---
LOOP_HZ = 75
LOOP_MS = 1000 // LOOP_HZ

# =============================================================================
# STS3215 REGISTER MAP
# =============================================================================

REG_MODE            = 33   # EEPROM: 0=position, 1=speed, 2=PWM, 3=step
REG_TORQUE_ENABLE   = 40   # SRAM:   1 = on, 0 = off
REG_GOAL_PWM        = 44   # SRAM:   Mode 2 PWM command (0x2C) — NOT reg 46
REG_LOCK            = 55   # SRAM:   0 = EEPROM unlocked, 1 = locked
REG_PRESENT_POS     = 56   # SRAM:   2 bytes, current output shaft position (0–4095)
REG_PRESENT_CURRENT = 69   # SRAM:   2 bytes, bit 15 = direction, 1 unit = 6.5 mA

INST_WRITE = 0x03
INST_READ  = 0x02

# =============================================================================
# ST BUS PROTOCOL
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
# HIGHER-LEVEL HELPERS
# =============================================================================

def set_mode(uart, scs_id, mode):
    """Write operating mode to EEPROM. Disables torque first. Power-cycle after."""
    _write_reg(uart, scs_id, REG_TORQUE_ENABLE, [0])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_LOCK, [0])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_MODE, [mode])
    time.sleep_ms(20)
    _write_reg(uart, scs_id, REG_LOCK, [1])
    time.sleep_ms(20)


def read_mode(uart, scs_id):
    raw = _read_reg(uart, scs_id, REG_MODE, 1)
    return raw[0] if raw is not None else None


def write_pwm(uart, scs_id, pwm):
    """
    Mode 2 PWM command. Signed, magnitude clamped to 50–1000.
    Register 44 (0x2C). Direction bit BIT10 (0x0400).
    pwm=0 is handled by the caller (disable torque instead).
    """
    mag = max(50, min(abs(pwm), 1000))
    raw = (mag | 0x0400) if pwm < 0 else mag
    _write_reg(uart, scs_id, REG_GOAL_PWM, [raw & 0xFF, (raw >> 8) & 0xFF])


def read_jaw_current(uart, scs_id):
    """Returns jaw current magnitude in STS units (1 unit = 6.5 mA), or None."""
    raw = _read_reg(uart, scs_id, REG_PRESENT_CURRENT, 2)
    if raw is None:
        return None
    return (raw[0] | (raw[1] << 8)) & 0x7FFF


def read_lever_position(uart, scs_id):
    """Returns lever encoder position (0–4095, centre = 2048), or None."""
    raw = _read_reg(uart, scs_id, REG_PRESENT_POS, 2)
    if raw is None:
        return None
    val = raw[0] | (raw[1] << 8)
    return -(val & 0x7FFF) if (val & 0x8000) else val

# =============================================================================
# FEEDBACK COMPUTATION
# =============================================================================

def compute_feedback(jaw_current):
    """
    Returns (net_pwm, state, feedback_mag).
    net_pwm:      what to command to lever servo (before dither offset)
    state:        "free" | "ramp" | "HOLD" | "MAX"
    feedback_mag: the feedback component only (for display)
    """
    if TIER == 1:
        if jaw_current < JAW_CURRENT_THRESHOLD:
            feedback, state = 0, "free"
        else:
            feedback, state = LEVER_TORQUE_HOLD, "HOLD"
    else:
        if jaw_current <= CURRENT_THRESHOLD:
            feedback, state = 0, "free"
        elif jaw_current >= CURRENT_MAX:
            feedback, state = TORQUE_MAX, "MAX"
        else:
            ratio = (jaw_current - CURRENT_THRESHOLD) / (CURRENT_MAX - CURRENT_THRESHOLD)
            feedback = int(TORQUE_MIN + ratio * (TORQUE_MAX - TORQUE_MIN))
            state = "ramp"

    base = FRICTION_COMP_PWM if FRICTION_COMP_ENABLED else 0
    total = base + feedback
    return (LEVER_DIR * total if total > 0 else 0), state, feedback

# =============================================================================
# DISPLAY
# =============================================================================

def _bar(ratio, width=16):
    n = max(0, min(width, round(ratio * width)))
    return '█' * n + '░' * (width - n)


def _lever_bar(deg, width=21):
    """Centre-relative bar. Centre marker at mid; position shown as block."""
    half = width // 2
    pos  = max(0, min(width - 1, half + round(deg / LEVER_ANGLE_RANGE * half)))
    chars = list('░' * width)
    chars[half] = '┼'
    chars[pos]  = '█'
    return ''.join(chars)


def render(lever_counts, jaw_current, state, net_pwm, feedback, dither_sym):
    deg = (lever_counts - 2048) * (360.0 / 4096)
    jaw_ratio = min(1.0, jaw_current / JAW_DISPLAY_MAX)
    line = (
        f"\r  lvr [{_lever_bar(deg)}] {deg:+6.1f}°"
        f"  jaw [{_bar(jaw_ratio)}] {jaw_current:3d}u {jaw_current * 6.5 / 1000:.2f}A"
        f"  {state:<4s} pwm={net_pwm:+4d} fb={feedback:3d} {dither_sym} "
    )
    print(line, end='')

# =============================================================================
# SETUP
# =============================================================================

def setup(lever_uart, jaw_uart):
    mode = read_mode(lever_uart, LEVER_ID)
    print(f"[setup] lever servo ID {LEVER_ID} current mode: {mode}")

    if mode is None:
        print("[setup] WARNING: could not read lever servo — check wiring and power.")

    if mode != 2:
        print("[setup] Writing Mode 2 to EEPROM (torque off first)...")
        set_mode(lever_uart, LEVER_ID, 2)
        print("[setup] Mode written.")
        print()
        print("  *** POWER-CYCLE the lever servo board (7.4V rail), then re-run. ***")
        print()
        raise SystemExit(0)

    print("[setup] Lever in Mode 2. Good.")

    if FRICTION_COMP_ENABLED:
        _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [1])
        write_pwm(lever_uart, LEVER_ID, LEVER_DIR * FRICTION_COMP_PWM)
        print(f"[setup] Friction comp ON: PWM = {LEVER_DIR * FRICTION_COMP_PWM:+d}")
        print( "        Lever should drift slowly toward open.")
    else:
        _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
        print("[setup] Friction comp OFF — lever is gear friction only.")
    print()

    pos = read_lever_position(lever_uart, LEVER_ID)
    if pos is not None:
        deg = (pos - 2048) * (360.0 / 4096)
        print(f"[setup] Lever position: {pos} counts ({deg:+.1f}° from centre)")

    jaw_current = read_jaw_current(jaw_uart, JAW_ID)
    if jaw_current is None:
        print(f"[setup] WARNING: jaw servo ID {JAW_ID} not responding — check 12V bus.")
    else:
        print(f"[setup] Jaw reads OK: {jaw_current} units ({jaw_current * 6.5:.0f} mA) at rest.")
    print()

# =============================================================================
# FEEDBACK LOOP
# =============================================================================

def run_feedback_loop(lever_uart, jaw_uart):
    tier_str  = f"Tier {TIER} ({'binary' if TIER == 1 else 'proportional'})"
    comp_str  = f"comp={'ON +' + str(FRICTION_COMP_PWM) if FRICTION_COMP_ENABLED else 'OFF'}"
    dith_str  = f"dither={'ON ±' + str(DITHER_AMPLITUDE) if DITHER_ENABLED else 'OFF'}"
    print(f"[loop] {tier_str}  {comp_str}  {dith_str}  {LOOP_HZ} Hz")
    if TIER == 2:
        print(f"       jaw {CURRENT_THRESHOLD}u→{CURRENT_MAX}u  "
              f"feedback {TORQUE_MIN}→{TORQUE_MAX} PWM")
    print()

    last_state    = None
    read_errors   = 0
    dither_phase  = 0
    display_tick  = 0
    torque_on     = FRICTION_COMP_ENABLED
    display_every = max(1, LOOP_HZ // DISPLAY_HZ)

    while True:
        t0 = time.ticks_ms()

        jaw_current  = read_jaw_current(jaw_uart, JAW_ID)
        lever_counts = read_lever_position(lever_uart, LEVER_ID)

        if jaw_current is None:
            read_errors += 1
            if read_errors % LOOP_HZ == 0:
                print(f"\n[warn] {read_errors} jaw read timeouts — check 12V bus")
        else:
            read_errors = 0
            net_pwm, state, feedback = compute_feedback(jaw_current)

            # Dither: flip sign each iteration, apply only when motor is on
            dither_phase ^= 1
            dither_offset = (DITHER_AMPLITUDE if dither_phase else -DITHER_AMPLITUDE) \
                            if (DITHER_ENABLED and net_pwm != 0) else 0

            # Drive lever
            if net_pwm == 0:
                if torque_on:
                    _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
                    torque_on = False
            else:
                if not torque_on:
                    _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [1])
                    torque_on = True
                write_pwm(lever_uart, LEVER_ID, net_pwm + dither_offset)

            # Print a newline on state transitions so they're preserved in scroll
            if state != last_state:
                deg = (lever_counts - 2048) * (360.0 / 4096) if lever_counts else 0
                print(f"\n  ── {last_state} → {state}  jaw={jaw_current}u  angle={deg:+.1f}°")
                last_state = state

            # Live display at DISPLAY_HZ
            display_tick += 1
            if display_tick >= display_every:
                display_tick = 0
                if lever_counts is not None:
                    dith_sym = '~' if (DITHER_ENABLED and dither_phase) else ' '
                    render(lever_counts, jaw_current, state, net_pwm, feedback, dith_sym)

        elapsed   = time.ticks_diff(time.ticks_ms(), t0)
        remaining = LOOP_MS - elapsed
        if remaining > 0:
            time.sleep_ms(remaining)

# =============================================================================
# ENTRY POINT
# =============================================================================

lever_uart = None
jaw_uart   = None

try:
    print("[boot] initialising UARTs...")
    lever_uart = UART(LEVER_UART_ID, baudrate=UART_BAUD, tx=LEVER_TX_PIN, rx=LEVER_RX_PIN)
    jaw_uart   = UART(JAW_UART_ID,   baudrate=UART_BAUD, tx=JAW_TX_PIN,   rx=JAW_RX_PIN)
    print(f"[boot] UART{LEVER_UART_ID}: TX=GPIO{LEVER_TX_PIN} RX=GPIO{LEVER_RX_PIN} (lever)")
    print(f"[boot] UART{JAW_UART_ID}:  TX=GPIO{JAW_TX_PIN}  RX=GPIO{JAW_RX_PIN}  (jaw)")

    setup(lever_uart, jaw_uart)
    run_feedback_loop(lever_uart, jaw_uart)

except KeyboardInterrupt:
    print("\n[stop] Ctrl+C — disabling lever torque.")
    if lever_uart is not None:
        _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
    print("[stop] Done.")

except Exception as e:
    print("[FATAL]", e)
    if lever_uart is not None:
        try:
            _write_reg(lever_uart, LEVER_ID, REG_TORQUE_ENABLE, [0])
        except:
            pass
    raise

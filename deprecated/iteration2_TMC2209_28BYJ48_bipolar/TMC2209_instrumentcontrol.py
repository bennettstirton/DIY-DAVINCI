# =============================================================================
# Fine Instrument Controller
# Target: ESP32 running MicroPython
#
# Hardware
# --------
# Motors (28BYJ-48 bipolar steppers, red wire cut, via TMC2209 driver boards):
#   VIO           → 3.3V (logic power)
#   VM            → 12V  (motor power)
#   EN            → GND  (always enabled; motor holds position when idle)
#   MS1/MS2       → set UART address (see below); microstep controlled via UART
#
#   UART bus: all four PDN_UART pins → GPIO 17 (star topology, no series resistor)
#
#   Driver addresses (set by MS1/MS2, latched at power-on):
#     Motor D – MS1=GND,  MS2=GND  → address 0
#     Motor E – MS1=3.3V, MS2=GND  → address 1
#     Motor F – MS1=GND,  MS2=3.3V → address 2
#     Motor G – MS1=3.3V, MS2=3.3V → address 3
#
#   UART config (written at boot): 1/2 step + interpolation → 4096 steps/rev
#
#   Motor D – Pitch      | STEP: GPIO 26  DIR: GPIO 25
#   Motor E – Roll       | STEP: GPIO 33  DIR: GPIO 32
#   Motor F – Yaw/Grip 1 | STEP: GPIO 13  DIR: GPIO 12
#   Motor G – Yaw/Grip 2 | STEP: GPIO 14  DIR: GPIO 27
#
# Joystick (3-axis analog, 10kΩ potentiometers):
#   Suggested wiring: VCC→3.3V, GND→GND
#   Forward/Back   (Pitch) → ADC GPIO 34
#   Left/Right     (Yaw)   → ADC GPIO 35
#   Axial Rotation (Roll)  → ADC GPIO 36
#   (All three use ADC1, which is more stable on ESP32 than ADC2)
#
# Motion Limits
# -------------
#   Roll:  ±175°  (from center)
#   Pitch:  ±90°  (from center)
#   Yaw:    ±80°  (soft limit; physical max ±95°, reduced to preserve grip range)
#   Grip:    0–25° (0 = fully closed; not joystick-controlled yet)
#
# Assumptions
# -----------
#   - The instrument is manually homed to its neutral/centered position before
#     power-on. All step counts begin at 0 (= neutral).
#   - Grip is held at 0° (fully closed) until a second input device is added.
#
# Axis Coupling / Inverse Kinematics
# ------------------------------------
# The instrument uses a cable-driven EndoWrist architecture
# (ref: https://patents.google.com/patent/US8540748B2/en).
# Cables are shared across axes, requiring coordinated motor motion:
#
#   Roll  (Motor E only — fully independent):
#     Motor E = roll_angle
#
#   Pitch (Motors D, F, G — cables F and G are routed through the pitch joint):
#     Motor D = pitch_angle
#     Motor F += pitch_angle × 0.5
#     Motor G += pitch_angle × 0.5
#
#   Yaw   (Motors F, G — both cables move same direction):
#     Motor F += yaw_angle
#     Motor G += yaw_angle
#
#   Grip  (Motors F, G — differential; grip_angle must remain ≥ 0):
#     Motor F += grip_angle × 0.5
#     Motor G -= grip_angle × 0.5
#     (0° = jaws closed, 25° = jaws fully open)
#
# Combined motor targets for a (pitch, yaw, roll, grip) command:
#   Motor D = pitch
#   Motor E = roll
#   Motor F = 0.5·pitch + yaw + 0.5·grip
#   Motor G = 0.5·pitch + yaw − 0.5·grip
# =============================================================================

print("[boot] imports starting...")
import sys
import select
import time
import uasyncio as asyncio
from machine import ADC, Pin, UART
print("[boot] imports OK")

# =============================================================================
# CONFIGURATION
# =============================================================================

STEPS_PER_REV = 4096                    # 28BYJ-48 bipolar 1/2 step: 32 full steps/rev × 2 microsteps × 64:1 gear
STEPS_PER_DEG = STEPS_PER_REV / 360.0  # ≈ 11.38 steps/degree

# Joystick ADC pins
ADC_PITCH_PIN = 34
ADC_YAW_PIN   = 35
ADC_ROLL_PIN  = 36

ADC_MAX    = 4095
ADC_CENTER = ADC_MAX / 2.0

# Deadband: normalized joystick noise threshold near center (0.0–1.0)
DEADBAND = 0.1

# Axis scale factors: multiply joystick deflection fraction by limit and scale
SCALE_PITCH = 1.0
SCALE_YAW   = 1.0
SCALE_ROLL  = 0.0  # disabled — roll pot disconnected, change back to 1.0 when replaced

# Axis soft limits (degrees from center)
LIMIT_PITCH    = 90.0
LIMIT_YAW      = 80.0    # Physical max is ±95°; soft-limited so grip always has room
LIMIT_ROLL     = 175.0
LIMIT_GRIP     = 0.0     # Placeholder — set to desired open angle when grip input is added
LIMIT_GRIP_MAX = 25.0    # Physical maximum grip opening; used for worst-case step limit calc

# Step rate: delay after each asyncio yield (milliseconds).
# 0 = yield immediately and re-enter the motor loop — maximises step throughput.
# Raise to 1+ if the joystick task feels starved.
STEP_DELAY_MS = 0

# Bresenham iterations per asyncio yield. Each iteration steps all motors once
# (those whose error accumulates enough), then waits STEP_PULSE_DELAY_US.
# Max slew rate ≈ STEPS_PER_TICK / (STEPS_PER_TICK × STEP_PULSE_DELAY_US×1e-6) / STEPS_PER_DEG
# e.g. 16 / (16 × 500µs) / 11.38 ≈ 175°/sec
# Lower if the joystick task feels starved.
STEPS_PER_TICK = 16

# Delay inserted once per Bresenham iteration (not per motor).
# All motors that need to step do so back-to-back, then we wait once.
# 500µs here + ~200-300µs Python overhead ≈ 750µs effective — matches tested motor limit.
STEP_PULSE_DELAY_US = 500

# EMA smoothing factor for joystick readings (0.0–1.0)
# Lower = smoother but laggier; higher = more responsive but noisier.
# 0.2 gives ~90ms time constant at 50 Hz — good starting point.
EMA_ALPHA = 0.2

# Motor GPIO pins [STEP, DIR]
MOTOR_D_PINS = (26, 25)   # Pitch
MOTOR_E_PINS = (33, 32)   # Roll
MOTOR_F_PINS = (13, 12)   # Yaw/Grip 1
MOTOR_G_PINS = (14, 27)   # Yaw/Grip 2

# Step count limits — worst-case combined angle for each motor
_sdeg = lambda d: round(d * STEPS_PER_DEG)
STEP_LIMIT_D  = _sdeg(LIMIT_PITCH)
STEP_LIMIT_E  = _sdeg(LIMIT_ROLL)
STEP_LIMIT_FG = _sdeg(0.5 * LIMIT_PITCH + LIMIT_YAW + 0.5 * LIMIT_GRIP_MAX)

# UART bus pin and TMC2209 register config
UART_TX_PIN    = 17
REG_GCONF      = 0x00
REG_IHOLD_IRUN = 0x10
REG_CHOPCONF   = 0x6C
GCONF_VALUE    = 0x000000C0   # pdn_disable=1, mstep_reg_select=1
CHOPCONF_VALUE = 0x17000053   # MRES=7 (1/2 step), intpol=1, toff=3, hstrt=5, tbl=2

# Current scaling (0–31 each).
# IRUN:  run current.  10/31 ≈ 32% of VREF max — safe starting point for 28BYJ-48 at 12V.
#          → still too hot?  try 7–8
#          → losing steps?   try 15–16
# IHOLD: hold current. 5/31  ≈ 16% — enough to hold position, much lower heat when idle.
#          → losing position at rest?  try 8–10
# IHOLDDELAY: ramp-down delay after last step (units of 2^18 clock ticks ≈ 0.5s at 6).
#          → reduce to 2–3 if motors feel warm even at idle
IRUN       = 10
IHOLD      = 5
IHOLDDELAY = 6
IHOLD_IRUN_VALUE = (IHOLDDELAY << 16) | (IRUN << 8) | IHOLD


# =============================================================================
# TMC2209 UART
# =============================================================================

def _crc8(data):
    """CRC-8 as specified in the TMC2209 datasheet."""
    crc = 0
    for byte in data:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 1):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            byte >>= 1
    return crc



def _write_reg(uart, driver_addr, reg, value):
    """Send a single 8-byte write datagram to the specified driver address."""
    payload = [
        0x05,
        driver_addr & 0xFF,
        (reg | 0x80) & 0xFF,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >>  8) & 0xFF,
        (value      ) & 0xFF,
    ]
    payload.append(_crc8(payload))
    uart.write(bytes(payload))
    time.sleep_ms(2)


def configure_drivers():
    """
    Open UART2 and write GCONF + CHOPCONF to all four TMC2209 drivers.
    Must be called after power-on, before any motor motion.
    """
    uart = UART(2, baudrate=115200, tx=UART_TX_PIN, rx=16)
    for addr, name in ((0, "D"), (1, "E"), (2, "F"), (3, "G")):
        print(f"[uart] configuring driver {addr} (Motor {name})...")
        _write_reg(uart, addr, REG_GCONF,      GCONF_VALUE)
        _write_reg(uart, addr, REG_CHOPCONF,   CHOPCONF_VALUE)
        _write_reg(uart, addr, REG_IHOLD_IRUN, IHOLD_IRUN_VALUE)
    print("[uart] all drivers configured")


# =============================================================================
# STEPPER MOTOR
# =============================================================================

class Stepper:
    """
    Drives one 28BYJ-48 (bipolar, red wire cut) via a TMC2209 STEP/DIR driver.
    Set target_steps (or call set_target_degrees), then call
    step_toward_target() repeatedly to move there.

    EN is tied to GND on the TMC2209, so the motor always holds its position.
    _deenergize() is a no-op — holding is desirable for a surgical instrument.
    """

    def __init__(self, pins, step_limit):
        self.step_pin = Pin(pins[0], Pin.OUT)
        self.dir_pin  = Pin(pins[1], Pin.OUT)
        self.current_steps = 0
        self.target_steps  = 0
        self.step_limit    = step_limit

    def _deenergize(self):
        pass  # EN tied to GND — motor holds position passively

    def set_target_degrees(self, degrees):
        """Convert a degree target to steps, clamped to hardware limits."""
        steps = round(degrees * STEPS_PER_DEG)
        self.target_steps = max(-self.step_limit, min(self.step_limit, steps))

    def step_toward_target(self):
        """Advance one step toward target via STEP/DIR pulse."""
        delta = self.target_steps - self.current_steps
        if delta == 0:
            return
        self.dir_pin.value(1 if delta > 0 else 0)
        self.current_steps += 1 if delta > 0 else -1
        self.step_pin.value(1)
        time.sleep_us(10)   # TMC2209 minimum STEP pulse width
        self.step_pin.value(0)


# =============================================================================
# ALIGNMENT
# =============================================================================

# Coarse jog step size for alignment (≈ 5.6° output rotation at 4096 steps/rev)
ALIGN_COARSE_STEPS = 16

def _jog(motor, steps):
    """Physically move a motor N steps without updating position tracking."""
    motor.dir_pin.value(1 if steps > 0 else 0)
    time.sleep_us(1)  # direction setup time before first pulse
    for _ in range(abs(steps)):
        motor.step_pin.value(1)
        time.sleep_us(10)
        motor.step_pin.value(0)
        time.sleep_us(STEP_PULSE_DELAY_US)


def run_alignment(motor_d, motor_e, motor_f, motor_g):
    """
    Optional interactive serial-terminal alignment routine. Runs synchronously
    before the asyncio joystick loop starts.

    On startup, waits 5 seconds for the user to type 'h' + Enter to begin.
    Any other input or timeout skips alignment and proceeds to joystick control.

    Commands (type and press Enter):
      +       Step forward  16 steps (~5.6°)
      -       Step backward 16 steps (~5.6°)
      +N      Step forward  N steps  (e.g. +32)
      -N      Step backward N steps  (e.g. -10)
      h       Confirm this motor is home (zeroes its position)
    """
    print()
    print("=" * 54)
    print("  Type 'h' + Enter within 5 seconds to begin")
    print("  motor alignment. Press Enter or wait to skip.")
    print("=" * 54)

    _poll = select.poll()
    _poll.register(sys.stdin, select.POLLIN)

    if _poll.poll(5000):
        line = sys.stdin.readline().strip()
        if line != 'h':
            print("Skipping alignment. Starting joystick control...")
            print()
            return
    else:
        print("Timed out. Skipping alignment. Starting joystick control...")
        print()
        return

    motors = [
        (motor_d, "D (Pitch)"),
        (motor_e, "E (Roll)"),
        (motor_f, "F (Yaw/Grip 1)"),
        (motor_g, "G (Yaw/Grip 2)"),
    ]

    print()
    print("=" * 54)
    print("  ALIGNMENT MODE")
    print("=" * 54)
    print("  Before starting: manually position the instrument")
    print("  to its neutral/centered position.")
    print()
    print("  For each motor, jog its coupling disc until it")
    print("  seats into the instrument shaft, then type 'h'.")
    print()
    print("  +       step forward  16 steps (~5.6 deg)")
    print("  -       step backward 16 steps (~5.6 deg)")
    print("  +N/-N   step forward/backward N steps")
    print("  h       confirm home for this motor")
    print("=" * 54)
    print()

    for motor, name in motors:
        net = 0
        print(f"Motor {name}  |  net: {net:+d} steps ({net / STEPS_PER_DEG:+.1f} deg)")

        while True:
            line = sys.stdin.readline().strip()

            if line == 'h':
                motor.current_steps = 0
                motor.target_steps  = 0
                motor._deenergize()
                print(f"  Zeroed. (moved {net:+d} steps = {net / STEPS_PER_DEG:+.1f} deg from start)")
                print()
                break
            elif line.startswith('+') or line.startswith('-'):
                try:
                    n = int(line[1:]) if len(line) > 1 else ALIGN_COARSE_STEPS
                    n = n if line[0] == '+' else -n
                    _jog(motor, n)
                    net += n
                    print(f"  net: {net:+d} steps ({net / STEPS_PER_DEG:+.1f} deg)")
                except ValueError:
                    print(f"  Unknown command '{line}'. Use +, -, +N, -N, or h.")
            elif line == '':
                pass  # ignore blank lines
            else:
                print(f"  Unknown command '{line}'. Use +, -, +N, -N, or h.")

    print("All motors aligned. Starting joystick control...")
    print("=" * 54)
    print()


# =============================================================================
# ASYNCIO TASKS
# =============================================================================

async def coordinated_motor_task(motor_d, motor_e, motor_f, motor_g):
    """
    Drives all four motors in kinematic sync using Bresenham error accumulation.
    On each tick the motor with the largest remaining distance steps once;
    all others step proportionally less often so all motors arrive at their
    targets simultaneously — preserving coupled-axis ratios throughout the move.
    """
    motors = [motor_d, motor_e, motor_f, motor_g]
    errors = [0] * 4

    while True:
        for _ in range(STEPS_PER_TICK):
            deltas = [abs(m.target_steps - m.current_steps) for m in motors]
            max_d = max(deltas)

            if max_d == 0:
                errors = [0] * 4
                for m in motors:
                    m._deenergize()
                break  # no work to do; skip remaining sub-ticks
            else:
                for i, motor in enumerate(motors):
                    if deltas[i] == 0:
                        motor._deenergize()
                        errors[i] = 0
                        continue
                    errors[i] += deltas[i]
                    if errors[i] >= max_d:
                        errors[i] -= max_d
                        motor.step_toward_target()
                time.sleep_us(STEP_PULSE_DELAY_US)  # one wait per iteration, not per motor

        await asyncio.sleep_ms(STEP_DELAY_MS)


async def joystick_task(motor_d, motor_e, motor_f, motor_g):
    """
    Read all three joystick axes at ~50 Hz, compute inverse kinematics,
    and update motor target positions.
    """
    pitch_adc = ADC(Pin(ADC_PITCH_PIN))
    yaw_adc   = ADC(Pin(ADC_YAW_PIN))
    roll_adc  = ADC(Pin(ADC_ROLL_PIN))

    for adc in (pitch_adc, yaw_adc, roll_adc):
        adc.atten(ADC.ATTN_11DB)    # Full 0–3.3V input range
        adc.width(ADC.WIDTH_12BIT)  # 12-bit resolution (0–4095)

    def read_normalized(adc, home_raw):
        """Read ADC, home-relative zero, apply deadband. Returns −1.0 to +1.0."""
        raw = adc.read()
        n = (raw - home_raw) / ADC_CENTER
        if abs(n) < DEADBAND:
            return 0.0
        return max(-1.0, min(1.0, n))

    # Warm up ADC (first reads after power-on are unreliable on ESP32),
    # then average several reads to capture a stable home position.
    # Motors treat wherever the joystick sits at startup as zero deflection.
    for _ in range(10):
        pitch_adc.read()
        yaw_adc.read()
        roll_adc.read()
    N_HOME = 16
    home_pitch = sum(pitch_adc.read() for _ in range(N_HOME)) // N_HOME
    home_yaw   = sum(yaw_adc.read()   for _ in range(N_HOME)) // N_HOME
    home_roll  = sum(roll_adc.read()  for _ in range(N_HOME)) // N_HOME
    print(f"[joystick] home captured — pitch: {home_pitch}  yaw: {home_yaw}  roll: {home_roll}")

    # Deflection at home is zero by definition, so seed EMA at 0.
    ema_pitch = 0.0
    ema_yaw   = 0.0
    ema_roll  = 0.0
    print_counter = 0

    while True:
        ema_pitch = EMA_ALPHA * read_normalized(pitch_adc, home_pitch) + (1 - EMA_ALPHA) * ema_pitch
        ema_yaw   = EMA_ALPHA * read_normalized(yaw_adc,   home_yaw)   + (1 - EMA_ALPHA) * ema_yaw
        ema_roll  = EMA_ALPHA * read_normalized(roll_adc,  home_roll)  + (1 - EMA_ALPHA) * ema_roll

        # Apply deadband to EMA output — clamps residual noise that leaked
        # through the per-read deadband to exactly zero before hitting the IK.
        pitch_n = ema_pitch if abs(ema_pitch) >= DEADBAND else 0.0
        yaw_n   = ema_yaw   if abs(ema_yaw)   >= DEADBAND else 0.0
        roll_n  = ema_roll  if abs(ema_roll)  >= DEADBAND else 0.0

        # Map normalized joystick position to axis angle targets
        pitch_deg = pitch_n * LIMIT_PITCH * SCALE_PITCH
        yaw_deg   = yaw_n   * LIMIT_YAW   * SCALE_YAW
        roll_deg  = roll_n  * LIMIT_ROLL  * SCALE_ROLL
        grip_deg  = LIMIT_GRIP  # constant until a grip input device is added

        # Inverse kinematics — see header for full derivation
        motor_d.set_target_degrees(pitch_deg)
        motor_e.set_target_degrees(roll_deg)
        motor_f.set_target_degrees(0.5 * pitch_deg + yaw_deg + 0.5 * grip_deg)
        motor_g.set_target_degrees(0.5 * pitch_deg + yaw_deg - 0.5 * grip_deg)

        print_counter += 1
        if print_counter >= 50:
            print_counter = 0
            print(f"pitch: {pitch_deg:+.1f} deg | yaw: {yaw_deg:+.1f} deg | roll: {roll_deg:+.1f} deg")

        await asyncio.sleep_ms(20)  # 50 Hz update rate


# =============================================================================
# ENTRY POINT
# =============================================================================

async def _control_loop(motor_d, motor_e, motor_f, motor_g):
    asyncio.create_task(coordinated_motor_task(motor_d, motor_e, motor_f, motor_g))
    asyncio.create_task(joystick_task(motor_d, motor_e, motor_f, motor_g))

    while True:
        await asyncio.sleep_ms(1000)


# Configure drivers, create motors, align, then hand off to the async joystick loop
try:
    print("[boot] configuring TMC2209 drivers via UART...")
    configure_drivers()

    print("[boot] creating motors...")
    motor_d = Stepper(MOTOR_D_PINS, STEP_LIMIT_D)
    print("[boot] motor D OK")
    motor_e = Stepper(MOTOR_E_PINS, STEP_LIMIT_E)
    print("[boot] motor E OK")
    motor_f = Stepper(MOTOR_F_PINS, STEP_LIMIT_FG)
    print("[boot] motor F OK")
    motor_g = Stepper(MOTOR_G_PINS, STEP_LIMIT_FG)
    print("[boot] motor G OK")

    print("[boot] entering alignment...")
    run_alignment(motor_d, motor_e, motor_f, motor_g)

    print("[boot] starting joystick control loop...")
    asyncio.run(_control_loop(motor_d, motor_e, motor_f, motor_g))
    print("[boot] control loop exited (unexpected)")

except Exception as e:
    print("[boot] FATAL ERROR:", e)
    raise

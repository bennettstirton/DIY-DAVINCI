# =============================================================================
# Fine Instrument Controller
# Target: ESP32 running MicroPython
#
# Hardware
# --------
# Motors (28BYJ-48 unipolar steppers via ULN2003 driver boards):
#   Motor D – Pitch       | IN1–IN4: GPIO 26, 25, 33, 32
#   Motor E – Roll        | IN1–IN4: GPIO 13, 12, 14, 27
#   Motor F – Yaw/Grip 1  | IN1–IN4: GPIO  2,  4, 16, 17
#   Motor G – Yaw/Grip 2  | IN1–IN4: GPIO  5, 18, 19, 21
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
from machine import ADC, Pin
print("[boot] imports OK")

# =============================================================================
# CONFIGURATION
# =============================================================================

# 28BYJ-48 wave-drive coil sequence [IN1, IN2, IN3, IN4]
# Wave drive energizes one coil at a time: lower current draw, higher speed ceiling,
# half the resolution of half-step mode (2048 vs 4096 steps/rev, ~0.18 deg/step).
COIL_SEQ = [
    (1, 0, 0, 0),
    (0, 1, 0, 0),
    (0, 0, 1, 0),
    (0, 0, 0, 1),
]
SEQ_LEN = len(COIL_SEQ)

STEPS_PER_REV = 2048                    # 28BYJ-48: 32 gear steps × 64:1 ratio, wave-drive mode
STEPS_PER_DEG = STEPS_PER_REV / 360.0  # ≈ 5.69 steps/degree

# Joystick ADC pins
ADC_PITCH_PIN = 34
ADC_YAW_PIN   = 35
ADC_ROLL_PIN  = 36

ADC_MAX    = 4095
ADC_CENTER = ADC_MAX / 2.0

# Deadband: normalized joystick noise threshold near center (0.0–1.0)
DEADBAND = 0.05

# Axis scale factors: multiply joystick deflection fraction by limit and scale
SCALE_PITCH = 1.0
SCALE_YAW   = 1.0
SCALE_ROLL  = 0.0  # disabled — roll pot disconnected, change back to 1.0 when replaced

# Axis soft limits (degrees from center)
LIMIT_PITCH = 90.0
LIMIT_YAW   = 80.0    # Physical max is ±95°; soft-limited so grip always has room
LIMIT_ROLL  = 175.0
LIMIT_GRIP  = 0.0     # Placeholder — set to desired open angle when grip input is added

# Step rate: delay between each half-step per motor (milliseconds)
# 1ms → ~1000 steps/sec → ~88°/sec maximum slew rate
STEP_DELAY_MS = 1 # was 1

# EMA smoothing factor for joystick readings (0.0–1.0)
# Lower = smoother but laggier; higher = more responsive but noisier.
# 0.2 gives ~90ms time constant at 50 Hz — good starting point.
EMA_ALPHA = 0.2

# Motor GPIO pins [IN1, IN2, IN3, IN4]
MOTOR_D_PINS = [26, 25, 33, 32]   # Pitch
MOTOR_E_PINS = [13, 12, 14, 27]   # Roll
MOTOR_F_PINS = [ 2,  4, 16, 17]   # Yaw/Grip 1
MOTOR_G_PINS = [ 5, 18, 19, 21]   # Yaw/Grip 2

# Step count limits — worst-case combined angle for each motor
_sdeg = lambda d: round(d * STEPS_PER_DEG)
STEP_LIMIT_D = _sdeg(LIMIT_PITCH)
STEP_LIMIT_E = _sdeg(LIMIT_ROLL)
STEP_LIMIT_FG = _sdeg(0.5 * LIMIT_PITCH + LIMIT_YAW + 0.5 * LIMIT_GRIP)


# =============================================================================
# STEPPER MOTOR
# =============================================================================

class Stepper:
    """
    Drives one 28BYJ-48 in half-step mode.
    Set target_steps (or call set_target_degrees), then call
    step_toward_target() repeatedly to move there.
    """

    def __init__(self, pins, step_limit):
        self.pins = [Pin(p, Pin.OUT) for p in pins]
        self._seq_idx    = 0
        self.current_steps = 0
        self.target_steps  = 0
        self.step_limit    = step_limit

    def _apply_coils(self):
        seq = COIL_SEQ[self._seq_idx % SEQ_LEN]
        for pin, val in zip(self.pins, seq):
            pin.value(val)

    def _deenergize(self):
        """Release all coils to reduce heat when motor is at rest."""
        for pin in self.pins:
            pin.value(0)

    def set_target_degrees(self, degrees):
        """Convert a degree target to steps, clamped to hardware limits."""
        steps = round(degrees * STEPS_PER_DEG)
        self.target_steps = max(-self.step_limit, min(self.step_limit, steps))

    def step_toward_target(self):
        """Advance one half-step toward target. De-energizes coils when arrived."""
        delta = self.target_steps - self.current_steps
        if delta == 0:
            self._deenergize()
            return
        if delta > 0:
            self._seq_idx = (self._seq_idx + 1) % SEQ_LEN
            self.current_steps += 1
        else:
            self._seq_idx = (self._seq_idx - 1) % SEQ_LEN
            self.current_steps -= 1
        self._apply_coils()


# =============================================================================
# ALIGNMENT
# =============================================================================

# Coarse jog step size for alignment (64 steps ≈ 5.6° output rotation)
ALIGN_COARSE_STEPS = 64

def _jog(motor, steps):
    """Physically move a motor N steps without updating position tracking."""
    direction = 1 if steps > 0 else -1
    for _ in range(abs(steps)):
        motor._seq_idx = (motor._seq_idx + direction) % SEQ_LEN
        motor._apply_coils()
        time.sleep_us(STEP_DELAY_MS * 1000)


def run_alignment(motor_d, motor_e, motor_f, motor_g):
    """
    Optional interactive serial-terminal alignment routine. Runs synchronously
    before the asyncio joystick loop starts.

    On startup, waits 5 seconds for the user to type 'h' + Enter to begin.
    Any other input or timeout skips alignment and proceeds to joystick control.

    Commands (type and press Enter):
      +       Step forward  64 steps (~11.3°)
      -       Step backward 64 steps (~11.3°)
      +N      Step forward  N steps  (e.g. +128)
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
    print("  +       step forward  64 steps (~11.3 deg)")
    print("  -       step backward 64 steps (~11.3 deg)")
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
        deltas = [abs(m.target_steps - m.current_steps) for m in motors]
        max_d = max(deltas)

        if max_d == 0:
            errors = [0] * 4
            for m in motors:
                m._deenergize()
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

    def read_normalized(adc):
        """Read ADC, center-zero, apply deadband. Returns −1.0 to +1.0."""
        raw = adc.read()
        n = (raw - ADC_CENTER) / ADC_CENTER
        if abs(n) < DEADBAND:
            return 0.0
        return max(-1.0, min(1.0, n))

    # Warm up ADC (first reads after power-on are unreliable on ESP32)
    # then seed EMA from the actual joystick position so there's no
    # startup transient driving the motors on first loop iteration.
    for _ in range(10):
        pitch_adc.read()
        yaw_adc.read()
        roll_adc.read()
    ema_pitch = read_normalized(pitch_adc)
    ema_yaw   = read_normalized(yaw_adc)
    ema_roll  = read_normalized(roll_adc)
    print_counter = 0

    while True:
        ema_pitch = EMA_ALPHA * read_normalized(pitch_adc) + (1 - EMA_ALPHA) * ema_pitch
        ema_yaw   = EMA_ALPHA * read_normalized(yaw_adc)   + (1 - EMA_ALPHA) * ema_yaw
        ema_roll  = EMA_ALPHA * read_normalized(roll_adc)  + (1 - EMA_ALPHA) * ema_roll

        pitch_n = ema_pitch
        yaw_n   = ema_yaw
        roll_n  = ema_roll

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


# Create motors, align, then hand off to the async joystick loop
try:
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

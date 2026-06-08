# =============================================================================
# Fine Instrument Controller
# Target: ESP32 running MicroPython
#
# Hardware:
#   - 3× 28BYJ-48 unipolar stepper motors via ULN2003 driver boards
#   - Motors control pitch, roll, and yaw of the surgical instrument (EndoWrist)
#   - 4th motor (grip) is stubbed in config but not wired for MVP
#
# Communication:
#   - Commands received from Raspberry Pi over USB serial (UART0 / the REPL port)
#   - Reading is non-blocking via select.poll(), so the REPL still works while
#     the program is running (useful for development / emergency stops)
#   - Protocol: one JSON object per line (\n terminated)
#
#     Pi → ESP32 (position command):  {"p": <steps>, "r": <steps>, "y": <steps>}
#     Pi → ESP32 (home all axes):     {"cmd": "home"}
#     Pi → ESP32 (stop all axes):     {"cmd": "stop"}
#     ESP32 → Pi (status reply):      {"p": <pos>, "r": <pos>, "y": <pos>}
#
# Notes on 28BYJ-48:
#   - Full-step mode (default): 2048 steps per output-shaft revolution
#   - Coils are de-energized when the motor reaches its target to prevent
#     overheating. Avoid leaving them energised at rest for extended periods.
#   - STEP_SPEED_RPS is the hard speed cap. Start conservative (0.15 RPS =
#     ~307 steps/sec). These motors stall easily under load at high speed.
#
# Motor-to-DOF mapping:
#   - Which physical motor controls pitch, roll, or yaw is TBD until mechanical
#     assembly confirms the cable routing. Flip the INVERT_DIR flags if a motor
#     runs backwards. Swap the PINS tuples to remap axes. Do not rewire.
#
# Soft limits:
#   - Each axis has configurable MIN/MAX step limits enforced on every incoming
#     command. The ESP32 will silently clamp targets to these limits.
#   - Set conservatively during first power-on. Widen after verifying range.
# =============================================================================

from machine import Pin
import sys
import select
import time
import ujson

# =============================================================================
# CONFIG — tune all parameters here
# =============================================================================

# --- 28BYJ-48 motor parameters ---
STEPS_PER_REV   = 2048     # Full-step mode. Use 4096 for half-step.
STEP_SPEED_RPS  = 0.15     # Max speed in revolutions per second.
                            # 0.15 RPS × 2048 steps/rev ≈ 307 steps/sec
                            # → ~3.3 ms per step. Increase once motion is confirmed.
STEP_INTERVAL_US = int(1_000_000 / (STEP_SPEED_RPS * STEPS_PER_REV))

# --- Soft travel limits (steps from home position) ---
# 1024 steps ≈ 180° at the motor shaft (before any EndoWrist cable gearing).
# Tighten these during first power-on and widen once motion is confirmed safe.
PITCH_MIN_STEPS = -1024
PITCH_MAX_STEPS =  1024
ROLL_MIN_STEPS  = -1024
ROLL_MAX_STEPS  =  1024
YAW_MIN_STEPS   = -1024
YAW_MAX_STEPS   =  1024

# --- Direction inversion ---
# Flip to True if a motor runs the wrong way. Do not rewire.
PITCH_INVERT_DIR = False
ROLL_INVERT_DIR  = False
YAW_INVERT_DIR   = False

# --- GPIO pin assignments (ULN2003 IN1–IN4 for each motor) ---
# Motor-to-DOF mapping is TBD pending mechanical assembly.
# Swap these tuples to remap which motor controls which DOF.
#
# Pins used: 4, 5, 13, 14, 21, 22, 23, 25, 26, 27, 32, 33
# All are safe output-capable GPIO on ESP32. Avoids: 0 (boot), 6–11 (flash),
# 12 (strapping), 15 (strapping), 34–39 (input-only), 1/3 (UART0 REPL).
PITCH_PINS = (13, 14, 27, 26)   # IN1, IN2, IN3, IN4
ROLL_PINS  = (25, 33, 32, 23)   # IN1, IN2, IN3, IN4
YAW_PINS   = (21, 22,  5,  4)   # IN1, IN2, IN3, IN4
# GRIP_PINS = (...)             # Reserved — not wired for MVP

# --- Status report interval ---
STATUS_REPORT_MS = 100    # How often to send position status back to Pi (ms)

# =============================================================================
# 28BYJ-48 full-step drive sequence
# Two coils energized simultaneously — more torque than wave drive.
# Coil order matches IN1, IN2, IN3, IN4 on the ULN2003 board.
# =============================================================================
_FULL_STEP = (
    (1, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 1),
    (1, 0, 0, 1),
)
_COILS_OFF = (0, 0, 0, 0)

# =============================================================================
# Stepper class
# =============================================================================

class Stepper28BYJ:
    """
    Non-blocking 28BYJ-48 stepper driver.

    Call update() as fast as possible in the main loop. It takes at most one
    step per call (rate-limited by STEP_INTERVAL_US). Coils are automatically
    de-energized when the motor reaches its target position.
    """

    def __init__(self, pins, invert=False):
        self.coils  = tuple(Pin(p, Pin.OUT) for p in pins)
        self.invert = invert
        self.seq    = 0           # current index into _FULL_STEP
        self.pos    = 0           # current position in steps from home
        self.target = 0           # commanded target position
        self._last_step_us = 0
        self._energized    = False
        self._apply(_COILS_OFF)

    def set_target(self, steps):
        self.target = steps
        if self.target != self.pos:
            self._energized = True

    def home(self):
        """Zero position counter and de-energize coils."""
        self.pos    = 0
        self.target = 0
        self._apply(_COILS_OFF)
        self._energized = False

    def stop(self):
        """Halt immediately at current position and de-energize coils."""
        self.target = self.pos
        self._apply(_COILS_OFF)
        self._energized = False

    def update(self):
        if self.pos == self.target:
            if self._energized:
                self._apply(_COILS_OFF)
                self._energized = False
            return

        now = time.ticks_us()
        if time.ticks_diff(now, self._last_step_us) < STEP_INTERVAL_US:
            return

        # Step toward target, accounting for direction inversion
        step_fwd = (self.target > self.pos) ^ self.invert
        if step_fwd:
            self.seq = (self.seq + 1) % 4
            self.pos += 1
        else:
            self.seq = (self.seq - 1) % 4
            self.pos -= 1

        self._apply(_FULL_STEP[self.seq])
        self._last_step_us = now

    def _apply(self, pattern):
        for coil, val in zip(self.coils, pattern):
            coil.value(val)


# =============================================================================
# Non-blocking serial I/O over UART0 (USB / REPL port)
# Uses select.poll() to check for data without blocking the motor loop.
# =============================================================================

_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_rx_buf = ""

def _try_read_line():
    """Return the next complete line from stdin, or None if not ready."""
    global _rx_buf
    while _poll.poll(0):
        c = sys.stdin.read(1)
        if not c:
            break
        if c == "\n":
            line = _rx_buf
            _rx_buf = ""
            return line
        _rx_buf += c
    return None

def _send(obj):
    """Serialize obj as JSON and write to stdout (Pi receives it)."""
    sys.stdout.write(ujson.dumps(obj) + "\n")


# =============================================================================
# Main
# =============================================================================

def main():
    pitch = Stepper28BYJ(PITCH_PINS, PITCH_INVERT_DIR)
    roll  = Stepper28BYJ(ROLL_PINS,  ROLL_INVERT_DIR)
    yaw   = Stepper28BYJ(YAW_PINS,   YAW_INVERT_DIR)

    last_status_ms = time.ticks_ms()
    print("Instrument controller ready.")

    while True:
        # --- Motor stepping (runs every iteration, rate-limited internally) ---
        pitch.update()
        roll.update()
        yaw.update()

        # --- Serial receive ---
        line = _try_read_line()
        if line:
            try:
                cmd = ujson.loads(line)
            except ValueError:
                cmd = None   # Malformed JSON — ignore

            if cmd is not None:
                if "cmd" in cmd:
                    if cmd["cmd"] == "home":
                        pitch.home()
                        roll.home()
                        yaw.home()
                    elif cmd["cmd"] == "stop":
                        pitch.stop()
                        roll.stop()
                        yaw.stop()
                else:
                    # Position command: clamp to soft limits and update targets
                    if "p" in cmd:
                        t = max(PITCH_MIN_STEPS, min(PITCH_MAX_STEPS, int(cmd["p"])))
                        pitch.set_target(t)
                    if "r" in cmd:
                        t = max(ROLL_MIN_STEPS, min(ROLL_MAX_STEPS, int(cmd["r"])))
                        roll.set_target(t)
                    if "y" in cmd:
                        t = max(YAW_MIN_STEPS, min(YAW_MAX_STEPS, int(cmd["y"])))
                        yaw.set_target(t)

        # --- Status report ---
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_status_ms) >= STATUS_REPORT_MS:
            _send({"p": pitch.pos, "r": roll.pos, "y": yaw.pos})
            last_status_ms = now_ms


main()

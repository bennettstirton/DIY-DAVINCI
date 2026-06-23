"""
pitch_sweep.py — OPEN-LOOP pitch motor-adequacy test with hand ROM calibration.

WHY THIS EXISTS
    Closed-loop "commanded vs actual" conflates two different things: control
    lag (soft gains) and a real torque/speed limit (underpowered motor). This
    script removes the PID entirely. It commands a CONSTANT step rate (the PWM
    hardware generates the pulses) and just watches the encoder. If the encoder
    keeps up, the motor has the torque at that speed/load. If the encoder falls
    behind the commanded angle (missed steps / stall), that's a physical limit.

TWO PHASES
    1. CALIBRATE (motor power OFF): you cut power to the stepper drivers and move
       the pitch axis by hand through its full available ROM for CALIB_SECONDS.
       The script records the min/max encoder angle (wrap-safe) = effective ROM.
    2. SWEEP (motor power ON): restore driver power, press RE-ARM, and the script
       sweeps pitch open-loop between the calibrated limits, ramping speed up.

HOW TO READ IT
    Prints axis_trace-compatible lines: PITCH "Tgt" = commanded open-loop angle
    (a clean triangle wave), "Act" = encoder. Watch on the live chart:
      - Lines overlay -> motor keeps up, torque is adequate at that speed
      - Act lags / clips below Tgt on the up-swings -> slipping = torque-limited
    The FREQ: column shows the commanded step rate (saturation check).

RUN IT
    Make sure config.py is on the device, then (does NOT overwrite main.py):
        mpremote connect /dev/cu.usbserial-21140 cp config.py :config.py + run pitch_sweep.py
    Watch live + log CSV in another terminal:
        source venv/bin/activate && python3 axis_trace.py

SAFETY
    Keep the e-stop in reach. E-stop or an encoder-read failure halts the test.
    The powered sweep insets SWEEP_MARGIN_DEG from the hand-found extremes and
    hard-aborts if the encoder leaves the band by HARD_MARGIN_DEG.
"""

import time
from machine import Pin, PWM, SoftI2C
from config import *

# --- Test parameters -------------------------------------------------------
CALIB_SECONDS   = 10.0   # hand-exercise window to capture ROM
SWEEP_MARGIN_DEG = 5.0   # inset the powered sweep from the hand-found extremes
HARD_MARGIN_DEG  = 10.0  # abort if encoder leaves [lo-margin, hi+margin]

# Speed staircase (rev/sec at the motor). Each stage runs SWEEPS_PER_STAGE
# traversals, then steps up. Tops out near PITCH_MAX_RPS.
SPEED_STAGES_RPS = [0.25, 0.5, 1.0, 1.5, 2.0]
SWEEPS_PER_STAGE = 4     # one traversal = lo->hi or hi->lo; 4 = two full cycles

# Speed profile at the ends: ramp down to MIN_SWEEP_RPS within DECEL_ZONE_DEG of
# a limit, then back up on the way out. The arm has real inertia, so this gives
# a gentle bounce instead of slamming a reversal at full speed. Measure motor
# adequacy from the CONSTANT-SPEED middle of each traversal, not these ramps.
DECEL_ZONE_DEG = 8.0     # degrees from a limit over which to ramp speed
MIN_SWEEP_RPS  = 0.1     # floor speed at the turnaround / start
# Time-based acceleration limit. Steppers can't jump to cruise speed from a
# standstill (they stall and gravity drags the arm back). This slews the applied
# speed up gradually so motion starts gently. Lower it if the arm still stalls
# at the start of a move; raise it to spend less time ramping.
ACCEL_RPS2     = 2.0     # rev/s^2 — how fast commanded speed may rise

# Reject single-tick encoder jumps bigger than this. At full sweep speed the arm
# moves ~36°/tick max, so a larger jump is a corrupted I2C read, not motion —
# ignore it (a clean read follows). Abort only if glitches persist.
MAX_GLITCH_DEG    = 45.0
MAX_CONSEC_GLITCH = 8

# Encoder reads can glitch from stepper EMI at higher speeds. On a failed read
# we stop the motor (removing the EMI source) and retry — the bus almost always
# recovers — and only abort if it stays dead this many attempts.
RECOVER_ATTEMPTS = 15    # ~0.3 s of retries with the motor stopped

LOOP_MS  = 50            # monitor/print cadence (20 Hz)

# --- Hardware (reuse config pins) ------------------------------------------
pitch_dir = Pin(PITCH_DIR_PIN, Pin.OUT, value=0)
pitch_pwm = PWM(Pin(PITCH_STEP_PIN), freq=1000, duty=0)
estop_pin = Pin(ESTOP_PIN, Pin.IN, Pin.PULL_UP)
rearm_pin = Pin(REARM_PIN, Pin.IN, Pin.PULL_UP)
i2c = SoftI2C(scl=Pin(AS5600_SCL_PIN), sda=Pin(AS5600_SDA_PIN), freq=AS5600_I2C_FREQ)

# Continuous-angle state: unwrap the 0-4095 encoder so a ROM that crosses the
# 0/360 boundary still reads as a monotonic arc. Maintained across BOTH phases.
_prev_raw = None
_turns    = 0


def tca_select(channel):
    i2c.writeto(TCA9548A_ADDR, bytes([1 << channel]))


def _read_pitch_raw():
    tca_select(PITCH_TCA_CHANNEL)
    for _ in range(3):
        try:
            d = i2c.readfrom_mem(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            return ((d[0] & 0x0F) << 8) | d[1]
        except OSError:
            time.sleep_us(500)
    return None


def read_pitch_cont():
    """Continuous (unwrapped) pitch angle in degrees, or None on I2C failure.

    Counts full turns by detecting >half-rev jumps between samples, so the
    value never wraps. Valid as long as it's sampled continuously and the arm
    never moves >180° between reads (trivially true at hand/sweep speeds)."""
    global _prev_raw, _turns
    raw = _read_pitch_raw()
    if raw is None:
        return None
    if _prev_raw is not None:
        d = raw - _prev_raw
        if d > 2048:
            _turns -= 1          # wrapped 0 -> 4095 (decreasing)
        elif d < -2048:
            _turns += 1          # wrapped 4095 -> 0 (increasing)
    _prev_raw = raw
    deg = ((_turns * 4096 + raw) / 4096.0) * 360.0 - PITCH_ENCODER_OFFSET_DEG
    return -deg if PITCH_ENCODER_INVERT else deg


_drive_freq = None     # last applied (freq, dir) so we don't re-init redundantly
_drive_dir  = None


def stop():
    global _drive_freq
    pitch_pwm.duty(0)
    _drive_freq = None


def drive(freq, dir_value):
    """Set direction and run the step PWM at freq (Hz). init() avoids timer leak.
    Re-inits only when freq (to ~5 Hz) or direction actually changes."""
    global _drive_freq, _drive_dir
    f = max(1, int(freq))
    if _drive_dir != dir_value:
        pitch_dir.value(dir_value)
        _drive_dir = dir_value
    if _drive_freq is None or abs(f - _drive_freq) >= 5:
        pitch_pwm.init(freq=f, duty=512)
        _drive_freq = f


def emit(cmd_deg, act_deg, freq):
    """axis_trace-compatible line. Pitch carries the test; roll/linear are dummy."""
    act_str = "{:6.1f}".format(act_deg) if act_deg is not None else " ???.?"
    print("| PITCH: Tgt:{:6.1f} Act:{} Err:{:5.1f}"
          " | ROLL: Tgt:   0.0 Act:   0.0 Err:  0.0"
          " | LINEAR: Cmd:  0.0mm Act:  0.0mm / 175mm"
          " | FREQ: P:{} R:0 L:0".format(
              cmd_deg, act_str,
              (cmd_deg - act_deg) if act_deg is not None else 0.0,
              int(freq)))


def wait_for_rearm(prompt):
    """Block until a fresh RE-ARM press, keeping the encoder unwrap alive.
    Returns False if e-stop is pressed instead."""
    print(prompt)
    # require release first so a held button doesn't auto-trigger
    while rearm_pin.value() == 0:
        read_pitch_cont()
        time.sleep_ms(20)
    while True:
        if estop_pin.value() == 1:
            return False
        read_pitch_cont()                  # keep continuous frame warm
        if rearm_pin.value() == 0:
            time.sleep_ms(30)              # debounce
            if rearm_pin.value() == 0:
                return True
        time.sleep_ms(20)


def calibrate():
    """Hand-exercise ROM capture. Returns (lo, hi) continuous degrees, or None."""
    if not wait_for_rearm(
            "CALIBRATE: cut stepper driver power, then press RE-ARM to start.\n"
            "  You'll have {:.0f}s to move pitch through its full ROM by hand.".format(
                CALIB_SECONDS)):
        return None

    # First read can glitch (intermittent I2C) — retry before giving up.
    a = None
    for _ in range(RECOVER_ATTEMPTS):
        a = read_pitch_cont()
        if a is not None:
            break
        time.sleep_ms(20)
    if a is None:
        print("ABORT: pitch encoder not reading after retries. Check TCA ch {} / wiring.".format(
            PITCH_TCA_CHANNEL))
        return None

    lo = hi = a
    print("Calibrating — move pitch through its full range now...")
    t_end = time.ticks_add(time.ticks_ms(), int(CALIB_SECONDS * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        if estop_pin.value() == 1:
            print("E-STOP — calibration aborted.")
            return None
        a = read_pitch_cont()
        if a is not None:
            if a < lo: lo = a
            if a > hi: hi = a
            emit((lo + hi) / 2, a, 0)      # live view; FREQ 0 = motor off
        time.sleep_ms(LOOP_MS)

    span = hi - lo
    print("ROM captured: [{:.1f}, {:.1f}]°  span {:.1f}°".format(lo, hi, span))
    if span < 5.0:
        print("ABORT: ROM span {:.1f}° too small — did the axis actually move?".format(span))
        return None
    return lo, hi


def sweep(lo, hi):
    sweep_lo = lo + SWEEP_MARGIN_DEG
    sweep_hi = hi - SWEEP_MARGIN_DEG
    if sweep_hi - sweep_lo < 3.0:
        print("ABORT: usable range after {:.0f}° margins is too small.".format(SWEEP_MARGIN_DEG))
        return
    print("Sweep band (inset {:.0f}°): [{:.1f}, {:.1f}]°".format(
        SWEEP_MARGIN_DEG, sweep_lo, sweep_hi))

    if not wait_for_rearm(
            "SWEEP: restore stepper driver power, then press RE-ARM to begin."):
        print("E-STOP — sweep not started.")
        return

    # First read can glitch as the drivers re-energize — retry before committing.
    act = None
    for _ in range(RECOVER_ATTEMPTS):
        act = read_pitch_cont()
        if act is not None:
            break
        time.sleep_ms(20)
    if act is None:
        print("ABORT: encoder not reading at sweep start. Check wiring / power.")
        return

    # Seed the "increases angle" direction bit from the config convention armcontrol
    # uses (set_motor drives dir = forward ^ invert; forward raises pitch). This is
    # usually correct, so it avoids a wrong-way twitch when starting at an extreme.
    # The first-motion auto-detect below still corrects it if the seed is wrong.
    dir_to_hi = 0 if PITCH_INVERT_DIR else 1
    dir_known = False
    cmd_deg = act
    target  = sweep_hi if act < (sweep_lo + sweep_hi) / 2 else sweep_lo
    applied_rps = MIN_SWEEP_RPS   # actually-commanded speed; slew-limited (soft start)

    for rps in SPEED_STAGES_RPS:
        sweeps_done = 0
        wrong_way_ms = 0
        print("--- stage {} rps  (cruise {} Hz) ---".format(rps, int(rps * PITCH_STEPS_PER_REV)))

        dir_value = dir_to_hi if target == sweep_hi else (1 - dir_to_hi)
        last = time.ticks_ms()
        last_print = last
        last_act = act
        glitch_count = 0

        while sweeps_done < SWEEPS_PER_STAGE:
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last) / 1000.0
            last = now

            if estop_pin.value() == 1:
                stop(); print("E-STOP — test halted."); return

            act = read_pitch_cont()
            if act is None:
                # Stop the motor (kills the EMI source) and retry; resume if it recovers.
                stop()
                for _ in range(RECOVER_ATTEMPTS):
                    time.sleep_ms(20)
                    act = read_pitch_cont()
                    if act is not None:
                        break
                if act is None:
                    stop(); print("ABORT: encoder read unrecoverable — motor stopped (likely EMI)."); return
                print("note: encoder glitch recovered, resuming.")
                cmd_deg = act; last_act = act; last = time.ticks_ms()
                applied_rps = MIN_SWEEP_RPS   # soft-start again (motor was stopped)
                continue

            # Reject corrupted reads: an impossible one-tick jump is a bad I2C
            # sample, not motion. Hold last position and let the next read recover.
            if abs(act - last_act) > MAX_GLITCH_DEG:
                glitch_count += 1
                if glitch_count > MAX_CONSEC_GLITCH:
                    stop()
                    print("ABORT: persistent encoder glitches (last {:+.0f}° jump) — bus too noisy.".format(
                        act - last_act))
                    return
                act = last_act         # ignore the spike this tick
            else:
                glitch_count = 0

            if act > sweep_hi + HARD_MARGIN_DEG or act < sweep_lo - HARD_MARGIN_DEG:
                stop(); print("ABORT: pitch {:.1f}° left safe band — stopped.".format(act)); return

            going_up = (target == sweep_hi)

            # Position profile: full rps in the middle, ramping to MIN_SWEEP_RPS within
            # DECEL_ZONE_DEG of the NEAREST limit (decel approaching, accel leaving).
            dist = min(act - sweep_lo, sweep_hi - act)
            frac = max(0.0, min(1.0, dist / DECEL_ZONE_DEG))
            target_rps = MIN_SWEEP_RPS + (rps - MIN_SWEEP_RPS) * frac

            # Soft start: slew the APPLIED speed up toward target at ACCEL_RPS2 so
            # the motor never jumps to cruise from a standstill (which stalls).
            # Decreases (decel near a limit) apply immediately via the min().
            applied_rps = min(target_rps, applied_rps + ACCEL_RPS2 * dt)
            applied_rps = max(applied_rps, MIN_SWEEP_RPS)
            cur_rps = applied_rps
            drive(cur_rps * PITCH_STEPS_PER_REV, dir_value)

            cmd_deg += (cur_rps * 360.0 * dt) * (1 if going_up else -1)

            moved = act - last_act
            if abs(moved) > 0.05:
                toward = (moved > 0) == going_up
                if not dir_known and abs(moved) > 0.3:
                    if not toward:
                        dir_value = 1 - dir_value
                        dir_to_hi = dir_value if going_up else (1 - dir_value)
                    dir_known = True
                wrong_way_ms = wrong_way_ms + LOOP_MS if (dir_known and not toward) else 0
                if wrong_way_ms > 600:
                    stop(); print("ABORT: moving away from target despite drive — check wiring."); return
            last_act = act

            if going_up and act >= sweep_hi:
                target = sweep_lo; cmd_deg = act; sweeps_done += 1; dir_value = 1 - dir_to_hi
            elif (not going_up) and act <= sweep_lo:
                target = sweep_hi; cmd_deg = act; sweeps_done += 1; dir_value = dir_to_hi

            if time.ticks_diff(now, last_print) >= LOOP_MS:
                last_print = now
                emit(cmd_deg, act, cur_rps * PITCH_STEPS_PER_REV)

            elapsed = time.ticks_diff(time.ticks_ms(), now)
            if LOOP_MS - elapsed > 0:
                time.sleep_ms(LOOP_MS - elapsed)

    stop()
    print("pitch_sweep complete. Motor stopped.")


def run():
    rom = calibrate()
    if rom is None:
        return
    sweep(*rom)


try:
    run()
finally:
    pitch_pwm.deinit()

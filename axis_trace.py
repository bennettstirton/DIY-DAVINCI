#!/usr/bin/env python3
"""
axis_trace.py — Scrolling trace for all 3 arm axes: pitch, roll, and linear.

Pitch + Roll subplots:
    Solid blue    = commanded target
    Dashed cyan   = actual position (AS5600 encoder)
    Dotted yellow = sinusoid fit to commanded data (orbit sanity check)

Linear subplot:
    Solid cyan    = position in mm (open-loop — no separate target/actual)
    Dashed white  = max travel limit (175 mm)

Run with the venv active:
    source venv/bin/activate
    python3 axis_trace.py
"""

import serial
import re
import threading
import time
import collections
import math
import sys
import csv
import os
import matplotlib
matplotlib.use('MacOSX')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =============================================================================
# CONFIG
# =============================================================================

SERIAL_PORT    = "/dev/cu.usbserial-21140"
BAUD_RATE      = 115200
WINDOW_S       = 10.0    # seconds of history visible at once
DEMO_ORBIT_RPS = 0.15    # must match config.py — used for sinusoid fit

# =============================================================================
# SERIAL PARSING
# =============================================================================

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
LINE_RE = re.compile(
    r'PITCH:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+).*?'
    r'ROLL:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+).*?'
    r'LINEAR:.*?Cmd:\s*([-\d.]+)mm.*?Act:\s*([-\d.?]+)mm',
    re.DOTALL
)

# Diagnostic input line from print_input_debug() in armcontrol.py:
#   INPUT | TRIM rawX:2048 rawY:2050 sclX:+0.00 sclY:+0.00 | IMU p:  +1.2 r:  -0.3
INPUT_RE = re.compile(
    r'INPUT.*?rawX:\s*([-+\d.]+).*?rawY:\s*([-+\d.]+).*?'
    r'sclX:\s*([-+\d.]+).*?sclY:\s*([-+\d.]+).*?'
    r'IMU\s*p:\s*([-+\d.]+).*?r:\s*([-+\d.]+)',
    re.DOTALL
)

# Optional commanded-frequency token appended to the PITCH/ROLL/LINEAR line:
#   ... | FREQ: P:1234 R:567 L:0
# Lets us see whether the controller is saturating (railed at *_MAX_FREQ =
# lagging because it's physically maxed) vs loafing (lagging because of soft
# gains). pitch_sweep.py also emits this. Absent on older firmware → logged "".
FREQ_RE = re.compile(r'FREQ:\s*P:\s*([-\d]+)\s*R:\s*([-\d]+)\s*L:\s*([-\d]+)')

# Trim joystick ADC reference values — must match config.py.
TRIM_CENTER   = 2048
TRIM_DEADBAND = 300

_MAXLEN    = int(WINDOW_S * 20)
times      = collections.deque(maxlen=_MAXLEN)
pitch_tgt  = collections.deque(maxlen=_MAXLEN)
pitch_act  = collections.deque(maxlen=_MAXLEN)
roll_tgt   = collections.deque(maxlen=_MAXLEN)
roll_act   = collections.deque(maxlen=_MAXLEN)
linear_cmd = collections.deque(maxlen=_MAXLEN)
linear_act = collections.deque(maxlen=_MAXLEN)

# Input diagnostic buffers (independent time axis — INPUT lines arrive at their
# own 10 Hz cadence, interleaved with the PITCH/ROLL/LINEAR debug lines).
in_times   = collections.deque(maxlen=_MAXLEN)
in_raw_x   = collections.deque(maxlen=_MAXLEN)
in_raw_y   = collections.deque(maxlen=_MAXLEN)
_last_input = {"raw_x": 0, "raw_y": 0, "scl_x": 0.0, "scl_y": 0.0,
               "imu_p": 0.0, "imu_r": 0.0, "seen": False}

buf_lock   = threading.Lock()
_status    = "Connecting..."
_t0        = time.monotonic()

LOG_DIR  = os.path.join(os.path.dirname(__file__), "logs")
os.makedirs(LOG_DIR, exist_ok=True)
_log_filename = os.path.join(LOG_DIR, time.strftime("axis_%Y%m%d_%H%M%S.csv"))
_log_file     = open(_log_filename, "w", newline="")
_csv          = csv.writer(_log_file)
_csv.writerow(["t_s", "pitch_tgt", "pitch_act", "roll_tgt", "roll_act",
               "linear_cmd_mm", "linear_act_mm", "pitch_freq"])


def serial_reader():
    global _status
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                _status = f"Connected  ({SERIAL_PORT})"
                print(f"[axis_trace] Serial connected: {SERIAL_PORT}")
                while True:
                    raw  = ser.readline().decode("utf-8", errors="replace")
                    line = ANSI_ESCAPE.sub("", raw).strip()
                    m    = LINE_RE.search(line)
                    if m:
                        t = time.monotonic() - _t0
                        lin_act_str = m.group(6).strip()
                        try:
                            lin_act_val = float(lin_act_str)
                        except ValueError:
                            lin_act_val = None  # encoder read failed ("?.?")
                        mf = FREQ_RE.search(line)
                        pitch_freq = mf.group(1) if mf else ""
                        with buf_lock:
                            times.append(t)
                            pitch_tgt.append(float(m.group(1)))
                            pitch_act.append(float(m.group(2)))
                            roll_tgt.append(float(m.group(3)))
                            roll_act.append(float(m.group(4)))
                            linear_cmd.append(float(m.group(5)))
                            if lin_act_val is not None:
                                linear_act.append(lin_act_val)
                        _csv.writerow([
                            f"{t:.3f}",
                            m.group(1), m.group(2),
                            m.group(3), m.group(4),
                            m.group(5),
                            "" if lin_act_val is None else f"{lin_act_val:.1f}",
                            pitch_freq
                        ])
                        _log_file.flush()
                        continue

                    mi = INPUT_RE.search(line)
                    if mi:
                        t = time.monotonic() - _t0
                        with buf_lock:
                            in_times.append(t)
                            in_raw_x.append(int(float(mi.group(1))))
                            in_raw_y.append(int(float(mi.group(2))))
                            _last_input["raw_x"] = int(float(mi.group(1)))
                            _last_input["raw_y"] = int(float(mi.group(2)))
                            _last_input["scl_x"] = float(mi.group(3))
                            _last_input["scl_y"] = float(mi.group(4))
                            _last_input["imu_p"] = float(mi.group(5))
                            _last_input["imu_r"] = float(mi.group(6))
                            _last_input["seen"]  = True
                        continue

                    if line:
                        print(f"[ESP32] {line}")
        except serial.SerialException as e:
            _status = f"NOT CONNECTED — retrying...  ({SERIAL_PORT})"
            print(f"[axis_trace] Serial error: {e}. Retrying in 2 s...")
            time.sleep(2)

# =============================================================================
# SINUSOID FIT
# =============================================================================

def fit_sinusoid(ts, ys, rps):
    """
    Fit y = C + Ac*cos(ωt) + As*sin(ωt) to (ts, ys) for known frequency rps.
    Returns (fitted_ys, amplitude_deg) or (None, None) if insufficient data.
    """
    n = len(ts)
    if n < 20:
        return None, None

    omega  = rps * 2.0 * math.pi
    mean_y = sum(ys) / n
    cos_v  = [math.cos(omega * t) for t in ts]
    sin_v  = [math.sin(omega * t) for t in ts]
    det_y  = [y - mean_y for y in ys]

    cc = sum(c * c for c in cos_v)
    ss = sum(s * s for s in sin_v)
    cs = sum(c * s for c, s in zip(cos_v, sin_v))
    cy = sum(c * d for c, d in zip(cos_v, det_y))
    sy = sum(s * d for s, d in zip(sin_v, det_y))

    det = cc * ss - cs * cs
    if abs(det) < 1e-9:
        return None, None

    Ac        = (ss * cy - cs * sy) / det
    As        = (cc * sy - cs * cy) / det
    amplitude = math.sqrt(Ac ** 2 + As ** 2)
    fitted    = [mean_y + Ac * cos_v[i] + As * sin_v[i] for i in range(n)]
    return fitted, amplitude

# =============================================================================
# PLOT SETUP
# =============================================================================

fig, (ax_p, ax_r, ax_l, ax_in) = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
fig.patch.set_facecolor('#1a1a2e')
fig.subplots_adjust(hspace=0.45)


def _style_ax(ax, title, ylabel):
    ax.set_facecolor('#1a1a2e')
    ax.set_ylabel(ylabel, color='white')
    ax.tick_params(colors='white')
    for spine in ('bottom', 'left'):
        ax.spines[spine].set_color('#555577')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True, color='#333355', linestyle='--', linewidth=0.5)
    ax.set_title(title, color='white', fontsize=11)


_style_ax(ax_p, "Pitch — Commanded vs Actual", "Pitch (°)")
_style_ax(ax_r, "Roll  — Commanded vs Actual", "Roll (°)")
_style_ax(ax_l, "Linear — Position",            "Linear (mm)")
_style_ax(ax_in, "Input — Trim joystick raw ADC (move stick to verify ESP32 sees it)",
          "ADC (0-4095)")
ax_in.set_xlabel("Time (s)", color='white')

_LEGEND_KW = dict(fontsize=8, framealpha=0.3, labelcolor='white',
                  facecolor='#1a1a2e', edgecolor='#444')

(lp_tgt,)  = ax_p.plot([], [], color='#4488ff', lw=1.5,  label='Commanded')
(lp_act,)  = ax_p.plot([], [], color='#55ddff', lw=1.5,  ls='--', label='Actual (encoder)')
(lp_pred,) = ax_p.plot([], [], color='#ffdd44', lw=1.0,  ls=':',  label='Sinusoid fit')
ax_p.legend(loc='upper left', **_LEGEND_KW)

(lr_tgt,)  = ax_r.plot([], [], color='#4488ff', lw=1.5,  label='Commanded')
(lr_act,)  = ax_r.plot([], [], color='#55ddff', lw=1.5,  ls='--', label='Actual (encoder)')
(lr_pred,) = ax_r.plot([], [], color='#ffdd44', lw=1.0,  ls=':',  label='Sinusoid fit')
ax_r.legend(loc='upper left', **_LEGEND_KW)

(ll_cmd,)  = ax_l.plot([], [], color='#4488ff', lw=1.5,  label='Commanded (steps→mm)')
(ll_act,)  = ax_l.plot([], [], color='#55ddff', lw=1.5,  ls='--', label='Actual (AS5600 encoder)')
# Static ceiling line — drawn once, stays in place
ax_l.axhline(y=175.0, color='#ffffff', lw=0.8, ls='--', alpha=0.4, label='Max travel (175 mm)')
ax_l.set_ylim(-5, 190)
ax_l.legend(loc='upper left', **_LEGEND_KW)

# --- Input subplot: raw trim ADC over time ---
(li_x,) = ax_in.plot([], [], color='#ff8800', lw=1.5, label='rawX (roll stick)')
(li_y,) = ax_in.plot([], [], color='#44ff88', lw=1.5, label='rawY (pitch stick)')
# Center line + deadband band — within this band the stick reads as zero command.
ax_in.axhline(y=TRIM_CENTER, color='#ffffff', lw=0.8, ls='--', alpha=0.4,
              label=f'center ({TRIM_CENTER})')
ax_in.axhspan(TRIM_CENTER - TRIM_DEADBAND, TRIM_CENTER + TRIM_DEADBAND,
              color='#555577', alpha=0.18)
ax_in.set_ylim(-100, 4195)
ax_in.legend(loc='upper left', **_LEGEND_KW)

# Live numeric readout — true even inside the deadband, so you can confirm the
# pin is moving at all. If these don't change when you move the stick → wiring.
input_text = ax_in.text(0.99, 0.95, "", transform=ax_in.transAxes,
                        color='#88ddff', fontsize=9, family='monospace',
                        ha='right', va='top',
                        bbox=dict(facecolor='#0d0d1a', alpha=0.8, edgecolor='#444'))

status_text = ax_in.text(0.99, 0.02, _status, transform=ax_in.transAxes,
                        color='#888899', fontsize=8, ha='right', va='bottom')

# =============================================================================
# ANIMATION
# =============================================================================

def _update_rotary_axis(ax, l_tgt, l_act, l_pred, ts, tgt, act, t_min):
    vis_t   = [t   for t in ts                  if t >= t_min]
    vis_tgt = [v   for t, v in zip(ts, tgt)     if t >= t_min]
    vis_act = [v   for t, v in zip(ts, act)     if t >= t_min]

    l_tgt.set_data(vis_t, vis_tgt)
    l_act.set_data(vis_t, vis_act)

    fitted, amplitude = fit_sinusoid(vis_t, vis_tgt, DEMO_ORBIT_RPS)
    if fitted is not None:
        l_pred.set_data(vis_t, fitted)
        ax.set_title(ax.get_title().split("  [")[0] +
                     f"  [fit amplitude: {amplitude:.1f}°]", color='white', fontsize=11)
    else:
        l_pred.set_data([], [])

    all_v = vis_tgt + vis_act
    if all_v:
        lo, hi = min(all_v), max(all_v)
        pad = max((hi - lo) * 0.25, 1.5)
        ax.set_ylim(lo - pad, hi + pad)


_ALL_ARTISTS = (lp_tgt, lp_act, lp_pred, lr_tgt, lr_act, lr_pred,
                ll_cmd, ll_act, li_x, li_y, input_text, status_text)


def update(_frame):
    with buf_lock:
        ts   = list(times)
        ptgt = list(pitch_tgt)
        pact = list(pitch_act)
        rtgt = list(roll_tgt)
        ract = list(roll_act)
        lcmd = list(linear_cmd)
        lact = list(linear_act)
        its  = list(in_times)
        ix   = list(in_raw_x)
        iy   = list(in_raw_y)
        last = dict(_last_input)

    # Scroll window tracks whichever stream is freshest. During active jogging
    # only INPUT lines arrive (handle_jogging skips debug_print), so the pitch/
    # roll times go stale — without this the input plot would freeze mid-jog.
    last_t = []
    if ts:
        last_t.append(ts[-1])
    if its:
        last_t.append(its[-1])
    if not last_t:
        return _ALL_ARTISTS

    now   = max(last_t)
    t_min = now - WINDOW_S
    ax_p.set_xlim(t_min, now + 0.3)

    if ts:
        _update_rotary_axis(ax_p, lp_tgt, lp_act, lp_pred, ts, ptgt, pact, t_min)
        _update_rotary_axis(ax_r, lr_tgt, lr_act, lr_pred, ts, rtgt, ract, t_min)

        vis_t    = [t for t in ts               if t >= t_min]
        vis_lcmd = [v for t, v in zip(ts, lcmd) if t >= t_min]
        ll_cmd.set_data(vis_t, vis_lcmd)

        # linear_act deque may be shorter than times if some reads failed
        if lact:
            lact_start = max(0, len(lact) - len(vis_t))
            ll_act.set_data(vis_t[-len(lact):], lact[lact_start:])

    # --- Input subplot ---
    vis_it = [t for t in its             if t >= t_min]
    vis_ix = [v for t, v in zip(its, ix) if t >= t_min]
    vis_iy = [v for t, v in zip(its, iy) if t >= t_min]
    li_x.set_data(vis_it, vis_ix)
    li_y.set_data(vis_it, vis_iy)

    if last["seen"]:
        input_text.set_text(
            "rawX:{:4d} ({:+5d})  sclX:{:+.2f}\n"
            "rawY:{:4d} ({:+5d})  sclY:{:+.2f}\n"
            "IMU  p:{:+.1f}  r:{:+.1f}".format(
                last["raw_x"], last["raw_x"] - TRIM_CENTER, last["scl_x"],
                last["raw_y"], last["raw_y"] - TRIM_CENTER, last["scl_y"],
                last["imu_p"], last["imu_r"]))
    else:
        input_text.set_text("no INPUT line yet\n(redeploy armcontrol.py)")

    status_text.set_text(_status)
    return _ALL_ARTISTS


ani = animation.FuncAnimation(fig, update, interval=50,
                               blit=False, cache_frame_data=False)

# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    period = 1.0 / DEMO_ORBIT_RPS
    print("=" * 60)
    print("  Axis Trace — Pitch / Roll / Linear")
    print(f"  Port:      {SERIAL_PORT}")
    print(f"  Window:    {WINDOW_S:.0f} s rolling")
    print(f"  Orbit:     {DEMO_ORBIT_RPS} Hz  →  {period:.1f} s period")
    print()
    print("  Pitch + Roll:")
    print(f"    Blue solid  = commanded target")
    print(f"    Cyan dashed = actual (AS5600 encoder)")
    print(f"    Yellow dot  = sinusoid fit to commanded (orbit sanity check)")
    print()
    print("  Linear:")
    print(f"    Blue solid  = commanded position (step count → mm)")
    print(f"    Cyan dashed = actual position (AS5600 encoder on NEMA17 shaft)")
    print(f"    White dash  = 175 mm travel limit")
    print()
    print("  Input (trim joystick raw ADC):")
    print(f"    Orange = rawX (roll stick)   Green = rawY (pitch stick)")
    print(f"    Grey band = deadband (±{TRIM_DEADBAND} of {TRIM_CENTER}) → reads as zero command")
    print(f"    If lines don't move when you wiggle the stick → wiring, not code")
    print()
    print("  Close the plot window to exit.")
    print(f"  Logging to: {_log_filename}")
    print("=" * 60)

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        _log_file.close()
        print(f"\nLog saved: {_log_filename}")
        sys.exit(0)

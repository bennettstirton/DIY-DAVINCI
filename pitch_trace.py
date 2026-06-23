#!/usr/bin/env python3
"""
pitch_trace.py — Scrolling pitch + roll commanded vs actual trace for jitter diagnosis.

Two subplots sharing a time axis:
    Solid blue    = commanded target (from PID input)
    Dashed cyan   = actual position  (from AS5600 encoder)
    Dotted yellow = sinusoid fit to commanded data (reference / sanity check)

Run with the venv active:
    source venv/bin/activate
    python3 pitch_trace.py
"""

import serial
import re
import threading
import time
import collections
import math
import sys
import matplotlib
matplotlib.use('MacOSX')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =============================================================================
# CONFIG
# =============================================================================

SERIAL_PORT    = "/dev/cu.usbserial-21140"   # same port as arm_visualizer.py
BAUD_RATE      = 115200
WINDOW_S       = 10.0    # seconds of history visible at once
DEMO_ORBIT_RPS = 0.15    # must match config.py — used for sinusoid fit

# =============================================================================
# SERIAL PARSING
# =============================================================================

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
LINE_RE = re.compile(
    r'PITCH:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+).*?'
    r'ROLL:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+)',
    re.DOTALL
)

_MAXLEN   = int(WINDOW_S * 20)
times     = collections.deque(maxlen=_MAXLEN)
pitch_tgt = collections.deque(maxlen=_MAXLEN)
pitch_act = collections.deque(maxlen=_MAXLEN)
roll_tgt  = collections.deque(maxlen=_MAXLEN)
roll_act  = collections.deque(maxlen=_MAXLEN)
buf_lock  = threading.Lock()
_status   = "Connecting..."
_t0       = time.monotonic()


def serial_reader():
    global _status
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                _status = f"Connected  ({SERIAL_PORT})"
                print(f"[pitch_trace] Serial connected: {SERIAL_PORT}")
                while True:
                    raw  = ser.readline().decode("utf-8", errors="replace")
                    line = ANSI_ESCAPE.sub("", raw).strip()
                    m    = LINE_RE.search(line)
                    if m:
                        t = time.monotonic() - _t0
                        with buf_lock:
                            times.append(t)
                            pitch_tgt.append(float(m.group(1)))
                            pitch_act.append(float(m.group(2)))
                            roll_tgt.append(float(m.group(3)))
                            roll_act.append(float(m.group(4)))
                    elif line:
                        print(f"[ESP32] {line}")
        except serial.SerialException as e:
            _status = f"NOT CONNECTED — retrying...  ({SERIAL_PORT})"
            print(f"[pitch_trace] Serial error: {e}. Retrying in 2 s...")
            time.sleep(2)

# =============================================================================
# SINUSOID FIT
# =============================================================================

def fit_sinusoid(ts, ys, rps):
    """
    Fit y = C + Ac*cos(ωt) + As*sin(ωt) to (ts, ys) for known frequency rps.
    Returns (fitted_ys, amplitude_deg) or (None, None) if insufficient data.

    Uses least-squares via the 2x2 normal equations — O(n), runs in <1ms.
    The fit phase-locks to whatever the data is doing, so it works regardless
    of when pitch_trace.py was started relative to the orbit cycle.
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

fig, (ax_p, ax_r) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
fig.patch.set_facecolor('#1a1a2e')
fig.subplots_adjust(hspace=0.35)


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
ax_r.set_xlabel("Time (s)", color='white')

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

status_text = ax_r.text(0.99, 0.02, _status, transform=ax_r.transAxes,
                        color='#888899', fontsize=8, ha='right', va='bottom')

# =============================================================================
# ANIMATION
# =============================================================================

def _update_axis(ax, l_tgt, l_act, l_pred, ts, tgt, act, t_min):
    vis_t   = [t       for t in ts                  if t >= t_min]
    vis_tgt = [v       for t, v in zip(ts, tgt)     if t >= t_min]
    vis_act = [v       for t, v in zip(ts, act)     if t >= t_min]

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


def update(_frame):
    with buf_lock:
        ts   = list(times)
        ptgt = list(pitch_tgt)
        pact = list(pitch_act)
        rtgt = list(roll_tgt)
        ract = list(roll_act)

    if not ts:
        return lp_tgt, lp_act, lp_pred, lr_tgt, lr_act, lr_pred

    now   = ts[-1]
    t_min = now - WINDOW_S
    ax_p.set_xlim(t_min, now + 0.3)

    _update_axis(ax_p, lp_tgt, lp_act, lp_pred, ts, ptgt, pact, t_min)
    _update_axis(ax_r, lr_tgt, lr_act, lr_pred, ts, rtgt, ract, t_min)

    status_text.set_text(_status)
    return lp_tgt, lp_act, lp_pred, lr_tgt, lr_act, lr_pred


ani = animation.FuncAnimation(fig, update, interval=50,
                               blit=False, cache_frame_data=False)

# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    period = 1.0 / DEMO_ORBIT_RPS
    print("=" * 60)
    print("  Pitch + Roll Trace — Commanded vs Actual")
    print(f"  Port:      {SERIAL_PORT}")
    print(f"  Window:    {WINDOW_S:.0f} s rolling")
    print(f"  Orbit:     {DEMO_ORBIT_RPS} Hz  →  {period:.1f} s period")
    print()
    print("  Predicted waveforms:")
    print(f"    Pitch commanded : cosine, ±10° around home, {period:.1f}s period")
    print(f"    Roll  commanded : sine,   ±10° around home, {period:.1f}s period (90° behind pitch)")
    print(f"    Actual (both)   : should track commanded closely — orbit only")
    print(f"                      needs ~9.4°/s peak, well within motor limits.")
    print(f"                      Expect <1° lag, small amplitude loss if any.")
    print()
    print("  Yellow dotted = least-squares sinusoid fit to commanded data.")
    print("  If fit diverges from commanded, the orbit math has a timing issue.")
    print("  Close the plot window to exit.")
    print("=" * 60)

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("\nExiting.")
        sys.exit(0)

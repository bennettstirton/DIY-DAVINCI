#!/usr/bin/env python3
"""
arm_visualizer.py  —  Real-time 3D digital twin for the 3-axis robot arm.

Reads the ESP32's existing debug serial output — no changes to armcontrol.py needed.
Shows commanded (target) vs actual positions for all three axes simultaneously.

SETUP (one-time):
    pip install pyserial matplotlib

BEFORE RUNNING:
    1. Set SERIAL_PORT below to match your ESP32's USB serial port.
       - Mac:     /dev/cu.usbserial-XXXXX  or  /dev/cu.SLAB_USBtoUART
                  (run: ls /dev/cu.* in Terminal to find it)
       - Windows: COM3  or  COM4  (check Device Manager)
       - Linux:   /dev/ttyUSB0
    2. Make sure armcontrol.py is running on the ESP32.
    3. Run: python3 arm_visualizer.py

WHAT YOU SEE:
    Blue  solid  = arm commanded position (what the joystick is asking for)
    Cyan  dashed = arm actual position    (what the encoders read)
    Red   solid  = linear extension (commanded / open-loop)
    Green bar    = roll orientation indicator at the tip (commanded)
    Yellow bar   = roll orientation indicator at the tip (actual)

GEOMETRY ASSUMPTIONS (adjust if the motion looks wrong):
    - Pitch tilts the arm in the vertical plane (positive = tip goes up)
    - Roll rotates around the arm's own long axis (shown as crossbar at tip)
    - Linear extends the end effector along the arm direction

    If pitch/roll axes are swapped or inverted on screen, flip PITCH_SIGN
    or ROLL_SIGN, or swap the axis labels below.
"""

import serial
import re
import threading
import time
import math
import sys
import matplotlib
matplotlib.use('TkAgg')   # works on Mac; change to 'Qt5Agg' if TkAgg fails
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (activates 3D projection)

# =============================================================================
# CONFIG — edit these
# =============================================================================

SERIAL_PORT  = "/dev/cu.usbserial-21140"   # <-- UPDATE THIS (see instructions above)
BAUD_RATE    = 115200

# Visual arm geometry (these are display lengths, not necessarily physical)
ARM_LENGTH   = 300    # mm — length of the main arm link in the visualisation
MAX_LINEAR   = 175    # mm — matches LINEAR_MAX_MM in armcontrol.py

# Sign flips: set to -1 if a motion appears inverted on screen
PITCH_SIGN   = 1
ROLL_SIGN    = 1

# =============================================================================
# SERIAL PARSING
# =============================================================================

# Strip ANSI bold/reset escape codes that armcontrol.py injects
ANSI_ESCAPE  = re.compile(r'\x1b\[[0-9;]*m')

# Match the debug_print output:
#   | PITCH: Tgt:  5.2 Act:  4.8 Err: 0.4 | ROLL: Tgt: ... | LINEAR:  45.2mm / 175mm
LINE_RE = re.compile(
    r'PITCH:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+).*?'
    r'ROLL:.*?Tgt:\s*([-\d.]+).*?Act:\s*([-\d.]+).*?'
    r'LINEAR:.*?([\d.]+)mm',
    re.DOTALL
)

# Shared state — updated by serial thread, read by the animation callback
state = {
    "pitch_tgt": 0.0,
    "pitch_act": 0.0,
    "roll_tgt":  0.0,
    "roll_act":  0.0,
    "linear_mm": 0.0,
    "status":    "Connecting...",
}
state_lock = threading.Lock()


def serial_reader():
    """Background thread: opens serial port, parses lines, updates state."""
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                with state_lock:
                    state["status"] = f"Connected ({SERIAL_PORT})"
                print(f"[visualizer] Serial connected: {SERIAL_PORT}")

                while True:
                    raw = ser.readline().decode("utf-8", errors="replace")
                    line = ANSI_ESCAPE.sub("", raw).strip()
                    m = LINE_RE.search(line)
                    if m:
                        with state_lock:
                            state["pitch_tgt"] = PITCH_SIGN * float(m.group(1))
                            state["pitch_act"] = PITCH_SIGN * float(m.group(2))
                            state["roll_tgt"]  = ROLL_SIGN  * float(m.group(3))
                            state["roll_act"]  = ROLL_SIGN  * float(m.group(4))
                            state["linear_mm"] = float(m.group(5))

        except serial.SerialException as e:
            with state_lock:
                state["status"] = f"NOT CONNECTED — {SERIAL_PORT}  (retrying...)"
            print(f"[visualizer] Serial error: {e}. Retrying in 2 s...")
            time.sleep(2)

# =============================================================================
# FORWARD KINEMATICS
# =============================================================================

def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def _normalize(v):
    mag = math.sqrt(sum(x*x for x in v))
    return [x/mag for x in v] if mag > 1e-9 else [0.0, 0.0, 1.0]

def arm_points(pitch_deg, roll_deg, linear_mm):
    """
    Compute 3D stick-figure points for the arm given its angles and extension.

    Returns:
        base      — fixed base at origin
        arm_tip   — tip of the rigid arm section (after pitch)
        ext_tip   — tip of the linear extension
        roll_pos  — one end of the roll-indicator crossbar at ext_tip
        roll_neg  — other end of the roll-indicator crossbar
    """
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)

    # Arm direction: nominally along +Y, pitched upward by pitch_deg
    arm_dir = [0.0, math.cos(p), math.sin(p)]

    base    = [0.0, 0.0, 0.0]
    arm_tip = [ARM_LENGTH * d for d in arm_dir]
    ext_tip = [arm_tip[i] + linear_mm * arm_dir[i] for i in range(3)]

    # Roll indicator — a short crossbar perpendicular to arm_dir, rotated by roll_deg
    # Find two basis vectors perpendicular to arm_dir
    ref  = [0.0, 0.0, 1.0] if abs(arm_dir[1]) > 0.9 else [0.0, 1.0, 0.0]
    perp1 = _normalize(_cross(arm_dir, ref))
    perp2 = _normalize(_cross(arm_dir, perp1))

    # Rotate perp1 by roll angle in the perp1/perp2 plane
    roll_vec = [
        perp1[i] * math.cos(r) + perp2[i] * math.sin(r)
        for i in range(3)
    ]

    INDICATOR = 40  # mm, half-length of the crossbar
    roll_pos = [ext_tip[i] + roll_vec[i] * INDICATOR for i in range(3)]
    roll_neg = [ext_tip[i] - roll_vec[i] * INDICATOR for i in range(3)]

    return base, arm_tip, ext_tip, roll_pos, roll_neg

# =============================================================================
# PLOT SETUP
# =============================================================================

fig = plt.figure(figsize=(12, 8))
fig.patch.set_facecolor('#1a1a2e')

ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor('#1a1a2e')

reach = ARM_LENGTH + MAX_LINEAR + 60
ax.set_xlim(-reach, reach)
ax.set_ylim(-reach, reach)
ax.set_zlim(-reach/2, reach)
ax.set_xlabel("X (mm)", color='white')
ax.set_ylabel("Y — forward (mm)", color='white')
ax.set_zlabel("Z — up (mm)", color='white')
ax.tick_params(colors='white')
for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#333355')
ax.grid(True, color='#333355', linestyle='--', linewidth=0.5)

# Draw ground-plane circle at Z=0 for depth reference
theta = [i * 2 * math.pi / 60 for i in range(61)]
ax.plot([reach * math.cos(t) for t in theta],
        [reach * math.sin(t) for t in theta],
        [0] * 61, color='#333355', lw=0.8, linestyle=':')

# Base marker
ax.scatter([0], [0], [0], c='white', s=80, zorder=10, marker='s')

# Lines
(line_arm_tgt,)  = ax.plot([], [], [], color='#4488ff', lw=3,   label='Arm commanded')
(line_arm_act,)  = ax.plot([], [], [], color='#88ddff', lw=2,   linestyle='--', label='Arm actual')
(line_ext_tgt,)  = ax.plot([], [], [], color='#ff4444', lw=2,   label='Linear (commanded)')
(line_roll_tgt,) = ax.plot([], [], [], color='#44ff88', lw=2.5, label='Roll indicator (cmd)')
(line_roll_act,) = ax.plot([], [], [], color='#ffff44', lw=1.5, linestyle='--', label='Roll indicator (act)')

# Tip markers
(pt_ext_tgt,)  = ax.plot([], [], [], 'o', color='#ff4444', ms=8)
(pt_arm_act,)  = ax.plot([], [], [], 'o', color='#88ddff', ms=6)

leg = ax.legend(loc='upper left', fontsize=8, framealpha=0.3,
                labelcolor='white', facecolor='#1a1a2e')

title_obj = ax.set_title("Connecting to ESP32...", color='white', fontsize=11, pad=12)

# Info text box
info_text = ax.text2D(0.02, 0.02, "", transform=ax.transAxes,
                      color='white', fontsize=8, va='bottom',
                      bbox=dict(facecolor='#1a1a2e', alpha=0.7, edgecolor='#444'))

def _set_line(line, p0, p1):
    line.set_data_3d([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])

def _set_point(pt, p):
    pt.set_data_3d([p[0]], [p[1]], [p[2]])


def update(frame):
    with state_lock:
        pt  = state["pitch_tgt"]
        pa  = state["pitch_act"]
        rt  = state["roll_tgt"]
        ra  = state["roll_act"]
        lm  = state["linear_mm"]
        status = state["status"]

    # Commanded arm
    b, tip_t, ext_t, rp_t, rn_t = arm_points(pt, rt, lm)
    _set_line(line_arm_tgt,  b,    tip_t)
    _set_line(line_ext_tgt,  tip_t, ext_t)
    _set_line(line_roll_tgt, rn_t, rp_t)
    _set_point(pt_ext_tgt, ext_t)

    # Actual arm (pitch/roll from encoders; linear is open-loop so same lm)
    b, tip_a, ext_a, rp_a, rn_a = arm_points(pa, ra, lm)
    _set_line(line_arm_act,  b,    tip_a)
    _set_line(line_roll_act, rn_a, rp_a)
    _set_point(pt_arm_act, tip_a)

    title_obj.set_text(
        f"Pitch  cmd={pt:+.1f}°  act={pa:+.1f}°  err={pt-pa:+.1f}°   |   "
        f"Roll  cmd={rt:+.1f}°  act={ra:+.1f}°  err={rt-ra:+.1f}°   |   "
        f"Linear  {lm:.1f} / {MAX_LINEAR:.0f} mm"
    )

    info_text.set_text(
        f"Status: {status}\n"
        f"Geometry: arm {ARM_LENGTH}mm + linear {lm:.0f}mm\n"
        f"Drag to rotate view  |  Scroll to zoom"
    )

    return (line_arm_tgt, line_arm_act, line_ext_tgt,
            line_roll_tgt, line_roll_act, pt_ext_tgt, pt_arm_act)


ani = animation.FuncAnimation(fig, update, interval=50, blit=False)  # 20 Hz

# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  Robot Arm Visualizer")
    print(f"  Serial port: {SERIAL_PORT}")
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

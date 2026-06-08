# SALAI Embedded/Control — Project Context

## What this project is

DIY 3-axis robot arm controlled by an ESP32 running MicroPython. Used in a surgical/medical robotics context (SALAI). The main controller file is `armcontrol.py` (runs on the ESP32). Full architecture, bug history, and hardware notes are in `primary_robot_arm_handoff.md` — read that first for deep context.

## Key files

| File | Purpose |
|------|---------|
| `armcontrol.py` | ESP32 MicroPython controller — runs on the device, do not run on Mac |
| `arm_visualizer.py` | Mac-side 3D digital twin visualizer — runs on the Mac |
| `primary_robot_arm_handoff.md` | Full architecture, hardware notes, bug history, tuning guide |
| `secondary_surgical_robot_handoff.md` | Secondary robot handoff doc |

## Python environment

There is a venv at `./venv/`. Always use it:

```bash
cd /Users/bennett/Documents/MAR2026SALAI
source venv/bin/activate
```

In VS Code: select interpreter at `./venv/bin/python3`. The venv has `pyserial` and `matplotlib` installed.

## arm_visualizer.py — what it does and why

Bennett was having trouble separating input/controller issues from mechanical arm issues. The visualizer solves this by reading the ESP32's existing serial debug output (no changes to `armcontrol.py` needed) and rendering a live 3D stick-figure of the arm's commanded vs actual position.

**How it works:** `armcontrol.py` already prints a debug line at 10Hz via serial USB:
```
| PITCH: Tgt:  5.2 Act:  4.8 Err: 0.4 | ROLL: Tgt: ... | LINEAR:  45.2mm / 175mm
```
The visualizer connects to the serial port, strips ANSI escape codes, parses this line, and animates a matplotlib 3D plot.

**To run it:**
1. Make sure `armcontrol.py` is running on the ESP32
2. In a terminal with the venv active:
   ```bash
   cd /Users/bennett/Documents/MAR2026SALAI
   python3 arm_visualizer.py
   ```
3. The serial port is set at the top of the file (`SERIAL_PORT`). Current value: `/dev/cu.usbserial-21140`. Run `ls /dev/cu.*` to find the right port if it changes.

**Known limitation:** `debug_print` in `armcontrol.py` is only called from `handle_joystick()`, not from `handle_jogging()`. The visualizer goes static while the trim joystick is active. Easy to fix by also calling `debug_print` from `handle_jogging()` if this becomes annoying.

**Geometry assumptions (may need adjusting once physically verified):**
- Pitch tilts the arm in the vertical plane
- Roll rotates around the arm's own long axis (shown as a crossbar indicator at the tip)
- Linear extends the end effector along the arm direction
- `PITCH_SIGN` and `ROLL_SIGN` at the top of the file can be set to `-1` to flip a direction

## Current debugging focus

Bennett is trying to determine whether observed jitter/jerk is coming from the input stack (joystick/encoder) or from the arm mechanics (suspected: encoder mount on roll axis). The visualizer is the primary tool for this — if the visualizer shows jitter with the arm disconnected, it's the input stack.

## Handoff doc discrepancy (noted)

`primary_robot_arm_handoff.md` §2 GPIO table describes an older version with rocker switches on GPIO 16–19. The actual running code uses a trim joystick (GPIO 32/33) and optical encoder (GPIO 16/17). The handoff doc needs updating to reflect this.

# SALAI Embedded/Control — Project Context

## What this project is

DIY 3-axis robot arm controlled by an ESP32 running MicroPython. Used in a surgical/medical robotics context (SALAI). The main controller file is `armcontrol.py` (runs on the ESP32). Full architecture, bug history, and hardware notes are in `primary_robot_arm_handoff.md` — read that first for deep context.

## Key files

| File | Purpose |
|------|---------|
| `armcontrol.py` | ESP32 MicroPython controller — runs on the device, do not run on Mac |
| `axis_trace.py` | **Mac-side primary visualizer** — scrolling pitch/roll/linear/input trace; writes `logs/axis_*.csv`. This is what `arm-deploy` launches. |
| `pitch_sweep.py` | One-off **open-loop pitch motor-adequacy test** (ESP32). Hand-calibrates ROM, then sweeps pitch at constant step rate (no PID) so missed steps/stall = torque limit. Flashed via `sweep-deploy`. |
| `pitch_trace.py` | Older Mac-side pitch/roll-only scrolling trace (not launched by deploy) |
| `arm_visualizer.py` | **DEPRECATED** 3D digital-twin visualizer — no longer used, kept for reference only |
| `armcontrol_refactor_plan.md` | **Planned** structural refactor of `armcontrol.py` (split into `sensors.py`/`motors.py`, then an `Axis` class). Behavior-preserving, do before the demo sprint. Not started. |
| `primary_robot_arm_handoff.md` | Full architecture, hardware notes, bug history, tuning guide |
| `secondary_surgical_robot_handoff.md` | Secondary robot handoff doc |
| `esp32_pinout.md` | **Authoritative pin-level wiring reference**, both ESP32 boards (arm + instrument) — pull-up/cap values, strapping-pin cautions, spare GPIO map |
| `esp32_arm_mermaid.md` | Mermaid wiring diagram for the arm controller, mirrors `esp32_pinout.md` |
| `estop_ic_wiring_guide.md` | Build guide for a *planned* hardware e-stop gate (74HC04/74HC08) — not yet built, gates STEP lines without cutting motor ENABLE |

`arm-deploy` is a zsh function (in `~/.zshrc`) that copies `armcontrol.py`→`main.py` and `config.py` to the ESP32 via `mpremote`, resets, then launches `python3 axis_trace.py`.

`sweep-deploy` (also in `~/.zshrc`) is the same pattern but flashes `pitch_sweep.py` as `main.py` for the open-loop pitch motor test — then `arm-deploy` restores the normal firmware. Both use `cp + reset` (not `mpremote run`) on purpose: `reset` releases the serial port so `axis_trace.py` can read/log it. `mpremote run` holds the port and blocks logging.

## Python environment

There is a venv at `./venv/`. Always use it:

```bash
cd /Users/bennett/Documents/MAR2026SALAI
source venv/bin/activate
```

In VS Code: select interpreter at `./venv/bin/python3`. The venv has `pyserial` and `matplotlib` installed.

## axis_trace.py — what it does and why (PRIMARY TOOL)

Bennett was having trouble separating input/controller issues from mechanical arm issues. `axis_trace.py` solves this by reading the ESP32's serial debug output (no special protocol — just the existing prints) and rendering live scrolling traces. **This is the chart that pops up when you run `arm-deploy`**, and it writes the `logs/axis_*.csv` files used for PID analysis.

**Four stacked subplots:**
1. Pitch — commanded vs actual (+ sinusoid fit for orbit sanity)
2. Roll — commanded vs actual (+ sinusoid fit)
3. Linear — commanded vs actual position (mm)
4. **Input** — trim joystick raw ADC (rawX/rawY) over time, with deadband band and a live numeric readout (raw, scaled, IMU). Use this to confirm the ESP32 is actually receiving joystick signals: if the lines don't move when you wiggle the stick, it's wiring, not code.

**How it works:** `armcontrol.py` prints two line types via serial USB at 10Hz:
```
| PITCH: Tgt:  5.2 Act:  4.8 Err: 0.4 | ROLL: ... | LINEAR: Cmd: 45.2mm Act: 44.8mm / 175mm | TRIM: P: 0.00 R: 0.00
INPUT | TRIM rawX:2048 rawY:2050 sclX:+0.00 sclY:+0.00 | IMU p:  +1.2 r:  -0.3
```
`axis_trace.py` strips ANSI codes, matches each line with `LINE_RE` / `INPUT_RE`, and animates matplotlib. `print_input_debug()` (the INPUT line) is called from both the pre-demo jog loop and the main loop, so the input subplot stays live even during active jogging (when `debug_print` is skipped).

**To run it:** just use `arm-deploy` (handles flashing + launch). Standalone: `python3 axis_trace.py` with the venv active. Serial port is `SERIAL_PORT` at the top (`/dev/cu.usbserial-21140`; `ls /dev/cu.*` to find it).

## Current debugging focus

Bennett is trying to determine whether observed jitter/jerk is coming from the input stack (joystick/encoder) or from the arm mechanics (suspected: encoder mount on roll axis). The Input subplot in `axis_trace.py` is the primary tool for this — if the raw ADC jitters with the arm disconnected, it's the input stack.

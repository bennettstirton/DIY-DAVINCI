# armcontrol.py Refactor Plan

**Status:** planned, not started. **Scope:** `armcontrol.py` only (the file that flashes to the ESP32 as `main.py`).

## Why we're doing this

`armcontrol.py` is a ~1,467-line monolith. The arm is **not yet working**, and a heavy
build/debug/integration sprint is coming before the demo. A refactor is an investment that
pays back over future development — so doing it *before* the sprint sharpens the axe before
we chop. We split the file so axis work during the sprint is fast to navigate and cheap to change.

Out of scope: `pitch_sweep.py` and other debug scripts. They're throwaway tooling — we leave
them alone (even though they duplicate some driver code).

## The one discipline that must hold

**This refactor is behavior-preserving. It changes *structure*, never *behavior*.**

- The proof each step is clean: the arm behaves **identically to before** in `axis_trace.py`
  — same angles, same `FREQ:` values, same `INPUT` line, same serial output.
- The arm is broken right now, so "identical" may mean "identically broken." **That's fine** —
  it proves we only moved code, didn't change logic.
- **Do not fix bugs during the refactor.** Finish the refactor, verify identical behavior,
  *then* start the fixing sprint on the clean structure. Mixing the two makes regressions
  impossible to bisect.
- Commit every phase separately so any regression reverts to the last good flash.

## Target architecture

One-directional imports, no cycles (an arrow means "imports"):

```
config.py   mcp23017.py        (leaves — unchanged)
   ^   ^        ^
sensors.py    motors.py        (new — flat function modules, import only config/mcp23017)
   ^              ^
        armcontrol.py          (main.py: ISRs, control loop, demo/jog, startup)
```

| File | Role |
|------|------|
| `config.py` | **Unchanged.** All pins, constants, PID gains. |
| `mcp23017.py` | **Unchanged.** Limit-switch GPIO-expander driver. |
| `sensors.py` *(new)* | Owns the `i2c` object. AS5600 reads, IMU reads, TCA mux select. |
| `motors.py` *(new)* | Owns the 3 PWM + 3 dir pins. PWM/dir drive helpers. |
| `armcontrol.py` | Stays `main.py`. ISRs, control layer, demo/jog, startup + 40 ms loop. |

## Phases (each flashed + verified before the next)

**Verification standard, every phase:** `arm-deploy`, wiggle the trim joystick + tilt the IMU,
confirm `axis_trace.py` shows behavior *identical* to the previous commit. Any deviation →
revert that phase only.

### Phase 0 — Update deploy scripts (B)
Edit `arm-deploy` in `~/.zshrc` to copy the new modules to the device. Add
`cp sensors.py :sensors.py +`, `cp motors.py :motors.py +`, and `cp mcp23017.py :mcp23017.py +`
to the `mpremote` chain (keep the existing `armcontrol.py→main.py` and `config.py` lines).
Flash the **unchanged** firmware first to prove the deploy path works before anything depends on it.

### Phase 1 — Extract `motors.py` (B, lowest risk)
Move `set_motor`, `stop_motor`, `stop_linear_motor`, `wrap_angle_error`, and the 3 PWM + 3 dir
Pin objects into `motors.py` (`from config import *` at top). In `armcontrol.py`, import the
names directly (`from motors import set_motor, pitch_pwm, ...`) so the `_estop_isr` and the
`try/finally` deinit keep bare-global references to the PWM objects (see Gotchas). Pure I/O,
no shared control state. **Flash + verify.**

### Phase 2 — Extract `sensors.py` (B)
Move `tca_select`, `_read_as5600_raw`, `read_pitch_angle_deg`, `read_roll_angle_deg`,
`update_linear_encoder_mm` (+ its `_linear_as5600_*` module state), `zero_linear_encoder`,
`imu_init`, `read_imu_angles`, `read_imu_commands`, and the `i2c` object into `sensors.py`.
Update `main()`'s startup I2C scan to reuse `sensors.i2c` — **do not create a second SoftI2C.**
Import the read functions by name into `armcontrol`. **Flash + verify.**

### Phase 3 — `Axis` class for pitch/roll PID (C, isolated, last)
The pitch and roll control logic are near-identical twins, distinguished only by a `pitch_`/
`roll_` prefix on ~16 globals each. Collapse them into one `Axis` class holding per-axis state
(`target`, `home`, `integral`, `last_error`, `last_actual`, `current_freq`, `last_forward`,
`decelerating`, `ema_joy`, plus its gains/clamp/deadband/max-freq/invert/pwm/dir-pin) and a
`run_pid(dt)` method. Instantiate `pitch = Axis(...)`, `roll = Axis(...)`. Rewrite `_run_pid`
to call them while **preserving exact cross-axis behavior** (a roll-read failure still stops
*both* axes; pitch early-return ordering unchanged). Update sites that poke per-axis state
(`handle_jogging`, `clear_axis_faults`, startup home-capture, e-stop re-arm resync) to use
`pitch.`/`roll.` attributes.

This is the only phase touching the control core, so it's done alone — a regression bisects to
exactly this commit. **Flash + verify carefully:** jog both axes full range, run the demo orbit
(sinusoid fit stays clean), trip + re-arm e-stop, trip + clear an axis crash fault.

This `Axis` class becomes the home for *future* per-axis features during the sprint.

## MicroPython gotchas to honor

1. **ISRs stay module-level in `armcontrol.py` and must not allocate.** Keep `_estop_isr` and
   `_encoder_irq` as module functions; `_estop_isr` holds direct PWM references (bare-global
   lookup), not `motors.pitch_pwm` attribute chains.
2. **Keep `_encoder_irq` with its pins + state** (`encoder_a/b`, `_enc_state`, `encoder_count`)
   together in `armcontrol.py`.
3. **Strict layering, no cycles:** `sensors`/`motors` import only `config`/`mcp23017` — never
   each other or `armcontrol`. `home_linear_axis` (needs a motor *and* `zero_linear_encoder`)
   stays in `armcontrol`, the layer allowed to depend on both.
4. **Serial output byte-identical.** `debug_print`/`print_input_debug` stay in `armcontrol`
   unchanged — `axis_trace.py`'s regexes anchor on the exact format strings and ANSI codes.
5. **Named imports in the 40 ms loop** (`from sensors import read_pitch_angle_deg`) — bare-global
   lookups are faster than `sensors.x` attribute access per tick.
6. **One `SoftI2C` owner** (`sensors.py`); startup scan reuses it.

## Do NOT change (churn = risk, no payoff)
- `config.py`, `mcp23017.py`.
- PID math/gains/deadbands/integral clamps/decel ramps — extraction must not "tidy" them.
- Cross-axis coupling and statement ordering in `_run_pid`.
- All ISRs + their `.irq()` registration; the `disable_irq`/`enable_irq` critical section in
  `handle_encoder_linear`; the e-stop poll / re-arm resync block in `main()`.
- `debug_print` / `print_input_debug` format strings.
- `home_linear_axis` homing/backoff logic.
- `try / main() / finally: deinit()` bottom block.

## Files involved
- `armcontrol.py` (split source) · `sensors.py` *(new)* · `motors.py` *(new)*
- `axis_trace.py` (verification tool — read-only)
- `~/.zshrc` (`arm-deploy`)

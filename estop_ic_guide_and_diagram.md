# Hardware E-Stop Gate — Guide & Perfboard Diagram

Stops all stepper STEP signals in hardware without cutting ENABLE.
Coils stay energized → motors hold position → arm doesn't drop under gravity.

Companion file: `estop_ic_perfboard_layout.svg` — visual perfboard hole grid matching
the coordinates in this doc.

## Why this circuit, not ENABLE pins

| Approach | Coil current on stop? | Safe? |
|---|---|---|
| Cut ENABLE | No — coils go limp | Arm drops under gravity |
| Gate STEP signals | Yes — coils stay energized | Arm holds last position |

---

## Parts needed (from your 74HCxx assortment)

| Qty | Part | Role |
|-----|------|------|
| 1 | **74HC04** (hex inverter) | Converts active-LOW e-stop signal to active-HIGH gate enable |
| 1 | **74HC08** (quad AND gate) | Gates 3 STEP lines per arm (1 chip handles 1 arm; 4 arms need 3 chips) |
| 1 | 10 kΩ resistor | Pull-up on e-stop signal line (optional — see below) |
| — | 3.3V supply | From ESP32 3V3 pin |

> **74HCT variants work too** — 74HCT04 / 74HCT08 accept 3.3V inputs on a 5V supply, useful if your motor drivers need 5V logic levels. For now, everything here assumes 3.3V throughout.

---

## Signal names

| Name | Logic level | Meaning |
|------|-------------|---------|
| `ESTOP_N` | Active LOW | T-junction on the existing NC→GPIO4 wire (shared node) |
| `ESTOP` | Active HIGH | Output of the 74HC04 inverter; HIGH = running, LOW = motors gate closed |

> **The NO terminal is not used in this circuit.** Only the NC contact provides fail-safe behavior. A broken NO wire would leave the gate open (motors run uncontrolled). A broken NC wire produces the safe state (gate closed = motors stopped).

---

## Perfboard placement & wire list

Physical build reference: which hole each pin/wire lands on a standard 0.1in
(2.54mm) pitch perfboard. See `estop_ic_perfboard_layout.svg` for the visual.

**The tables below are the authoritative build reference — the SVG is a visual aid.**
Wires are insulated hookup wire; they can run over or near other pads without
shorting. Only the listed endpoints matter electrically.

### Coordinate system

Columns 1–24 numbered left→right, rows A–N lettered top→bottom, each step = 0.1in.
Count holes from your board's top-left corner to find a coordinate, e.g. "row D,
col 4" = 4 holes right, 3 holes down from the corner hole (row A = row 1).

### Component placement

| Component | Position | Notes |
|---|---|---|
| U1 — 74HC04 | Pins straddle rows D and G, columns 4–10 | Pin 1 at D4 (top-left), notch toward column 1 |
| U2 — 74HC08 | Pins straddle rows D and G, columns 14–20 | Pin 1 at D14 (top-left), notch toward column 1 |
| 3.3V rail | Bus wire along row A, col 1–24 | Solder one continuous wire (or copper rail strip) here first |
| GND rail | Bus wire along row N, col 1–24 | Same — do this before placing ICs |
| J1 (ESTOP_N junction) | Row C, col 2 | Free hole, not a component — just a solder junction for 3 wires |
| Pull-up resistor (optional) | Between J1 (C2) and 3.3V rail (A2) | 10kΩ. Skip it — ESP32's internal pull-up on GPIO4 already covers this. |

### U1 — 74HC04 pin map

| Pin | Hole | Function | Connects to |
|---|---|---|---|
| 1 | D4 | 1A in | J1 (C2) — ESTOP_N |
| 2 | D5 | 1Y out | ESTOP bus → U2 pins 2, 5, 10 |
| 3 | D6 | 2A (unused in) | tie to pin 7 (D10) |
| 4 | D7 | 2Y (unused out) | leave floating |
| 5 | D8 | 3A (unused in) | tie to pin 7 (D10) |
| 6 | D9 | 3Y (unused out) | leave floating |
| 7 | D10 | GND | GND rail (N10) |
| 8 | G10 | 4Y (unused out) | leave floating |
| 9 | G9 | 4A (unused in) | GND rail (N9) |
| 10 | G8 | 5Y (unused out) | leave floating |
| 11 | G7 | 5A (unused in) | GND rail (N7) |
| 12 | G6 | 6Y (unused out) | leave floating |
| 13 | G5 | 6A (unused in) | GND rail (N5) |
| 14 | G4 | VCC | 3.3V rail (A4) |

### U2 — 74HC08 pin map

| Pin | Hole | Function | Connects to |
|---|---|---|---|
| 1 | D14 | 1A in | GPIO14 lead (Pitch STEP from ESP32) |
| 2 | D15 | 1B in | ESTOP bus (from U1 pin 2) |
| 3 | D16 | 1Y out | DM556T STEP lead (Pitch driver) |
| 4 | D17 | 2A in | GPIO26 lead (Roll STEP from ESP32) |
| 5 | D18 | 2B in | ESTOP bus (from U1 pin 2) |
| 6 | D19 | 2Y out | TB6600 STEP lead (Roll driver) |
| 7 | D20 | GND | GND rail (N20) |
| 8 | G20 | 3Y out | TB6600 STEP lead (Linear driver) |
| 9 | G19 | 3A in | GPIO13 lead (Linear STEP from ESP32) |
| 10 | G18 | 3B in | ESTOP bus — same column as pin 5, short vertical jumper D18→G18 |
| 11 | G17 | 4Y (unused out) | leave floating |
| 12 | G16 | 4A (unused in) | GND rail (N16) |
| 13 | G15 | 4B (unused in) | GND rail (N15) |
| 14 | G14 | VCC | 3.3V rail (A14) |

### Full wire list (every solder connection)

| # | From | To | Net | Notes |
|---|---|---|---|---|
| 1 | E-STOP terminal A | GND rail (N3, or any rail hole) | GND | |
| 2 | E-STOP terminal B | J1 (C2) | ESTOP_N | same wire as existing GPIO4 tap |
| 3 | ESP32 GPIO4 | J1 (C2) | ESTOP_N | existing wire, just land it on J1 instead of a bare splice |
| 4 | J1 (C2) | U1 pin 1 (D4) | ESTOP_N | |
| 5 | (optional) J1 (C2) | 3.3V rail (A2) | pull-up | 10kΩ resistor, skip if relying on internal pull-up |
| 6 | U1 pin 14 (G4) | 3.3V rail (A4) | VCC | |
| 7 | U1 pin 7 (D10) | GND rail (N10) | GND | |
| 8 | U1 pin 3 (D6) | U1 pin 7 (D10) | GND | unused input tie |
| 9 | U1 pin 5 (D8) | U1 pin 7 (D10) | GND | unused input tie |
| 10 | U1 pin 9 (G9) | GND rail (N9) | GND | unused input tie |
| 11 | U1 pin 11 (G7) | GND rail (N7) | GND | unused input tie |
| 12 | U1 pin 13 (G5) | GND rail (N5) | GND | unused input tie |
| 13 | U1 pin 2 (D5) | U2 pin 2 (D15) | ESTOP bus | |
| 14 | U1 pin 2 (D5) | U2 pin 5 (D18) | ESTOP bus | daisy-chain from #13 or run separately |
| 15 | U2 pin 5 (D18) | U2 pin 10 (G18) | ESTOP bus | short vertical jumper, same column |
| 16 | U2 pin 14 (G14) | 3.3V rail (A14) | VCC | |
| 17 | U2 pin 7 (D20) | GND rail (N20) | GND | |
| 18 | U2 pin 12 (G16) | GND rail (N16) | GND | unused input tie |
| 19 | U2 pin 13 (G15) | GND rail (N15) | GND | unused input tie |
| 20 | ESP32 GPIO14 | U2 pin 1 (D14) | STEP in | cut the old direct GPIO14→DM556T wire |
| 21 | U2 pin 3 (D16) | DM556T STEP input | STEP out | |
| 22 | ESP32 GPIO26 | U2 pin 4 (D17) | STEP in | cut the old direct GPIO26→TB6600 wire |
| 23 | U2 pin 6 (D19) | TB6600 (Roll) STEP input | STEP out | |
| 24 | ESP32 GPIO13 | U2 pin 9 (G19) | STEP in | cut the old direct GPIO13→TB6600 wire |
| 25 | U2 pin 8 (G20) | TB6600 (Linear) STEP input | STEP out | |

---

## Build order

1. **Prep the e-stop signal tap.** The NC terminal wire currently runs to ESP32 GPIO4. Add a T-junction anywhere on that wire: one leg stays on GPIO4 (existing, unchanged), the other leg lands on J1. The ESP32's internal pull-up on GPIO4 is already in place — the 10kΩ external pull-up is optional belt-and-suspenders. **Do not connect the NO terminal to this circuit.** Leave it unused.
2. Solder the two rail bus wires first (row A = 3.3V, row N = GND) — do this before placing ICs so you have full access to every hole.
3. Socket U1 at D4–D10/G4–G10, U2 at D14–D20/G14–G20. Double-check pin 1 orientation (notch toward column 1) on both before soldering.
4. Wire #6–19 (power + unused-pin ties) — get both chips fully powered and quiet before touching signal wires.
5. Wire #1–5 (e-stop tap + junction + optional pull-up).
6. Wire #13–15 (ESTOP bus).
7. Wire #20–25 (STEP routing) — **cut the old direct ESP32→driver STEP wires first**, then land them on the gate inputs instead.
8. Verify per the fail-safe table below before reconnecting motor power.

---

## Scaling to 4 arms

Each arm adds 3 more STEP lines = 3 more AND gates.

| Arms | AND gates needed | 74HC08 chips | 74HC04 chips |
|------|-----------------|--------------|--------------|
| 1 | 3 | 1 | 1 |
| 2 | 6 | 2 | 1 |
| 3 | 9 | 3 | 1 |
| 4 | 12 | 3 | 1 |

One 74HC04 output can drive all 12 AND gate B inputs — CMOS outputs can sink/source ~25 mA, and each CMOS input draws <1 µA. No buffer needed.

**Wiring pattern for multiple arms:**
- ESTOP_N (NC contact) → single 74HC04 pin 1
- 74HC04 pin 2 (ESTOP) → daisy-chain to all 74HC08 B inputs across all chips
- Each arm's ESP32 STEP outputs → corresponding A inputs on their chip
- Each chip's Y outputs → that arm's motor driver STEP inputs

---

## Fail-safe verification

After building, verify each failure mode:

| Condition | ESTOP_N | 74HC04 out | AND output | Motors |
|-----------|---------|-----------|------------|--------|
| Normal running | LOW (NC closed) | HIGH | Passes STEP | Running |
| E-stop pressed | HIGH (NC open) | LOW | Blocked (0) | Stopped, holding |
| E-stop wire breaks | HIGH (pull-up) | LOW | Blocked (0) | Stopped, holding ✓ |
| 74HC04 loses power | — | — | AND input B floats LOW | Stopped ✓ |

> **Test this** before trusting it: with arm running, disconnect the NC terminal B wire. Motors should stop immediately. Reconnect — motors should resume on next STEP pulse.

---

## Notes

- **DIR signals don't need gating** — direction only matters while STEP pulses are active.
- **ENABLE pins** — leave them connected to ESP32 as-is (or tie ENABLE LOW = always enabled). Don't use them for e-stop.
- **74LS vs 74HC** — prefer 74HC for 3.3V operation. 74LS series needs 5V supply and its output LOW level (~0.3V) is fine as an input to HC gates, but its HIGH level (~2.5V) is marginal for 3.3V HC inputs. Stick to HC/HCT if you have the choice.

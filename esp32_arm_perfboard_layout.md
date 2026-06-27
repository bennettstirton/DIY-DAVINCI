# Arm Controller ESP32 — Perfboard Placement & Wire List

Physical build reference for `armcontrol.py` / `config.py`'s GPIO wiring on a
standard 0.1in (2.54mm) pitch perfboard. See `esp32_arm_perfboard_layout.svg`
for the visual. Board: **DOIT ESP32 DevKit V1, 30-pin** — physical pin order
verified against two independently-drawn prior diagrams in this repo.
**Verify your specific board's pin order and row spacing against its
silkscreen before drilling/soldering — clone boards vary.**

**The tables below are the authoritative build reference — the SVG is a visual aid.**
Wires are insulated hookup wire; they can run near other pads without shorting.
Only the listed endpoints matter electrically. Steppers, sensors, joystick,
encoder, and switches are **remote/panel-mounted** — they reach this board via
cable, not soldered directly to it. Only the ESP32, the TCA9548A mux breakout,
and a handful of passives (pull-ups, debounce cap) live on this perfboard.

If you've built the hardware e-stop gate (`estop_ic_guide_and_diagram.md`),
GPIO4/14/26/13 route through that board instead of straight to the e-stop
switch / drivers — noted inline below.

## Coordinate system

Columns 1–36 numbered left→right, rows A–X lettered top→bottom, each step = 0.1in.

## Component placement

| Component | Position | Notes |
|---|---|---|
| ESP32 DevKit | Pins straddle rows D–R, columns 10 (left) and 19 (right) | Pin spacing between columns assumed 0.9in — **measure your board** |
| TCA9548A mux breakout | Control pins row T, channel pins row V, columns 22–28 | Generic breakout pinout — adjust columns to your module's silkscreen |
| 3.3V rail | Bus wire along row A, col 1–36 | Solder before placing the ESP32 |
| GND rail | Bus wire along row X, col 1–36 | Same |
| SCL pull-up (2.2kΩ) | Inline on the wire from ESP32 GPIO22 up to the 3.3V rail | |
| SDA pull-up (2.2kΩ) | Inline on the wire from ESP32 GPIO21 up to the 3.3V rail | |
| GPIO4 debounce cap (100nF) | Inline on the wire from ESP32 GPIO4 down to GND rail | |
| Encoder dividers (optional, 1kΩ+2kΩ ×2) | Near GPIO16/17 leads | **Only if your encoder runs on 5V** — skip for a 3.3V encoder |

## ESP32 pin map (DOIT DevKit V1, 30-pin)

| Row | Left pin | Function | Connects to | Right pin | Function | Connects to |
|---|---|---|---|---|---|---|
| D | 3V3 | regulated 3.3V out | 3.3V rail (A10) | VIN | 5V in | external 5V/USB supply lead |
| E | EN | reset | unused, no wire | 23 | free | unused — MCP23017 limit switches moved to I2C, INTA/INTB unwired |
| F | 36 (VP) | free, input-only | unused | 22 | I2C SCL | TCA9548A SCL + 2.2k pull-up + MPU6050 tap |
| G | 39 (VN) | free, input-only | unused | 1 (TX0) | USB serial | avoid — leave free |
| H | 34 | free, input-only | unused | 3 (RX0) | USB serial | avoid — leave free |
| I | 35 | free, input-only | unused | 21 | I2C SDA | TCA9548A SDA + 2.2k pull-up + MPU6050 tap |
| J | 32 | TRIM_JOY_X (pitch trim) | joystick X wiper lead | 19 | free | unused |
| K | 33 | TRIM_JOY_Y (roll trim) | joystick Y wiper lead | 18 | free | unused |
| L | 25 | ROLL_DIR | TB6600 Roll DIR lead | 5 | REARM | re-arm button (NO) → GND, **strapping pin** |
| M | 26 | ROLL_STEP | → e-stop gate → TB6600 Roll STEP | 17 | ENCODER_B | optical encoder ch.B lead (÷ if 5V) |
| N | 27 | PITCH_DIR | DM556T Pitch DIR lead | 16 | ENCODER_A | optical encoder ch.A lead (÷ if 5V) |
| O | 14 | PITCH_STEP | → e-stop gate → DM556T STEP | 4 | ESTOP | 100nF→GND here, then → e-stop gate J1, **strapping pin** |
| P | 12 | free | unused — **strapping pin, leave floating** | 0 | BOOT | onboard button, no wiring needed, **strapping pin** |
| Q | 13 | LINEAR_STEP | → e-stop gate → TB6600 Linear STEP | 2 | onboard LED | no wiring needed, **strapping pin** |
| R | GND | ground | GND rail (X10) | 15 | LINEAR_DIR | TB6600 Linear DIR lead, **strapping pin** |

## TCA9548A mux pin map (generic breakout, addr 0x70)

| Pin | Function | Connects to |
|---|---|---|
| VCC | power | 3.3V rail |
| GND | ground | GND rail |
| SDA | I2C data | ESP32 GPIO21 |
| SCL | I2C clock | ESP32 GPIO22 |
| A0 | addr bit 0 | GND rail (tie low → addr 0x70) |
| A1 | addr bit 1 | GND rail |
| A2 | addr bit 2 | GND rail |
| SD0/SC0 | channel 0 | AS5600 (roll), remote |
| SD1/SC1 | channel 1 | AS5600 (pitch), remote |
| SD2/SC2 | channel 2 | AS5600 (linear), remote |

MPU6050 IMU (0x68) is **not** behind the mux — it taps the main SDA/SCL lines
directly, in parallel with the TCA9548A, via its own cable run.

## MCP23017 GPIO expander pin map (limit/crash switches, addr 0x20)

Sits on the **main** SDA/SCL lines (not behind the TCA9548A mux), same as the
MPU6050 — its address (0x20) doesn't collide with the mux (0x70) or any AS5600
(0x36, mux-selected only). INTA/INTB are left unconnected — `armcontrol.py`
polls the chip once per 40ms tick instead of wiring an interrupt.

| Pin | Function | Connects to |
|---|---|---|
| VCC | power | 3.3V rail |
| GND | ground | GND rail |
| SDA | I2C data | ESP32 GPIO21 |
| SCL | I2C clock | ESP32 GPIO22 |
| A0 | addr bit 0 | GND rail (tie low → addr 0x20) — **see Waveshare note** |
| A1 | addr bit 1 | GND rail — **see Waveshare note** |
| A2 | addr bit 2 | GND rail — **see Waveshare note** |
| INTA, INTB | interrupt out | not connected |
| PA0 | linear retract/home limit | limit switch (NC) → GND, remote |
| PA1 | linear extend limit | limit switch (NC) → GND, remote |
| PA2 | pitch min limit | limit switch (NC) → GND, remote |
| PA3 | pitch max limit | limit switch (NC) → GND, remote |
| PA4 | roll min limit | limit switch (NC) → GND, remote |
| PA5 | roll max limit | limit switch (NC) → GND, remote |
| PA6, PA7, PB0-7 | free | unused — spare expander inputs |

All six switches use the MCP's internal pull-ups (`GPPU` registers set in
`mcp23017.py`) — no external pull-up resistors needed on the perfboard.

### Waveshare board variant (the board actually in use)

This build uses a **Waveshare MCP23017 board**, not a bare DIP/generic breakout
(`config.py` confirms it). Two things differ from the generic pinout above:

- **Address pins:** the Waveshare board sets A0/A1/A2 via **onboard jumpers/solder
  pads**, which default to all-low → **0x20** (exactly what `MCP23017_ADDR`
  expects). So wires **#17–19 below are optional/redundant** — leave the jumpers
  at default and you do not need to run A0/A1/A2 to the GND rail at all. Only wire
  them (or move a jumper) if you're relocating the address off 0x20.
- **Onboard I2C pull-ups:** some Waveshare units include their own SDA/SCL
  pull-ups in parallel with the perfboard's 2.2kΩ. That's harmless (slightly
  stronger total pull-up); only revisit if the bus acts flaky. Verify against the
  silkscreen, and confirm the screw terminals are labelled PA0–PA5 (some Waveshare
  boards label them GPA0–GPA5).

Net effect: the host-side connection is **4 wires** (VCC, GND, SDA, SCL), not 7.

## Full wire list (every on-board solder connection)

| # | From | To | Net | Notes |
|---|---|---|---|---|
| 1 | ESP32 3V3 (D-left) | 3.3V rail | VCC | |
| 2 | ESP32 GND (R-left) | GND rail | GND | |
| 3 | ESP32 GPIO22/SCL (F-right) | TCA9548A SCL | I2C | |
| 4 | ESP32 GPIO22/SCL (F-right) | 3.3V rail | pull-up | inline 2.2kΩ |
| 5 | ESP32 GPIO21/SDA (I-right) | TCA9548A SDA | I2C | |
| 6 | ESP32 GPIO21/SDA (I-right) | 3.3V rail | pull-up | inline 2.2kΩ |
| 7 | TCA9548A VCC | 3.3V rail | VCC | |
| 8 | TCA9548A GND | GND rail | GND | |
| 9 | TCA9548A A0 | GND rail | addr | |
| 10 | TCA9548A A1 | GND rail | addr | |
| 11 | TCA9548A A2 | GND rail | addr | |
| 12 | ESP32 GPIO4/ESTOP (O-right) | GND rail | debounce | inline 100nF cap |
| 13 | MCP23017 VCC | 3.3V rail | VCC | |
| 14 | MCP23017 GND | GND rail | GND | |
| 15 | MCP23017 SDA | ESP32 GPIO21/SDA (I-right) | I2C | shares the SDA pull-up at #5/#6 |
| 16 | MCP23017 SCL | ESP32 GPIO22/SCL (F-right) | I2C | shares the SCL pull-up at #3/#4 |
| 17 | MCP23017 A0 | GND rail | addr | **optional on Waveshare** — onboard jumper defaults low (0x20); skip unless relocating address |
| 18 | MCP23017 A1 | GND rail | addr | **optional on Waveshare** — see #17 |
| 19 | MCP23017 A2 | GND rail | addr | **optional on Waveshare** — see #17 |

Everything else is an **external lead** — a wire leaving the board to a
remote/panel component, not a board-to-board solder joint. See the pin map
table above for where each one lands.

## Build order

1. Solder the two rail bus wires first (row A = 3.3V, row X = GND).
2. Socket the ESP32 at rows D–R. Double check pin-1 (3V3/VIN row) orientation against your board's silkscreen before soldering — USB connector end should match row D.
3. Wire #1–2 (ESP32 power).
4. Socket the TCA9548A breakout at rows T/V. Wire #7–11 (mux power + address tie-low).
5. Wire #3–6 (I2C bus + pull-ups).
6. Wire #12 (e-stop debounce cap).
7. Socket the MCP23017 breakout. Wire #13-16 (power + I2C taps). On the Waveshare board, skip #17-19 — leave the address jumpers at default (0x20). On a generic breakout, add #17-19 to tie A0/A1/A2 low.
8. Land external leads per the pin map: steppers, joystick, encoder, switches, remote sensors. If the hardware e-stop gate is built, route GPIO4/14/26/13 to it per `estop_ic_guide_and_diagram.md` instead of straight to the switch/drivers.
9. Power up without motor PSU connected first; confirm 3.3V rail voltage and I2C bus scan (TCA9548A + 3× AS5600 + MPU6050 + MCP23017) before connecting drivers. `armcontrol.py` prints a boot-time MCP23017 health check (PA0-5 state) — all six should read `0` at rest.

---

## Strapping-pin cautions (matter once soldered)

GPIO **0, 2, 5, 15** are ESP32 strapping pins — their level at power-on affects boot:
- **GPIO0** must be HIGH at boot (BOOT button released) to run normally — fine as-is.
- **GPIO5** must be HIGH at boot — the NO re-arm button leaves it pulled HIGH at rest, OK. Don't hold re-arm during power-up.
- **GPIO15** must be HIGH at boot — it's `LINEAR_DIR` (output). Should be fine, but if the board won't boot with the driver attached, this is the first suspect (add a pull-up).
- **GPIO2** is the onboard LED — harmless.
- **GPIO12** also affects flash voltage at boot — leave floating/unused (already the case here).

## Spare GPIO map (arm board)

Used: 0, 2, 4, 5, 13, 14, 15, 16, 17, 21, 22, 25, 26, 27, 32, 33.
Free for future expansion: **18, 19, 23**, plus input-only **34, 35, 36, 39** (no internal pull-up — add external pull-ups if used as digital inputs).
Avoid for new outputs: 6–11 (flash), 34–39 (input-only).

GPIO23 was `LINEAR_LIMIT` before the limit switches moved to the MCP23017 expander — now free.

## Power notes

- ESP32 from USB or regulated **5V → VIN**.
- **3.3V rail** → AS5600 ×3, TCA9548A, MPU6050, joystick pots, I2C pull-ups — all remote devices tap this rail via their cable runs; any point on the rail is electrically equivalent.
- Stepper **motor supply** (24–48V typ.) → drivers only; **common GND** between driver supply and ESP32.

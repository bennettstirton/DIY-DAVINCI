# SALAI — ESP32 Pinout / Wiring Reference (Instrument Controller)

Bench reference for moving off the breadboard to perfboard.

> The **Arm Controller** ESP32 (`armcontrol.py` + `config.py`) has its own
> dedicated perfboard build reference now: see `esp32_arm_perfboard_layout.md`
> and `esp32_arm_perfboard_layout.svg`. This file covers the **Instrument
> Controller** only.

This board shares GPIO numbers (16/17, 21/22) with the arm controller for
*different* functions, so the two cannot be combined onto one ESP32 without
remapping — keep them as two boards.

Pin facts are pulled directly from the code (`imu_instrumentcontrol.py`) as of
this writing — verify against the file if it changes.

---

# INSTRUMENT CONTROLLER ESP32 (`imu_instrumentcontrol.py`)

## 1.1 Servo bus — UART2 → Waveshare Bus Servo Adapter (A)

| GPIO | Signal | → Destination | Notes |
|------|--------|---------------|-------|
| 17 | UART2 TX | Waveshare TX | **TX→TX** (not crossover; adapter is half-duplex) |
| 16 | UART2 RX | Waveshare RX | **RX→RX** |
| — | GND | Waveshare GND | common ground |

- Baud **1,000,000**. Adapter jumper cap on the **A** position.
- STS3215-C047 servos on the bus by **ID**: 4 = pitch, 5 = roll, 6 = yaw/grip-1, 7 = yaw/grip-2.
- Adapter logic power = 5 V; **servo bus power = 12 V** (separate from ESP32). Common GND.

## 1.2 I2C bus — `I2C(0)`, SDA=21 SCL=22, 400 kHz

| GPIO | Signal | Notes |
|------|--------|-------|
| 21 | I2C SDA | pull-ups typically on the breakout boards |
| 22 | I2C SCL | |

| Device | Addr | Notes |
|--------|------|-------|
| BNO055 IMU | 0x28 | **ADR → GND** (0x29 if tied HIGH). VIN→3.3 V, GND→GND |
| AS5600 (jaw/finger) | 0x36 | fixed addr, no ADDR pin. VDD→3.3 V, GND→GND |

- No mux here — the two devices have distinct addresses, so they share the bus directly.
- A **clutch button** to re-capture home is mentioned in code as *not yet wired* — leave a spare GPIO + GND pad for it (e.g. GPIO 0 or any free input).

## 1.3 Power (instrument)
- ESP32 from USB or 5 V → VIN.
- **3.3 V** → BNO055, AS5600.
- **5 V** → Waveshare adapter logic.
- **12 V** → servo bus (Waveshare servo terminals). All grounds common.

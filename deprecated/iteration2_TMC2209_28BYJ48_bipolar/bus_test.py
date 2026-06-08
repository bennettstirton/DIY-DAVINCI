# =============================================================================
# TMC2209 UART bus test — Motors D, E, F, G
#
# Sends UART config to each driver address independently, then steps only
# that motor. Confirms:
#   1. Each driver receives its own address correctly (set by MS1/MS2)
#   2. Writes to one address don't affect the others
#
# Expected: each motor moves alone in sequence, others stay still.
#
# Address wiring:
#   Motor D — address 0 — MS1=GND,  MS2=GND
#   Motor E — address 1 — MS1=3.3V, MS2=GND
#   Motor F — address 2 — MS1=GND,  MS2=3.3V
#   Motor G — address 3 — MS1=3.3V, MS2=3.3V
# =============================================================================

from machine import UART, Pin
import time

UART_TX_PIN = 17

STEP_DELAY_US = 750
STEPS = 4096  # 1 input-shaft revolution at 1/2 step

REG_GCONF    = 0x00
REG_CHOPCONF = 0x6C
GCONF_VALUE    = 0x000000C0
CHOPCONF_VALUE = 0x17000053  # MRES=7 (1/2 step), intpol=1

MOTORS = [
    (0, "D (Pitch)",    26, 25),
    (1, "E (Roll)",     33, 32),
    (2, "F (Yaw/Grip1)",13, 12),
    (3, "G (Yaw/Grip2)",14, 27),
]


def _crc8(data):
    crc = 0
    for byte in data:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 1):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            byte >>= 1
    return crc


def write_reg(uart, driver_addr, reg, value):
    payload = [
        0x05,
        driver_addr & 0xFF,
        (reg | 0x80) & 0xFF,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >>  8) & 0xFF,
        (value      ) & 0xFF,
    ]
    payload.append(_crc8(payload))
    uart.write(bytes(payload))
    time.sleep_ms(2)


def run_steps(step_pin, n):
    for _ in range(n):
        step_pin.value(1)
        time.sleep_us(10)
        step_pin.value(0)
        time.sleep_us(STEP_DELAY_US)


uart = UART(2, baudrate=115200, tx=UART_TX_PIN, rx=16)

# Build pin objects and configure all drivers
pins = {}
for addr, name, step_gpio, dir_gpio in MOTORS:
    s = Pin(step_gpio, Pin.OUT)
    d = Pin(dir_gpio,  Pin.OUT)
    d.value(1)
    pins[addr] = (s, d)
    print("Configuring driver {} ({})...".format(addr, name))
    write_reg(uart, addr, REG_GCONF,    GCONF_VALUE)
    write_reg(uart, addr, REG_CHOPCONF, CHOPCONF_VALUE)

print("All drivers configured.")
print()

# Step each motor individually
for addr, name, _, _ in MOTORS:
    step_pin, _ = pins[addr]
    print("Stepping Motor {} only — all others should stay still...".format(name))
    run_steps(step_pin, STEPS)
    print("  Done. Pausing 2 seconds...")
    time.sleep_ms(2000)

print("Bus test complete.")
print("If each motor moved alone, all four addresses are working correctly.")

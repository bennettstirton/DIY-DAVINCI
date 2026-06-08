# =============================================================================
# TMC2209 UART — 1/2 step interpolation binary test
# Run 1: 1/2 step, interpolation OFF
# Run 2: 1/2 step, interpolation ON
# Both at 200us delay. Compare smoothness between the two runs.
# =============================================================================

from machine import UART, Pin
import time

# --- Pins ---
UART_TX_PIN = 17
STEP_PIN    = 26
DIR_PIN     = 25

STEP_DELAY_US = 750  # confirmed reliable at 1/2 step via speed_test.py
STEPS = 4096         # 1 input-shaft revolution at 1/2 step

# --- Registers ---
REG_GCONF    = 0x00
REG_CHOPCONF = 0x6C
GCONF_VALUE  = 0x000000C0

CHOPCONF_NO_INTERP = 0x07000053  # MRES=7 (1/2 step), intpol=0
CHOPCONF_INTERP    = 0x17000053  # MRES=7 (1/2 step), intpol=1


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
step = Pin(STEP_PIN, Pin.OUT)
dirp = Pin(DIR_PIN,  Pin.OUT)
dirp.value(1)

write_reg(uart, 0, REG_GCONF, GCONF_VALUE)

print("Run 1: 1/2 step, NO interpolation")
write_reg(uart, 0, REG_CHOPCONF, CHOPCONF_NO_INTERP)
time.sleep_ms(10)
run_steps(step, STEPS)
print("Done. Pausing 2 seconds...")
time.sleep_ms(2000)

print("Run 2: 1/2 step, interpolation ON")
write_reg(uart, 0, REG_CHOPCONF, CHOPCONF_INTERP)
time.sleep_ms(10)
run_steps(step, STEPS)
print("Done.")

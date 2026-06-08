# =============================================================================
# Motor D speed ramp test — UART mode, 1/2 step, interpolation OFF
# Configures driver 0 via UART then steps through delays slow → fast.
# Watch/listen for stalling — note the last delay that ran cleanly.
# =============================================================================

from machine import UART, Pin
import time

UART_TX_PIN = 17
STEP_PIN    = 26
DIR_PIN     = 25

REG_GCONF    = 0x00
REG_CHOPCONF = 0x6C
GCONF_VALUE  = 0x000000C0
CHOPCONF_VALUE = 0x07000053  # MRES=7 (1/2 step), intpol=0


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


uart = UART(2, baudrate=115200, tx=UART_TX_PIN, rx=16)
step = Pin(STEP_PIN, Pin.OUT)
dirp = Pin(DIR_PIN,  Pin.OUT)
dirp.value(1)

print("Configuring driver 0: 1/2 step via UART...")
write_reg(uart, 0, REG_GCONF,    GCONF_VALUE)
write_reg(uart, 0, REG_CHOPCONF, CHOPCONF_VALUE)
print("Done. Starting speed ramp.")
print()

STEPS_PER_RUN = 4096  # 1 input-shaft revolution at 1/2 step

delays_us = [3000, 2000, 1500, 1000, 750, 500, 400, 300, 200]

for delay in delays_us:
    steps_per_sec = 1_000_000 // (delay + 10)
    print("delay: {:4d} us  (~{:5d} steps/sec) — running...".format(delay, steps_per_sec))

    for _ in range(STEPS_PER_RUN):
        step.value(1)
        time.sleep_us(10)
        step.value(0)
        time.sleep_us(delay)

    print("  done. Smooth? (continuing in 1 sec...)")
    time.sleep_ms(1000)

print()
print("Ramp complete.")

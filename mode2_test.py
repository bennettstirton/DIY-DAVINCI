import time
from machine import UART

SERVO_ID = 6   # change this to match whichever servo you're testing

uart = UART(1, baudrate=1_000_000, tx=4, rx=5)

def checksum(body):
    return (~sum(body)) & 0xFF

def packet(scs_id, instruction, params):
    length = len(params) + 2
    body = [scs_id, length, instruction] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def read_reg(scs_id, address, length):
    uart.read()
    uart.write(packet(scs_id, 0x02, [address, length]))
    time.sleep_ms(50)
    resp = uart.read()
    print(f"read id={scs_id} addr={address}: raw={list(resp) if resp else None}")
    return resp

def send_pwm(pwm):
    # Mode 2 PWM register is 0x2C = 44 (NOT 46).
    # Direction bit is BIT10 (0x0400), NOT BIT15 (0x8000).
    # Valid magnitude range: 50–1000.
    mag = max(50, min(abs(pwm), 1000)) if pwm != 0 else 0
    raw = (mag | 0x0400) if pwm < 0 else mag
    p = packet(SERVO_ID, 0x03, [44, raw & 0xFF, (raw >> 8) & 0xFF])
    uart.write(p)
    print(f"sent PWM {pwm:+d}  raw=0x{raw:04X}  reg=44  bytes={list(p)}")

# --- Comms check ---
print(f"--- comms check: ID register from servo {SERVO_ID} ---")
read_reg(SERVO_ID, 5, 1)

# --- Check and set mode 2 (torque OFF before EEPROM write) ---
resp = read_reg(SERVO_ID, 33, 1)
mode = resp[5] if resp and len(resp) > 5 else None
print(f"current mode: {mode}")

if mode != 2:
    print("disabling torque before EEPROM write...")
    uart.write(packet(SERVO_ID, 0x03, [40, 0]))   # torque OFF
    time.sleep_ms(50)
    print("writing mode 2 to EEPROM...")
    uart.write(packet(SERVO_ID, 0x03, [55, 0]))   # unlock EEPROM
    time.sleep_ms(20)
    uart.write(packet(SERVO_ID, 0x03, [33, 2]))   # mode = 2
    time.sleep_ms(20)
    uart.write(packet(SERVO_ID, 0x03, [55, 1]))   # lock EEPROM
    time.sleep_ms(20)
    print()
    print("*** POWER-CYCLE the servo board, then re-run. ***")
    raise SystemExit(0)

print("mode is 2 — proceeding")

# --- Enable torque ---
uart.write(packet(SERVO_ID, 0x03, [40, 1]))
time.sleep_ms(50)
resp = read_reg(SERVO_ID, 40, 1)
print(f"torque enable readback: {list(resp) if resp else None}  (byte 5 should be 1)")

print()
print("--- Mode 2 PWM test (reg 44, BIT10 direction) ---")
print("    +900 for 3s...")
send_pwm(900)
time.sleep(3)

print("    -900 for 3s...")
send_pwm(-900)
time.sleep(3)

print("    disabling torque (stop).")
uart.write(packet(SERVO_ID, 0x03, [40, 0]))
print("done — did the servo move?")

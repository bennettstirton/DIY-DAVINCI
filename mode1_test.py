import time
from machine import UART

uart = UART(1, baudrate=1_000_000, tx=4, rx=5)

def checksum(body):
    return (~sum(body)) & 0xFF

def pkt(scs_id, instruction, params):
    length = len(params) + 2
    body = [scs_id, length, instruction] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def read_reg(address, length):
    uart.read()
    uart.write(pkt(6, 0x02, [address, length]))
    time.sleep_ms(50)
    resp = uart.read()
    return resp

def write_reg(address, data):
    uart.write(pkt(6, 0x03, [address] + list(data)))
    time.sleep_ms(20)
    uart.read()

def write_speed(speed):
    """Mode 1: command target speed. Signed, -1000 to +1000. Bit 15 = direction."""
    mag = min(abs(speed), 1000)
    raw = mag | 0x8000 if speed < 0 else mag
    # Mirrors SDK WriteSpec: write acc + pos(ignored) + time(ignored) + speed at reg 41
    write_reg(41, [0, 0, 0, 0, 0, raw & 0xFF, (raw >> 8) & 0xFF])
    print(f"speed={speed:+d}  raw=0x{raw:04X}")

# --- Check/set mode ---
resp = read_reg(33, 1)
mode = resp[5] if resp and len(resp) > 5 else None
print(f"current mode: {mode}")

if mode != 1:
    print("writing mode 1 to EEPROM...")
    write_reg(55, [0])    # unlock EEPROM
    time.sleep_ms(20)
    write_reg(33, [1])    # mode = 1 (continuous rotation / speed)
    time.sleep_ms(20)
    write_reg(55, [1])    # lock EEPROM
    print()
    print("*** POWER-CYCLE the servo board, then re-run. ***")

else:
    print("mode is 1 — enabling torque and testing speeds")
    write_reg(40, [1])    # torque enable
    time.sleep_ms(50)

    print()
    print("phase 1: speed +300 for 3s (positive direction)")
    print("         servo should rotate. note which way — that is your 'positive' direction.")
    write_speed(300)
    time.sleep(3)

    print()
    print("phase 2: speed 0 for 3s (brake)")
    print("         servo should hold still and resist if you try to move it.")
    write_speed(0)
    time.sleep(3)

    print()
    print("phase 3: speed -300 for 3s (negative direction)")
    write_speed(-300)
    time.sleep(3)

    print()
    print("phase 4: speed 0 — stopping")
    write_speed(0)
    write_reg(40, [0])    # torque off
    print("done.")
    print()
    print("Report: did all three phases produce movement/braking as described?")

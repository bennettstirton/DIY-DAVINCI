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

# --- Check current mode ---
resp = read_reg(33, 1)
mode = resp[5] if resp and len(resp) > 5 else None
print(f"current mode: {mode}")

if mode != 0:
    print("writing mode 0 to EEPROM...")
    write_reg(55, [0])       # unlock EEPROM
    time.sleep_ms(20)
    write_reg(33, [0])       # mode = 0 (position)
    time.sleep_ms(20)
    write_reg(55, [1])       # lock EEPROM
    print()
    print("*** POWER-CYCLE the servo board, then re-run. ***")

else:
    print("mode is 0 — enabling torque and sending position commands")
    write_reg(40, [1])       # torque enable
    time.sleep_ms(50)

    print("moving to position 1000...")
    write_reg(41, [50, 232, 3, 0, 0, 100, 0])   # acc=50, pos=1000, speed=100
    time.sleep(4)

    print("moving to position 3000...")
    write_reg(41, [50, 184, 11, 0, 0, 100, 0])  # acc=50, pos=3000, speed=100
    time.sleep(4)

    print("returning to center (2048)...")
    write_reg(41, [50, 0, 8, 0, 0, 100, 0])     # acc=50, pos=2048, speed=100
    time.sleep(4)

    write_reg(40, [0])   # torque off
    print("done — did the servo move?")

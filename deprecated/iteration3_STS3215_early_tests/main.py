from machine import UART
import time

# UART setup (UART2)
uart = UART(2, baudrate=1000000, tx=17, rx=16)

def checksum(data):
    return (~sum(data)) & 0xFF

def send_packet(packet):
    uart.write(bytearray(packet))
    time.sleep(0.01)
    if uart.any():
        resp = uart.read()
        print("Response:", resp)

# --- Ping servo (ID = 1) ---
def ping(servo_id=1):
    packet = [0xFF, 0xFF, servo_id, 0x02, 0x01]
    packet.append(checksum(packet[2:]))
    send_packet(packet)

# --- Move servo ---
def move(servo_id, position):
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF
    
    packet = [
        0xFF, 0xFF,
        servo_id,
        0x07,
        0x03,
        0x2A,      # goal position address
        pos_l, pos_h,
        0x00, 0x00 # time
    ]
    packet.append(checksum(packet[2:]))
    
    send_packet(packet)

# --- Test ---
print("Pinging servo...")
ping(1)

print("Sweeping...")
while True:
    move(1, 200)   # one side
    time.sleep(1)
    move(1, 800)   # other side
    time.sleep(1)
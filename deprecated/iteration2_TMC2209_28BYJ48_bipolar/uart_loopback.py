# =============================================================================
# UART loopback test — confirms ESP32 UART2 is transmitting
#
# Wiring for this test ONLY:
#   Connect GPIO 17 directly to GPIO 16 with a single jumper wire.
#   (Disconnect from the TMC2209 bus first!)
#
# Expected result: TX bytes are received back on RX → UART is working.
# If nothing is received: UART2 is misconfigured or the pins are wrong.
# =============================================================================

from machine import UART
import time

uart = UART(2, baudrate=115200, tx=17, rx=16, timeout=100)

TEST_BYTES = b'\x05\x00\x80\x00\x00\x00\xC0\x4A'  # a sample GCONF write datagram

print("Sending {} bytes on UART2 TX (GPIO 17)...".format(len(TEST_BYTES)))
uart.write(TEST_BYTES)
time.sleep_ms(10)

received = uart.read()
print("Received:", received)

if received and len(received) > 0:
    print("PASS — UART2 is transmitting. Got {} bytes back.".format(len(received)))
    if received == TEST_BYTES:
        print("Bytes match exactly.")
    else:
        print("Note: bytes don't match exactly (may be partial read — still a pass).")
else:
    print("FAIL — nothing received. UART2 is not transmitting on GPIO 17/16.")
    print("Try: check MicroPython firmware supports UART2, or try UART(1).")

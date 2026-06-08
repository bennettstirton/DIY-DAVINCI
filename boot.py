from machine import Pin
import time

_btn = Pin(0, Pin.IN, Pin.PULL_UP)
_led = Pin(2, Pin.OUT)   # onboard LED — GPIO 2 on most ESP32 DevKit v1 boards; adjust if needed

_TIMEOUT_MS = 5000
_deadline = time.ticks_add(time.ticks_ms(), _TIMEOUT_MS)

print("Press BOOT within 5s to start armcontrol.py.")

while _btn.value() == 1:
    if time.ticks_diff(_deadline, time.ticks_ms()) <= 0:
        print("Timeout — REPL ready.")
        raise SystemExit

_led.value(1)
time.sleep_ms(200)
_led.value(0)

print("Starting armcontrol.py")
import armcontrol

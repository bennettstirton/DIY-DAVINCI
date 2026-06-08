# 28BYJ-48 (bipolar) via TMC2209 — back and forth
# STEP -> GPIO 26, DIR -> GPIO 25, EN -> GND (hardwired)

from machine import Pin
import time

STEP = Pin(26, Pin.OUT)
DIR  = Pin(25, Pin.OUT)

# 28BYJ-48: 32 steps/rev motor * 64:1 gear = 2048 full steps/output shaft rev
# TMC2209 default microstep = 8x -> 2048 * 8 = 16384 microsteps per full revolution
# STEPS = 2048 here means 1/8 of a revolution — good for initial testing
# Change to 16384 when you want full rotations
STEPS = 4096
STEP_DELAY_US = 500 # was 3000

def move(steps, direction):
    DIR.value(direction)
    time.sleep_us(5)  # DIR setup time
    for _ in range(steps):
        STEP.value(1)
        time.sleep_us(STEP_DELAY_US)
        STEP.value(0)
        time.sleep_us(STEP_DELAY_US)

cycle = 0
while True:
    print(f"Cycle {cycle}: moving forward...")
    move(STEPS, 1)
    time.sleep_ms(500)
    print(f"Cycle {cycle}: moving backward...")
    move(STEPS, 0)
    time.sleep_ms(500)
    cycle += 1
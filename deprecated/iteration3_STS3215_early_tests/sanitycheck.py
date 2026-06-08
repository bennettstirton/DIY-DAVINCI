from machine import UART
import time

uart = UART(2, baudrate=1000000, tx=17, rx=16)

print("hello")
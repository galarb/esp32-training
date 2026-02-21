# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()
from machine import Pin, PWM
import time

# List of GPIO pins to set low (modify this based on your setup)
pins_to_reset = [0, 2, 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33]  

# Initialize all pins as outputs and set them LOW
for pin in pins_to_reset:
    p = Pin(pin, Pin.OUT)
    p.value(0)

print("All pins set to LOW on boot.") 

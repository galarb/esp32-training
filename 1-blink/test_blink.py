from blink import blink

led = blink(2)
for _ in range(10):
    led.blink(1)

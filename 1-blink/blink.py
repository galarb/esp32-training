from machine import Pin
import time
class blink:
    def __init__(self, led_pin):
        self.led = Pin(led_pin, Pin.OUT)

    def blink(self, speed):
        self.led.value(1)   # was: led.value(1)
        print("led on")
        time.sleep(speed)
        self.led.value(0)   # was: led_pin.value(0)
        print("led off")
        time.sleep(speed)
                            


from machine import Pin, time_pulse_us
import time

class Ultrasonic:
    def __init__(self, trig_pin, echo_pin, timeout_us=30000):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.timeout = timeout_us
        self.trig.value(0)
        time.sleep_us(2)

    def _send_pulse(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

    def distance_cm(self):
        self._send_pulse()
        try:
            duration = time_pulse_us(self.echo, 1, self.timeout)
        except OSError:
            return None  # Timeout / out of range

        # Speed of sound = 343 m/s
        # Distance = (duration / 2) * 0.0343
        return (duration / 2) * 0.0343

    def distance_mm(self):
        d = self.distance_cm()
        if d is None:
            return None
        return d * 10


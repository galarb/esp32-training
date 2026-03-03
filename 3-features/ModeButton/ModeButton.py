from machine import Pin
import time

class ModeButton:
    def __init__(self, pin_num, modes=3, debounce_ms=300):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.modes = modes
        self.debounce_ms = debounce_ms
        
        self._mode = 0
        self._last_press = 0
        self._pressed_flag = False
        
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self._irq_handler)

    # --- IRQ (VERY SMALL) ---
    def _irq_handler(self, pin):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_press) > self.debounce_ms:
            self._last_press = now
            self._pressed_flag = True   # just set flag

    # --- Call this from main loop ---
    def update(self):
        if self._pressed_flag:
            self._pressed_flag = False
            self._mode = (self._mode + 1) % self.modes

    def get_mode(self):
        return self._mode

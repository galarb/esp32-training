import math
from machine import Pin, PWM
import time

class ev3lego:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.encoder1_irq_handler)

        self.encoder2 = Pin(encoder2_pin, Pin.IN)
        
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm1 = PWM(self.in1)
        self.pwm2 = PWM(self.in2)
        self.pwm1.freq(1000)
        self.pwm2.freq(1000)

        self.wheel_size = wheel_size
        self.degrees = 0

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False

    def encoder1_irq_handler(self, pin):
        encoder1_state = self.encoder1.value()
        encoder2_state = self.encoder2.value()
        
        if encoder1_state == encoder2_state:
            self.degrees += 1
        else:
            self.degrees -= 1

        print("Degrees: ", self.degrees)

    def motgo(self, speed):
        pwm_value = int(min(max(abs(speed), 0), 100) * 10.23)  # Map -100 to 100 to 0 to 1023

        if speed > 0:
            # Forward direction
            self.pwm1.duty(pwm_value)
            self.pwm2.duty(0)
        elif speed < 0:
            # Reverse direction
            self.pwm1.duty(0)
            self.pwm2.duty(pwm_value)
        else:
            # Stop the motor
            self.pwm1.duty(0)
            self.pwm2.duty(0)
            
    def PIDcalc(self, sp, inp, kp, ki, kd):
        current_time = time.ticks_ms()
        elapsed_time = (current_time - self.previous_time) / 1000.0

        error = sp - inp
        
        if error * self.last_error < 0:
            self.integral_flag = True
            self.cum_error = 0
            print("Error changed direction, resetting integral accumulator.")
        else:
            self.integral_flag = False

        if not self.integral_flag:
            self.cum_error += error * elapsed_time

        if elapsed_time > 0:
            rate_error = (error - self.last_error) / elapsed_time
            out = kp * error + ki * self.cum_error + kd * rate_error

            self.last_error = error
            self.previous_time = current_time

            out = max(-254, min(254, out))  # Clamp the output to [-254, 254]

            print("Degrees: ", self.degrees)
            print("PID output value: ", out)
            return out

        return 0

    def godegrees(self, angle, times):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, 1, 1, 0)
            motspeed = max(-254, min(254, motspeed))  # Clamp the speed to [-254, 254]
            self.motgo(motspeed)

    def godegreesp(self, angle, times, kp, ki, kd):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, kp, ki, kd)
            motspeed = max(-254, min(254, motspeed))
            self.motgo(motspeed)

    def gomm(self, distance, times):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegrees(deg, times)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0
        return dist_covered

    def gommp(self, distance, times, kp, ki, kd):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegreesp(deg, times, kp, ki, kd)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0
        return dist_covered
    
    def stophard(self):
        self.pwm1.duty(1023)
        self.pwm2.duty(1023)
        
    def release(self):
        # Stop PWM signals
        self.pwm1.deinit()
        self.pwm2.deinit()

        # Set pins low
        self.in1.value(0)
        self.in2.value(0)

        # Disable IRQs
        self.encoder1.irq(handler=None)
        self.encoder2.irq(handler=None)

        print("Motor stopped and resources released.")


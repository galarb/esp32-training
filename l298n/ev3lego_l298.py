import math
from machine import Pin, PWM
import time

class ev3lego_l298:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, ena_pin, wheel_size):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.encoder1_irq_handler)

        self.encoder2 = Pin(encoder2_pin, Pin.IN)
        
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.ena = Pin(ena_pin, Pin.OUT)
        

        self.pwm = PWM(self.ena)
        self.pwm.freq(500)
        
        #stop the motor 
        self.pwm.duty(0)
        self.in1.value(0)
        self.in2.value(0)


        self.wheel_size = wheel_size
        self.degrees = 0

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False

    def encoder1_irq_handler(self, pin):
        if self.encoder1.value() == self.encoder2.value():
            self.degrees += 1
        else:
            self.degrees -= 1

        if self.degrees % 10 == 0:
            #print("Degrees: ", self.degrees)
            pass
            
    def motgo(self, speed):
        if speed > 0:
            self.in1.value(0)
            self.in2.value(1)
        elif speed < 0:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(0)

        pwm_value = int((abs(speed) / 254) * 1023)
        self.pwm.duty(pwm_value)
    
    def brake(self):
        self.in1.value(1)
        self.in2.value(1)
        

    def PIDcalc(self, sp, inp, kp, ki, kd):
        current_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(current_time, self.previous_time) / 1000.0

        error = sp- inp
        
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

            #print("Degrees: ", self.degrees)
            #print("PID output value: ", out)
            return out

        return 0

    def godegrees(self, angle, times):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, 54, 168, 4.3)
            motspeed = max(-254, min(254, motspeed))  # Clamp the speed to [-254, 254]
            self.motgo(motspeed)
            time.sleep_ms(1)

    def godegreesp(self, angle, times, kp, ki, kd):
        for _ in range(times):
            #print("degrees = ", self.degrees) #
            motspeed = self.PIDcalc(angle, self.degrees, kp, ki, kd)
            motspeed = max(-254, min(254, motspeed))
            self.motgo(motspeed)
            time.sleep_ms(1)


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
    
    def stop(self):

        # Stop motor
        self.pwm.duty(0)
        self.in1.value(0)
        self.in2.value(0)

        # Disable interrupts
        self.encoder1.irq(handler=None)

        # Optional: deinit PWM completely
        self.pwm.deinit()

        print("Motor and IRQ stopped")

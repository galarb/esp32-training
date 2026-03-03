import math
from machine import Pin, PWM
import time

class ev3lego_zk:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.encoder1_irq_handler)

        self.encoder2 = Pin(encoder2_pin, Pin.IN)
        
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)

        self.pwm1 = PWM(self.in1)
        self.pwm1.freq(500)
        self.pwm2 = PWM(self.in2)
        self.pwm2.freq(500)
        #stop the motor 
        self.pwm1.duty(0)
        self.pwm2.duty(0)   

        self.wheel_size = wheel_size
        self.degrees = 0

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False
        
    
        self.start_time = time.ticks_ms()
        self.log_counter = 0
        self.log_file = open("pid_log.csv", "w")
        self.log_file.write("time,error,output,degrees\n")
        self.start_time = time.ticks_ms()

    def encoder1_irq_handler(self, pin):
        if self.encoder1.value() == self.encoder2.value():
            self.degrees += 1
        else:
            self.degrees -= 1

        if self.degrees % 10 == 0:
            #print("Degrees: ", self.degrees)
            pass
            
    def motgo(self, speed):
        pwm_value = int((abs(speed) / 254) * 1023)
        
        if speed > 0:
            self.pwm1.duty(0)
            self.pwm2.duty(pwm_value)   
        elif speed < 0:
            self.pwm1.duty(pwm_value)
            self.pwm2.duty(0)   
        else:
            self.in1.value(0)
            self.in2.value(0)
    
    def brake(self):
        self.pwm1.duty(1023)
        self.pwm2.duty(1023)
        
    def reset_encoder(self):
        self.degrees = 0

    def get_degrees(self):
        return self.degrees

    def degrees_to_mm(self, deg):
        return (deg * self.wheel_size * math.pi) / 360.0

    def mm_to_degrees(self, mm):
        return (mm / (self.wheel_size * math.pi)) * 360
    
    def stop(self):

        # Stop motor
        self.pwm1.duty(0)
        self.pwm2.duty(0)

        # Disable interrupts
        self.encoder1.irq(handler=None)

        # Optional: deinit PWM completely
        self.pwm1.deinit()
        self.pwm2.deinit()
        self.log_file.close()

        print("Motor , IRQ and log stopped")

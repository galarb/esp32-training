import time
class MotorController:
    def __init__(self, motor, pid, us):
        self.motor = motor
        self.pid = pid
        self.us = us

    def go_degrees(self, target_deg):
        self.pid.reset()
        self.motor.reset_encoder()

        while True:
            current = self.motor.get_degrees()
            speed = self.pid.compute(target_deg, current)
            self.motor.motgo(int(speed))

            if abs(target_deg - current) < 1:
                break

            time.sleep_ms(2)

        self.motor.brake()

    def go_mm(self, distance_mm):
        target_deg = self.motor.mm_to_degrees(distance_mm)
        self.go_degrees(target_deg)
    
    def go_us(self, target_dist):
        self.pid.reset()

        while True:
            current = self.us.distance_cm()
            speed = self.pid.compute(target_dist, current)
            self.motor.motgo(int(speed))

            if abs(target_dist - current) < 1:
                break

            time.sleep_ms(2)

        self.motor.brake()

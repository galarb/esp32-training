import time
class MotorController:
    def __init__(self, motor, pid):
        self.motor = motor
        self.pid = pid

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
    

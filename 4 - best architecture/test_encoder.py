import time
from ev3lego_zk import ev3lego_zk
from pid import PID
from MotorController import MotorController
from Ultrasonic import Ultrasonic

encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)
trig_pin = 17
echo_pin = 5

robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
pid = PID(54, 168, 4.3)
us = Ultrasonic(trig_pin, echo_pin)
controller = MotorController(robot, pid, us)

controller.go_degrees(180)

time.sleep_ms(2)
print(robot.degrees)
print(us.distance_cm())
robot.stop()


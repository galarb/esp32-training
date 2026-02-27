import time
from ev3lego_l298 import ev3lego_l298

encoder1_pin = 12
encoder2_pin = 14
in1_pin = 15
in2_pin = 2
ena_pin = 4
wheel_size = 65 # wheel size (in mm)

# Create an instance of ev3lego
robot = ev3lego_l298(encoder1_pin, encoder2_pin, in1_pin, in2_pin, ena_pin, wheel_size)


#example usage: go to an exact angle in degrees
robot.godegreesp(180, 1000, 50, 2, 2) #required angle, number of iterations
time.sleep(1)
print(robot.degrees)
robot.brake() 
robot.stop()

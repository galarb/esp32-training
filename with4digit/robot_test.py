import time
from ev3lego_zk import ev3lego_zk
from tm1650_1 import TM1650
from PIDLogger import PIDLogger

encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)

# Create an instance of ev3lego
robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
display = TM1650(18, 19)


#example usage: go to an exact angle in degrees
robot.godegreesp(720, 1000, 30, 94, 6.4) #required angle, number of iterations

        
        
display.ShowNum(robot.degrees)

robot.brake() 
robot.stop()



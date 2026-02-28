import time
from ev3lego_zk import ev3lego_zk
from tm1650_1 import TM1650
from ModeButton import ModeButton
from Ultrasonic import Ultrasonic

encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)

# Create an instance of ev3lego
robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
display = TM1650(18, 19)
mode_button = ModeButton(pin_num=15, modes=3)
us = Ultrasonic(17, 5)

#example usage: go to an exact angle in degrees

while True:
    mode_button.update()      # <---- REQUIRED
    
    mode = mode_button.get_mode()
    

    if mode == 0:
        print('run_ultrasonic()')
        d = us.distance_cm()
        if d is None:
            print("Out of range")
        else:
            print("Distance: {:.1f} cm".format(d))
            display.ShowNum(int(d))
            display.displayDot(4, 1)

        time.sleep(1)

    elif mode == 1:
        print('run motor encoder')
        
        robot.godegreesp(90, 1000, 60, 200, 7) #required angle, number of iterations
        display.ShowNum(robot.degrees)
        display.displayDot(4, 0)
        
        time.sleep(1)
        

    elif mode == 2:
        print('run_idle()')
        robot.brake() 
        robot.degrees = 0
        display.ShowStr('IdLE')
        time.sleep(1)





robot.brake() 
robot.stop()



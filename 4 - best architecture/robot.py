import time
from ev3lego_zk import ev3lego_zk
from tm1650_1 import TM1650
from ModeButton import ModeButton
from Ultrasonic import Ultrasonic
from pid import PID
from MotorController import MotorController

encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)
trig_pin = 17
echo_pin = 5

# Create objects
robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
display = TM1650(18, 19)
mode_button = ModeButton(pin_num=15, modes=4)
pid = PID(54, 168, 4.3)
us = Ultrasonic(trig_pin, echo_pin)
controller = MotorController(robot, pid, us)

#example usage: go to an exact angle in degrees

while True:
    mode_button.update()      # <---- REQUIRED
    
    mode = mode_button.get_mode()
    

    if mode == 0:
        print('run distance measurement')
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
        controller.go_degrees(180)
        display.ShowNum(robot.degrees)
        display.displayDot(4, 0)
        
        time.sleep(1)
        

    elif mode == 2:
        print('go to 20 cm')
        controller.go_us(20)
        time.sleep(1)

        
    elif mode == 3:
        print('run_idle()')
        robot.brake() 
        robot.degrees = 0
        display.ShowStr('IdLE')
        time.sleep(1)



robot.brake() 
robot.stop()



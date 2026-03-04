import time
from ev3lego_zk import ev3lego_zk
from tm1650_1 import TM1650
from ModeButton import ModeButton
from Ultrasonic import Ultrasonic
from pid import PID
from MotorController import MotorController
import neopixel

encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)
trig_pin = 17
echo_pin = 5
leds_pin = 16
NUM_LEDS = 2

# Create objects
robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
display = TM1650(18, 19)
mode_button = ModeButton(pin_num=15, modes=4)
pid = PID(54, 168, 4.3)
us = Ultrasonic(trig_pin, echo_pin)
controller = MotorController(robot, pid, us)
np = neopixel.NeoPixel(Pin(leds_pin), NUM_LEDS)
def set_color(i, r, g, b):
    np[i] = (g, r, b)  # Swap R and G
def blink_idle():
    set_color(0, 255, 0, 0)   # Red
    set_color(1, 0, 255, 0)   # Green
    np.write()
    time.sleep(0.5)

    # Swap colors
    set_color(0, 0, 255, 0)   # Green
    set_color(1, 255, 0, 0)   # Red
    np.write()
    time.sleep(0.5)
    
    
while True:
    mode_button.update()      # <---- REQUIRED
    
    mode = mode_button.get_mode()
    

    if mode == 0:
        print('run distance measurement')
        set_color(0, 0, 0, 255)  # real BLUE    
        set_color(1, 0, 0, 255)  # real BLUE
        np.write()
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
        set_color(0, 0, 255, 0)  # real GREEN    
        set_color(1, 0, 255, 0)  # real GREEN
        np.write()

        controller.go_degrees(180)
        display.ShowNum(robot.degrees)
        display.displayDot(4, 0)
        robot.brake() 

        time.sleep(1)
        

    elif mode == 2:
        print('go to 20 cm')
        set_color(0, 80, 0, 80)     
        set_color(1, 80, 0, 80)
        np.write()
        controller.go_us(20)
        display.ShowNum(int(us.distance_cm()))
        robot.brake() 

        time.sleep(1)
        
        
    elif mode == 3:
        print('run_idle()')
        robot.brake() 
        robot.degrees = 0
        display.ShowStr('IdLE')
        blink_idle()        



robot.brake() 
robot.stop()



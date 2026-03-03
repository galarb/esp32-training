from ModeButton import ModeButton
from time import sleep
from Ultrasonic import Ultrasonic
mode_button = ModeButton(pin_num=15, modes=3)

us = Ultrasonic(17, 5)

while True:
    d = us.distance_cm()
    if d is None:
        print("Out of range")
    else:
        print("Distance: {:.1f} cm".format(d))
    time.sleep(0.5)


    mode_button.update()      # <---- REQUIRED
    
    mode = mode_button.get_mode()
    

    if mode == 0:
        print('run_ultrasonic()')
        time.sleep(1)

    elif mode == 1:
        print('run_line_follower()')
        time.sleep(1)

    elif mode == 2:
        print('run_idle()')
        time.sleep(1)

from ModeButton import ModeButton

mode_button = ModeButton(pin_num=15, modes=3)

while True:
    mode_button.update()      # <---- REQUIRED
    
    mode = mode_button.get_mode()
    

    if mode == 0:
        print('run_ultrasonic()')

    elif mode == 1:
        print('run_line_follower()')

    elif mode == 2:
        print('run_idle()')

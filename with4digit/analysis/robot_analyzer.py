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
logger = PIDLogger()

run = 1

tests = [
    (3.0, 3.0, 1),
    (63.0, 240.0, 8.0),
    (63.0, 240.0, 7.0),
    (63.0, 240.0, 6.0),
    (63.0, 240.0, 5.0),
    (63.0, 240.0, 4.0),
    (63.0, 240.0, 3.0),
    (63.0, 240.0, 2.0),
    (63.0, 245.0, 1.0),
    (63.0, 240.0, 0.0),
    
]
for Kp, Ki, Kd in tests:

    print("Run", run, "Kp=", Kp, "Ki=", Ki, "Kd=", Kd)

    # RESET STATE
    robot.degrees = 0
    robot.last_error = 0
    robot.cum_error = 0
    robot.previous_time = time.ticks_ms()

    start = time.ticks_ms()
    setpoint = 45

    while True:

        now = time.ticks_diff(time.ticks_ms(), start) / 1000.0

        position = robot.degrees
        error = setpoint - position

        motspeed = robot.PIDcalc(setpoint, position, Kp, Ki, Kd)
        motspeed = max(-254, min(254, motspeed))

        robot.motgo(motspeed)

        logger.log(
            run,
            Kp, Ki, Kd,
            now,
            setpoint,
            position,
            error,
            motspeed
        )

        if now > 3.0:
            break

        time.sleep_ms(10)

    robot.brake()
    time.sleep(1)

    run += 1
#example usage: go to an exact angle in degrees
#robot.godegreesp(720, 3000, 30, 94, 6.4) #required angle, number of iterations

        
        
display.ShowNum(robot.degrees)

robot.brake() 
robot.stop()


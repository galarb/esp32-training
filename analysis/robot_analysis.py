import time
from ev3lego_zk import ev3lego_zk
from Ultrasonic import Ultrasonic
from pid import PID
from MotorController import MotorController
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
    (1.0, 0.0, 0),
    (2.0, 0.0, 0.0),
    (2.0, 2.0, 0.0),
    (10.0, 10.0, 3.0),
    (5.0, 3.0, 1.0),
    (5.0, 1.0, 2.0),
    (10.0, 4.0, 3.0),
    (63.0, 245.0, 9.0),
    (54.0, 168.0, 4.3),
    (30.0, 94.0, 6.4),
    
]
for Kp, Ki, Kd in tests:

    print("Run", run, "Kp=", Kp, "Ki=", Ki, "Kd=", Kd)

    robot.degrees = 0

    pid = PID(Kp, Ki, Kd)
    pid.reset()

    start = time.ticks_ms()
    setpoint = 45

    while True:

        now = time.ticks_diff(time.ticks_ms(), start) / 1000.0

        position = robot.degrees
        error = setpoint - position

        motspeed = pid.compute(setpoint, position)
        motspeed = max(-254, min(254, motspeed))

        robot.motgo(int(motspeed))

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
display.ShowNum(robot.degrees)

robot.brake() 
robot.stop()


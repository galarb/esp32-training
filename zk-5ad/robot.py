import time
from ev3lego_zk import ev3lego_zk
encoder1_pin = 39
encoder2_pin = 36
in1_pin = 33
in2_pin = 32
wheel_size = 65 # wheel size (in mm)
# Create an instance of ev3lego
robot = ev3lego_zk(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)
#example usage: go an exact distance in mm
#print('distance covered =', robot.gomm(0, 1000))#required distance, number of iterations
#example usage: go to an exact angle in degrees
robot.godegrees(360, 1000) #required angle, number of iterations
#robot.motgo(120)
#time.sleep(1)
robot.brake() 
robot.stop()

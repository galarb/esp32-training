import time
from ev3lego import ev3lego  

encoder1_pin = 16 
encoder2_pin = 17 
in1_pin = 0       
in2_pin = 2       
wheel_size = 65    # wheel diameter (in mm)

# Create an instance of ev3lego
robot = ev3lego(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)

#example usage: go an exact distance in mm
#print('distance covered =', robot.gomm(90, 1000)) #required distance, number of iterations

#example usage: go to an exact angle in degrees
#robot.godegrees(150, 400) #required angle, number of iterations


robot.stophard()
robot.release()

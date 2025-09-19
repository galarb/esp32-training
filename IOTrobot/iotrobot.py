import time
from ev3lego import ev3lego  
import network
from umqtt.simple import MQTTClient

encoder1_pin = 16 
encoder2_pin = 17 
in1_pin = 2       
in2_pin = 0       
wheel_size = 65    # wheel diameter (in mm)


# Create an instance of ev3lego
robot = ev3lego(encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size)

robot.connect_wifi()
robot.connect_mqtt()


#example usage: go an exact distance in mm
#print('distance covered =', robot.gomm(90, 1000)) #required distance, number of iterations

#example usage: go to an exact angle in degrees
#robot.godegrees(90, 20000) #required angle, number of iterations
#robot.motgo(100)
#time.sleep(2)
robot.motgo(0)

for _ in range(200):
    robot.check_message() # waits for new message
    time.sleep(0.1)
robot.publish_angle()
robot.stophard()
robot.release()

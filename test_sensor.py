import RPi.GPIO as GPIO 
import time


def read_sensor(inPin):
    return GPIO.input(inPin)

#GPIO.setmode(GPIO.BOARD)
#inPin1 = 15
#inPin2 = 19
#GPIO.setup(inPin1, GPIO.IN)
#GPIO.setup(inPin2, GPIO.IN)

#while True:
    #print("Current sensor1 reading is " + str(GPIO.input(inPin1)))
    #print("Current sensor2 reading is " + str(GPIO.input(inPin2)))
    #time.sleep(2)




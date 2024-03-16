import Adafruit_PCA9685
from motor import *
import time

# Initialize the PCA9685 using the default address (0x40).
servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
servo.set_pwm_freq(60)

# Define the channel numbers for the motors.
motor_channel_left = 0
motor_channel_right = 1
motor_unload = 2
v = 3
vl = 3
vr = 3

# print("Testing moving forward")
# move_forward(servo, motor_channel_left, motor_channel_right, v, 5)
# time.sleep(5)

# print("Testing moving backward")
# move_backward(servo, motor_channel_left, motor_channel_right, v, 5)
# time.sleep(5)

# print("Testing stopping")
# move_forward(servo, motor_channel_left, motor_channel_right, v)
# stop(servo, motor_channel_left, motor_channel_right)
# time.sleep(5)

# print("Testing turning left")
# turnL(servo, motor_channel_left, motor_channel_right, vr, 5)
# time.sleep(5)

# print("Testing turning right")
# turnR(servo, motor_channel_left, motor_channel_right, vl, 5)
# time.sleep(5)

# print("Testing turning with vl, vr")
# turn(servo, motor_channel_left, motor_channel_right, vl, vr, duration = 5)
# time.sleep(5)

print("Testing unloading")
unload(servo, motor_unload, 1000, 4)
time.sleep(5)

#print("Testing loading")
#load(servo, motor_unload, 100, 2)
#time.sleep(5)

#print("testing stoping turn")
#turn(servo, motor_channel_left, motor_channel_right, vl, vr)
#time.sleep(5)
#stop(servo, motor_channel_left, motor_channel_right)

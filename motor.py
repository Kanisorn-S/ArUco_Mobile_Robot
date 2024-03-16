#pip install adafruit-circuitpython-servokit
#install be fore use

import time
import Adafruit_PCA9685
import importlib
from datetime import datetime, timedelta
from test_servo_pwm import ccw, cw

# Initialize the PCA9685 using the default address (0x40).
# kit = ServoKit(channels=16)

# Define the channel numbers for the motors.
# motor_channel_left = 0
# motor_channel_right = 1
# motor_unload = 2

# Define the PWM values for forward and backward movement.
# forward_pwm = 4095
# backward_pwm = 0

def move_forward(pca, motor_channel_left, motor_channel_right, v, duration = 0, pwml=0, pwmr=0):
    '''
    Move forward with motor speed v 
    '''
    if pwml == 0:
        pwml = int(ccw(v))
    if pwmr == 0:
        pwmr = int(cw(v))
    print("pwml is " + str(pwml))
    print("pwmr is " + str(pwmr))
    if not duration:
        pca.set_pwm(motor_channel_left, 0, pwml)
        pca.set_pwm(motor_channel_right, 0, pwmr)
    else:
        end_time = datetime.now() + timedelta(seconds = duration)
        while datetime.now() < end_time:
            pca.set_pwm(motor_channel_left, 0, pwml)
            pca.set_pwm(motor_channel_right, 0, pwmr)
        stop(pca, motor_channel_left, motor_channel_right)



def move_backward(pca, motor_channel_left, motor_channel_right, v, duration = 0):
    pwml = int(cw(v))
    pwmr = int(ccw(v))
    print("pwml is " + str(pwml))
    print("pwmr is " + str(pwmr))
    if not duration:
        pca.set_pwm(motor_channel_left, 0, pwml)
        pca.set_pwm(motor_channel_right, 0, pwmr)
    else:
        end_time = datetime.now() + timedelta(seconds = duration)
        while datetime.now() < end_time:
            pca.set_pwm(motor_channel_left, 0, pwml)
            pca.set_pwm(motor_channel_right, 0, pwmr)
        stop(pca, motor_channel_left, motor_channel_right)


def stop(pca, motor_channel_left, motor_channel_right):
    '''
    Stop the robot
    '''
    pca.set_pwm(motor_channel_left, 0, 0)
    pca.set_pwm(motor_channel_right, 0, 0)


def turnL(pca, motor_channel_left, motor_channel_right, vr, duration = 0):
    pwmr = int(cw(vr))
    print("pwmr is " + str(pwmr))
    if not duration:
        pca.set_pwm(motor_channel_left, 0, 0)
        pca.set_pwm(motor_channel_right, 0, pwmr)
    else:
        end_time = datetime.now() + timedelta(seconds = duration)
        while datetime.now() < end_time:
            pca.set_pwm(motor_channel_left, 0, 0)
            pca.set_pwm(motor_channel_right, 0, pwmr)
        stop(pca, motor_channel_left, motor_channel_right)

def turnR(pca, motor_channel_left, motor_channel_right, vl, duration = 0):
    pwml = int(ccw(vl))
    print("pwml is " + str(pwml))
    if not duration:
        pca.set_pwm(motor_channel_left, 0, pwml)
        pca.set_pwm(motor_channel_right, 0, 0)
    else:
        end_time = datetime.now() + timedelta(seconds = duration)
        while datetime.now() < end_time:
            pca.set_pwm(motor_channel_left, 0, pwml)
            pca.set_pwm(motor_channel_right, 0, 0)
        stop(pca, motor_channel_left, motor_channel_right)

def turn(pca, motor_channel_left, motor_channel_right, vl, vr, spinning = True,  duration = 0, pwml = 0, pwmr = 0):
    '''
    Turn the robot without moving using left wheel velocity and right wheel velocity as vl and vr respectively
    '''
    print("Turning!")
    if not pwml and not pwmr:
        pwml = int(ccw(vl))
        pwmr = int(cw(vr))
    print("pwml is " + str(pwml))
    print("pwmr is " + str(pwmr))
    if not duration:
        while spinning:
            pca.set_pwm(motor_channel_left, 0, pwml)
            pca.set_pwm(motor_channel_right, 0, pwmr)
            time.sleep(2)
        stop(pca, motor_channel_left, motor_channel_right) 
    else:
        end_time = datetime.now() + timedelta(seconds = duration)
        while datetime.now() < end_time:
            pca.set_pwm(motor_channel_left, 0, pwml)
            pca.set_pwm(motor_channel_right, 0, pwmr)
        stop(pca, motor_channel_left, motor_channel_right)

def unload(pca, motor_unload, pwm, duration):
    end_time = datetime.now() + timedelta(seconds = duration)
    while datetime.now() < end_time:
        pca.set_pwm(motor_unload, 0, pwm)
    pca.set_pwm(motor_unload, 0, 0)

def load(pca, motor_unload, pwm, duration):
    end_time = datetime.now() + timedelta(seconds = duration)
    while datetime.now() < end_time:
        pca.set_pwm(motor_unload, 0, pwm)
    pca.set_pwm(motor_unload, 0, 0)

# Move forward for 10 seconds.
# v = input("Mobe with speed:")

# move_forward(v)
# time.sleep(10)


# stop()
# time.sleep(10)

# move_backward(v)
# time.sleep(10)

# turnL(v)
# time.sleep(10)

# turnR(v)
# time.sleep(10)

# stop()

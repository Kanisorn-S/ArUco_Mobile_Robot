import cv2 as cv
import nanocamera as nano
import Adafruit_PCA9685
import importlib
import time
from datetime import datetime, timedelta
import sys

duration = 10
servo_max_cw = 98 # full throtle clockwise is 4.87 rad/s
servo_min_cw = 350 # min throtle counter-clockwise is 0.94 rad/s
servo_max_ccw = 632 # full throtle counter-clockwise is 4.87 rad/s
servo_min_ccw = 380 # min throtle counter-clockwise is 0.89 rad/s

max_w = 4
min_w_ccw = 0.833
min_w_cw = 0.833

# for mini servo
mini_cw = 60
mini_ccw = 1100



# servo = Adafruit_PCA9685.PCA9685(address=0x40, busnum = 1)

# servo.set_pwm_freq(60)

# end_time = datetime.now() + timedelta(seconds = duration)
# while datetime.now() < end_time:


#     servo.set_pwm(2, 0, mini_cw)




# servo.set_pwm(2, 0, 0)

def ccw(w):
    '''
    Takes in the rotational speed omega(w) for turning ccw then returns the pwm needed
    '''
    if w < 0:
        return cw(-1*w)
    elif w < min_w_ccw or w > max_w:
        print("Over/Under speed limit")
        sys.exit()
    else:
        full_range = max_w - min_w_ccw
        adjusted_w = w - min_w_ccw
        pwm_range = servo_max_ccw - servo_min_ccw
        pwm = (adjusted_w * pwm_range) / full_range
        return servo_min_ccw + pwm 

def cw(w):
    '''
    Takes in the rotational speed omega(w) for turning ccw then returns the pwm needed
    '''
    if w < 0:
        return ccw(-1*w)
    elif w < min_w_cw or w > max_w:
        print("Over/Under speed limit")
        sys.exit()
    else:
        full_range = max_w - min_w_cw
        adjusted_w = w - min_w_cw
        pwm_range = servo_min_cw - servo_max_cw
        pwm = (adjusted_w * pwm_range) / full_range
        return servo_min_cw - pwm 

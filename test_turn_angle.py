import cv2 as cv
import nanocamera as nano
import time
from motor import *
from threading import Thread
import RPi.GPIO as GPIO 
import multiprocessing as mp
from test_sensor import read_sensor
from datetime import datetime, timedelta
from check_scan import scan
import cv2.aruco as aruco
import numpy as np
from move import inv_kine
import math
import concurrent.futures
from locate import complete
from obstacle import obstacleAvoidance

servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
servo.set_pwm_freq(60)
motor_channel_left = 0
motor_channel_right = 1
motor_unload = 2
v = 3
vl = 3
vr = 3
cap1 = nano.Camera(device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)
GPIO.setmode(GPIO.BOARD)
inPin = 15
GPIO.setup(inPin, GPIO.IN)
N_ARUCO = 2 # max aruco ID
t_aruco = 0
rel_dis = {}
SCAN_TIME = 10
R = 0.03
BASELINE = 0.24
MAX_SPEED = 255
marker_size = 100
t_bot = np.array([0, 0, 0])
atTarget = False
# divide t_angle by pi
NINETY = math.pi / 2
ROT_TIME_CCW = 6.5
FACTOR_CCW = ROT_TIME_CCW / NINETY
ROT_TIME_CW = 7.5
t_angle = math.pi / 2
FACTOR_CW = ROT_TIME_CW / NINETY
with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)
# ArUco markers dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

def turn_set_angle(servo, motor_channel_left, motor_channel_right, t_angle):

    # Turning CW
    if t_angle < 0:
        pwml = 380
        pwmr = 380
        ROT_TIME = FACTOR_CW * t_angle
        turn(servo, motor_channel_left, motor_channel_right, 0, 0, duration = ROT_TIME, pwml = pwml, pwmr = pwmr)
    # Turning CCW
    else:
        pwml = 350
        pwmr = 350
        ROT_TIME = FACTOR_CCW * t_angle
        turn(servo, motor_channel_left, motor_channel_right, 0, 0, duration = ROT_TIME, pwml = pwml, pwmr = pwmr)

def main():
    servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
    servo.set_pwm_freq(60)
    motor_channel_left = 0
    motor_channel_right = 1
    motor_unload = 2
    v = 3
    vl = 3
    vr = 3
    cap1 = nano.Camera(device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
    cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)
    GPIO.setmode(GPIO.BOARD)
    inPin = 15
    GPIO.setup(inPin, GPIO.IN)
    N_ARUCO = 2 # max aruco ID
    t_aruco = 0
    rel_dis = {}
    SCAN_TIME = 10
    R = 0.03
    BASELINE = 0.24
    MAX_SPEED = 255
    marker_size = 100
    t_bot = np.array([0, 0, 0])
    atTarget = False
    # divide t_angle by pi
    NINETY = math.pi / 2
    ROT_TIME_CCW = 10
    FACTOR_CCW = ROT_TIME_CCW / NINETY
    ROT_TIME_CW = 12.5
    t_angle = math.pi / 2
    FACTOR_CW = ROT_TIME_CW / NINETY
    with open('camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)
    # ArUco markers dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    
    vl, vr = inv_kine(FACTOR_CW * t_angle, R, BASELINE, MAX_SPEED, theta = t_angle)

    # move_forward(servo, motor_channel_left, motor_channel_right, 0, 1, 632, 98) 
    turn(servo, motor_channel_left, motor_channel_right, vl, vr, duration = FACTOR_CW * t_angle, pwml = 380, pwmr = 380)


    
if __name__ == "__main__":
     main()

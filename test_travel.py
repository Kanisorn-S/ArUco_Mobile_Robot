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
    t_angle = 0
    ROT_TIME = 3
    tvec = np.array([0, 300, 0])
    with open('camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)
    # ArUco markers dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    
    vl, vr = inv_kine(ROT_TIME, R, BASELINE, MAX_SPEED, tvec = tvec, theta = t_angle)
    move_forward(servo, motor_channel_left, motor_channel_right, v, duration = ROT_TIME)


    
if __name__ == "__main__":
     main()

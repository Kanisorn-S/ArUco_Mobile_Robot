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
from obstacle import obstacleAvoidance2
import importlib.util
import sys
import importlib
from god import god

def main():
    HALF_ANGLE_LEFT = 55 * math.pi / 180
    HALF_ANGLE_RIGHT = 62 * math.pi / 180
    servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
    servo.set_pwm_freq(60)
    motor_channel_left = 0
    motor_channel_right = 1
    motor_unload = 2
    v = 3
    vl = 3
    vr = 3
    # Right cam
    cap1 = nano.Camera(device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
    # Left cam
    cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)
    GPIO.setmode(GPIO.BOARD)
    inPin = 15
    inPin2 = 19
    GPIO.setup(inPin, GPIO.IN)
    GPIO.setup(inPin2, GPIO.IN)
    N_ARUCO = 3 # max aruco ID
    t_aruco = 1
    rel_dis = {}
    SCAN_TIME = 25
    R = 0.03
    BASELINE = 0.24
    MAX_SPEED = 255
    marker_size = 100
    t_bot = np.array([0, 0, 0])
    atTarget = False
    with open('/home/ai/Desktop/Code2/ArUCo_Bot/camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)
    # divide t_angle by pi
    NINETY = math.pi / 2
    ROT_TIME_CCW = 6.5
    FACTOR_CCW = ROT_TIME_CCW / NINETY
    ROT_TIME_CW = 7.5
    t_angle = math.pi / 2
    FACTOR_CW = ROT_TIME_CW / NINETY
    # ArUco markers dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    
    # Trigger to start the program
    result = 1
    while result:
        result = read_sensor(inPin)
        print("current sensor reading is " + str(result))
    print("Object detected. Begin spinning and scanning")
    
    god(SCAN_TIME, R, BASELINE, MAX_SPEED, servo, motor_channel_left, motor_channel_right, cap1, cap2, N_ARUCO, t_aruco, rel_dis, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, inPin2)
    
    # reached target
    unload(servo, motor_unload, 1000, 20)
    load(servo, motor_unload, 100, 20)
    
if __name__ == "__main__":
     main()

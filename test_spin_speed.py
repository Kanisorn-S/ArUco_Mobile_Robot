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
from god import god2

def main():
    # Initialize PCA
    servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
    servo.set_pwm_freq(60)

    # Set motor channel
    motor_channel_left = 0
    motor_channel_right = 1
    motor_unload = 2

    # Set default wheel speed
    v = 3
    vl = 3
    vr = 3

    # Initialize USB cam
    cap = nano.Camera(camera_type = 1, device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
    GPIO.setmode(GPIO.BOARD)

    # Set input pin for ir sensors
    inPin = 15
    inPin2 = 19

    # Initialize input pin for ir sensors
    GPIO.setup(inPin, GPIO.IN)
    GPIO.setup(inPin2, GPIO.IN)

    # Set up aruco
    N_ARUCO = 3 # max aruco ID
    t_aruco = 1 # id of target aruco
    rel_dis = {}
    marker_size = 100
    t_bot = np.array([0, 0, 0])
    atTarget = False
    with open('/home/ai/Desktop/Code2/ar_single/camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)
    
    # Other variables
    SCAN_TIME = 25
    R = 0.03
    BASELINE = 0.24
    MAX_SPEED = 255
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
    
    time.sleep(5)

    vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    print("vl is " + str(vl))
    print("vr is " + str(vr))
    turn(servo, motor_channel_left, motor_channel_right, vl, vr)

    # V = 1.6
    # SPEED = 0.104
    # TRAVEL_TIME = 5 
    # move_forward(servo, motor_channel_left, motor_channel_right, V, TRAVEL_TIME)


if __name__ == "__main__":
     main()

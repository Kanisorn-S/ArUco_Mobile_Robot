import cv2 as cv
import nanocamera as nano
import time
from motor import move_backward, move_forward, turn, stop
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
import Adafruit_PCA9685
# from god import god

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
    with open('/home/ai/Desktop/Code2/ArUCo_Bot/camera_cal.npy', 'rb') as f:
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
    
    # Start spinning and Scanning
    vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    print("vl is " + str(vl))
    print("vr is " + str(vr))
    turn(servo, motor_channel_left, motor_channel_right, vl, vr)
    print("Code reaches here")
    scan(cap, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, N_ARUCO)
    print("Code ends")
    stop(servo, motor_channel_left, motor_channel_right)

    print("Complete scanning")
    print("Result:")
    print(rel_dis)
    
    rel_dis[0][2] = 0
    rel_dis[1][2] = - math.pi / 2
    rel_dis[2][2] = math.pi
    rel_dis[3][2] = math.pi / 2
    
    f = open("rel_dis2.txt", "w")
    f.write(str(rel_dis))
    f.close()
    alpha, t_angle, tvec = complete(cap, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis) 
    print("t_angle is " + str(t_angle))
    target_angle = t_angle
    if (target_angle < -180) or (target_angle > 180):
        target_angle = - (360 - target_angle)
    # target_angle = rel_dis[t_aruco][2] - current_angle
    f = open("rel_dis2.txt", "a")
    f.write(f'current angle is : {str(math.degrees(alpha))}, tvec is : {str(tvec)}, target_angle is : {str(math.degrees(target_angle))}')
    f.close()
    print("sleeping")
    time.sleep(5)
    TURN_SPEED = 0.404 # rad / s
    DURATION_FACTOR = 1 / TURN_SPEED
    vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    if t_angle < 0:
        vl = -vl 
        vr = -vr
    print("vl is " + str(vl))
    print("vr is " + str(vr))
    print("spin time is " + str(abs(DURATION_FACTOR * target_angle)))
    turn(servo, motor_channel_left, motor_channel_right, vl, vr, duration = abs(DURATION_FACTOR * target_angle))

if __name__ == "__main__":
    main()

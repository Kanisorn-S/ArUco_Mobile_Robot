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
import importlib.util
import sys
import importlib

''' List of functions that needs to be change for single camera
    - god #
    - scan #
    - complete #
    - obstacle avoidance #
''' 


def god(SCAN_TIME, R, BASELINE, MAX_SPEED, servo, motor_channel_left, motor_channel_right, cap, N_ARUCO, t_aruco, rel_dis, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, inPin2):
    
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

    print("sleeping")
    time.sleep(5)

    TRAVEL_TIME = 15
    vl, vr = inv_kine(TRAVEL_TIME, R, BASELINE, MAX_SPEED, tvec = tvec)
    #TRAVEL_TIME = 0
    move_forward(servo, motor_channel_left, motor_channel_right, vl)
    end_time = datetime.now() + timedelta(seconds = TRAVEL_TIME)

    # Obstacle Avoidance
    while True:
            if datetime.now() > end_time:
                break
            ret = obstacleAvoidance(cap)
            TURN_SPEED = 0.404 # rad / s
            DURATION_FACTOR = 1 / TURN_SPEED
            result = read_sensor(inPin2)
            vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
            print("ret is " + str(ret))
            if ret == 1:
                print("Object on the left")
                t_angle = - math.pi / 5
                # Stop moving
                if t_angle < 0:
                    vl = -vl 
                    vr = -vr
                stop(servo, motor_channel_left, motor_channel_right)
                # Obstacle detected on the left side
                turn(servo, motor_channel_left, motor_channel_right, vl, vr, t_angle * DURATION_FACTOR)
                move_forward(servo, motor_channel_left, motor_channel_right, vl, 3)
                print("Dodging to the right")
                rel_dis.clear()
                god(SCAN_TIME, R, BASELINE, MAX_SPEED, servo, motor_channel_left, motor_channel_right, cap, N_ARUCO, t_aruco, rel_dis, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, inPin2)
                
            elif ret == -1:
                print("Object on the right")
                t_angle = math.pi / 5
                # Stop moving
                stop(servo, motor_channel_left, motor_channel_right)
                # Obstacle detected on the left side
                turn(servo, motor_channel_left, motor_channel_right, vl, vr, t_angle * DURATION_FACTOR)
                move_forward(servo, motor_channel_left, motor_channel_right, vl, 3)
                print("Dodging to the left")
                rel_dis.clear()
                god(SCAN_TIME, R, BASELINE, MAX_SPEED, servo, motor_channel_left, motor_channel_right, cap, N_ARUCO, t_aruco, rel_dis, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, inPin2)
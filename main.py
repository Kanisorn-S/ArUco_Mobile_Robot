from locate import r_dis, locate, check_complete_dict, intersect, find_tvec, clean_tvec, complete
from move import find_target_angle, inv_kine
from motor import move_forward, stop, turnL, turnR, turn, unload, load
from detect import isRotationMatrix, rotationMatrixToEulerAngles
from obstacle import obstacleAvoidance, whileMoving
from move import locate_and_move
from scan import scan
import numpy as np
import math 
from threading import Thread # Using multi-threading for rotating the robot while running the ArUCo scan
import cv2 as cv 
import cv2.aruco as aruco 
import nanocamera as nano
import RPi.GPIO as GPIO 
import Adafruit_PCA9685

# set GPIO PIN for ir sensor input
GPIO.setmode(GPIO.BOARD)
inPin = 15
GPIO.setup(inPin, GPIO.IN)

# set const parameters
MAX_SPEED = 1
R = 0
BASELINE = 0
N_ARUCO = 5
SCAN_TIME = 20
ROT_TIME = 5
TRAVEL_TIME = 10
t_bot = np.array([0, 0, 0])
thirty_degrees = 30 * math.pi / 180

# define useful variable
rel_dis = {}
t_aruco = 0
anchors = []
obstacle = False
atTarget = False
atBase = True
turnVelocity = 0
forwardVelocity = 0
duration = 0

# Initialize the PCA9685 using the default address (0x40).
servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
servo.set_pwm_freq(60)

# Define the channel numbers for the motors.
motor_channel_left = 0
motor_channel_right = 1
motor_unload = 2
unload_motor_v = 0.5
unload_duration = load_duration = 5

# ArUco marker size
marker_size = 100 

with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

# ArUco markers dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# Initialize two CSI cameras
cap1 = nano.Camera(device_id = 0, flip = 2, width  = 640, height = 480, fps = 60)
cap2 = nano.Camera(device_id = 1, flip = 2, width  = 640, height = 480, fps = 60)

# initial scan
scan(cap1, cap2, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot)

while True:
    if (atBase and not GPIO.input(inPin)) or t_aruco == 0:

        locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)

        # Run while moving
        whileMoving(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, turnVelocity, forwardVelocity, duration, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
        unload(servo, motor_unload, unload_motor_v, unload_duration)
        load(servo, motor_unload, unload_motor_v, load_duration)
        # Arrived at target
        if t_aruco == 1:
            t_aruco = 0
            atTarget = False 
        else:
            t_aruco = 1
            atTarget = False
            atBase = True

        

            

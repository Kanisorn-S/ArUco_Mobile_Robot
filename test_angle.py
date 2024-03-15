from check_scan import scan, test
from locate2 import complete
import cv2 as cv
import cv2.aruco as aruco
import nanocamera as nano
import numpy as np
import time
import math

cap = nano.Camera(flip = 2, width  = 640, height = 480, fps = 60)

N_ARUCO = 2
t_aruco = 0
rel_dis = {}
SCAN_TIME = 10
R = 20
BASELINE = 20
MAX_SPEED = 255

marker_size = 100
t_bot = np.array([690, 0, 0])

with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

# ArUco markers dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

scan(cap, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, N_ARUCO)

print("Complete scanning")
print("Result:")
print(rel_dis)

print("-----------------------------------------")
print("Beginning new phase")
time.sleep(10)

t_angle, tvec = complete(cap, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis)

print("Target translational vector is: ")
print(tvec)
print("Target angle is: ")
print(math.degrees(t_angle))
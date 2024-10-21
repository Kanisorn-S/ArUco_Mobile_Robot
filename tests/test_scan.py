from src.check_scan import scan
import cv2 as cv
import cv2.aruco as aruco
import nanocamera as nano
import numpy as np

cap1 = nano.Camera(flip = 2, width  = 640, height = 480, fps = 60)
cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 60)

N_ARUCO = 2
t_aruco = 0
rel_dis = {}
SCAN_TIME = 10
R = 20
BASELINE = 20
MAX_SPEED = 255

marker_size = 100
t_bot = np.array([0, 0, 0])

with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

# ArUco markers dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

scan(cap1, cap2, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, N_ARUCO)

print("Complete scanning")
print("Result:")
print(rel_dis)

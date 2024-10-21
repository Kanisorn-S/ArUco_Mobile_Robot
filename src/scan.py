from locate import r_dis, locate, check_complete_dict, clean_tvec
from move import find_target_angle, inv_kine
from motor import move_forward, stop, turn 
from calibration.detect import isRotationMatrix, rotationMatrixToEulerAngles
import numpy as np
import math 
from threading import Thread # Using multi-threading for rotating the robot while running the ArUCo scan
import cv2 as cv 
import cv2.aruco as aruco 

def scan(cap1, cap2, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot):
    # 360 Scan
    
    vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    Thread(target = turn, daemon = True, args = (vl, vr)).start()

    rel_dis1 = {}
    rel_dis2 = {}

    while not check_complete_dict(N_ARUCO, rel_dis):
        frame1 = cap1.read()
        frame2 = cap2.read()

        gray_frame1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        gray_frame2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

        corners1, ids1, rejected1 = aruco.detectMarkers(gray_frame1, aruco_dict, camera_matrix, camera_distortion)
        corners2, ids2, rejected2 = aruco.detectMarkers(gray_frame2, aruco_dict, camera_matrix, camera_distortion)

        if ids1 is not None:

            rvec_list_all1, tvec_list_all1, _objPoints1 = aruco.estimatePoseSingleMarkers(corners1, marker_size, camera_matrix, camera_distortion)

            realworld_tvec1 = clean_tvec(ids1, tvec_list_all1, rvec_list_all1)

            locate(ids1, realworld_tvec1, rel_dis1, t_bot, N_ARUCO)

        if ids2 is not None:

            rvec_list_all2, tvec_list_all2, _objPoints2 = aruco.estimatePoseSingleMarkers(corners2, marker_size, camera_matrix, camera_distortion)

            realworld_tvec2 = clean_tvec(ids2, tvec_list_all2, rvec_list_all2)

            locate(ids2, realworld_tvec2, rel_dis2, t_bot, N_ARUCO)
    
    for i in range(N_ARUCO + 1):
        rel_dis[i] = (rel_dis1[i] + rel_dis2[i]) / 2
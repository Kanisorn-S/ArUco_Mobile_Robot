from locate import r_dis, locate, check_complete_dict, clean_tvec
from move import find_target_angle, inv_kine
from motor import turn
#from motor import move_forward, stop, turn 
#from detect2 import isRotationMatrix, rotationMatrixToEulerAngles
import numpy as np
import math 
from threading import Thread # Using multi-threading for rotating the robot while running the ArUCo scan
import cv2 as cv 
import cv2.aruco as aruco 
import time 
import multiprocessing as mp

# def test(cap):
#     while True:
#         frame = cap.read()
#         correct = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
#         cv.imshow("display", correct)
#         # print("turning")
#         time.sleep(5)

def scan(cap, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, n_aruco):
    # 360 Scan
    print("scanning")
    # vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    # spin = mp.Process(target = turn, args = (vl, vr))
    # spin.start()

    while not check_complete_dict(N_ARUCO, rel_dis):
        print("rel_dis is not complete")
        frame = cap.read()


        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

        if ids is not None:
            rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            # realworld_tvec points from aruco to camera
            realworld_tvec = clean_tvec(ids, tvec_list_all, rvec_list_all)

            locate(ids, realworld_tvec, rel_dis, t_bot, n_aruco)

            print("rel_dis is now: ")
            print(rel_dis)
        
        # cv.imshow("display1", frame1)
        # cv.imshow("display2", frame2)

        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    
    # cap1.release()
    # cap2.release()
    # cv.destroyAllWindows()


    

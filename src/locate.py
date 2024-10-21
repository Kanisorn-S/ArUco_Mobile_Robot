import numpy as np
import math
import importlib
import cv2 as cv
import cv2.aruco as aruco
from move import find_target_angle

def r_dis(t_bot, tvec):
    '''
    Takes the position of the robot and a tvec pointing from 
    a aruco to the robot then returns the position of the aruco
    '''

    return t_bot - tvec

def locate(ids, realworld_tvec, rel_dis, t_bot, n_aruco):
    # Unwrap ids
    ids = [id[0] for id in ids]
    print("locating")
    for i, id in enumerate(ids):
        if id <= n_aruco:
            if id not in rel_dis.keys():
                rel_dis[id] = [r_dis(t_bot, realworld_tvec[i])]
            elif len(rel_dis[id]) < 20:
                rel_dis[id].append(r_dis(t_bot, realworld_tvec[i]))

def average(rel_dis):
    for id in rel_dis.keys():
        sum = np.array([0, 0, 0])
        length = len(rel_dis[id])
        for tvec in rel_dis[id]:
            sum = sum + tvec
        rel_dis[id] = sum / length

def check_complete_dict(n_aruco, rel_dis):
    for i in range(n_aruco + 1):
        if i not in rel_dis.keys():
            return False 
    # for value in rel_dis.values():
    #     if len(value) < 10:
    #         return False
    average(rel_dis)
    return True

def find_tvec(ids, tvec_list_all, t_id, rel_dis):
    '''
    Takes an input ids : a list of ArUCo ids detected in the frame
                   tvec_list_all : a list of translational matrices in the order of ids
                   t_id : the ArUCo id of the target ArUCo 
                   rel_dis : a dictionary containing relative distance between ArUCo markers
    returns a translational vector that points from the target ArUCo to the camera
    '''
    # checkin ids
    print(str(ids))
    # Unwrap ids
    # ids = [id[0] for id in ids]

    # if t_id in ids:
    #     return tvec_list_all[ids.index(t_id)]
    # elif ids[0] in rel_dis.keys():
    #     id = ids[0]
    #     a = rel_dis[id]
    #     b = rel_dis[t_id]
    #     c = a - b 
    #     d = tvec_list_all[0]
    #     return c + d 
    # else:
    #     return np.array([0, 1, 0])
    return rel_dis[t_id]


def clean_tvec(ids, tvec_list_all, rvec_list_all):
    '''
    Utility function that takes raw input from aruco detection and clean it by
    1. Unwrapping both tvec_list_all and rvec_list_all
    2. Flips all rvecs and tvecs
    3. Return a list of tvecs in order of ids ready for use
    '''

    # Unwrap tvec_list_all and rvec_list_all
    tvec_list_all = [tvec[0] for tvec in tvec_list_all]
    rvec_list_all = [rvec[0] for rvec in rvec_list_all]

    # Flip al rvecs and tvecs
    rvec_list_all = [rvec * -1 for rvec in rvec_list_all]
    tvec_list_all = [tvec * -1 for tvec in tvec_list_all]

    # Create a list of rotational matrices in order of ids
    rotational_matrices = [cv.Rodrigues(rvec)[0] for rvec in rvec_list_all]

    # Create final list of tvecs that points from the target to the camera
    realworld_tvec = []
    for i in range(len(ids)):
        realworld_tvec.append(np.dot(rotational_matrices[i], tvec_list_all[i]))

    return realworld_tvec

def complete(cap, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, find_pos = False):
    '''
    Takes in all information of a single camera and returns a tvec pointing from the target to the camera
    '''
    # Start moving towards target ArUCo
    frame = cap.read()

    gray_frame = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

    if ids is not None:

        aruco.drawDetectedMarkers(frame, corners)
        rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        # Clean tvecs
        realworld_tvec = clean_tvec(ids, tvec_list_all, rvec_list_all)

        if not find_pos:
            # clean ids
            ids = [id[0] for id in ids]
            # Finding current position of robot
            id = ids[0]
            # Find translational vector that points from the target ArUCo to the camera
            target_tvec = find_tvec(ids, realworld_tvec, t_aruco, rel_dis)

            rvec = rvec_list_all[0][0]
            tvec = tvec_list_all[0][0]

            # Find target angle to reorient robot
            current_angle, target_angle = find_target_angle(id, target_tvec, rvec, rel_dis, t_aruco)

        else:
            # clean ids
            # ids = [id[0] for id in ids]
            # Finding current position of robot
            id = ids[0]
            a = rel_dis[id]
            d = tvec_list_all[0]
            target_tvec = a + d 
            target_angle = 0




        return current_angle, target_angle, target_tvec
    
    else:
        return rel_dis[t_aruco][2], 0, rel_dis[t_aruco]

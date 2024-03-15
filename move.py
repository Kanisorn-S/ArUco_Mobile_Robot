from locate import *
from motor import *
import numpy as np
import math
import cv2 as cv  
import importlib


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def find_target_angle(id, tvec, rvec, rel_dis, t_aruco):
    '''
    Takes an input tvec : a translational vector which points from the target to the camera
                   rvec : a rotational vector of either the target ArUCo or an anchor ArUCo
    returns absolute alpha
    (note: the returned angle is in radian)
    '''
    if id not in rel_dis.keys():
        return 0, 0
    rvec_flipped = rvec * -1
    rotation_matrix, jacobian = cv.Rodrigues(rvec_flipped)

    pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
    print("yaw is " + str(math.degrees(yaw)))

    # target_angle = yaw + math.atan(tvec[0] / tvec[1])
    # flip tvec to point from camera to target
    tvec_r = rel_dis[t_aruco]
    tvec_r[0] = -tvec_r[0]
    target_angle = math.atan(tvec_r[0]/tvec_r[1])
    current_angle = yaw + rel_dis[id][2]
    print(math.degrees(current_angle))
    print(str(rel_dis[t_aruco][2]) + ", " + str(current_angle) + ", " + str(target_angle))
    t_angle = rel_dis[t_aruco][2] - current_angle + target_angle

    return current_angle, t_angle

def inv_kine(time, r, baseline, max_speed,tvec = np.array([0, 0, 0]), theta = 0):
    '''
    Takes an input time : the amount of time that the robot will take to reach the target
                   r : the radius of the wheel
                   baseline : the distance between two wheels measured from the center of both wheels
                   max_speed : the speed of the wheel in rad / sec when the motor is set to 255
                   tvec : a translational vector which points from the target to the camera
                   theta : the angle that the robot needs to turn in radian
    returns pwml, pwmr which is the pwm of left and right servo motors
    '''

    # Inverse kinematics based on an analytical model of differential drive mobile robot
    tvec = tvec / 1000
    dis = math.sqrt(tvec[0]**2 + tvec[1]**2)
    forward_vel = dis / time 
    l = baseline / 2
    w = theta / time 
    vl = (forward_vel - w * l) / r
    vr = (forward_vel + w * l) / r
    # left turns counter-clockwise, right turns clockwise
    print("vl is " + str(vl) + " rad/s")
    print("vr is " + str(vr) + " rad/s")
    return vl, vr

def locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME):
    # Initial orioentation and movement
    alpha, tvec1 = complete(cap1, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis) # cap1 is left, t_angle1 is alpha
    beta, tvec2 = complete(cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis) # cap2 is right, t_angle2 is beta

    t_angle = ((thirty_degrees + beta) + (alpha - thirty_degrees)) / 2
    tvec = (tvec1 + tvec2) / 2

    # Use inverse kinematics to find wheel velocity and reorient the robot
    vl, vr = inv_kine(ROT_TIME, R, BASELINE, MAX_SPEED, theta = t_angle)
    turn(vl, vr)

    # Move towards the target
    vl, vr = inv_kine(TRAVEL_TIME, R, BASELINE, MAX_SPEED, tvec = tvec)
    move_forward(vl)





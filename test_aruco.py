from locate2 import complete
import nanocamera as nano
import cv2 as  cv
import numpy as np
import cv2.aruco as aruco
import time

with open('camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

rel_dis = {
    0 : np.array([710, 1105, 0]),
    1 : np.array([74, 765, 0]),
    2 : np.array([1285, 626, 0]),
    3 : np.array([366, 380, 0]),
    4 : np.array([1100, 930, 0]),
    5 : np.array([434, 1099, 0])
}
t_aruco = 0
marker_size = 100

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
cap = nano.Camera(flip = 2, width  = 640, height = 480, fps = 60)

while True:
    print("Running the program")
    frame = cap.read()
    target_angle, target_tvec = complete(cap, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis)
    print("Target Angle is " + str(target_angle))
    print("Target Tvec is " + str(target_tvec))
    tvec_str = "x=%4.0f, y=%4.0f, direction=%4.0f"%(target_tvec[0], target_tvec[1], target_angle)
    cv.putText(frame, tvec_str, (20, 400), cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv.LINE_AA)
    cv.imshow("frame", frame)

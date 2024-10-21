import cv2 as cv
import nanocamera as nano
import time
from src.motor import *
from threading import Thread
import RPi.GPIO as GPIO 
import multiprocessing as mp
from test_sensor import read_sensor
from datetime import datetime, timedelta
from src.check_scan import scan
import cv2.aruco as aruco
import numpy as np
from src.move import inv_kine
import math
import concurrent.futures
from src.locate import complete
from src.obstacle import obstacleAvoidance

def display(cap1, cap2, duration):
    while True:
            frame1 = cap1.read()
            frame2 = cap2.read()
            cv.imshow("display1", frame1)
            cv.imshow("display2", frame2)
            cv.waitKey(0)
def main():
    servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
    servo.set_pwm_freq(60)
    motor_channel_left = 0
    motor_channel_right = 1
    motor_unload = 2
    v = 3
    vl = 3
    vr = 3
    cap1 = nano.Camera(device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
    cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)
    GPIO.setmode(GPIO.BOARD)
    inPin = 15
    GPIO.setup(inPin, GPIO.IN)
    N_ARUCO = 2 # max aruco ID
    t_aruco = 0
    rel_dis = {}
    SCAN_TIME = 10
    R = 0.03
    BASELINE = 0.24
    MAX_SPEED = 255
    marker_size = 100
    t_bot = np.array([0, 0, 0])
    atTarget = False
    with open('camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)
    # ArUco markers dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    result = 1
    while result:
        result = read_sensor(inPin)
        print("current sensor reading is " + str(result))
    
    
    # vl, vr = inv_kine(SCAN_TIME, R, BASELINE, MAX_SPEED, theta = 2 * math.pi)
    # print("vl is " + str(vl))
    # print("vr is " + str(vr))
    # spin = mp.Process(target = turn, args = (servo, motor_channel_left, motor_channel_right, vl, vr))
    # spin.start()
    scan(cap1, cap2, N_ARUCO, t_aruco, rel_dis, SCAN_TIME, R, BASELINE, MAX_SPEED, aruco_dict, camera_matrix, camera_distortion, marker_size, t_bot, N_ARUCO)
    # spin.terminate()
    stop(servo, motor_channel_left, motor_channel_right)

    print("Complete scanning")
    print("Result:")
    print(rel_dis)

    left = mp.Value('i', 0)
    right = mp.Value('i', 0)

    avoid_left = mp.Process(target = obstacleAvoidance, args = (cap1, left, 1))
    avoid_right = mp.Process(target = obstacleAvoidance, args = (cap2, right, 2))
    # move = mp.Process(target = move_forward, args = (servo, motor_channel_left, motor_channel_right, v))

    avoid_left.start()
    avoid_right.start()
    # move.start()
    # with concurrent.futures.ProcessPoolExecutor() as executer:
    #              avoid_left = executer.submit(obstacleAvoidance, cap1)
    #              avoid_right = executer.submit(obstacleAvoidance, cap2)

    while not atTarget:
            # Get current robot position from surrounding ArUCo markers
            t_angle1, pos1 = complete(cap1, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, find_pos= True)
            t_angle2, pos2 = complete(cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, find_pos = True)

            pos = (pos1 + pos2) / 2

            # if at target, break
            #if rel_dis[t_aruco] - 5 <= pos <= rel_dis[t_aruco] + 5:
                #atTarget = True
                #break

            left_cam = left.value
            right_cam = right.value

            print("Suggested direction from left_cam is " + str(left_cam))
            print("Suggested direction from right_cam is " + str(right_cam))

            if left_cam == 1:
                print("Obstacle on the left!")
                time.sleep(5)
                # Obstacle detected on the left side
                #while left_cam != 0:
                    #print("Keep turning right")
                #     turnR(turnVelocity)
                # move_forward(forwardVelocity, duration)
                # locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
            elif right_cam == -1:
                print("Obstacle on the right!")
                time.sleep(5)
                #while right_cam != 0:
                    #print("Keep turning left")
                #     turnL(turnVelocity)
                # move_forward(forwardVelocity, duration)
                # locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
            elif (left_cam == -1 or right_cam == 1):
                print("Object detected ahead!")
                time.sleep(5)
                #while left_cam != 0:
                    #print("Keep turning right")
                #     turnR(turnVelocity)
                # move_forward(forwardVelocity, duration)
                # locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
    avoid_left.terminate()
    avoid_right.terminate()
    # move.terminate()
    # # while True:
        
    # #     #displaying = multiprocessing.Process(target = display, args = (cap1, cap2, 10))
    # #     moving = mp.Process(target = move_forward, args = (servo, motor_channel_left, motor_channel_right, v, 5))
    # #     frame1 = cap1.read()
    # #     frame2 = cap2.read()
    # #     cv.imshow("display1", frame1)
    # #     cv.imshow("display2", frame2)
    # #     #displaying.start()
    # #     moving.start()
    # #     #displaying.join()
    # #     #moving.join()
    # #     key = cv.waitKey(1) & 0xFF
    # #     if key == ord("q"):
    # #         break
    # #     print("Multiprocess is completed")
    
    # # cap1.release()
    # # cap2.release()
    # # cv.destroyAllWindows()
if __name__ == "__main__":
     main()

from time import sleep
from locate import complete
# from motor import *
import cv2 as cv 
import numpy as np
import math 
import os 
import nanocamera as nano
from threading import Thread
# from move import locate_and_move
import importlib
import multiprocessing as mp


def calc_dist(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist 

def getChunks(l, n):

    """Yield successive n-sized chunks from l."""

    a = []

    for i in range(0, len(l), n):   

        a.append(l[i:i + n])

    return a

def obstacleAvoidance(cap):
    '''
    Function that takes in a frame captured from the camera and
    returns the direction the robot should move
    '''
    StepSize = 5

    while True:

        frame = cap.read()

        img = frame.copy()

        # img = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        blur = cv.bilateralFilter(img,9,40,40)

        edges = cv.Canny(blur,50,100)

        img_h = img.shape[0] - 1

        img_w = img.shape[1] - 1

        EdgeArray = []

        for j in range(0,img_w,StepSize):

            pixel = (j,0)

            for i in range(img_h-5,0,-1):

                if edges.item(i,j) == 255:

                    pixel = (j,i)

                    break

            EdgeArray.append(pixel)


        #for x in range(len(EdgeArray)-1):

            #cv.line(img, EdgeArray[x], EdgeArray[x+1], (0,255,0), 1)



        #for x in range(len(EdgeArray)):

            #cv.line(img, (x*StepSize, img_h), EdgeArray[x],(0,255,0),1)


        chunks = getChunks(EdgeArray,int(len(EdgeArray)/3)) # 5

        max_dist = 0

        c = []

        for i in range(len(chunks)-1):        

            x_vals = []

            y_vals = []

            for (x,y) in chunks[i]:

                x_vals.append(x)

                y_vals.append(y)


            avg_x = int(np.average(x_vals))

            avg_y = int(np.average(y_vals))

            c.append([avg_y,avg_x])

            #cv.line(frame,(320,480),(avg_x,avg_y),(255,0,0),2)  

        # print(c)

        forwardEdge = c[1]
        # print(forwardEdge)

        #cv.line(frame,(320,480),(forwardEdge[1],forwardEdge[0]),(0,255,0),3)   
        
        y = (min(c))
        # print(y)
        
        if forwardEdge[0] > 300: #200 # >230 works better 

            if y[1] < 310:
                direction = "left "
                ret = -1
                # print(direction)

            else: 
                direction = "right "
                ret = 1
                # print(direction)

        else:
            direction = "forward "
            ret = 0
            # print(direction)
        
        # cv.imshow("frame" + str(num),frame)

        key = cv.waitKey(5) & 0xFF
        if key == 27:
            break



        return ret
    cap.release()
    cv.destroyAllWindows()


# def whileMoving(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, turnVelocity, forwardVelocity, duration, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME):
#     # Loop check for obstacle until target is reach
#         while not atTarget:
#             # Get current robot position from surrounding ArUCo markers
#             t_angle1, pos1 = complete(cap1, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, find_pos= True)
#             t_angle2, pos2 = complete(cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, find_pos = True)

#             pos = (pos1 + pos2) / 2

#             # if at target, break
#             if rel_dis[t_aruco] - 5 <= pos <= rel_dis[t_aruco] + 5:
#                 atTarget = True
#                 break
            
#             # Obstacle avoidance
#             left_cam = Thread(target = obstacleAvoidance, daemon = True, args = (cap1, )).start()
#             right_cam = Thread(target = obstacleAvoidance, daemon = True, args = (cap2, )).start()

#             if left_cam == "right":
#                 # Obstacle detected on the left side
#                 while left_cam != "straight":
#                     turnR(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
#             elif right_cam == "left":
#                 while right_cam != "straight":
#                     turnL(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
#             elif (left_cam == "left" or right_cam == "right"):
#                 while left_cam != "straight":
#                     turnR(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)


# def whileMoving2(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, turnVelocity, forwardVelocity, duration, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME):
#     # Loop check for obstacle until target is reach
#         while True:
#             var_l = 0
#             var_r = 0
#             # Obstacle avoidance
#             left_cam = mp.Process(target = obstacleAvoidance, args = (cap2, var_l))
#             right_cam = mp.Process(target = obstacleAvoidance, args = (cap1, var_r))
#             left_cam.start()
#             right_cam.start()
            
#             if var_l == -1:
#                 # Obstacle detected on the left side
#                 while var_l == -1:
#                     turnR(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
#             elif right_cam == "left":
#                 while right_cam != "straight":
#                     turnL(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)
#             elif (left_cam == "left" or right_cam == "right"):
#                 while left_cam != "straight":
#                     turnR(turnVelocity)
#                 move_forward(forwardVelocity, duration)
#                 locate_and_move(cap1, cap2, aruco_dict, camera_matrix, camera_distortion, marker_size, t_aruco, rel_dis, thirty_degrees, ROT_TIME, R, BASELINE, MAX_SPEED, TRAVEL_TIME)




def obstacleAvoidance2(cap, cap_id, queue):
        '''
        Function that takes in a frame captured from the camera and
        returns the direction the robot should move
        '''
        StepSize = 5

    

        frame = cap.read()

        img = frame.copy()

        # img = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        blur = cv.bilateralFilter(img,9,40,40)

        edges = cv.Canny(blur,50,100)

        img_h = img.shape[0] - 1

        img_w = img.shape[1] - 1

        EdgeArray = []

        for j in range(0,img_w,StepSize):

            pixel = (j,0)

            for i in range(img_h-5,0,-1):

                if edges.item(i,j) == 255:

                    pixel = (j,i)

                    break

            EdgeArray.append(pixel)


        for x in range(len(EdgeArray)-1):

            cv.line(img, EdgeArray[x], EdgeArray[x+1], (0,255,0), 1)



        for x in range(len(EdgeArray)):

            cv.line(img, (x*StepSize, img_h), EdgeArray[x],(0,255,0),1)


        chunks = getChunks(EdgeArray,int(len(EdgeArray)/3)) # 5

        max_dist = 0

        c = []

        for i in range(len(chunks)-1):        

            x_vals = []

            y_vals = []

            for (x,y) in chunks[i]:

                x_vals.append(x)

                y_vals.append(y)


            avg_x = int(np.average(x_vals))

            avg_y = int(np.average(y_vals))

            c.append([avg_y,avg_x])

            cv.line(frame,(320,480),(avg_x,avg_y),(255,0,0),2)  

        #print(c)
        forwardEdge = c[0]
        Edge = c[1]
        if cap_id == 0:
            Edge = c[2]
        else:
            Edge = c[0]
        #print(Edge)

        cv.line(frame,(320,480),(forwardEdge[1],forwardEdge[0]),(0,255,0),3)   
        
        y = (min(c))
        #print(y)
        # Left cam
        if cap_id == 0:
            if Edge[0] > 330: #200 # >230 works better 
                direction = "left"
                ret = -1

            else:
                direction = "forward "
                ret = 0
                # print(direction)
            
            #cv.imshow("frame" + str(num),frame)
        # Right cam
        else:
            if Edge[0] > 260: #200 # >230 works better 
                direction = "right"
                ret = 1

            else:
                direction = "forward "
                ret = 0
                # print(direction)
            
            #cv.imshow("frame" + str(num),frame)

        #key = cv.waitKey(5) & 0xFF
        #if key == 27:
            #break

        queue.put((cap_id, ret))
        # cv.imshow("result", img)
        # return direction
    
    #cap.release()
    #cv.destroyAllWindows()
    
#cap1 = nano.Camera(device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
#cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)

#var = {0 : 0}
#while True:
    #obstacleAvoidance2(cap2, var, 1)
    #print("var is " + str(var[0]))
    

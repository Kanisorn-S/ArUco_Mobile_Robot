from time import sleep
import cv2 as cv 
import numpy as np 
import math
import os 
import nanocamera as nano

# Moves the robot forward for a set time
def forward():
    #TODO
    print("Moving forward!")

# Moves the robot backward for a set time
def backward():
    #TODO
    print("Moving backward!")

# Moves the robot to the right for a set time
def right():
    #TODO
    print("Moving to the right!")

# Moves the robot to the left for a set time
def left():
    #TODO 
    print("Moving to the left!") 

# Stop the robot
def stop():
    #TODO
    print("Stopping!")

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

testmode = 2

cap = nano.Camera(device_id = 0, flip = 2, width  = 640, height = 480, fps = 60)

try:
   if not os.path.exists('data'):
      os.makedirs('data')
except OSError:
   print ('Error: Creating directory of data')

StepSize = 5
currentFrame = 0

if testmode == 1:
   F = open("./data/imagedetails.txt",'a')
   F.write("\n\nNew Test \n")


while(1):

    frame = cap.read()

    if testmode == 1:
        name = './data/frame' + str(currentFrame) + '.jpg'
        print ('Creating...' + name)
    
    img = frame.copy()

    img = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

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

    print(c)

    forwardEdge = c[1]
    print(forwardEdge)

    cv.line(frame,(320,480),(forwardEdge[1],forwardEdge[0]),(0,255,0),3)   
    # cv.imwrite("feed", frame)
     
    y = (min(c))
    print(y)
    
    if forwardEdge[0] > 250: #200 # >230 works better 

       if y[1] < 310:
          left()
          #pwm.start(0)
          #pwm1.start(40)
          direction = "left "
          print(direction)

       else: 
          right()
          direction = "right "
          print(direction)

    else:
       forward()
#       sleep(0.005)
       direction = "forward "
       print(direction)
       
    if testmode == 1:
       F.write ("frame"+str(currentFrame)+".jpg" +" | " + str(c[0]) + " | " + str(c[1]) + " | " +str(c[2])  + " | " + direction + "\n") 
       currentFrame +=1

    if testmode == 2:

       cv.imshow("frame",frame)

       cv.imshow("Canny",edges)

       cv.imshow("result",img)


    k = cv.waitKey(5) & 0xFF  ##change to 5

    if k == 27:

        break


cv.destroyAllWindows
cap.release()

import cv2 as cv
import nanocamera as nano
import time


cap1 = nano.Camera(camera_type = 1, device_id = 0, flip = 2, width = 640, height = 480, fps = 30)
#cap2 = nano.Camera(device_id = 1, flip = 2, width = 640, height = 480, fps = 30)
#cap = cv.VideoCapture('/dev/video0')
while True:
    #ret, frame = cap.read()
    #cv.imshow("display", frame)
    frame1 = cap1.read()
    #frame2 = cap2.read()
    cv.imshow("left view", frame1)
    #cv.imshow("right view", frame2)

    key = cv.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap1.release()
#cap2.release()
cv.destroyAllWindows()

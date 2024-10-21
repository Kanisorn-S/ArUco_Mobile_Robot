import cv2 as cv 
import numpy as np 

cap = cv.VideoCapture(0)
stepSize = 5
while True:
    ret, frame = cap.read()
    img = frame.copy()
    blur = cv.bilateralFilter(img, 9, 40, 40)
    edges = cv.Canny(blur, 50, 100)
    cv.imshow("Canny", edges)
    img_h = img.shape[0] - 1
    img_w = img.shape[1] - 1
    EdgeArray = []
    for j in range(0, img_w, stepSize):
        pixel = (j, 0)
        for i in range(img_h - 5, 0, -1):
            if edges.item(i, j) == 255:
                pixel = (j, i)
                break
        EdgeArray.append(pixel)
    
    for x in range(len(EdgeArray) - 1):
        cv.line(img, EdgeArray[x], EdgeArray[x + 1], (0, 255, 0), 1)
    for x in range(len(EdgeArray)):
        cv.line(img, (x*stepSize, img_h), EdgeArray[x], (0, 255, 0), 1)
    cv.imshow("result", img)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()
cap.release()
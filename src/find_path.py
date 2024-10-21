import nanocamera as nano
import cv2 as cv
import cv2.aruco as aruco
import os

file_path_nano = nano.__file__
file_path_cv = cv.__file__
#file_path_aruco = aruco.__file__

dir_nano = os.path.dirname(file_path_nano)
dir_cv = os.path.dirname(file_path_cv)
#dir_aruco = os.path.dirname(file_path_aruco)

print("Path for nanocamera : " + dir_nano)
print("Path for cv : " + dir_cv)
#print("Path for aruco : " + dir_aruco)

# path for nanocamera
# /home/ai/.local/lib/python3.6/site-packages/nanocamera

# path for cv
# /home/ai/.local/lib/python3.6/site-packages/cv2

# path for aruco

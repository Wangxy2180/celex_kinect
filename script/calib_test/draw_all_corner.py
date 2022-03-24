'''
Author: wangxy
Date: 2022-03-24 14:39:52
LastEditTime: 2022-03-24 15:05:44
LastEditors: wangxy
Description: none
FilePath: /src/celex_kinect/script/calib_test/draw_all_corner.py
'''
import os
import cv2
from cv2 import IMREAD_GRAYSCALE



# src_path = '/home/free/catkin_cd/celex_kinect_calib/color'
# src_path = '/home/free/catkin_cd/celex_kinect_calib/ir'
src_path = '/home/free/catkin_cd/celex_kinect_calib/ig_gray'
print (os.listdir(src_path))
image_name=os.listdir(src_path)

criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 50, 0.001)


for name in image_name:
    image = cv2.imread(os.path.join(src_path,name),IMREAD_GRAYSCALE)
    retval, corners = cv2.findChessboardCorners(image, (5,7),None)
    corners2 = cv2.cornerSubPix(image, corners, (11,11), (-1,-1), criteria)
    image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(image, (11,8), corners2, retval)   # 记住，OpenCV的绘制函数一般无返回值
    cv2.imshow('img', image)
    cv2.waitKey(500)

'''
Author: wangxy
Date: 2022-01-03 15:00:00
LastEditTime: 2022-01-03 15:09:08
LastEditors: wangxy
Description: none
FilePath: /src/script/OpenCVTest.py
'''
from cv2 import cv2
import numpy as np

# from cv2 import cv2

img_in = np.zeros((10,10,1))
channel=0
hist_info=[]
hist_size=128
hist_range=[1,128]
cv2.calcHist(img_in,1,channel,np.zeros((10,10,1)),hist_info,1,hist_size,hist_range[0])

print(len(hist_info))
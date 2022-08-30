import cv2
import numpy as np
import os
from glob import glob

basePath = "/home/krri/brtdata/krri_dataset/Scenario/"
frame_array = []
concat_path = basePath
pathOut = "/home/krri/brtdata/krri_dataset/Scenario/Scenario.mp4"  

file_list = glob(basePath)
print(file_list)

for file in file_list:
    img = cv2.imread(file)
    height, width, layers = img.shape
    size = (width,height)
    print(file,end='\r')
    frame_array.append(img)
    
    
out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), 10, size)
print(pathOut)
for i in range(len(frame_array)):
    out.write(frame_array[i])
out.release()
        
    
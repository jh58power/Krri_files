import cv2
import numpy as np
import os

labelPath = "/home/krri/HDD/dataset/철도_샘플데이터/Cam/ver1/20d1c1/obj_train_data/"
imgPath = "/home/krri/brtdata/exp_20201111/raw/Train_Test Dataset/Train/Day/D1/c1/"

labelfile = os.listdir(labelPath)
imgfile = os.listdir(imgPath)

for label, imgfile in zip(labelfile,imgfile):
    f = open(labelPath+label,"r")
    img = cv2.imread(imgPath+label.split(".")[0]+".jpg")
    while True:
        line = f.readline()   
        if not line:
            break
        x_cen = float(line.split()[1]) * 1920
        y_cen = float(line.split()[2]) * 1024
        w = float(line.split()[3]) * 1920
        h = float(line.split()[4]) * 1024

        x1 = x_cen - w/2
        y1 = y_cen - h/2
        x2 = x_cen + w/2
        y2 = y_cen + h/2

        img = cv2.rectangle(img,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),1)

    print(label,imgfile)
    cv2.imshow("img",img)
    key_input = cv2.waitKey(0)
    if key_input & 0xFF == ord('q'): 
        break
    
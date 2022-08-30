import cv2
import numpy as np
import os
import glob

filePath = "/home/krri/brtdata/krri_dataset/Scenario/"

imgfile = glob.glob(filePath+"*.jpg")
labelfile = glob.glob(filePath+"*.txt")

imgfile.sort()
labelfile.sort()

class_dict = {0:"0 Car",
              1:"1 Bus",
              2:"2 Truck",
              3:"3 Bike",
              4:"4 Pedestrian"
              }

for label in labelfile:
    f = open(label,"r")
    img = cv2.imread(label.split(".")[0]+".jpg")
    print(label.split(".")[0]+".jpg",end='\r')
    
    while True:
        line = f.readline()   
        if not line:
            break
        class_n = int(line.split()[0])
        x_cen = float(line.split()[1]) * 1920
        y_cen = float(line.split()[2]) * 1024
        w = float(line.split()[3]) * 1920
        h = float(line.split()[4]) * 1024

        x1 = x_cen - w/2
        y1 = y_cen - h/2
        x2 = x_cen + w/2
        y2 = y_cen + h/2

        try:
            img = cv2.putText(img,class_dict[class_n],(int(x1),int(y1)),cv2.FONT_HERSHEY_PLAIN,1.5,(0,255,0),2)
        except:
            continue
        img = cv2.rectangle(img,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),2)

    # cv2.imwrite("/home/krri/brtdata/krri_dataset/Img_with_label",img)
    
    cv2.imshow("img",img)
    key_input = cv2.waitKey(0)
    if key_input & 0xFF == ord('q'): 
        break
    
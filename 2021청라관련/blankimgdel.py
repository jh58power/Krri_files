import numpy as np
import os
import shutil

basePath = "rosbag2_2021_06_08-20_46_28"

imgPath_1 = "/home/krri/brtdata/2021_06_BAG/raw/" + basePath + "/sync_5_sec/c1/"
imgPath_2 = "/home/krri/brtdata/2021_06_BAG/raw/" + basePath + "/sync_5_sec/c2/"
lidarPath_1 = "/home/krri/brtdata/2021_06_BAG/raw/" + basePath + "/sync_5_sec/l1/"        
lidarPath_2 = "/home/krri/brtdata/2021_06_BAG/raw/" + basePath + "/sync_5_sec/l2/"   

img_list_1 = os.listdir(imgPath_1)
img_list_2 = os.listdir(imgPath_2)
lidar_list_1 = os.listdir(lidarPath_1)
lidar_list_2 = os.listdir(lidarPath_2)
last_index = [int(img_list_1[-1][:-4]),int(img_list_2[-1][:-4]),int(lidar_list_1[-1][:-4]),int(lidar_list_2[-1][:-4])] 
        
first_index = [int(img_list_1[0][:-4]),int(img_list_2[0][:-4]),int(lidar_list_1[0][:-4]),int(lidar_list_2[0][:-4])] 
max_index = max(last_index)
min_index = min(first_index)

for i in range(min_index,max_index,50):
    counter = 0
    img1 = imgPath_1 + str(i).zfill(6) + ".jpg"
    img2 = imgPath_2 + str(i).zfill(6) + ".jpg"
    ld1 = lidarPath_1 + str(i).zfill(6) + ".pcd"
    ld2 = lidarPath_2 + str(i).zfill(6) + ".pcd"

    if not os.path.exists(img1):
        counter += 1
    if not os.path.exists(img2):
        counter += 1
    if not os.path.exists(ld1):
        counter += 1
    if not os.path.exists(ld2):
        counter += 1
    
    if counter != 0 and counter != 4:
        if os.path.isfile(img1):
            os.remove(img1)
        if os.path.isfile(img2):
            os.remove(img2)
        if os.path.isfile(ld1):
            os.remove(ld1)        
        if os.path.isfile(ld2):
            os.remove(ld2)
        print(i)
        
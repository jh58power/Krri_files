import os
import shutil

from psutil import cpu_count

scenPath = "/home/krri/brtdata/2021_06_BAG/raw/Train_Test Dataset/Scenario/"

dstPath = "/home/krri/SSD/sfa3d_master/dataset/kitti/testing/velodyne_2021_final/"

scenList = os.listdir(scenPath)

for folder in scenList:
    l1_path = scenPath + folder + "/l1/" 
    l2_path = scenPath + folder + "/l2/"
    
    try:
        l1 = os.listdir(l1_path)
        l2 = os.listdir(l2_path)
    except:
        continue
    
    for i,j in zip(l1,l2):
        shutil.copyfile(l1_path + i, dstPath+i)
        shutil.copyfile(l2_path + j, dstPath+j)
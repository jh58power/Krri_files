from gettext import find
from tempfile import tempdir
from attr import has
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import shutil
import argparse
import math

from pandas import concat

class Img_Pcd_Viewer:
    def __init__(self, basePath):
        
        raw_path = "/home/krri/brtdata/exp_20201111/raw/Train_Test Dataset/Train/Night/"
        self.imgPath_1 = raw_path + basePath + "/c1"
        self.imgPath_2 = raw_path + basePath + "/c2"
        self.imgPath_3 = raw_path + basePath + "/c3"
        self.imgPath_4 = raw_path + basePath + "/c4"
        self.lidarPath_1 = raw_path + basePath + "/ld"
        
        self.savePath = raw_path + basePath + "/concat_img"
        if not os.path.exists(self.savePath):
            os.makedirs(self.savePath)
        self.rotation = False

        #폴더 이미지 리스트
        self.img_list_1 = os.listdir(self.imgPath_1)
        self.img_list_2 = os.listdir(self.imgPath_2)
        self.img_list_3 = os.listdir(self.imgPath_3)
        self.img_list_4 = os.listdir(self.imgPath_4)
        self.lidar_list_1 = os.listdir(self.lidarPath_1)

        #공백 이미지 생성
        self.tempImg = cv2.imread(self.imgPath_1 + "/" + self.img_list_1[0])
        self.blackImg = np.zeros((self.tempImg.shape[0],self.tempImg.shape[1],3),np.uint8)

        self.bevX = 900
        self.bevY = 600
        
    def rotMatrix(self,theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        return R
    
    def pcd_projection(self,pcd_data):
        x_size = self.bevX
        y_size = self.bevY
        x_range = x_size*0.03 #20.0
        y_range = y_size*0.03 #60.0
        grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size])
        image_size = np.array([x_size, y_size])
        pc = o3d.io.read_point_cloud(pcd_data)
        
        xyz = np.asarray(pc.points)
        R = self.rotMatrix(-0.785398) 
        xyz[:,[0,2]] = np.dot(R, xyz[:,[0,2]].T).T
        
        if self.rotation == True:
            xy = xyz[:,1:3]
        else:
            xy = np.asarray(pc.points)[:,0:2]
            
        #좌표 변환
        shifted_coord = xy[:, :2] + np.array([x_range, y_range])
        # image index
        index = np.floor(shifted_coord / grid_size).astype(np.int64)

        # choose illegal index
        bound_x = np.logical_and(index[:, 0] >= 0, index[:, 0] < image_size[0])
        bound_y = np.logical_and(index[:, 1] >= 0, index[:, 1] < image_size[1])
        bound_box = np.logical_and(bound_x, bound_y)
        index = index[bound_box]
        # show image
        image = np.zeros((x_size, y_size), dtype=np.uint8)
        image[index[:, 0], index[:, 1]] = 255
        return image

    def makeImg(self,imgPath):
        if os.path.exists(imgPath):
            img = cv2.imread(imgPath)
        else:
            img = self.blackImg
        
        return img
    
    def run(self):
        for c1,c2,c3,c4,ld in zip(self.img_list_1,self.img_list_2,self.img_list_3,self.img_list_4,self.lidar_list_1):
            imgPt1 = self.imgPath_1 + "/" + c1
            imgPt2 = self.imgPath_2 + "/" + c2
            imgPt3 = self.imgPath_3 + "/" + c3
            imgPt4 = self.imgPath_4 + "/" + c4
            pcdPt1 = self.lidarPath_1 + "/" + ld
            
            img1 = self.makeImg(imgPt1)
            img2 = self.makeImg(imgPt2)
            img3 = self.makeImg(imgPt3)
            img4 = self.makeImg(imgPt4)
            
            #이미지 4개 concat
            _img1 = np.hstack((img1,img2))
            _img2 = np.hstack((img3,img4))
            concat_img = np.vstack((_img1,_img2))
            
            #pcd bev 크기를 concat 한 후의 이미지 크기와 매칭

            if os.path.exists(pcdPt1):
                pc_image = self.pcd_projection(pcdPt1)
                pc_image = cv2.cvtColor(pc_image,cv2.COLOR_GRAY2BGR)
            else:
                pc_image = self.blackImg
                
            pc_image = cv2.resize(pc_image,dsize=(int(concat_img.shape[0]/1.5), concat_img.shape[0]), interpolation=cv2.INTER_CUBIC)
            
            pc_image_rot = cv2.rotate(pc_image,cv2.ROTATE_180)

            concat_img = np.hstack((concat_img,pc_image_rot))
            
            concat_img = cv2.resize(concat_img, dsize=(0,0),fx = 0.3, fy = 0.3, interpolation=cv2.INTER_AREA)
            
            name = self.savePath + "/" + c1[7:17]
            cv2.imwrite(name,concat_img)
            print(f"Saving file >> {c1[7:13]}",end='\r')

            # cv2.imshow("a",concat_img)
            # key_input = cv2.waitKey(0)
            # if key_input & 0xFF == ord('q'): 
            #     break
pathList = ["N1","N2","N3","N4"]
for path in pathList:
    print(f"Reading>> {path}")
    Img_Pcd_Viewer(path).run()
            
            
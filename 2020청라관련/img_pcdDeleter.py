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

        self.imgPath_1 = "/home/krri/brtdata/exp_20201111/raw/" + basePath + "/sync_5_sec/c1"
        self.imgPath_2 = "/home/krri/brtdata/exp_20201111/raw/" + basePath + "/sync_5_sec/c2"
        self.imgPath_3 = "/home/krri/brtdata/exp_20201111/raw/" + basePath + "/sync_5_sec/c3"
        self.imgPath_4 = "/home/krri/brtdata/exp_20201111/raw/" + basePath + "/sync_5_sec/c4"
        self.lidarPath_1 = "/home/krri/brtdata/exp_20201111/raw/" + basePath + "/sync_5_sec/ld"        

        #폴더 이미지 리스트
        self.img_list_1 = os.listdir(self.imgPath_1)
        self.img_list_2 = os.listdir(self.imgPath_2)
        self.img_list_3 = os.listdir(self.imgPath_3)
        self.img_list_4 = os.listdir(self.imgPath_4)
        self.lidar_list_1 = os.listdir(self.lidarPath_1)
        if "Thumbs.db" in self.img_list_1:
            self.img_list_1.remove("Thumbs.db")
        #공백 이미지 생성
        self.tempImg = cv2.imread(self.imgPath_1 + "/" + self.img_list_1[0])
        self.blackImg = np.zeros((self.tempImg.shape[0],self.tempImg.shape[1],3),np.uint8)


        #탐색할 첫번째 파일 인덱스와 마지막 인덱스 찾기
        last_index = [int(self.img_list_1[-1][:-4]),int(self.img_list_2[-1][:-4]),
                      int(self.img_list_3[-1][:-4]),int(self.img_list_4[-1][:-4]),int(self.lidar_list_1[-1][:-4])] 
        
        first_index = [int(self.img_list_1[0][:-4]),int(self.img_list_2[0][:-4]),
                       int(self.img_list_3[0][:-4]),int(self.img_list_4[0][:-4]),int(self.lidar_list_1[0][:-4])] 
        self.max_index = max(last_index)
        self.min_index = min(first_index)
        
        self.bevX = 900
        self.bevY = 600
        
        self.frame = 50
        self.flag = False
        
    def rotMatrix(self,theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        return R
    
    def pcd_projection(self,pcd_data):
        x_size = self.bevX
        y_size = self.bevY
        x_range = x_size*0.04 #20.0
        y_range = y_size*0.04 #60.0
        grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size])
        image_size = np.array([x_size, y_size])
        pc = o3d.io.read_point_cloud(pcd_data)
        
        xyz = np.asarray(pc.points)
        R = self.rotMatrix(-0.785398) 
        xyz[:,[0,2]] = np.dot(R, xyz[:,[0,2]].T).T

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
        i = self.min_index
        while i <= self.max_index:
            imgPt1 = self.imgPath_1 + "/" + str(i).zfill(6) + ".jpg" 
            imgPt2 = self.imgPath_2 + "/" + str(i).zfill(6) + ".jpg" 
            imgPt3 = self.imgPath_3 + "/" + str(i).zfill(6) + ".jpg" 
            imgPt4 = self.imgPath_4 + "/" + str(i).zfill(6) + ".jpg" 
            pcdPt1 = self.lidarPath_1 + "/" + str(i).zfill(6) + ".pcd"
            
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
            
            cv2.imshow("a",concat_img)
            print(f"Frame No.{i}",end="\r")
            if self.flag == True:
                if os.path.isfile(d_img1):
                    os.remove(d_img1)
                    print(d_img1," file Deleted!")
                if os.path.isfile(d_img2):
                    os.remove(d_img2)
                    print(d_img2," file Deleted!")
                if os.path.isfile(d_img3):
                    os.remove(d_img3)
                    print(d_img3," file Deleted!")
                if os.path.isfile(d_img4):
                    os.remove(d_img4)
                    print(d_img4," file Deleted!")
                if os.path.isfile(d_pcd):
                    os.remove(d_pcd)
                    print(d_pcd," file Deleted!")
                    
                flag = False

            key_input = cv2.waitKey(0)
            if key_input & 0xFF == ord('q'): 
                break
            elif key_input & 0xFF == ord('b'):
                i -= self.frame
                continue
            elif key_input & 0xFF == ord('d'):
                d_img1 = imgPt1
                d_img2 = imgPt2
                d_img3 = imgPt3
                d_img4 = imgPt4
                d_pcd = pcdPt1
                self.flag = True
                i += self.frame
                continue
            else:
                i += self.frame
                continue
            
pathList = "2020-11-11-16-08-26"

print(f"Reading>> {pathList}")
Img_Pcd_Viewer(pathList).run()
            
            

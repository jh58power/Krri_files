from gettext import find
from tempfile import tempdir
from attr import has
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os


from pandas import concat

class Img_Pcd_Viewer:
    def __init__(self, basePath):
        rawPath = "/home/krri/brtdata/2021_06_BAG/raw/"
        self.imgPath_1 = rawPath + basePath + "/sync2/c1"
        self.imgPath_2 = rawPath + basePath + "/sync2/c2"
        self.lidarPath_1 = rawPath + basePath + "/sync2/l1"
        self.lidarPath_2 = rawPath + basePath + "/sync2/l2"
        
        self.savePath = rawPath + basePath + "/sync2/concat_img"
        if not os.path.exists(self.savePath):
            os.makedirs(self.savePath)

        #폴더 이미지 리스트
        self.img_list_1 = os.listdir(self.imgPath_1)
        self.img_list_2 = os.listdir(self.imgPath_2)
        self.lidar_list_1 = os.listdir(self.lidarPath_1)
        self.lidar_list_2 = os.listdir(self.lidarPath_2)

        #공백 이미지 생성
        self.tempImg = cv2.imread(self.imgPath_1 + "/" + self.img_list_1[0])
        self.blackImg = np.zeros((self.tempImg.shape[0],self.tempImg.shape[1],3),np.uint8)


        #탐색할 첫번째 파일 인덱스와 마지막 인덱스 찾기
        last_index = [int(self.img_list_1[-1][:-4]),int(self.img_list_2[-1][:-4]),
                      int(self.lidar_list_1[-1][:-4]),int(self.lidar_list_2[-1][:-4])] 
        
        first_index = [int(self.img_list_1[0][:-4]),int(self.img_list_2[0][:-4]),
                       int(self.lidar_list_1[0][:-4]),int(self.lidar_list_2[0][:-4])] 
        self.max_index = max(last_index)
        self.min_index = min(first_index)
        print(self.max_index)
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
        
        for file in range(self.min_index, self.max_index):
            imgPt1 = self.imgPath_1 + "/" + str(file).zfill(6) + ".jpg" 
            imgPt2 = self.imgPath_2 + "/" + str(file).zfill(6) + ".jpg" 
            pcdPt1 = self.lidarPath_1 + "/" + str(file).zfill(6) + ".pcd"
            pcdPt2 = self.lidarPath_2 + "/" + str(file).zfill(6) + ".pcd"

            img1 = self.makeImg(imgPt1)
            img2 = self.makeImg(imgPt2)

            #이미지 4개 concat
            _img1 = np.vstack((img2,img1))
           
            #pcd bev 크기를 concat 한 후의 이미지 크기와 매칭

            if os.path.exists(pcdPt1):
                pc_image_1 = self.pcd_projection(pcdPt1)
                pc_image_1 = cv2.cvtColor(pc_image_1,cv2.COLOR_GRAY2BGR)
            else:
                pc_image_1 = self.blackImg
                
            if os.path.exists(pcdPt2):
                pc_image_2 = self.pcd_projection(pcdPt2)
                pc_image_2 = cv2.cvtColor(pc_image_2,cv2.COLOR_GRAY2BGR)
            else:
                pc_image_2 = self.blackImg
                
            pc_image_1 = cv2.resize(pc_image_1, dsize=(int(_img1.shape[0]/1.5), _img1.shape[0]),interpolation=cv2.INTER_CUBIC)
            pc_image_2 = cv2.resize(pc_image_2, dsize=(int(_img1.shape[0]/1.5), _img1.shape[0]),interpolation=cv2.INTER_CUBIC)
            pc_image_1 = cv2.rotate(pc_image_1,cv2.ROTATE_180)
            pc_image_2 = cv2.rotate(pc_image_2,cv2.ROTATE_180)
            
            concat_img = np.hstack((pc_image_1,_img1))
            concat_img = np.hstack((concat_img,pc_image_2))
            
            concat_img = cv2.resize(concat_img, dsize=(0,0),fx = 0.35, fy = 0.35, interpolation=cv2.INTER_AREA)
            
            name = self.savePath + "/" + str(file).zfill(6) + ".jpg"
            
            cv2.imwrite(name,concat_img)
            print(f"Saving file >> {str(file).zfill(6)}",end='\r')
            
            # cv2.imshow("a",concat_img)
            # key_input = cv2.waitKey(0)
            # if key_input & 0xFF == ord('q'): 
            #     break
            
pathList = ["rosbag2_2021_06_08-12_22_53"]#os.listdir("/home/krri/brtdata/2021_06_BAG/raw")


for path in pathList:
    print(f"Reading>> {path}")
    Img_Pcd_Viewer(path).run()
            
            
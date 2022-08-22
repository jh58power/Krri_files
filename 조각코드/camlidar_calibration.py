import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import operator
import openpyxl
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm, trange
import json
import os
import open3d as o3d

Vres = 1024
Hres = 1920
HFOV = np.deg2rad(120)

camT = [0,0.1,-0.1]

camR = [np.deg2rad(-6),np.deg2rad(21),np.deg2rad(-3)]#[0,0.2,1] #rad yaw pitch roll

def rotMatrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R

def getPoint(vBox):
    x = vBox[0]
    y = vBox[1]
    z = vBox[2]
    h = vBox[3]
    w = vBox[4]
    l = vBox[5]
    ry = vBox[6]
    
    flh = np.array([x+l/2, y+w/2, z+h/2])
    fll = np.array([x+l/2, y+w/2, z-h/2])
    frh = np.array([x+l/2, y-w/2, z+h/2])
    frl = np.array([x+l/2, y-w/2, z-h/2])
    rlh = np.array([x-l/2, y+w/2, z+h/2])
    rll = np.array([x-l/2, y+w/2, z-h/2])
    rrh = np.array([x-l/2, y-w/2, z+h/2])
    rrl = np.array([x-l/2, y-w/2, z-h/2])
    
    R = rotMatrix(ry)
    
    points = np.concatenate([[flh],[fll],[frh],[frl],[rlh],[rll],[rrh],[rrl]])
    print(points)
    points[:,0:2] = np.dot(R,points[:,0:2].T).T
    
    return points

def readJson(jsonPath):
    json_file = open(jsonPath,'r')
    obj_list = []
    for obj in json.load(json_file):
        obj_name = obj["obj_type"]
        cat_id = 0
        h,w,l = obj["psr"]["3D_Dimenstion"]["height"],obj["psr"]["3D_Dimenstion"]["width"],obj["psr"]["3D_Dimenstion"]["length"]
        x,y,z = obj["psr"]["Location"]["x"],obj["psr"]["Location"]["y"],obj["psr"]["Location"]["z"]
        ry = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Yaw"]
        pitch = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Pitch"]
        roll = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Roll"]
        object_label = [x,y,z,h,w,l,ry]
        obj_list.append(object_label)
    
    return obj_list

def loadFromPcd(path):
    pcd = o3d.t.io.read_point_cloud(path,format='pcd')
    pcdXYZ = np.asarray(pcd.point['positions'].numpy(),dtype=np.float32)
    
    return pcdXYZ

#camera Matrix 가 없을시 구하는 공식 (왜곡이 없다는 가정)
#왜곡 있을시 체커보드 이용해서 직접 구해야함
def camMatrix():
    f = (Hres/2)/math.tan(HFOV/2)
    cx = (Hres+1)/2
    cy = (Vres+1)/2
    return np.array([[f,0,cx],
                     [0,f,cy],
                     [0,0,1]])

def world2local(t,r,x,y,z):
    # 평행이동
    nx = x-t[0]
    ny = y-t[1]
    nz = z-t[2]
    
    # 회전변환
    a = r[2]
    b = r[1]
    g = r[0]
    rotMat = R.from_euler('zyx', [-g, -b, -a])
    rotMat.as_quat()
    
    return rotMat.apply([nx,ny,nz])  


def cam2img(F,point):
    #이미지 프레임 축에 맞게 조정
    rearXY=np.array([[-point[1]],[-point[2]],[point[0]]])
    
    #X 축 차원 제거
    #1/x * [CamMatrix] * [-y,-z,x]

    return list(map(int, np.multiply(1/point[0],np.dot(F,rearXY)))) 

def drawRectangle(src,pos1, pos2):
    cv2.line(src, pos1, pos2, color=(191, 255, 0),thickness=4)

#rt Matrix 가 이미 존재할때 사용하는 함수
def rot(F,x,y,z):
    rt_matrix = np.array([[0.05347218, -0.99636744, -0.06627701, 0.06748746],
                [-0.05432555, 0.06337102, -0.99651034, -1.29283624],
                [0.9970905, 0.05688612, -0.05073963, 0.45057703]])
    
    result = np.dot(rt_matrix,[[x],[y],[z],[1]]) 
    
    u,v,p = np.dot(F,result)
    
    a= np.multiply(1/p,[u,v,p])
    
    return list(map(int,a.T.squeeze()))

def main():
    
    lidar_path = "/home/krri/brtdata/krri_dataset/0804/2021/D3/L2/lidar/"
    img_path  = "/home/krri/brtdata/2021_06_BAG/raw/Train_Test Dataset/Train/Day/D3/c1/"
    
    # F = np.array([[952.49962794, 0, 925.0661249],[0, 954.36418124, 548.74277633],[0, 0, 1,]]) #camMatrix()
    # dist = np.array([[-3.20089119e-01,  1.07167829e-01,  8.79506045e-05,  7.20942526e-04, -1.65905540e-02]])
    F = np.array([[1.02810602e+03, 0.00000000e+00, 9.62100527e+02],
       [0.00000000e+00, 1.03390483e+03, 5.88910166e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) #camMatrix()
    dist = np.array([[-0.34069842,  0.11933445,  0.00122974, -0.00068699, -0.01881615]])
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(F,dist,(Hres,Vres),0,(Hres,Vres))
    x,y,w,h = roi
    
    for lidar in os.listdir(lidar_path)[60:]:

        pcd = loadFromPcd(lidar_path+lidar)
        RGB = cv2.imread(img_path+"21D3C1_"+lidar[7:13]+'.jpg')
        RGB = cv2.undistort(RGB, F, dist, None, newcameramtx)
        # RGB = RGB[y:y+h, x:x+w]
        print(img_path+"21D1C1_"+lidar[7:13]+'.jpg',end='\r')
        
        for i in pcd:
            if i[0] > 0 and i[2] < 1.0:
                localPoint = world2local(camT,camR, i[0], i[1],i[2])
                
                # point = rot(F,i[0], i[1],i[2])
                # print("local",localPoint)
                
                point = cam2img(F,localPoint)
                # print("point", point)

                drawRectangle(RGB, (point[0],point[1]),(point[0],point[1]))

        cv2.imshow("a",RGB)
        a = cv2.waitKey(0)
        if  a & 0xFF == 27: # esc 키를 누르면 닫음
            break
        cv2.destroyAllWindows()
         
if __name__ == "__main__":
	main()
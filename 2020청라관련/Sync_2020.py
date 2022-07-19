import cv2
from time import time
import numpy as np
import os
import matplotlib.pyplot as plt
import shutil
import open3d as o3d

device = o3d.core.Device("CPU:0")
dtype = o3d.core.float32

def rotMatrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R


def PcdRotate(pcdFile):
    pcd = o3d.t.io.read_point_cloud(pcdFile,format='pcd')
    pcdXYZ = np.asarray(pcd.point['positions'].numpy(),dtype=np.float32)

    R = rotMatrix(0.724440610153788) # 약 41도
    R2 = rotMatrix(1.5708*2)#0.785398 = 45도  1.5708 = 90도 

    permutation = [1,2,0] # 순서를 y,z,x -> x,y,z

    pcdXYZ[:] = pcdXYZ[:, permutation] 

    pcdXYZ[:,[1,2]] = np.dot(R, pcdXYZ[:,[1,2]].T).T #y,z 평면 상으로 회전
    pcdXYZ[:,[0,1]] = np.dot(R2, pcdXYZ[:,[0,1]].T).T # x,y 평면상으로 회전

    pcd.point['positions'] = o3d.core.Tensor(pcdXYZ,dtype,device)
    
    return pcd

basic_path = "/home/krri/brtdata/exp_20201111/raw"
raw_list = ["2020-11-12-16-15-14"]



for time_path in raw_list:
    time_path = "/" + time_path +"/"
    save_path = basic_path + time_path + "sync/"
    
    if (not os.path.isdir(save_path)):
        os.makedirs(save_path+"c1/")
        os.makedirs(save_path+"c2/")
        os.makedirs(save_path+"c3/")
        os.makedirs(save_path+"c4/")
        os.makedirs(save_path+"ld/")
    
    for i in range(1, 6):
        if i != 5:
            # 1. jpg, pcd 파일 각각 list로 불러오기(파일명 : 19자리)
            topic_path = basic_path + time_path + f'camera{i}/usb_cam{i}/image_raw'
            if (not os.path.exists(topic_path)):
                print('Path Doesn\'t exist')

            if "Thumbs.db" in os.listdir(topic_path):
                print("망할 Thumbs")
                pass

            # 2. 겹치는 건 제외하고, ms단위이기 때문에 뒤의 단위는 제거한 파일명을 생성해서 저장.
            # 0000000부터 시작해보았으나 겹쳐지는 파일이나 각각 카메라에서 찍은 파일의 개수 등 다 달라서 총 파일의 개수가 다르게 나와서 timestamp가 일치하지 않음. 그래서 다음과 같이 변경함.
            print("Reading Cam "+ f"{i}")
            t = int(os.listdir(topic_path)[0][10]) - 1 
            for topic in os.listdir(topic_path):
                try:
                    if int(topic[10]) != t:
                        cp_img_name = f'{str(topic[:11])}.jpg'
                        print(save_path + f'c{i}/' + cp_img_name,end="\r")

                        shutil.copyfile(topic_path + '/' + topic, save_path + f'c{i}/' + cp_img_name)

                        t = int(topic[10])

                    # ms가 같으면 전에꺼 사용하고 다음껀 날려버림.
                    else:
                        continue
                except:
                    continue    
                
        else:
            topic_path = basic_path + time_path + 'os1_cloud_node/points'
            if (not os.path.exists(topic_path)):
                print('Path Doesn\'t exist')

            t = int(os.listdir(topic_path)[0][10]) - 1 
            for topic in os.listdir(topic_path):
                if int(topic[10]) != t:
                    cp_img_name = f'{str(topic[:11])}.pcd'
                    pcd = PcdRotate(topic_path + "/" + topic)
                    print(save_path + f'ld/' + cp_img_name, end="\r")
                    o3d.t.io.write_point_cloud(save_path + "/ld" + "/" + cp_img_name, pcd)

                    t = int(topic[10])

                # ms가 같으면 전에꺼 사용하고 다음껀 날려버림.
                else:
                    continue
import numpy as np
import open3d as o3d
import os
import shutil

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

raw_folder = "/home/krri/brtdata/exp_20201111/raw" # raw 데이터들 있는 폴더
data_folder = "/2020-11-11-14-58-29/" # 원하는 데이터
topic_list = ["camera1/usb_cam1/image_raw","camera2/usb_cam2/image_raw","camera3/usb_cam3/image_raw","camera4/usb_cam4/image_raw","os1_cloud_node"]


output_path = raw_folder + "/rotated" + data_folder

if (not os.path.isdir(output_path)):
    os.makedirs(output_path)

for i in topic_list:
    data_out = output_path + "/" + i
    if (not os.path.isdir(data_out)):
        os.makedirs(data_out)
        
camera_1_path = raw_folder + data_folder + topic_list[1]
lidar_path = raw_folder + data_folder + topic_list[-1] + "/" + "points/"

camera1_files = os.listdir(camera_1_path)
lidar_files = os.listdir(lidar_path)

idx = 0
c_count = 0
l_count = 0
while l_count < len(lidar_files):
    
    lidar_time = int(lidar_files[l_count][:-4])
    camera1_time = int(camera1_files[c_count][:-4])
    pcd = PcdRotate(lidar_path + lidar_files)
    
    if lidar_time - camera1_time > 300000000: # lidar 가 더 느림 
        o3d.t.io.write_point_cloud(output_path + "os1_cloud_node" + str("{:06d}".format(idx))+".pcd", pcd)
        shutil.copy(camera_1_path+"/"+camera1_files, output_path + topic_list[1] + str("{:06d}".format(idx))+".jpg")
        c_count +=2
        l_count +=1

    elif lidar_time - camera1_time < 300000000: #camera가 더 느림
        o3d.t.io.write_point_cloud(output_path + "os1_cloud_node" + str("{:06d}".format(idx))+".pcd", pcd)
        c_count +=1
        l_count +=1
        
    else:
        o3d.t.io.write_point_cloud(output_path + "os1_cloud_node" + str("{:06d}".format(idx))+".pcd", pcd)
        shutil.copy(camera_1_path+"/"+camera1_files, output_path + topic_list[1] + str("{:06d}".format(idx))+".jpg")
        c_count +=1
        l_count +=1
        
    # print(str("{:06d}".format(idx))+" done",end="\r")
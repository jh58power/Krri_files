from attr import has
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import shutil
import argparse
import math
def rotMatrix(theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        return R
    
def rot_pcd_projection(pcd_data):
    pc = o3d.io.read_point_cloud(pcd_data)
    
    xyz = np.asarray(pc.points)
    print(xyz.shape)
    R = rotMatrix(-0.785398) #0.785398  1.5708
    xyz[:,[0,2]] = np.dot(R, xyz[:,[0,2]].T).T
    
    xy = xyz[:,1:3]
    # xy = np.asarray(pc.points)[:,1:3]
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
def pcd_projection(pcd_data):
    pc = o3d.io.read_point_cloud(pcd_data)
    xyz = np.asarray(pc.points)
    R = rotMatrix(-0.785398) #0.785398  1.5708
    xyz[:,[0,2]] = np.dot(R, xyz[:,[0,2]].T).T
    
    xy = xyz[:,1:3]
    # xy = np.asarray(pc.points)[:,1:3]
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

parser = parser = argparse.ArgumentParser(description='PointCloud Viewer & Saver')
parser.add_argument('--pcd_path', default = './',help='PCD 파일의 위치')
parser.add_argument('--img_path', default = '',help = '이미지 파일 위치')
parser.add_argument('--start_index', default = 0, help='시작 인덱스')

args = parser.parse_args()  

pcd_path = args.pcd_path
img_path = args.img_path

pcd_output_path = './copy_data/pcd'#pcd_path + 'copy_data/pcd'
img_out = './copy_data/img'#pcd_path + 'copy_data/img'

pcd_list = os.listdir(pcd_path)

if os.path.isdir(img_path):
    img_list = os.listdir(img_path)
else:
    img_list = pcd_list
for file in pcd_list:
    assert file.endswith(".pcd"), ".pcd가 없거나 아닌 파일이 있습니다."    

x_size = 900
y_size = 600
x_range = x_size*0.04
y_range = y_size*0.04

grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size])
image_size = np.array([x_size, y_size])
print("Press Any Key to see next")
print("Press \"B\" to see previous")
print("Press \"Q\" to exit")
print("Press \"J\" to Jump 10 Frame")
print("Press \"S\" to save")
i = int(args.start_index)

while i <= len(img_list):
    print(f"Reading file >> {pcd_list[i]} {img_list[i]}",end='\r')
    
    if os.path.isdir(img_path):
        origin_img = img_path+'/'+img_list[i]
        copy_img = img_out + '/' + img_list[i]
        cam_image = cv2.imread(origin_img,cv2.IMREAD_COLOR)
        cv2.imshow("Cam Image",cam_image)
        
    # near_pcd = find_nearest(pcd_path,int(img_list[i].split('.')[0]))
    origin_pcd = pcd_path+'/'+ pcd_list[i] #near_pcd
    copy_pcd = pcd_output_path +'/'+pcd_list[i] #near_pcd
    
    pc_image = rot_pcd_projection(origin_pcd)
    cv2.imshow('PointCloud',pc_image)
    
    key_input = cv2.waitKey(0)
    if key_input & 0xFF == ord('q'): 
        break
    elif key_input & 0xFF == ord('b'):
        i -= 1
        continue
    elif key_input & 0xFF == ord('j'):
        i += 10
        continue
    elif key_input & 0xFF == ord('h'):
        i += 100
        continue
    elif key_input & 0xFF == ord('s'):
        if (not os.path.isdir(pcd_output_path)):
            os.makedirs(pcd_output_path)
        if (not os.path.isdir(img_out)):
            os.makedirs(img_out)
        shutil.copy(origin_pcd,copy_pcd)
        if os.path.isdir(img_path):
            shutil.copy(origin_img,copy_img)
        continue
    else:
        i += 1
        continue
        
cv2.destroyAllWindows()

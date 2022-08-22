from attr import has
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math

def cal_rad(x1,y1,x2,y2):
    rad = math.atan2(y2-y1,x2-x1)
    return rad

def rotMatrix(theta):
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        return R
def LSM(x,y):
    x_bar = x.mean()
    y_bar = y.mean()
    calculated_weight = ((x - x_bar) * (y - y_bar)).sum() / ((x - x_bar)**2).sum()
    calculated_bias = y_bar - calculated_weight * x_bar
    return calculated_weight, calculated_bias

def rot_pcd_projection(pcd_data):
    x_size = 300
    y_size = 300
    z_size = 300
    
    x_range = 3 #20.0
    y_range = 3 #60.0
    z_range = 2
    
    bus_height = 3.48
    
    grid_size = np.array([2 * x_range / x_size, 2 * y_range / y_size, 2 * z_range / z_size])
    image_size = np.array([x_size, y_size, z_size])

    
    shifted_coord = np.asarray(pcd_data)
    
    #좌표 변환
    shifted_coord[:,0] += x_range
    shifted_coord[:,1] += y_range
    shifted_coord[:,2] = -shifted_coord[:,2] + bus_height
    # image index
    index = np.floor(shifted_coord / grid_size).astype(np.int64)

    # choose illegal index
    bound_x = np.logical_and(index[:, 0] >= 0, index[:, 0] < image_size[0])
    bound_y = np.logical_and(index[:, 1] >= 0, index[:, 1] < image_size[1])
    bound_z = np.logical_and(index[:, 2] >= 0, index[:, 2] < image_size[2])

    bound_box = np.logical_and(np.logical_and(bound_x, bound_y), bound_z)
    index = index[bound_box]

    # show image
    image = np.zeros((x_size, y_size,z_size), dtype=np.uint8)
    image[index[:, 2],index[:, 0], index[:, 1]] = index[:, 2]
    image = np.amax(image, axis=0) 
    return image

pcd_path = '/home/krri/HDD/dataset/철도_샘플데이터/2020_11_11_OnlyPcd/train/000121.pcd' # hinged 000191 000121

pc = o3d.io.read_point_cloud(pcd_path)
    
xyz = np.asarray(pc.points)
permutation = [1,2,0] # 순서를 y,z,x -> x,y,z
xyz[:] = xyz[:, permutation] 
print(xyz.shape)

R = rotMatrix(0.724440610153788) #0.785398  1.5708
xyz[:,[1,2]] = np.dot(R, xyz[:,[1,2]].T).T

r_pc_image = rot_pcd_projection(xyz)


exist_X = np.where(r_pc_image>0)[1]
exist_Y = np.where(r_pc_image>0)[0]

front_X = exist_X[np.where(exist_Y<150)]
front_Y = exist_Y[np.where(exist_Y<150)]

rear_X = exist_X[np.where(exist_Y>=160)]
rear_Y = exist_Y[np.where(exist_Y>=160)]

front_w, front_b = LSM(front_Y,front_X) # Y값에 따른 X 증가량
rear_w, rear_b = LSM(rear_Y,rear_X)

new_frontX = front_w*front_Y + front_b
new_rearX = rear_w*rear_Y + rear_b

front_line_max = [new_frontX[front_Y.argmax()],front_Y[front_Y.argmax()]] #y값이 가장 클때의 X,Y 값
front_line_min = [new_frontX[front_Y.argmin()],front_Y[front_Y.argmin()]]

rear_line_max = [new_rearX[rear_Y.argmax()],rear_Y[rear_Y.argmax()]] #y값이 가장 클때의 X,Y 값
rear_line_min = [new_rearX[rear_Y.argmin()],rear_Y[rear_Y.argmin()]]

rad1 = cal_rad(front_line_min[0],front_line_min[1],front_line_max[0],front_line_max[1])
rad2 = cal_rad(rear_line_min[0],rear_line_min[1],rear_line_max[0],rear_line_max[1])
print(np.rad2deg(rad1))
print(np.rad2deg(rad2))

backtorgb = cv2.cvtColor(r_pc_image,cv2.COLOR_GRAY2RGB)


# for i in range(len(front_X)):
#     cv2.line(backtorgb,(front_X[i],int(new_frontY[i])),(front_X[i],int(new_frontY[i])),(0,0,255),2)
cv2.line(backtorgb,(int(new_frontX[front_Y.argmax()]),front_Y[front_Y.argmax()]),(int(new_frontX[front_Y.argmin()]),front_Y[front_Y.argmin()]),(0,0,255),2)
cv2.line(backtorgb,(int(new_rearX[rear_Y.argmax()]),rear_Y[rear_Y.argmax()]),(int(new_rearX[rear_Y.argmin()]),rear_Y[rear_Y.argmin()]),(0,0,255),2)

# cv2.line(backtorgb,(rear_X[0],int(new_rearY[0])),(rear_X[-1],int(new_rearY[-1])),(0,0,255),2)

# hinge = 180-rad1-rad2
cv2.imshow('rot1',r_pc_image)
# print(hinge)
cv2.imshow('rot',backtorgb)
cv2.waitKey(0)
    
        
cv2.destroyAllWindows()

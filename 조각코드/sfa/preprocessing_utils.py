# -*- coding: utf-8 -*-


import pcl_lib
import numpy as np
import cv2
import multiprocessing
import time
import torch
import copy

from functools import partial
import config.kitti_config as cnf

def rosmsg2cvimg(cam_msg):

	np_arr = np.array(cam_msg.data)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	return image_np


def multi_lidar_proc(lidar1, lidar2):

	l_list = [lidar1, lidar2]
	pool = multiprocessing.Pool(processes=12)
	result = pool.map(rosmsg2bevimg, l_list)
	pool.close()
	pool.join()

	return result[0], result[1]


def bevmap_front_vs_back(lidar_msg):

	scan = pcl_lib.read_points_arr(lidar_msg, field_names = ("x", "y", "z"), skip_nans=True)
	# scan = pcl_lib.read_points_arr(lidar_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)
	scan = np.reshape(scan, (-1,3))

	# scan_ = np.zeros(scan.shape)

	# scan_[:,0] = scan[:, 2]
	# scan_[:,1] = scan[:, 0]
	# scan_[:,2] = scan[:, 1]

	front_lidar = get_filtered_lidar(scan, cnf.boundary)
	front_bevmap = makeBEVMap(front_lidar, cnf.boundary,None)
	front_bevmap = torch.from_numpy(front_bevmap)

	back_lidar = get_filtered_lidar(scan, cnf.boundary_back)
	back_bevmap = makeBEVMap(back_lidar, cnf.boundary_back,None)
	back_bevmap = torch.from_numpy(back_bevmap)


	return front_bevmap, back_bevmap


def rosmsg2bevimg(lidar_msg, lidar_loc):

	scan = pcl_lib.read_points_arr(lidar_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)
	scan = np.reshape(scan, (-1,4))

	scan = get_filtered_lidar(scan, cnf.boundary)
	bevmap = makeBEVMap(scan, cnf.boundary,None)
	bevmap = torch.from_numpy(bevmap)

	return bevmap

# def rosmsg2bevimg(lidar_msg, lidar_loc):

# 	scan = pcl_lib.read_points_arr(lidar_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)
# 	scan = np.reshape(scan, (-1,4))

# 	if lidar_loc == 1: # front
# 		scan = get_filtered_lidar(scan, cnf.boundary)
# 		scan = makeBEVMap(scan, cnf.boundary, lidar_loc)

# 	elif lidar_loc == 2: # rear
# 		scan = get_filtered_lidar(scan, cnf.boundary_back)
# 		scan = makeBEVMap(scan, cnf.boundary_back, lidar_loc)

# 	return scan

def bevimg2cvimg(bev):

	bev_map = (bev.squeeze().transpose(1, 2, 0) * 255).astype(np.uint8)
	bev_map = cv2.resize(bev_map, (cnf.BEV_WIDTH, cnf.BEV_HEIGHT))
	bev_map = cv2.rotate(bev_map, cv2.ROTATE_180)

	return bev_map

def get_filtered_lidar(lidar, boundary):

	minX = boundary['minX']
	maxX = boundary['maxX']
	minY = boundary['minY']
	maxY = boundary['maxY']
	minZ = boundary['minZ']
	maxZ = boundary['maxZ']

	# Remove the point out of range x,y,z
	mask = np.where((lidar[:, 0] >= minX) & (lidar[:, 0] <= maxX) &
					(lidar[:, 1] >= minY) & (lidar[:, 1] <= maxY) &
					(lidar[:, 2] >= minZ) & (lidar[:, 2] <= maxZ))
	lidar = lidar[mask]
	lidar[:, 2] = lidar[:, 2] - minZ - 0.3
	# lidar[:, 2] = lidar[:, 2] - minZ - 0.7
	# lidar[:, 3] = lidar[:, 3]
	# lidar[:, 2] = lidar[:, 2] + 1.3
	# lidar[:, 2] = lidar[:, 2] + 1.75
	# lidar[:, 3] = (lidar[:, 3] / 65535) * 255

	return lidar

def convert_det_to_real_values(detections, num_classes=3):
    kitti_dets = []
    for cls_id in range(num_classes):
        if len(detections[cls_id]) > 0:
            for det in detections[cls_id]:
                # (scores-0:1, x-1:2, y-2:3, z-3:4, dim-4:7, yaw-7:8)
                _score, _x, _y, _z, _h, _w, _l, _yaw = det
                _yaw = -_yaw
                x = _y / cnf.BEV_HEIGHT * cnf.bound_size_x + cnf.boundary['minX']
                y = _x / cnf.BEV_WIDTH * cnf.bound_size_y + cnf.boundary['minY']
                z = _z + cnf.boundary['minZ']
                w = _w / cnf.BEV_WIDTH * cnf.bound_size_y
                l = _l / cnf.BEV_HEIGHT * cnf.bound_size_x

                kitti_dets.append([cls_id, x, y, z, _h, w, l, _yaw])

    return np.array(kitti_dets)

def show_rgb_image_with_boxes(img, labels, calib):
    for box_idx, label in enumerate(labels):
        cls_id, location, dim, ry = label[0], label[1:4], label[4:7], label[7]
        if location[2] < 2.0:  # The object is too close to the camera, ignore it during visualization
            continue
        if cls_id < 0:
            continue
        corners_3d = compute_box_3d(dim, location, ry)
        corners_2d = project_to_image(corners_3d, calib.P2)
        img = draw_box_3d(img, corners_2d, color=cnf.colors[int(cls_id)])

    return img

def roty(angle):
    # Rotation about the y-axis.
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def compute_box_3d(dim, location, ry):
    # dim: 3
    # location: 3
    # ry: 1
    # return: 8 x 3
    R = roty(ry)
    h, w, l = dim
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

    corners = np.array([x_corners, y_corners, z_corners], dtype=np.float32)
    corners_3d = np.dot(R, corners)
    corners_3d = corners_3d + np.array(location, dtype=np.float32).reshape(3, 1)
    return corners_3d.transpose(1, 0)

def project_to_image(pts_3d, P):
    # pts_3d: n x 3
    # P: 3 x 4
    # return: n x 2
    pts_3d_homo = np.concatenate([pts_3d, np.ones((pts_3d.shape[0], 1), dtype=np.float32)], axis=1)
    pts_2d = np.dot(P, pts_3d_homo.transpose(1, 0)).transpose(1, 0)
    pts_2d = pts_2d[:, :2] / pts_2d[:, 2:]

    return pts_2d.astype(np.int) 

def draw_box_3d(image, corners, color=(0, 0, 255)):
    ''' Draw 3d bounding box in image
        corners: (8,3) array of vertices for the 3d box in following order:
            1 -------- 0
           /|         /|
          2 -------- 3 .
          | |        | |
          . 5 -------- 4
          |/         |/
          6 -------- 7

    '''

    face_idx = [[0, 1, 5, 4],
                [1, 2, 6, 5],
                [2, 3, 7, 6],
                [3, 0, 4, 7]]
    for ind_f in range(3, -1, -1):
        f = face_idx[ind_f]
        for j in range(4):
            cv2.line(image, (corners[f[j], 0], corners[f[j], 1]),
                     (corners[f[(j + 1) % 4], 0], corners[f[(j + 1) % 4], 1]), color, 2, lineType=cv2.LINE_AA)
        if ind_f == 0:
            cv2.line(image, (corners[f[0], 0], corners[f[0], 1]),
                     (corners[f[2], 0], corners[f[2], 1]), color, 1, lineType=cv2.LINE_AA)
            cv2.line(image, (corners[f[1], 0], corners[f[1], 1]),
                     (corners[f[3], 0], corners[f[3], 1]), color, 1, lineType=cv2.LINE_AA)

    return image 

def makeBEVMap(PointCloud_, boundary, lidar_loc):
	Height = cnf.BEV_HEIGHT + 1
	Width = cnf.BEV_WIDTH + 1

	# Discretize Feature Map
	PointCloud = np.copy(PointCloud_)
	PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / cnf.DISCRETIZATION))
	PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / cnf.DISCRETIZATION) + Width / 2)

	# sort-3times
	sorted_indices = np.lexsort((-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
	PointCloud = PointCloud[sorted_indices]
	_, unique_indices, unique_counts = np.unique(PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
	PointCloud_top = PointCloud[unique_indices]

	# Height Map, Intensity Map & Density Map
	heightMap = np.zeros((Height, Width))
	intensityMap = np.zeros((Height, Width))
	densityMap = np.zeros((Height, Width))

	# some important problem is image coordinate is (y,x), not (x,y)
	max_height = float(np.abs(boundary['maxZ'] - boundary['minZ']))
	heightMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 2] / max_height

	normalizedCounts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))
	# intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
	densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

	RGB_Map = np.zeros((3, Height - 1, Width - 1))
	RGB_Map[2, :, :] = densityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # r_map
	RGB_Map[1, :, :] = heightMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # g_map
	# RGB_Map[0, :, :] = intensityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # b_map

	if lidar_loc == 1: # front
		'''
		# -10 ~ 40
		'''
		# rear = np.copy(RGB_Map[:, 487:,:])

		# RGB_Map[:, 121:, :] = RGB_Map[:, :487, :]
		# RGB_Map[:, :121, :] = rear

		rear = np.copy(RGB_Map[:, :304,:])
		RGB_Map[:, :304, :] = RGB_Map[:, 304:, :]
		RGB_Map[:, 304:, :] = rear
	elif lidar_loc == 2: # rear

		'''
		# -35 ~ 15
		'''
		# rear = np.copy(RGB_Map[:, 183:,:])

		# RGB_Map[:, 425:, :] = RGB_Map[:, :183, :]
		# RGB_Map[:, :425, :] = rear

		'''
		# -25 ~ 25
		'''
		rear = np.copy(RGB_Map[:, :304,:])

		RGB_Map[:, :304, :] = RGB_Map[:, 304:, :]
		RGB_Map[:, 304:, :] = rear


	return RGB_Map

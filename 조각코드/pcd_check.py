import open3d as o3d
import numpy as np
import cv2
import json
import open3d as o3d
import os


lidar_file = "/home/krri/brtdata/2021_06_BAG/raw/Train_Test Dataset/Train/Day/D3/l2/21D3L2_003907.pcd"
label_file = "/home/krri/brtdata/krri_dataset/0804/2021/D3/L2/3d_label/ver0/21D3L2_003907.json"

pcd = o3d.t.io.read_point_cloud(lidar_file,format='pcd')
pcdXYZ = np.asarray(pcd.point['positions'].numpy(),dtype=np.float32)
pcdcolors = np.zeros(pcdXYZ.shape)
pcdcolors[:,2] = 255
pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(pcdXYZ)
pcd.colors = o3d.utility.Vector3dVector(pcdcolors)

#-----------label--------------------
labels = []

json_file = open(label_file,'r')

for obj in json.load(json_file):
    obj_name = obj["obj_type"]
    cat_id = 0
    h,w,l = obj["psr"]["3D_Dimenstion"]["height"],obj["psr"]["3D_Dimenstion"]["width"],obj["psr"]["3D_Dimenstion"]["length"]
    x,y,z = obj["psr"]["Location"]["x"],obj["psr"]["Location"]["y"],obj["psr"]["Location"]["z"]
    ry = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Yaw"]
    pitch = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Pitch"]
    roll = obj["psr"]["Rotation_Roll_Pitch_Yaw"]["Roll"]
    # object_label = [[x, y, z],
    #                 [x,y,z+h/2],
    #                 [x,y,z-h/2],
    #                 [x+w/2,y,z],
    #                 [x-w/2,y,z],
    #                 [x,y+l/2,z],
    #                 [x,y-l/2,z]]
    object_label = np.array([x,y,z,h,w,l,ry])
    bounding_box = np.array([
        [-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
        [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
        [-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2]])
    yaw_rotation_matrix = np.array([
        [np.cos(ry), -np.sin(ry), 0.0],
        [np.sin(ry), np.cos(ry), 0.0],
        [0.0, 0.0, 1.0]])
    roll_rotation_matrix = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(roll), -np.sin(roll)],
        [0.0, np.sin(roll), np.cos(roll)]])
    pitch_rotation_matrix = np.array([
        [np.cos(pitch), 0.0, np.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-np.sin(pitch), 0.0, np.cos(pitch)]])
    
    eight_points = np.tile(object_label[0:3], (8, 1))
    yaw_rotated = np.dot(yaw_rotation_matrix, bounding_box)
    pitch_rotated = np.dot(pitch_rotation_matrix, yaw_rotated)
    roll_rotated = np.dot(roll_rotation_matrix, pitch_rotated)
    
    coners = (roll_rotated + eight_points.transpose()).transpose()
    
    labels.append(coners)

labels = np.array(labels).reshape(-1,3)

line = [[0, 1], [1, 2], [2, 3], [0, 3],
         [4, 5], [5, 6], [6, 7], [4, 7],
         [0, 4], [1, 5], [2, 6], [3, 7]]

lines = [np.array(line) + 8*i for i in range(int(len(labels)/8))]
lines = np.array(lines).reshape(-1,2)


colors = [[1, 0, 0] for _ in range(len(lines))]

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(labels)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)
   


# pcd_obj = o3d.geometry.PointCloud()
# pcd_obj.points = o3d.utility.Vector3dVector(labels)
# pcd_obj.colors = o3d.utility.Vector3dVector(objcolors)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.Default
# vis = o3d.visualization.VisualizerWithEditing

vis.add_geometry(pcd)
vis.add_geometry(line_set)
vis.run()

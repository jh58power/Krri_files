import open3d as o3d
import numpy as np
import cv2
import json
import open3d as o3d
import os

lidar_file = "/home/krri/brtdata/2021_06_BAG/raw/Train_Test Dataset/Scenario/BL2-UC5/l1/21D1L1_017448.pcd"


pcd = o3d.t.io.read_point_cloud(lidar_file,format='pcd')
pcdXYZ = np.asarray(pcd.point['positions'].numpy(),dtype=np.float32)
pcdcolors = np.zeros(pcdXYZ.shape)
pcdcolors[:,2] = 255
pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(pcdXYZ)
pcd.colors = o3d.utility.Vector3dVector(pcdcolors)

o3d.visualization.draw_geometries_with_editing([pcd])
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.Default

vis.add_geometry(pcd)
vis.run()

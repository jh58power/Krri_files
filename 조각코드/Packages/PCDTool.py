import open3d as o3d
import numpy as np

'''
pcd_Projection(pcd파일 경로 + 파일명, 이미지 넓이(pixel), 이미지 높이(pixel), x방향 거리, y 방향 거리 )
    - x 방향 거리는 m 단위로, 라이다로부터 몇미터 까지 나타낼껀지 설정한다.( y방향도 동일 )
    - 이미지 넓이는 pixel 값으로 이미지 해상도를 얼마로 나타낼껀지를 설정한다.

'''

def pcd_projection(pcd_data,x_size,y_size,x_range,y_range):
    
    x_size = x_size
    y_size = y_size
    x_range = x_range
    y_range = y_range
    
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
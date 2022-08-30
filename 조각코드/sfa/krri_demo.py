# -*- coding: utf-8 -*-

############################################################
import sys
import os
import time
import cv2
import torch
import warnings
import numpy as np
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
############################################################
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
############################################################
from rclpy.clock import ClockType
from rclpy.time import Time
############################################################
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
import message_filters
from custom_msg.msg import Bbox3d
from custom_msg.msg import Lidarobj
############################################################
import pcl_lib
from preprocessing_utils import rosmsg2bevimg, rosmsg2cvimg, bevimg2cvimg, multi_lidar_proc, bevmap_front_vs_back
from post_processing_utils import convert_det_to_real_values
from models.model_utils import create_model
from utils.evaluation_utils import draw_predictions
import config.kitti_config as cnf
from utils.demo_utils import parse_demo_configs, do_detect, write_credit
############################################################
warnings.filterwarnings("ignore", category=UserWarning)

src_dir = os.path.dirname(os.path.realpath(__file__))
while not src_dir.endswith("sfa"):
    src_dir = os.path.dirname(src_dir)
if src_dir not in sys.path:
    sys.path.append(src_dir)
############################################################

qos_depth = 5
QOS_RKL5V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)



class Collect(Node):
    def __init__(self):
        super().__init__('collect')


        self.configs = parse_demo_configs()
        self.model = create_model(self.configs)
        print('\n\n' + '-*=' * 30 + '\n\n')

        print(self.configs.pretrained_path)
        assert os.path.isfile(self.configs.pretrained_path), "No file at {}".format(self.configs.pretrained_path)
        self.model.load_state_dict(torch.load(self.configs.pretrained_path, map_location='cpu'))
        print('Loaded weights from {}\n'.format(self.configs.pretrained_path))

        self.configs.device = torch.device('cpu' if self.configs.no_cuda else 'cuda:{}'.format(self.configs.gpu_idx))
        self.model = self.model.to(device=self.configs.device)

        self.model.eval()

        self.br = CvBridge()

        # self.subscription = self.create_subscription(PointCloud2, '/group1/os_cloud_node/points', self.callback, 10)
        # self.subscription = self.create_subscription(PointCloud2, '/points_out/raw/ground_removed', self.callback, 10)
        self.subscription = self.create_subscription(PointCloud2, '/points_out/hinged', self.callback, 10)
        # self.subscription = self.create_subscription(PointCloud2, '/points_out/used_ndt', self.callback, 10)
        self.subscription

        self.nh = Node('lidar_detection_Node_handle')
        self.result_pub = self.nh.create_publisher(Lidarobj, 'lidar_result',QOS_RKL5V)
        # self.result_pub = self.nh.create_publisher(Lidarobj, 'front_obj',QOS_RKL5V)

    def callback(self, sub_front_lidar):


        start = time.time()


        front_bevmap, back_bevmap = bevmap_front_vs_back(sub_front_lidar)
        with torch.no_grad():

            front_detections, front_bevmap, fps = do_detect(self.configs, self.model, front_bevmap, is_front=True)
            back_detections, back_bevmap, _ = do_detect(self.configs, self.model, back_bevmap, is_front=False)

            # Draw prediction in the image
            front_bevmap = (front_bevmap.permute(1, 2, 0).numpy() * 255).astype(np.uint8)
            front_bevmap = cv2.resize(front_bevmap, (cnf.BEV_WIDTH, cnf.BEV_HEIGHT))
            front_bevmap = draw_predictions(front_bevmap, front_detections, self.configs.num_classes)
            # Rotate the front_bevmap
            front_bevmap_rot = cv2.rotate(front_bevmap, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Draw prediction in the image
            back_bevmap = (back_bevmap.permute(1, 2, 0).numpy() * 255).astype(np.uint8)
            back_bevmap = cv2.resize(back_bevmap, (cnf.BEV_WIDTH, cnf.BEV_HEIGHT))
            back_bevmap = draw_predictions(back_bevmap, back_detections, self.configs.num_classes)
            # Rotate the back_bevmap
            back_bevmap_rot = cv2.rotate(back_bevmap, cv2.ROTATE_90_CLOCKWISE)

            # merge front and back bevmap
            full_bev = np.concatenate((back_bevmap_rot, front_bevmap_rot), axis=1)
            full_bev = cv2.rotate(full_bev, cv2.ROTATE_90_COUNTERCLOCKWISE)

            end = time.time()
            proc = end-start

            print(proc)
            
            c_v = int(full_bev.shape[0]/2)
            c_u = int(full_bev.shape[1]/2)

            full_bev = cv2.circle(full_bev, (c_u, c_v), 3, (255, 255, 255), -1)
            
            # # score, x, y, z, h, w, l, yaw
            front_objects = convert_det_to_real_values(front_detections)
            back_objects = convert_det_to_real_values(back_detections)

            result_msg = self.make_ros_msg(front_objects, back_objects, full_bev, self.br)

            # print(front_objects)
            # print(back_objects)
            # front_bevmap_rot = cv2.rotate(front_bevmap_rot, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # back_bevmap_rot = cv2.rotate(back_bevmap_rot, cv2.ROTATE_90_COUNTERCLOCKWISE)

            self.result_pub.publish(result_msg)
            # full_bev = cv2.rotate(full_bev, cv2.ROTATE_90_CLOCKWISE)

            cv2.imshow('bev', full_bev)
            # cv2.imshow('f', front_bevmap)
            # cv2.imshow('r', back_bevmap)
            cv2.waitKey(1)

    def make_ros_msg(self, front_boxes, rear_boxes, bev_img, br):
        front_output = []
        for i in front_boxes:
            f_bbox = Bbox3d(
                x = round(float(i[1]),2),
                y = round(i[2],2),
                z = round(i[3],2),
                h = round(i[4],2),
                w = round(i[5],2),
                l = round(i[6],2),
                yaw =round(i[7],2))

            front_output.append(f_bbox)

        rear_output = []
        for j in rear_boxes:
            # print('x : {:.3f}'.format(j[1]))
            # print('y : {:.3f}'.format(j[2]))
            r_bbox = Bbox3d(
                x = -round(j[1],2),
                y = -round(j[2],2),
                z = round(j[3],2),
                h = round(j[4],2),
                w = round(j[5],2),
                l = round(j[6],2),
                yaw =round(j[7],2))

            rear_output.append(r_bbox)
            
        header_ = Header(
                stamp = self.nh.get_clock().now().to_msg()
                )
        output_msg = Lidarobj(
            header = header_,
            bev_image = br.cv2_to_imgmsg(bev_img),
            front_bbox = front_output,
            rear_bbox = rear_output
            )
        
        return output_msg


def main(args=None):
    rclpy.init(args=args)
    collect = Collect()
    rclpy.spin(collect)


if __name__ == '__main__':
    main()
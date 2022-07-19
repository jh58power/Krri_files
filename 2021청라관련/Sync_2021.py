import cv2
from time import time
import numpy as np
import os
import shutil

basic_path = "/home/krri/brtdata/2021_06_BAG/raw"
raw_list = ["rosbag2_2021_06_08-12_22_53"]#os.listdir(basic_path)


for time_path in raw_list:
    print(time_path)
    time_path = "/" + time_path +"/"
    save_path = basic_path + time_path + "sync2/"
    
    if (not os.path.isdir(save_path)):
        os.makedirs(save_path+"c1/")
        os.makedirs(save_path+"c2/")
        os.makedirs(save_path+"l1/")
        os.makedirs(save_path+"l2/")

    
    for i in range(1,3):
        # 1. jpg, pcd 파일 각각 list로 불러오기
        topic_path = basic_path + time_path + f'usb_cam_{i}/image_comp/'
        if (not os.path.exists(topic_path)):
            print('Path Doesn\'t exist')

        if "Thumbs.db" in os.listdir(topic_path):
            print("망할 Thumbs")
            pass

        # 2. 겹치는 건 제외하고, ms단위이기 때문에 뒤의 단위는 제거한 파일명을 생성해서 저장.
        # 0000000부터 시작해보았으나 겹쳐지는 파일이나 각각 카메라에서 찍은 파일의 개수 등 다 달라서 총 파일의 개수가 다르게 나와서 timestamp가 일치하지 않음. 그래서 다음과 같이 변경함.
        print("Reading Cam "+ f"{i}")
        try:
            t = int(os.listdir(topic_path)[0][10]) - 1 
        except:
            continue
        for topic in os.listdir(topic_path):
            try:
                if int(topic[10]) != t:
                    origin_camPath = topic_path + '/' + topic #기존 경로
                    
                    cp_img_name = f'{str(topic[:11])}.jpg'
                    dst_camPath = save_path + f'c{i}/' + cp_img_name #복사할 경로
                    
                    print(save_path + f'c{i}/' + cp_img_name,end="\r")
                    shutil.copyfile(origin_camPath, dst_camPath)

                    t = int(topic[10])

                # ms가 같으면 전에꺼 사용하고 다음껀 날려버림.
                else:
                    continue
            except:
                continue 
               
    for i in range(1,3):            
        topic_path = basic_path + time_path + f'group{i}/os_cloud_node/points'
        if (not os.path.exists(topic_path)):
            print('Path Doesn\'t exist')
            
        print("Reading Lidar "+ f"{i}")
        try:
            t = int(os.listdir(topic_path)[0][10]) - 1 
        except:
            continue
        for topic in os.listdir(topic_path):
            if int(topic[10]) != t:
                origin_ldPath = topic_path + '/' + topic
                
                cp_img_name = f'{str(topic[:11])}.pcd'
                cp_ldPath = save_path + f"l{i}/" + cp_img_name
                
                print(save_path + f'l{i}/' + cp_img_name, end="\r")
                shutil.copyfile(origin_ldPath, cp_ldPath)

                t = int(topic[10])

            # ms가 같으면 전에꺼 사용하고 다음껀 날려버림.
            else:
                continue
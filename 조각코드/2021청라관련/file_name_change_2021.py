import cv2
import numpy as np
import os

date_list = ["rosbag2_2021_06_08-12_22_53"] #os.listdir("/home/krri/brtdata/2021_06_BAG/raw/")


for date in date_list:
    basic_path = f"/home/krri/brtdata/2021_06_BAG/raw/{date}/sync2"

    #Cam
    for i in range(1, 3):
        topic_path = os.path.join(basic_path, f'c{i}')
        if (not os.path.exists(topic_path)):
            print('Path Doesn\'t exist')
            break

        start_num = int(os.listdir(topic_path)[0][-5])
        criterion = int(os.listdir(topic_path)[0][-11:-4])

        for topic in os.listdir(topic_path):
            diff = int(topic[-11:-4]) - criterion

            file_num = start_num + diff

            file_old_name = os.path.join(topic_path, topic)
            file_new_name = topic_path + '/' + str(file_num).zfill(6) + '.jpg'
            
            print(file_new_name,end="\r")

            os.rename(file_old_name, file_new_name)

    # #Lidar
    for i in range(1,3):
        topic_path = os.path.join(basic_path, f'l{i}')
        if (not os.path.exists(topic_path)):
            print('Path Doesn\'t exist')
            break

        start_num = int(os.listdir(topic_path)[0][-5])
        criterion = int(os.listdir(topic_path)[0][-11:-4])

        for topic in os.listdir(topic_path):
            diff = int(topic[-11:-4]) - criterion
        
            file_num = start_num + diff
        
            file_old_name = os.path.join(topic_path, topic)
            file_new_name = topic_path + '/' + str(file_num).zfill(6) + '.pcd'
            
            print(file_new_name,end="\r")
        
            os.rename(file_old_name, file_new_name)
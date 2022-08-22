import os
import shutil

from torch import dstack

# 18-15-52 해야됨
# date_list = ['2020-11-11-19-23-23', '2020-11-12-11-33-13', '2020-11-12-13-58-55', '2020-11-12-15-08-26', '2020-11-12-16-15-14', '2020-11-12-18-21-51', '2020-11-12-19-29-10']

date_list = os.listdir("/home/krri/brtdata/2021_06_BAG/raw")
date_list.remove("Train_Test Dataset")
date_list.remove("rosbag2_2021_06_07-17_19_15_")

for date in date_list:
    print(date)
    basic_path = os.path.join('/home/krri/brtdata/2021_06_BAG/raw', date, 'sync2')
    criterion_num = int(os.listdir(os.path.join(basic_path, 'c1'))[0].split('.')[0])

    for n in range(1, 3):
        compare_num = int(os.listdir(os.path.join(basic_path, 'c' + str(n)))[0].split('.')[0])
        if criterion_num < compare_num:
            criterion_num = compare_num
            
    for n in range(1, 3):
            compare_num = int(os.listdir(os.path.join(basic_path, 'l' + str(n)))[0].split('.')[0])
            if criterion_num < compare_num:
                criterion_num = compare_num

    for i in range(1, 3):
            file_path = os.path.join(basic_path, 'c' + str(i))
            file_list = os.listdir(file_path)

            dst_path = os.path.join('/home/krri/brtdata/2021_06_BAG/raw', date, 'sync_5_sec', 'c' + str(i))
            if not os.path.exists(dst_path):
                os.makedirs(dst_path)

            for file in file_list:
                num, extend = file.split('.')
                
                if int(num) % 50 == criterion_num:
                    print(dst_path+f'{file}',end="\r")
                    shutil.copyfile(os.path.join(file_path, file), os.path.join(dst_path, file))
                    
    for i in range(1, 3):
            file_path = os.path.join(basic_path, 'l' + str(i))
            file_list = os.listdir(file_path)

            dst_path = os.path.join('/home/krri/brtdata/2021_06_BAG/raw', date, 'sync_5_sec', 'l' + str(i))
            if not os.path.exists(dst_path):
                os.makedirs(dst_path)    

            for file in file_list:
                num, extend = file.split('.')
                
                if int(num) % 50 == criterion_num:
                    print(dst_path+f'{file}',end="\r")
                    shutil.copyfile(os.path.join(file_path, file), os.path.join(dst_path, file))
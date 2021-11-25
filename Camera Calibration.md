## Camera Calibration

- https://github.com/heethesh/lidar_camera_calibration 참고

#### 1-1. Camera Calibration Matrix 찾기 (from bag)

```bash
$ roslaunch lidar_camera_calibration play_rosbag.launch
```

#### 1-2. Camera Calibration Matrix 찾기 (from Camera)

```bash
$ rosrun camera_calibration cameacalibrator.py --size <Checkboard_W x Checkboard_H> --square <SquareSize> image:=<Image_raw Topic> camera:=<Camera Topic>
```



#### 필요 항목들 

- image_width : 카메라 넓이
- image_height : 카메라 높이
- camera_name : 카메라 이름 (토픽명)
- camera_matrix (3x3) 
- distortion_model : 일반적으로 **"plumb_bob"**
- distortion_coefficients(1x5)
- rectification_matrix(3x3)
- projection_matrix(3x4)

위 항목들 yaml 기록



#### 2. 추출된 Matrix 들을 이용하여 기존의 bag 의 header에 Calibration matrix 업데이트

```bash
$ rosrun lidar_camera_calibration update_camera_info.py <original_file.bag> <calibration_file.yaml>
```



#### 3. update 된 Rosbag 실행 (2가지 방법) 

1. launch 로 실행 

```bash
$ ROS_NAMESPACE=usb_cam roslaunch lidar_camera_calibration display_camera_calibration.launch
```

2. 직접 bag 으로 실행

```bash 
$ rosbag play <Updated_bagfile.bag>
```

```bash
$ ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
```

#### 4. rviz로 확인






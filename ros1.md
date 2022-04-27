

### ros 실행 순서

```bash
$ cd <RosFolder>
$ source /opt/ros/<RosVersion>/setup.bash
$ source devel/setup.bash
```

다른 터미널에서 

```bash
$ cd <RosFolder>
$ source /opt/ros/<RosVersion>/setup.bash
$ roscore
```



## Ros1 bag 참고 사이트

https://bigbigpark.tistory.com/36

### Rosbag to Video 

```bash
$ rosrun image_view_recorder video_recorder image:='<Topic>' _filename:='<VideoFileName>' _fps:=<fps>
```

### Rosbag 1 deserial

deserial 이란?

- ros1 bag 의 serial 화 된 데이터를 파이썬을 통해 해독하여 bag 자체에서 데이터를 추출하는 방법

- ``` python 
  import rosbag
  from sensor_msgs.msg import *
  bag = rosbag.Bag('/home/krri/HDD/Bags/2021-10-22-14-56-19.bag')
  
  for topic, msg, t in bag.read_messages(topics = ['/os1_cloud_node/points']):
      print(msg.data)
  bag.close()
  ```

* 장점 : bag 을 직접 실행시키지 않고 빠르게 데이터 취득 가능
* 단점 : 코드를 직접 작성해야함

##### 주로 추출하는 데이터 정보 (Canival or Brt)

#### **PointCloud Data** 

* **Topic :**  /os1_cloud_node/points

* **msg_type :** sensor_msgs/PointCloud2

* **msgs :** 

  | 메세지명 | 타입           | 정보                                                |
  | -------- | -------------- | --------------------------------------------------- |
  | data     | uint8[] (byte) | Pointcloud 데이터 정보 포함 ( decode 필요, float32) |
  | fields   | list (str)     | x, y, z, intensity, ring, reflexity, t 등등         |

  

#### **Camera Data**

* **Topic :**  /usb/cam/camera_info

* **msg_type :**sensor_msgs/CompressedImage

* **msgs :** 

  | 메세지명 | 타입  | 정보                                                   |
  | -------- | ----- | ------------------------------------------------------ |
  | data     | byte  | Camera 데이터 정보 포함 (cv_bridge 사용 decoding 필요) |
  | height   | uint8 | 높이 정보                                              |
  | width    | uint8 | 너비 정보                                              |



Rosbag Lidar Pointcloud 보는법

1. pointcloud 토픽의 frame id 확인

```bash
$ rostopic echo <PointCloudTopic> | grep=frame_id
```

2. rviz 실행
3. rviz 에서 Add -> by topic -> 원하는 토픽의 pointcloud 선택
4. 확인한 frame id를 Global Options > Fixed Frame 에 적기 

### 대용량 파일 읽기

bag 의 idx 를 읽는 idx 파일 먼저 생성 후 읽는 방식

```bash
$ cd ~/catkin_ws
$ . devel/setup.bash
#bag 이 있는 위치로 이동
$ rosrun rosbag rosbag index-file 2020-11-11-12-58-07.bag 
```


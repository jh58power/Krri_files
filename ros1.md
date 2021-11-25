

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
$ rosrun image_view_recoder image:='<Topic>' _filename:='<VideoFileName>' _fps:=<fps>
```

### Rosbag Lidar Pointcloud 보는법

1. pointcloud 토픽의 frame id 확인

```bash
$ rostopic echo <PointCloudTopic> | grep=frame_id
```

2. rviz 실행
3. rviz 에서 Add -> by topic -> 원하는 토픽의 pointcloud 선택
4. 확인한 frame id를 Global Options > Fixed Frame 에 적기 

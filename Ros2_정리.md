### Rosbag Lidar Pointcloud 보는법

1. pointcloud 토픽의 frame id 확인

```bash
$ ros2 topic echo <PointCloudTopic> | grep=frame_id
```

2. rviz 실행
3. rviz 에서 Add -> by topic -> 원하는 토픽의 pointcloud 선택
4. 확인한 frame id를 Global Options > Fixed Frame 에 적기 

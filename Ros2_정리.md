## Ros2

참고 사이트 :  https://hostramus.tistory.com/112

* 기초적인 메시지 publish 방법이나 subscribe 방법은 위 사이트 참고.

#### Ros Install

참고 사이트 : https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html



#### Ros 실행 방법

* Ros 빌드된 폴더에서

```bash
$ source /opt/ros/<VERSION>/setup.bash 
$ source install/setup.bash 
```

* /opt/~ : 터미널 전체 ros 셋업 -> rqt, rviz, topic 등등 현재 Ros 환경 내의 topic 등을 확인 할 수 있음
* <VERSION> : 상황에 맞는 ros 버전 선택. (Canival, VTD = foxy, 4층 컴퓨터 = galactic)
* install/setup.bash : 빌드된 ros로 셋업 -> 직접 빌드된 패키지의 커스텀 메세지, 파이썬 파일 등 실행 가능. publish 와 subscribe 할 수 있음

```bash
$ ros2 run <패키지명> <파이썬 파일>
```



#### Ros 빌드 방법

* 빌드란? 
  * ros2 관련 패키지들을 사용하기 위한 환경 구축
  * 패키지 내의 수정된 사항이 있을시 항상 새로 빌드를 해줘야 한다

```bash
$ colcon build --symlink-install
```

- --symlink-install : 원래는 파이썬 수정 할 때마다 colcon build를 해줘야하지만 symlink-install 을 해주면 수정 할 때마다 colcon build 해줄 필요가 없음



#### RQT

* RQT란? : 현재 ros 환경 내 발행중인 topic 들을 한번에 확인 가능.
* 단점) ros2 topic echo <토픽명> 으로 확인하는것에 비해 속도가 느리다.

#### RVIZ2

* rviz2 란? : pointcloud 데이터, camera 데이터 등을 시각화 시켜주는 툴



#### Ros2 Bag

bag 기록

```bash
$ ros2 bag record -a
```

bag play

```bash
$ ros2 bag play <Bag경로>
```



### Rosbag Lidar Pointcloud 보는법

1. pointcloud 토픽의 frame id 확인

```bash
$ ros2 topic echo <PointCloudTopic> | grep=frame_id
```

2. rviz 실행
3. rviz 에서 Add -> by topic -> 원하는 토픽의 pointcloud 선택
4. 확인한 frame id를 Global Options > Fixed Frame 에 적기 

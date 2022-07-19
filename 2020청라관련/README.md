# 2020 청라관련

# Instruction

---

2020년 청라 현장 시험 데이터 (/home/krri/brtdata/exp_20201111)

- **/2020_11_11(청라)**
  
    <aside>
    ℹ️ Ros1 Bag 으로 취득한 현장시험 데이터
    
    </aside>
    
    - 총 5개의 센서들로 4개의 카메라 1개의 라이다 센서 사용됨.
    - Bag 용량이 너무 커서 일반적인 방법으는 여는데 너무 오래걸림
    - .idx 파일을 만들어 놓음 → *사용법은 /bag_fastRead 에서 설명*

---

- ~~/bag_fastRead~~    →  /home/catkin_ws

    - 대용량 Bag 파일을 빠르게 읽을 수 있도록 하는 코드

    ### <원리>

    > bag 의 인덱스를 미리 불러와 나중에 읽을때 인덱스를 기반으로 빠르게 읽어오게 함
    > 

    > Ros1 기반이기 때문에 Ros1 사용방법 숙지 필요
    > 
    1. .idx 파일 생성
    2. Bag 읽기

    ### <사용법>

    1. .idx 파일 생성

    ```bash
    # idx 파일 생성
    $ cd ~/catkin_ws
    $ . devel/setup.bash
    #bag 이 있는 위치로 cd
    $ rosrun rosbag rosbag index-file 2020-11-11-12-58-07.bag
    ```

    1. idx 파일 기반으로 Bag 읽기

    ```bash
    $ cd ~/catkin_ws
    $ . devel/setup.bash
    $ rosbag play <name of largeBag>
    ```

---

- **/code**

    <aside>
    ℹ️ 주로 Raw 폴더에 있는 데이터 가공용으로 사용
    </aside>

    - Bag 에서 추출된 데이터는 파일명이 해당 프레임의 timestamp(nano sec)로 뽑혀있음
    - 각 센서마다 프레임이 들어온 시간이 완전히 동일하지 않아 싱크를 맞추는 작업이 필요한데 대부분 싱크를 맞추는 용의 코드들임.

    ### 싱크 맞추는 순서

    ![](./Images/Screenshot_from_2022-07-19_14-38-41.png)

    1. Sync_2020.py
        - 1605074309909406660.jpg → 16050743099.jpg 로 잘라주는 역할 (0.1초 간격)
        - 0.1 간격 내에 중복되는 프레임이 있을 시 더 늦은 시간의 프레임 제거
    2. file_name_change.py
        - Sync_2020을 통해 0.1 초 간격으 잘린 프레임들의 파일을 000001.jpg 형식으로 변환
        - 위 코드는 labeling 데이터 만들 때 파일의 포맷을 맞춰야하기 때문에 사용
    3. 2020imgpcdConcat.py
        - 위 sync_2020, file_name_change 후 5개의 센서들을 한번에 보기 편하게 시각화 하기 위해 사용.
        - 예시)

        ![](./Images/Untitled.png)

    4. file_10_sec.py
        - 가공된 데이터 10초 간격으로 잘라서 저장

---

- **/ErrorFile**

    <aside>
    ℹ️ 사용 불가 Bag 모음

---

- **/raw**

    <aside>
    ℹ️ Bag에서부터 추출한 센서 데이터들 모음
    - /Train_Test Dataset :
      
        > 라벨링용 데이터로 train 은 전체 데이터셋을 5초 간격으로 자른 후 차량이 있는 부분만 따로 모아둔 데이터 셋이 Test(Scenario) 는 brt표준문서 시나리오에 해당하는 부분만 따로 뽑아놓은 파일.
        > 

---

- **/ros1_bagExtracter**

    <aside>
    ℹ️ bag 을 play 시키지 않고 db 형식으로 바로 읽어와 파일 추출하는 코드
    ### <사용법>

    ros1bag_Extracter.py 

    **init** 함수만 변경

    ```python
    self.bag_uri = "bag 파일 경로 (절대경로)"
    output_dir_name = "추출 할 output 경로"
    self.camera_topics = ["/cam1 topic","/cam2 topic" . . .]
    self.lidar_topics = ["/lidar topic"]
    ```

    위 변수만 상황에 맞게 변경 후 사용.

    

    *세부 코드는 많이 어렵지 않기 때문에 잘 숙지하면 많은 도움이 될거라 예상합니다. (특히 pcd 관련 코드)*

## Ros2

참고 사이트 :  https://hostramus.tistory.com/112

* 기초적인 메시지 publish 방법이나 subscribe 방법은 위 사이트 참고.

#### Ros2 Install

참고 사이트 : https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html

or : https://blog.naver.com/PostView.naver?blogId=hdh7485&logNo=222378987294



#### Ros2 실행 방법

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



#### Ros2 빌드 방법

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
$ ros2 topic echo <PointCloudTopic> | grep frame_id
```

2. rviz 실행
3. rviz 에서 Add -> by topic -> 원하는 토픽의 pointcloud 선택
4. 확인한 frame id를 Global Options > Fixed Frame 에 적기 



### Ros Topic에서 이상한 값이 나온다?

* 3층에 있는 시뮬레이션 pc 의 값이 이더넷 네터워크를 통해 들어온것이다.

  ``` bash
  $ export ROS_LOCALHOST_ONLY=1
  ```

  입력시 ros 환경이 로컬 환경에서만 사용 가능하게 설정되어 안뜨게 된다.

* 위 명령어는 1회성으로 각 터미널마다 해줘야하는데 영구적으로 설정할 시

  /etc/bash.bashrc 에 해당 명령어 추가해 준뒤 

  ```bash 
  $ source /etc/bash.bashrc
  ```

  



### RQT Plugin 추가

참고 사이트 : https://cafe.naver.com/ca-fe/ArticleRead.nhn?clubid=25572101&page=1&inCafeSearch=true&searchBy=1&query=rqt+plugin&includeAll=&exclude=&include=&exact=&searchdate=all&media=0&sortBy=date&articleid=25073&referrerAllArticles=true

rqt 플러그인? rqt에서 확인하는 데이터를 QT designer 를 사용하여 자신만의 GUI 형태로 구현할 수 있다. 

1. install

```bash
$ sudo apt install qtcreator
```

2. 패키지 생성

```bash
$ cd ~/<폴더명>/src
$ ros2 pkg create <PACKAGE NAME> --build-type ament_cmake --dependencies rclpy rqt_gui rqt_gui_py python_qt_binding
```

3.  CMakelist.txt 에 install 항목들 추가

```
ament_python_install_packages(${PROJECT_NAME}
PACKAGE_DIR src/${PROJECT_NAME})

install(FILES
	plugin.xml
	DESTINATION share/${PROJECT_NAME}
	)
install(DIRECTORY
	resource
	launch
	DESTINATION share/${PROJECT_NAME}
	)	
install(PRO
	plugin.xml
	DESTINATION share/${PROJECT_NAME}
	)
```

4.  <패키지>/resource/<패키지명>.ui 파일 생성 및 qt 디자인
5. 전체 코드

```python
import os

from ament_index_python.resources import get_resource

from custom_msg.msg import GUI
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtWidgets import *
import rclpy

from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool
from rclpy.node import Node


from rqt_gui_py.plugin import Plugin

class ExamplesWidget(QWidget):
    def __init__(self, node):
        super(ExamplesWidget, self).__init__()
        self.setObjectName('ExamplesWidget')
        self.node = node
        self.PUBLISH_INTERVAL = 100 # 100 = 10hz
        pkg_name = 'rqt_Auto_status'
        ui_filename = 'rqt_Auto_status.ui'
        topic_name = 'GUI'
        
        #만들어 놓은 qt ui 불러옴
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        #custom msg 객체 호출
        self.pub_status = GUI()
        self.pub_status.autonomous_status = True 
        self.sub_status = GUI()
        self.sub_status.autonomous_status = True 

        #Publish 타이머 
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_auto)
        self.publish_timer.start(self.PUBLISH_INTERVAL)
        
        qos = QoSProfile(depth=100)
        
        #Publisher, Subscriber 선언
        self.publisher = self.node.create_publisher(GUI, topic_name, qos)
        self.subscriber = self.node.create_subscription(GUI, topic_name, self.get_auto, qos)
    
        #Slider Option    
        self.verticalSlider.setMaximum(50)
        self.verticalSlider.setMinimum(0)
        self.verticalSlider.setSingleStep(0.5)       
        self.verticalSlider.valueChanged.connect(self.showSliderValue)
        
        #radio button $ push Button Option
        self.radio_button_on.setChecked(True)
        self.push_button_status.clicked.connect(self.call_switch_service) #Connect(<해당 이벤트 발생시 호출할 함수>)
        self.push_button_status.clicked.connect(self.call_led_service)
        self.radio_button_on.clicked.connect(self.call_led_service)
        self.radio_button_off.clicked.connect(self.call_led_service)
        self.radio_button_on.setShortcut('o')
        self.radio_button_off.setShortcut('f')
        
        #Spin Box Option
        self.LH_spinbox.setRange(0,50)
        self.LH_spinbox.setSingleStep(0.5) 
        self.LH_spinbox.valueChanged.connect(self.Spinbox_changed)
        
    #spinbox 값 변화시 호출되는 함수    
    def Spinbox_changed(self):
        self.verticalSlider.setValue(float(self.LH_spinbox.value())) #spinbox 값 -> Slider
   
    # Publish 함수    
    def send_auto(self):
        self.publisher.publish(self.pub_status)
        
    def get_auto(self, msg):
        self.sub_status = msg
    
    #slider value    
    def showSliderValue(self):
        self.LH_spinbox.setValue(float(self.verticalSlider.value()))
        self.pub_status.lhdistance = float(self.verticalSlider.value())
        
    def call_led_service(self):
        request = self.pub_status
        if self.radio_button_on.isChecked():
            print("RADIO ON status")
            request.autonomous_status = True
            
        elif self.radio_button_off.isChecked():
            print("RADIO OFF status")
            request.autonomous_status = False
            
        if request.autonomous_status:
            self.push_button_status.setText('ON')
            self.push_button_status.setStyleSheet('color: rgb(255, 170, 0);')
            print('Auto ON')
            
        elif not request.autonomous_status:
            self.push_button_status.setText('OFF')
            self.push_button_status.setStyleSheet('')
            print('Auto OFF')    
   
    def call_switch_service(self):
        print("new Function")
        request = self.pub_status

        if self.push_button_status.text() == 'ON':
            print("on pushed")
            request.autonomous_status = False
            self.radio_button_on.setChecked(False)
            self.radio_button_off.setChecked(True)
        elif self.push_button_status.text() == 'OFF':
            print("off pushed")
            request.autonomous_status = True
            self.radio_button_on.setChecked(True)
            self.radio_button_off.setChecked(False)
    
    #종료 메서드        
    def shutdown_widget(self):
        self.node.destroy_subscription(self.subscriber)
        self.node.destroy_publisher(self.publisher)
 
class Examples(Plugin):
    
    def __init__(self, context):
        super(Examples, self).__init__(context)
        self.setObjectName('Auto_status')
        self.widget = ExamplesWidget(context.node)
        
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the Auto_status.')
        self.widget.shutdown_widget()
```


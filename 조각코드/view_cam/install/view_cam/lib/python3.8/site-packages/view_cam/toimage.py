import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
import sys
class HelloworldSubscriber(Node):

    def __init__(self):
        super().__init__('toimage')
        
        topic = '/usb_cam_1/image_comp' #토픽명
        self.output_path = '/home/krri/HDD/open3d_rgbd/input_data/' # 저장할 위치
        self.output_type = 'depth' # 저장할 폴더 (없을시 공백)
        
        print(topic,end='\r')
        self.image_sub = self.create_subscription(
            CompressedImage, #메세지 타입
            '/usb_cam_1/image_comp',
            self.subscribe_topic_message, #콜백 함수
            1)
        self.br = CvBridge()
        self.frame_no = 0
        
    def subscribe_topic_message(self, msg):
        print('reading',end='\r')
        # image = self.br.imgmsg_to_cv2(msg)
        np_img = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        image = cv2.resize(image, dsize=(0,0),fx = 0.35, fy = 0.35, interpolation=cv2.INTER_AREA)
        # cv2.imwrite(self.output_path + self.output_type + '/' + str(self.frame_no) + '.png',image)
        # self.frame_no += 1
        
        cv2.imshow("a",image)
        if cv2.waitKey(1) & 0xFF == 27: # esc 키를 누르면 닫음
            sys.exit(0)
            
def main(args=None):
    rclpy.init(args=args)
    node = HelloworldSubscriber()  #HelloworldSubscriber 클래스 생성
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import pandas as pd
import pytesseract
import torch
import re
from PIL import Image, ImageEnhance, ImageFilter

model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', force_reload = True, source='local') # 학습 모델 Load

ids = {601779:0, 595292:1, 479039:2}
cap = cv2.VideoCapture(0)

class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_sub') # Node 이름 설정
     self.find_ID = 1 # 찾아야 할 책의 Index ID
     self.publish_id() # 책의 좌표 Publish
     self.image_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                self.status_callback, 10) # Goal 여부 확인
     
     self.image = np.empty(shape=[1])
     self.library_db = pd.read_csv('library_db.csv', sep=',')
     self.status = ''

   def publish_id(self):
     # DB에서 목표 좌표 추출
     pose_x = self.library_db.loc[self.library_db['id']==self.find_ID, 'pose_x'] 
     pose_y = self.library_db.loc[self.library_db['id']==self.find_ID, 'pose_y']
     ori_z = self.library_db.loc[self.library_db['id']==self.find_ID, 'ori_z']
     ori_w = self.library_db.loc[self.library_db['id']==self.find_ID, 'ori_w']
     goals = PoseStamped()
  
     goals.header.frame_id = 'map'
     goals.pose.position.x = float(pose_x)
     goals.pose.position.y = float(pose_y)
     goals.pose.position.z = 0.0
     goals.pose.orientation.x = 0.0
     goals.pose.orientation.y = 0.0
     goals.pose.orientation.z = ori_z
     goals.pose.orientation.w = ori_w
     goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
     goal_pub.publish(goals) # 목표 좌표 Publish

   def status_callback(self, data):
     self.status = data.status_list[-1].status
     results = []
     print(1)
     if(self.status==4): # 목표 도착
       print("Goal Succeeded!")
       while (len(results)<2): # 위치를 찾기 위해 최소 2개의 기준점 필요
        if cap.isOpened():
          ret, image = cap.read() # 영상 획득
          if ret:
            results = self.detect_book_label(image) # 영상에서 기준점 추출
        else:
          print("cam not opened!")

       key_id1 = results[0] # 기준점 1
       key_id2 = results[1] # 기준점 2

       # 기준점 1,2를 기준으로 정리될 책의 위치 표시
       key_id1_cx = (key_id1[1]+key_id1[3])//2
       key_id2_cx = (key_id2[1]+key_id2[3])/ 2
       x_length = (key_id1_cx-key_id2_cx)//(key_id1[0]-key_id2[0])
       cx = key_id1_cx-key_id1[0]*x_length
       imgs = image.copy()
       x_start = int(cx-x_length//2)
       x_end = int(cx+x_length//2)
       y_start = int(key_id1[2])-50
       y_end = int(key_id1[4])+50
       result_img = cv2.rectangle(imgs, (int(key_id1[1]), int(key_id1[2])), (int(key_id1[3]), int(key_id1[4])), (0, 0, 255), 2)
       result_img = cv2.rectangle(result_img, (int(key_id2[1]), int(key_id2[2])), (int(key_id2[3]), int(key_id2[4])), (255, 0, 0), 2)
       result_img = cv2.rectangle(result_img, (x_start, y_start), (x_end, y_end), (123, 255, 0), -1)
       cv2.imshow('imgs', result_img)
       key = cv2.waitKey(0)


   def detect_book_label(self, image):
     results = model(image) # 모델에 영상 입력
     results_array = np.empty((0, 5))

     # 사용할 데이터 추출
     pred = results.pandas().xyxy[0]
     label_area = pred[(pred['name'] == 'label') & (pred['confidence'] > 0.5)]
     if len(label_area) != 0:
      # 데이터에서 라벨 데이터 추출
      for it in range(label_area.shape[0]):
          xmin, ymin, xmax, ymax = label_area.iloc[it][['xmin', 'ymin', 'xmax', 'ymax']]
          label_image = (image.copy())[int(ymin):int(ymax), int(xmin):int(xmax)]

          # PIL을 통해 영상 개선
          image_pil = Image.fromarray(cv2.cvtColor(label_image, cv2.COLOR_BGR2GRAY))
          enhancer = ImageEnhance.Contrast(image_pil)
          image_enhanced = enhancer.enhance(2)
          image_sharpened = image_enhanced.filter(ImageFilter.SHARPEN)

          # pytesseract를 통해 라벨 추출
          label_text = pytesseract.image_to_string(image_sharpened, config='--psm 8 digits')
          ttext = label_text[:-2]
          num_str = re.sub(r'[^0-9]', '', ttext)
          numm=9999

          if len(num_str)==6: # EM 코드는 6자리 숫자
            numm = int(num_str)
            if self.library_db['EM_num'].isin([numm]).any(): # 추출된 코드가 DB에 존재하는지 판단
             index_id = self.library_db.loc[self.library_db['EM_num']==numm, 'id'].iloc[0]
             results_array = np.vstack((results_array, np.array([index_id, int(xmin), int(ymin), int(xmax), int(ymax)])))
      results_array[:,0] = results_array[:,0]-self.find_ID # 찾아야할 책과 상대 정렬값 계산
      results_array = results_array[np.abs(results_array[:,0]).argsort()] # 가까운 순으로 정렬
     return results_array

     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber() # 노드 설정

  try :
    rclpy.spin(node) # 노드 실행

  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__' :
  main()

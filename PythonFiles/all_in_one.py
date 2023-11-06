import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
import pandas as pd
import pytesseract
import torch
import re

model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights/best.pt', force_reload = True)

bridge = CvBridge()

class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_sub')
     self.find_ID = 188
     self.publish_id()
     self.image_sub = self.create_subscription(
        Image,
        '/camera/image_raw',
        self.image_callback,
        qos_profile_sensor_data)
     self.image_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                self.status_callback, 10)
     
     self.image = np.empty(shape=[1])
     self.status = ''

   def publish_id(self):
     library_db = pd.read_csv('library_db.csv', sep=',')
     pose_x = library_db.loc[library_db['id']==self.find_ID, 'pose_x']
     pose_y = library_db.loc[library_db['id']==self.find_ID, 'pose_y']
     goals = PoseStamped()
  
     goals.header.frame_id = 'map'
     goals.pose.position.x = float(pose_x)
     goals.pose.position.y = float(pose_y)
     goals.pose.position.z = 0.0
     goals.pose.orientation.x = 0.0
     goals.pose.orientation.y = 0.0
     goals.pose.orientation.z = 0.7
     goals.pose.orientation.w = 0.7
     goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
     goal_pub.publish(goals)

   def status_callback(self, data):
     self.status = data.status_list[-1].status
     print("Goal Succeeded!")

   def image_callback(self, data):
     if(self.status==4):
      self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
     
      results = self.detect_book_label()
      print(results)
      key_id1 = results[0]
      key_id2 = results[1]
      key_id1_cx = (key_id1[1]+key_id1[3])//2
      key_id2_cx = (key_id2[1]+key_id2[3])//2

      x_length = (key_id1_cx-key_id2_cx)//(key_id1[0]-key_id2[0])
      cx = key_id1_cx-key_id1[0]*x_length
      print(x_length, cx)
      imgs = self.image.copy()
      x_start = int(cx-x_length//2)
      x_end = int(cx+x_length//2)
      y_start = int(key_id1[2])-50
      y_end = int(key_id1[4])+50
      result_img = cv2.rectangle(imgs, (int(key_id1[1]), int(key_id1[2])), (int(key_id1[3]), int(key_id1[4])), (0, 0, 255), 2)
      result_img = cv2.rectangle(result_img, (int(key_id2[1]), int(key_id2[2])), (int(key_id2[3]), int(key_id2[4])), (255, 0, 0), 2)
      result_img = cv2.rectangle(result_img, (x_start, y_start), (x_end, y_end), (123, 255, 0), -1)
      cv2.imshow('imgs', result_img)
      key = cv2.waitKey(0)

   def detect_book_label(self):
     results = model(self.image)
     results_array = np.empty((0, 5))

     pred = results.pandas().xyxy[0]
     label_area = pred[(pred['name'] == 'label') & (pred['confidence'] > 0.5)]
     if len(label_area) == 0:
         return None, None
     for it in range(label_area.shape[0]):
         xmin, ymin, xmax, ymax = label_area.iloc[it][['xmin', 'ymin', 'xmax', 'ymax']]
         label_image = (self.image.copy())[int(ymin):int(ymax), int(xmin):int(xmax)]
        
         gray = cv2.cvtColor(label_image, cv2.COLOR_BGR2GRAY)
         label_text = pytesseract.image_to_string(gray, lang = 'digits')
         ttext = label_text[:-2]
         num_str = re.sub(r'[^0-9]', '', ttext)
         numm=9999
         if len(num_str)==6:
           numm = int(num_str)
           results_array = np.vstack((results_array, np.array([numm, int(xmin), int(ymin), int(xmax), int(ymax)])))
     results_array[:,0] = results_array[:,0]-self.find_ID
     results_array = results_array[np.abs(results_array[:,0]).argsort()]
     return results_array

     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()

  try :
    rclpy.spin(node)

  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__' :
  main()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import pandas as pd
import pytesseract
import torch

model = torch.hub.load('ultralytics/yolov5', 'custom', path='python_files/weights/best.pt', force_reload = True)

bridge = CvBridge()

class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_sub')
     self.image_sub = self.create_subscription(
        Image,
        '/camera/image_raw',
        self.image_callback,
        qos_profile_sensor_data)
     self.image = np.empty(shape=[1])


   def image_callback(self, data) :
     self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
     cv2.imshow('imgs', self.image)
     key = cv2.waitKey(1)

     if key == ord('g'):
       results = self.detect_book_label()
       print(results)


   def detect_book_label(self):
     results = model(self.image)
     texts = []
     boxs = []    

     pred = results.pandas().xyxy[0]
     label_area = pred[(pred['name'] == 'label') & (pred['confidence'] > 0.5)]
     if len(label_area) == 0:
         return None, None
     for it in range(label_area.shape[0]):
         xmin, ymin, xmax, ymax = label_area.iloc[it][['xmin', 'ymin', 'xmax', 'ymax']]
         label_image = (self.image.copy())[int(ymin):int(ymax), int(xmin):int(xmax)]
        
         gray = cv2.cvtColor(label_image, cv2.COLOR_BGR2GRAY)
         label_text = pytesseract.image_to_string(gray, lang = 'digits')
         texts.append(label_text)
         boxs.append([int(xmin), int(ymin), int(xmax), int(ymax)])
        
     return texts

     
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

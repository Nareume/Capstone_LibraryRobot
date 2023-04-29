import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import pandas as pd

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
     cv2.imshow('img', self.image)
     cv2.imwrite('img.jpg', self.image)
     print('')
     cv2.waitKey(1)
     
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
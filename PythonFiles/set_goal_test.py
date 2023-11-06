import rclpy as rp
import pandas as pd
import numpy as np
from IPython.display import display
from geometry_msgs.msg import PoseStamped

def main(args=None) :
  rp.init(args=args)
  library_db = pd.read_csv('python_files/library_db.csv', sep=',')
  example_id = 99
  pose_x = library_db.loc[library_db['id']==example_id, 'pose_x']
  pose_y = library_db.loc[library_db['id']==example_id, 'pose_y']

  set_goal_node = rp.create_node('Set_goal')
  goals = PoseStamped()
  
  goals.header.frame_id = 'map'
  goals.pose.position.x = float(pose_x)
  goals.pose.position.y = float(pose_y)
  goals.pose.position.z = 0.0
  goals.pose.orientation.x = 0.0
  goals.pose.orientation.y = 0.0
  goals.pose.orientation.z = 0.7
  goals.pose.orientation.w = 0.7

  goal_pub = set_goal_node.create_publisher(PoseStamped, '/goal_pose', 10)
  goal_pub.publish(goals)

if __name__ == '__main__' :
  main()
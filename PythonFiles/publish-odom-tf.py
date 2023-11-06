import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from math import cos, sin
from rclpy.clock import Clock


class Test_TF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.time = 0
        self.pre_time = 0
        self.subscription = self.create_subscription(
            Odometry,
            '/micro_ros_arduino_node_publisher',
            self.cmd_callback,
            10
        )
        
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_z = 0.0

    def cmd_callback(self, msg):
        clock = self.get_clock()
        self.time = clock.now().nanoseconds

        sec = int(self.time//1e9)
        ns = int(self.time%1e9)

        transform = TransformStamped()
        transform.header.stamp.sec = sec
        transform.header.stamp.nanosec = ns
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y

        q = Quaternion()
        q.z = msg.pose.pose.orientation.z
        q.w = msg.pose.pose.orientation.w
        transform.transform.rotation = q

        odom_msg = Odometry()
        odom_msg.header.stamp.sec = sec
        odom_msg.header.stamp.nanosec = ns
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = msg.pose.pose.position.x
        odom_msg.pose.pose.position.y = msg.pose.pose.position.y
        odom_msg.pose.pose.orientation = q

        self.tf_broadcaster.sendTransform(transform)
        self.pub_odom.publish(odom_msg)

rp.init()
odom_to_tf_node = Test_TF()
rp.spin(odom_to_tf_node)

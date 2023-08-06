import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rosgraph_msgs.msg import Clock
from math import cos, sin

class Test_TF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.time_stamp = Clock()
        self.pre_time = 0
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.sub_clock = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
        
        self.position_x = -1.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_z = 0.0
        
    def clock_callback(self, msg):
        self.time_stamp = msg

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.time_stamp.clock
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        time = self.time_stamp.clock.sec+self.time_stamp.clock.nanosec*1e-9
        
        dt = time-self.pre_time
        V = msg.twist.twist.linear.x * dt
        self.position_x += V * cos(self.orientation_z)
        self.position_y += V * sin(self.orientation_z)

        transform.transform.translation.x = self.position_x
        transform.transform.translation.y = self.position_y

        delta_yaw = msg.twist.twist.angular.z * dt
        self.orientation_z += delta_yaw

        q = Quaternion()
        q.z = sin(self.orientation_z / 2.0)
        q.w = cos(self.orientation_z / 2.0)
        transform.transform.rotation = q

        self.pre_time = time

        self.tf_broadcaster.sendTransform(transform)
        print('x: ', self.position_x, 'y: ', self.position_y, 'r_z : ', self.orientation_z)

rp.init()
odom_to_tf_node = Test_TF()
rp.spin(odom_to_tf_node)

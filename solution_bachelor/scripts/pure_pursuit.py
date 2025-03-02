import rclpy as ros
from rclpy.node import Node
import numpy as np
from numpy import cos, sin
from utils import angle_from_vec_to_vec, parse_odometry

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class PurePursuit():

    def __init__(self, max_lookahead=4):
        self.max_lookahead = max_lookahead
        self.lookahead = max_lookahead
        self.wheelbase = 2.5

    def get_angle(self, cur_heading, next_point):
        heading_angle = angle_from_vec_to_vec(cur_heading, next_point)
        return (2 * self.lookahead * sin(heading_angle)) / self.wheelbase
    
class PurePursuitController(Node):

    def __init__(self, max_lookahead=4):
        super().__init__('pure_pursuit')

        self.pure_pursuit = PurePursuit(max_lookahead)

        # listen for closes points on the road
        self.target_trajectory = None
        self.target_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic='/contour',
            qos_profile=10,
            callback=self.target_callback
        )

        # and listen for odometry callbacks
        # everything is in local, so no need probably 
        # self.odom_sub = self.create_subscription(
        #     msg_type=Odometry,
        #     topic='/odom',
        #     callback=self.odom_callback,
        #     qos_profile=10
        # )

    def target_callback(self, msg:PointCloud2):
        pass

    def odom_callback(self, msg:Odometry):
        pos, vel = parse_odometry(msg)

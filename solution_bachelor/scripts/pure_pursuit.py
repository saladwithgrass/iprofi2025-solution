import rclpy as ros
from rclpy.node import Node
import numpy as np
from numpy import cos, sin
from utils import angle_from_vec_to_vec, parse_odometry

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class PurePursuitController():

    def __init__(self, min_lookahead):
        self.min_lookahead = min_lookahead
        self.lookahead = min_lookahead
        self.wheelbase = 4.3

    def get_angle(self, cur_heading, next_point):

        vec_len = np.linalg.norm(next_point)
        heading_angle = angle_from_vec_to_vec(cur_heading, next_point)
        return np.arctan(
            (2 * self.wheelbase * sin(heading_angle)) / vec_len
        )

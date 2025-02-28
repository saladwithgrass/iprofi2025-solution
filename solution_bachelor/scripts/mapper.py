import rclpy as ros
from rclpy.node import Node


import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import Odometry

import numpy as np
from utils import parse_odometry

class Mapper(Node):
    def __init__(self):

        super().__init__('mapper')

        # callback for odom
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            qos_profile=10,
            callback=self.odom_callback
        )

    def odom_callbac(self, msg:Odometry):
        pass

def main():
    ros.init()
    mapper = Mapper()

    ros.spin(mapper)
    ros.shutdown()

if __name__ == '__main__':
    main()

    
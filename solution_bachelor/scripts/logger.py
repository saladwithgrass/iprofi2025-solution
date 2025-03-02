import rclpy as ros
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

from utils import parse_odometry, time_to_sec, sec_to_time

import pickle

class Logger(Node):

    def __init__(self, logging_data='logs/'):
        super().__init__('logger')
    
        self.log_folder = logging_data

        # add init time
        self.init_time = self.get_clock().now().nanoseconds / 1e9

        # add listeners
        
        self.pos_data = []
        self.rot_data = []
        self.vel_data = []
        self.time_data = [0.]

        self.pos_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.pos_callback,
            qos_profile=10
        )

        # control
        self.control_data = []
        self.control_times = [0.]
        self.control_sub = self.create_subscription(
            msg_type=Twist,
            topic='/cmd_vel',
            callback=self.control_callback,
            qos_profile=10

        )

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.init_time
    
    def pos_callback(self, msg:Odometry):
        time = self.time()
        pos, rot, vel = parse_odometry(msg)

        self.pos_data.append(pos)
        self.rot_data.append(rot)
        self.vel_data.append(vel)
        self.time_data.append(time)
        
    
    def control_callback(self, msg:Twist):
        u1 = msg.linear.x
        u2 = msg.angular.z
        self.control_times.append(self.time())
        self.control_data.append([u1, u2])

    def save_data(self):
        filename = self.log_folder + f'data_{self.time()}.pkl'
        with open(filename, 'wb') as out_file:
            pickle.dump(
                {
                    'pos' : np.array(self.pos_data),
                    'rot' : np.array(self.rot_data),
                    'vel' : np.array(self.vel_data),
                    'control' : np.array(self.control_data),
                    'time' : np.array(self.time_data),
                    'control_time' : np.array(self.control_times)
                },
                out_file
            )
        print('saved to: \n', filename)

def main():
    ros.init()
    logger = Logger()
    try:
        ros.spin(logger)
    except KeyboardInterrupt:
        print('Keyboard interrupt. Stopping logs')

    
    logger.save_data()
    logger.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
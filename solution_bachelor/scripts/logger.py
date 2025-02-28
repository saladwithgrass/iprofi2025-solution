import rclpy as ros
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

import pickle

class Logger(Node):

    def __init__(self, logging_data='logs/'):
        super().__init__('logger')
    
        self.log_folder = logging_data

        # add init time
        self.init_time = self.get_clock().now().nanoseconds / 1e9

        # add listeners
        
        # pos
        self.pos_data = []
        self.pos_sub = self.create_subscription(
            msg_type=Pose2D,
            topic='/car_pose/pose_2d',
            callback=self.pos_callback,
            qos_profile=10
        )

        # vel
        self.vel_data = []
        self.vel_sub = self.create_subscription(
            msg_type=Pose2D,
            topic='/car_pose/velocity',
            callback=self.vel_callback,
            qos_profile=10

        )

        # control
        self.control_data = []
        self.control_sub = self.create_subscription(
            msg_type=Twist,
            topic='/cmd_vel',
            callback=self.control_callback,
            qos_profile=10

        )

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.init_time
    
    def pos_callback(self, msg:Pose2D):
        pos = [
            msg.x,
            msg.y,
            msg.theta
        ]
        time = self.time()
        self.pos_data.append([pos, time])

    def vel_callback(self, msg:Pose2D):
        vel = [
            msg.x,
            msg.y,
            msg.theta
        ]
        time = self.time()
        self.vel_data.append([vel, time])

    def control_callback(self, msg:Twist):
        u1 = msg.linear.x
        u2 = msg.angular.z
        time = self.time()
        self.control_data.append([[u1, u2], time])

    def save_data(self):
        filename = self.log_folder + f'data_{self.time()}.pkl'
        with open(filename, 'wb') as out_file:
            pickle.dump(
                {
                    'pos' : self.pos_data,
                    'vel' : self.vel_data,
                    'control' : self.control_data
                },
                out_file
            )
        print('saved to: \n', filename)

def main():
    ros.init()
    logger = Logger()
    try:
        ros.spin(logger)
    except:
        print('Keyboard interrupt. Stopping logs')
    finally:
        logger.save_data()
        logger.destroy_node()

if __name__ == '__main__':
    main()
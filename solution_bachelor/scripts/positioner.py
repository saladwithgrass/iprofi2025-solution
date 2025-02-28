#!/usr/bin/env python3
from copy import copy
import rclpy as ros
from rclpy.node import Node

from scipy.spatial.transform import Rotation as R
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion, Polygon, Point32, Transform, Pose, PoseArray

def quat_to_arr(quat:Quaternion):
    return np.array([
        quat.x,
        quat.y,
        quat.z,
        quat.w
    ])

class Positioner(Node):
    def __init__(self):
        super().__init__('odometer')

        self.last_time:float = self.get_clock().now().nanoseconds / 1e9

        # odometry topic subscriber
        self.odom_subscriber = self.create_subscription(
                msg_type=Odometry, 
                topic="/odom", 
                callback=self.odom_callback, 
                qos_profile=10
            )

        # current odometry readings
        self.pos = np.zeros((3,), dtype=np.float64)
        self.velocity = np.zeros((3,), dtype=np.float64)
        self.angle = 0
        self.transform = R.from_euler('xyz', [0, 0, self.angle])

        # odometry publisher
        self.pose2d_publisher = self.create_publisher(
            msg_type=Pose2D,
            topic='/car_pose/pose_2d',
            qos_profile=10
        )

        # velocity publisher
        self.velocity_publisher = self.create_publisher(
            msg_type=Pose2D,
            topic='/car_pose/velocity',
            qos_profile=10
        )

        # acceleration publisher
        self.accel_publisher = self.create_publisher(
            msg_type=Pose2D,
            topic='/car_pose/accel',
            qos_profile=10
        )

        # hitbox publisher
        self.hitbox_publisher = self.create_publisher(
            msg_type=Polygon,
            topic='/hitbox',
            qos_profile=10
        )

        # car parameters
        self.length = 4.3
        self.width = 1.8
        self.height = 0.9

        # car corners
        self.base_corners = np.array([
            [ self.length / 2,  self.width / 2, 0],
            [ self.length / 2, -self.width / 2, 0],
            [-self.length / 2, -self.width / 2, 0],
            [-self.length / 2,  self.width / 2, 0]
        ])


    def odom_callback(self, msg:Odometry):

        # save prev time for velocity
        prev_time = self.last_time

        # udpate time
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # update position
        prev_pos = copy(self.pos)
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z

        # update rotation
        self.transform = R.from_quat(quat_to_arr(msg.pose.pose.orientation))

        # self prev angle
        prev_angle = self.angle
        # update angle
        self.angle = self.transform.as_euler('xyz', degrees=False)[-1]
        self.angle = self.angle % (2 * np.pi)
        # self.transform = self.transform.as_matrix()

        # creae pose message
        pose_msg = Pose2D(x=self.pos[0], y=self.pos[1], theta=self.angle)
        # publish pose

        # update velocity
        prev_velocity = copy(self.velocity)

        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y
        self.velocity[2] = msg.twist.twist.angular.z
        
        vel_msg = Pose2D(x=self.velocity[0], y=self.velocity[1], theta=self.velocity[2])

        # update acceleration
        acceleration = (self.velocity - prev_velocity) / (self.last_time - prev_time)
        accel_msg = Pose2D(x=acceleration[0], y=acceleration[1], theta=acceleration[2])

        # update hitboxes
        # hitbox_msg = Polygon()
        # for corner in self.base_corners:
        #     new_corner = self.transform @ corner + self.pos
        #     point = Point32()
        #     point.x = new_corner[0]
        #     point.y = new_corner[1]
        #     point.z = new_corner[2]
        #     hitbox_msg.points.append(point)
        
        # publish data
        self.accel_publisher.publish(accel_msg)
        self.velocity_publisher.publish(vel_msg)
        self.pose2d_publisher.publish(pose_msg)
        # self.hitbox_publisher.publish(hitbox_msg)

def main(args=None):
    ros.init(args=args)
    odomet = Positioner()

    ros.spin(odomet)
    odomet.destroy_node()

if __name__ == '__main__':
    main()
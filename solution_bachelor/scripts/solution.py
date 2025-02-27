#!/usr/bin/env python3

import numpy as np
import os

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimpleMover(Node):

    def __init__(self, rate = 60):
        super().__init__('solution')
        self.get_logger().info("...solution node started2 ..................")

        self.gui = os.getenv('GUI')=='true' or os.getenv('GUI')=='True'

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom', self.gps_cb, 10)
        self.create_subscription(PointCloud2, '/camera/points', self.points_cb, 10)

        self.cv_bridge = CvBridge()

        self.rate = rate # Hz
        self.init_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.finish_time = 6*60 # ride duration seconds

        timer_period = 1.0 / float(self.rate)
        self.timer = self.create_timer(timer_period, self.move)

        # states 
        self.simulation_started = False

    def camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = cv_image.shape # выводим размер изображения в логи
            self.get_logger().info(f"Image received: {width}x{height}, channels: {channels}")
            if self.gui != False:
                cv2.imshow("output", cv_image)
                cv2.waitKey(1)


        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def gps_cb(self, msg):
        self.get_logger().info(f"The position of robot is: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")

    def points_cb(self, msg):
        self.get_logger().info(f"Points was received")

    def move(self):
        # Control algorithm
        msg = Twist()
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed_sec = now_sec - self.init_time_sec
        if now_sec > 0.0:
            self.simulation_started = True
        if self.simulation_started:
            if self.finish_time > elapsed_sec:
                if now_sec - self.init_time_sec < 10:
                    msg.linear.x = 2.6 # Момент на колеса в % от максимального (вдоль оси Х) 
                    msg.angular.z = 0.1 # Угол поворота колеса (вокруг оси Z)
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info("moving..!")
            else:
                self.cmd_vel_pub.publish(msg) # stop car 
                self.finish_contol()

    def finish_contol(self):
        self.get_logger().info(f"Controller stoped")
        self.timer.cancel()# Останавливаем таймер
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover(rate = 30)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
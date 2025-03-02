#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ControlSummator(Node):

    def __init__(self):

        super().__init__('control_summator')

        # create sub for angular control
        self.angular_sub = self.create_subscription(
            msg_type=Float64,
            topic='/ang_cmd',
            callback=self.ang_callback,
            qos_profile=10
        )

        # create sub for throttle control
        self.throttle_sub = self.create_subscription(
            msg_type=Float64,
            topic='/throttle_cmd',
            callback=self.throttle_callback,
            qos_profile=10
        )

        # create publisher for cmd vel
        self.cmd_pub = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10
        )

        # store control values to update
        self.throttle_value = float(0)
        self.ang_value = float(0)

    def publish_control(self):
        msg = Twist()
        print(self.ang_value)
        msg.linear.x = float(self.throttle_value)
        msg.angular.z = float(self.ang_value)
        self.cmd_pub.publish(msg)

    def throttle_callback(self, msg:Float64):
        self.throttle_value = msg.data
        self.publish_control()

    def ang_callback(self, msg:Float64):
        self.ang_value = float(msg.data)
        self.publish_control()

def main():
    ros.init()
    summator = ControlSummator()
    ros.spin(summator)
    summator.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
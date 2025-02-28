#!/usr/bin/env python3

import rclpy as ros

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist
from solution_bachelor.msg import TwistStampedArray 
from builtin_interfaces.msg import Time

def time_to_nsec(time:Time):
    secs = time.sec
    nsecs = time.nanosec
    return secs * 1e9 + nsecs

def time_to_sec(time:Time):
    secs = time.sec
    nsecs = time.nanosec
    return secs + float(nsecs) / 1e9


class ControlScheduler(Node):
    
    def __init__(self):
        super().__init__('control_publisher')

        # scheduled controls will arrive here
        self.control_sub = self.create_subscription(
            msg_type=TwistStampedArray,
            topic='/scheduled_controls',
            qos_profile=10,
            callback=self.control_callback
        )

        # current controls will be dispatched here
        self.control_pub = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10
        )

        # timer for dispatching controls
        self.control_timer = None

        # list of current scheduled controls
        self.scheduled_controls:list[TwistStamped] = []
        
        self.log('CONTROL SCHEDULER READY')

    def time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def log(self, msg:str):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warn(msg)

    def control_callback(self, msg:TwistStampedArray):
        # save messages
        self.scheduled_controls = msg.twists
        # immediately send the first signal
        self.dispatch_control()

    def set_timer(self):
        # safety
        if len(self.scheduled_controls) <= 1:
            self.get_logger().error('TRYING TO SET TIMER WITH NO CONTROLS')
            return

        # interval to next control
        cur_control_time = self.scheduled_controls[0].header.stamp
        next_control_time = self.scheduled_controls[1].header.stamp
        interval = time_to_sec(next_control_time) - time_to_sec(cur_control_time)
        print('intv: ', interval)

        # create timer
        self.control_timer = self.create_timer(
            timer_period_sec=interval,
            callback=self.dispatch_control
        )

    def dispatch_control(self):
        if self.control_timer is not None:
            self.control_timer.cancel()

        if len(self.scheduled_controls) == 0:
            self.warn('OUT OF CONTROLS')
            return
        
        self.log(f'{len(self.scheduled_controls)} left')
        # publish first control
        self.control_pub.publish(self.scheduled_controls[0].twist)

        # if there are none left, warn and do nothing
        if len(self.scheduled_controls) == 0:
            self.warn('OUT OF CONTROLS')
            return

        # set timer to next control
        self.set_timer()

        # remove first control
        self.scheduled_controls = self.scheduled_controls[1:]

def main():
    ros.init()
    scheduler = ControlScheduler()

    ros.spin(scheduler)
    scheduler.destroy_node()

if __name__ == '__main__':
    main()
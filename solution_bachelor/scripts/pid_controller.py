#!/usr/bin/env python3

import numpy as np
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from utils import parse_odometry, time_to_sec

class PIDController():
    def __init__(
            self, 
            target_vel,
            Kp,
            Ki,
            Kd
            ):

        self.target_speed = target_vel
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0
        self.prev_error = 0

    def two_var_update(self, cur_vel, dt):
        print('cur: ', cur_vel[1])
        print('target: ', self.target_speed)

        # save previous state
        self.prev_error = self.error

        # update new error
        error = self.target_speed - cur_vel[0]
        # error += np.abs(cur_vel[1])
        self.error = error

        # construct proportional part
        P = self.error * self.Kp

        # update integral
        self.error_integral += error * dt
        I = self.Ki * self.error_integral

        # update derivative
        self.error_derivative = (self.prev_error - error) / dt
        D = self.Kd * self.error_derivative

        # combine all parts
        output = P + I + D
        # clip to make sure it is in range
        output = np.clip(output, -100, 100)
        print('control: ', output)
        print('--------')

        return output

    def update(self, cur_speed:float, dt:float):
        print('cur: ', cur_speed)
        print('target: ', self.target_speed)

        # save previous state
        self.prev_error = self.error

        # update new error
        error = self.target_speed - cur_speed
        self.error = error

        # construct proportional part
        P = self.error * self.Kp

        # update integral
        self.error_integral += error * dt
        I = self.Ki * self.error_integral

        # update derivative
        self.error_derivative = (self.prev_error - error) / dt
        D = self.Kd * self.error_derivative

        # combine all parts
        output = P + I + D
        # clip to make sure it is in range
        output = np.clip(output, -100, 100)
        print('control: ', output)
        print('--------')

        return output

class ROSPIDController(Node, PIDController):

    def __init__(
            self,
            target_vel=3,
            Kp=80,
            Ki=0.01,
            Kd=0.01
            ):
        Node.__init__(self, 'pid_controller')
        PIDController.__init__(
            self, 
            target_vel=target_vel,
            Kp=Kp,
            Kd=Kd,
            Ki=Ki
            )

        # create publisher to throttle cmd 
        self.cmd_pub = self.create_publisher(
            msg_type=Float64,
            topic='/throttle_cmd',
            qos_profile=10
        )

        # create odom sub
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.odom_callback,
            qos_profile=10
        )

        # listen for targe speed
        self.target_sub = self.create_subscription(
            msg_type=Float64,
            topic='/target_vel',
            callback=self.target_callback,
            qos_profile=10
        )

        self.get_logger().info('WAITING TO INIT')
        self.is_initialized = False
        self.last_callback_time = 0
        

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def target_callback(self, msg:Float64):
        self.target_speed = float(msg.data)

    def odom_callback(self, msg:Odometry):
        if not self.is_initialized:
            self.is_initialized = True
            self.last_callback_time = self.time()
            self.get_logger().info('INIT COMPLETE')
            return
        
        time = self.time()
        pos, rot, vel = parse_odometry(msg)

        control = self.update(vel[0], time - self.last_callback_time)
        # self.get_logger().info(f'publishing control: {control}')
        self.last_callback_time = time
        self.cmd_pub.publish(Float64(data=control))

def main():
    ros.init()
    pid = ROSPIDController(
        Kd=0.5,
        target_vel=1.5
    )
    ros.spin(pid)
    ros.shutdown()

if __name__ == '__main__':
    main()
        
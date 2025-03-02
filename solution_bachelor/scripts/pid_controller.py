import numpy as np
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from utils import parse_odometry, time_to_sec

class PIDController():
    def __init__(
            self, 
            target_vel=1,
            Kp=1.,
            Ki=1.,
            Kd=1.
            ):

        self.target_speed = target_vel
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0
        self.prev_error = 0

    def update(self, cur_speed:float, dt:float):

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

        return output

class ROSPIDController(Node, PIDController):

    def __init__(self):
        super(Node).__init__('pid_controller')

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

        self.get_logger().log('WAITING TO INIT')
        self.is_initialized = False
        self.last_callback_time = 0
        while not self.is_initialized:
            ros.spin_once(self)


    def odom_callback(self, msg:Odometry):
        if not self.is_initialized:
            self.is_initialized = True
            self.last_callback_time = time_to_sec(msg.header.stamp)
            return
        
        pos, rot, vel, time = parse_odometry(Odometry)

        control = self.update(vel[0], time - self.last_callback_time)
        self.get_logger().log('publishing control: ', control)
        self.last_callback_time = time
        self.cmd_pub.publish(Float64(control))
        
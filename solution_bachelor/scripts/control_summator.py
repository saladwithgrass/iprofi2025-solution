import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float64

class ControlSummator(Node):

    def __init__(self):

        super().__init__('control_summator')

        # create sub for throttle control
        self.throttle_sub = self.create_subscription(

        )
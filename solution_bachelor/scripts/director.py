#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
from scipy.spatial.transform import Rotation as R
import cv2
import rclpy as ros
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from utils import load_grid, parse_odometry, points_from_map, points_to_map, ROAD_WIDTH, normal_2d, marker_msg
from utils import remap_value
from history_keeper import HistoryKeeper
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

from pure_pursuit import PurePursuitController

class Director(Node, PurePursuitController):

    def __init__(self):
        Node.__init__(self, 'director')
        self.lookahead = 7 # m
        PurePursuitController.__init__(self, self.lookahead)

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            qos_profile=10,
            callback=self.odom_callback
        )

        self.roi_sub = self.create_subscription(
            msg_type=OccupancyGrid,
            topic='/car_roi',
            qos_profile=10,
            callback=self.roi_callback
        )

        self.target_pub = self.create_publisher(
            msg_type=Marker,
            topic='/target',
            qos_profile=10
        )
        self.vel_pub = self.create_publisher(
            msg_type=Marker,
            topic='/vel',
            qos_profile=10
        )

        # target velocity publisher
        self.target_vel_pub = self.create_publisher(
            msg_type=Float64,
            topic='/target_vel',
            qos_profile=10
        )

        # create control publisher
        self.control_publisher = self.create_publisher(
            msg_type=Float64,
            topic='/ang_cmd',
            qos_profile=10
        )

        self.map_resolution = 0.1
        self.car_pos = [0, 0]
        self.lookahead_on_map = int(self.lookahead / self.map_resolution)
        self.map_size = 800
        self.lookahead_thickness = 5 # m
        self.lookahead_thickness = int(self.lookahead_thickness / self.map_resolution)
        self.cur_map = np.zeros((self.map_size, self.map_size))
        self.lookahead_circle_mask = None
        self.update_lookahead_circle()

        self.steering_angle = 0
        self.base_target_vel = 2.5
        self.stuck_counter = 0

        self.hist_keeper = HistoryKeeper()

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def update_lookahead_circle(self):
        self.lookahead_on_map = int(self.lookahead / self.map_resolution)
        self.lookahead_circle_mask = np.zeros_like(self.cur_map, np.uint8)
        cv2.circle(
            img=self.lookahead_circle_mask,
            center=(int(self.lookahead_circle_mask.shape[0]/2), int(self.lookahead_circle_mask.shape[1]/2)),
            radius=self.lookahead_on_map,
            color=255,
            thickness=self.lookahead_thickness
        )

    def get_possible_targets(self, map):
        return cv2.bitwise_and(map, self.lookahead_circle_mask)

    def select_road_ahead(self, map, rot):
        heading = R.from_euler('xyz', rot).apply([1, 0, 0])[:2][::-1]
        center = [
            int(map.shape[0] / 2),
            int(map.shape[1] / 2)
        ]

        width = map.shape[0]
        normal_1, normal_2 = normal_2d(heading)

        normal_1 = (normal_1 * width).astype(int)
        normal_2 = (normal_2 * width).astype(int)
        close_points = np.array([normal_1 + center, normal_2 + center])

        far_points = close_points + (heading*2*width).astype(int)

        all_points = np.concatenate((close_points, far_points[::-1]))

        mask = np.zeros_like(map)
        cv2.fillPoly(mask, [all_points], color=255)
        return cv2.bitwise_and(mask, map)
    
    def select_point_ahead(self, map, resolution, origin):
        x_indeces, y_indeces = np.where(map == 255)
        map_points = np.vstack((x_indeces, y_indeces)).T
        avg_index = np.mean(map_points, axis=0, dtype=int)
        return points_from_map([avg_index], resolution, origin)[0]

    def publish_target_velocity(self, target:float):
        self.target_vel_pub.publish(Float64(data=float(target)))
    
    def publish_angle_control(self, angle):
        # make it a bit more aggressive
        # if np.abs(angle) > 0.2:
        #     angle += np.sign(angle)*0.2

        msg = Float64(data=float(angle))
        # self.get_logger().info(f'publishing angle: {angle}')
        # print(angle)
        self.control_publisher.publish(msg)

    def deal_with_slip_angle(self, slip_angle):
        print('slip: ', np.rad2deg(slip_angle))
        abs_angle = abs(np.rad2deg(slip_angle))

        # LOW ZONE 
        low_bound = 2
        # nothing happens
        if abs_angle <= low_bound:
            return self.steering_angle, self.base_target_vel
        
        # MODERATE ZONE
        medium_bound = 5
        # reduce target velocity and steering angle by 5-10 percent 
        min_mod_reduction = 0.05
        max_mod_reduction = 0.1
        if low_bound < abs_angle <= medium_bound:
            reduction_coeff = 1 - remap_value(abs_angle, low_bound, medium_bound, min_mod_reduction, max_mod_reduction)
            return self.steering_angle * reduction_coeff, self.base_target_vel * reduction_coeff

        # HIGH ZONE
        high_bound = 10
        # reduce target velocity and steering angle by 10-20 percent
        min_high_reduction = 0.1
        max_high_reduction = 0.2
        if medium_bound < abs_angle <= high_bound:
            reduction_coeff = 1 - remap_value(abs_angle, medium_bound, high_bound, min_high_reduction, max_high_reduction)
            return self.steering_angle * reduction_coeff, self.base_target_vel * reduction_coeff

        # reduce velocity by 20-30 percent and countersteer non-aggressively
        # CRITICAL ZONE
        reduction_percent = 0.5
        reduction_coeff = 1 - reduction_percent
        return -np.sign(slip_angle) * 0.6, -self.base_target_vel*reduction_coeff 

    def approximate_yaw_rate(self, fwd_vel):
        return fwd_vel * self.steering_angle / self.wheelbase

    def deal_with_yaw_rate(self, yaw_rate, vel):
        if yaw_rate * self.steering_angle >= 0 and np.abs(self.approximate_yaw_rate(vel) - yaw_rate) < 0.5:
            return self.steering_angle, self.base_target_vel
        else:
            self.get_logger().info('countering wheel slip')
            return -self.steering_angle * 1.2, -self.base_target_vel * 0.5

    def roi_callback(self, msg:OccupancyGrid):
        time = self.time()

        # load position, rotation and velocity
        pos, rot, vel, _ = self.hist_keeper.find_closest_time(time) 

        # check if car has been moving recently
        if abs(vel[0]) < 0.1 or self.stuck_counter > 0:
            recent_vels = self.hist_keeper.get_latest_n_data(100, True)
            recent_vels = np.linalg.norm(recent_vels, axis=1)
            weights = np.exp(np.linspace(1, 0, len(recent_vels)))
            avg = np.average(recent_vels, weights=weights)
            if avg < 0.2 or self.stuck_counter > 0:
                if self.stuck_counter == 0:
                    self.stuck_counter = 10
                self.stuck_counter -= 1
                self.get_logger().warning('IM STUCK')
                self.publish_angle_control(-self.steering_angle)
                self.publish_target_velocity(-1.5)
                return

        # adjust lookahead
        self.lookahead = self.min_lookahead + np.abs(vel[0] * 0.5)
        self.update_lookahead_circle()

        # load map
        data, resolution, origin = load_grid(msg)

        # make sure to use only points that were seen at least thrice
        _, data = cv2.threshold(
            data,
            thresh=6,
            maxval=255,
            type=cv2.THRESH_BINARY
        )

        # dilate to fill empty spots on the road
        ksize = 5
        data = cv2.dilate(data, kernel=np.ones((ksize, ksize)))

        # make road thinner
        ksize = int(ROAD_WIDTH * 0.6 / self.map_resolution)
        if ksize % 2 == 0:
            ksize -= 1
        data = cv2.erode(data, kernel=np.ones((ksize,ksize)))

        # select only road ahead
        data = self.select_road_ahead(data, rot)

        # select poinst on lookahead distance
        data = self.get_possible_targets(data)

        # select point based on lookahead
        point = self.select_point_ahead(data, resolution, origin)
        # convert it to vector from vehicle
        point_local = point - pos[:2]
        point_local = R.from_euler('xyz', rot).inv().apply(np.concatenate((point_local, [0])))[:2]


        cur_heading = vel[:2]
        cur_heading = [1, 0]
        # cur_heading = [1, 0]
        # get angle to control
        self.steering_angle = self.get_angle(cur_heading, point_local)

        # calculate slip angle
        new_vel = self.base_target_vel
        if np.linalg.norm(vel) > 0.5:
            new_steer, new_vel = self.deal_with_yaw_rate(vel[2], vel[0])
            self.steering_angle = new_steer

        self.publish_target_velocity(new_vel)
        self.publish_angle_control(self.steering_angle)

        point3d = np.concatenate((point_local, [pos[2] + 2]))
        self.target_pub.publish(marker_msg(point3d, 'chassis'))
        self.vel_pub.publish(marker_msg(cur_heading, 'chassis'))
        # cv2.imshow('huh', data)
        # cv2.waitKey(1)


    def odom_callback(self, msg:Odometry):
        time = self.time()
        pos, rot, vel = parse_odometry(msg)
        self.hist_keeper.update(pos, rot, vel, time)

    def get_next_target(self):
        pass

def main():
    ros.init()
    director = Director()
    ros.spin(director)
    director.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
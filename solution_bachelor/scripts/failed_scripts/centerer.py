import rclpy as ros
from rclpy.node import Node


import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import Odometry

import numpy as np
import cv2

from copy import copy
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from utils import (
    parse_odometry, 
    time_to_sec, 
    point_cloud, 
    ROAD_WIDTH,
    interpolate_points
    )



class Centerer(Node):
    def __init__(self):

        super().__init__('centerer')

        # callback for odom
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            qos_profile=10,
            callback=self.odom_callback
        )

        # lidar callback
        self.lidar_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic='/processed_points',
            callback=self.lidar_callback,
            qos_profile=10
        )

        # for debug only
        self.pcd_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic='/road_center',
            qos_profile=10
        )

        # history of poses
        self.history_len = 200
        self.pos_hist = np.zeros((self.history_len, 3), np.float32)
        self.vel_hist = np.zeros((self.history_len, 3), np.float32)
        self.rot_hist = np.zeros((self.history_len, 3), np.float32)
        self.time_hist = np.zeros((self.history_len, 3), np.float32)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def find_closest_time(self, time):
        min_idx = np.argmin(np.abs(self.time_hist - time))
        return ( 
            np.array(self.pos_hist[min_idx]), 
            np.array(self.rot_hist[min_idx]),
            np.array(self.vel_hist[min_idx]),
            np.array(self.time_hist[min_idx]),
        )

    def extract_contour(self, points_2d:np.ndarray):
        distance = 1.

        # first, discard float part with some precision
        precision = 0.2
        distance /= precision
        unfloated_points = points_2d / precision
        unfloated_points = unfloated_points.astype(np.int32)

        unfloated_points = np.unique(unfloated_points, axis=0)

        min_x = np.min(unfloated_points[:, 0])
        max_x = np.max(unfloated_points[:, 0])
        
        min_y = np.min(unfloated_points[:, 1])
        max_y = np.max(unfloated_points[:, 1])

        pic_padding = 50 # px
        pic_height = int((max_y - min_y) / distance) + pic_padding
        pic_width = int((max_x - min_x) / distance) + pic_padding
        map = np.zeros((pic_height, pic_width), dtype=np.uint8)
        map[
            (unfloated_points[:, 1] / distance + pic_height / 2).astype(np.int32),
            (unfloated_points[:, 0] / distance + pic_width / 2).astype(np.int32)
        ] = 255

        # extract contours
        map = cv2.adaptiveThreshold(map, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 3, 2)
        contours, hierarchy = cv2.findContours(map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        largest_contour = largest_contour.squeeze()

        # convert contour back to points in original coordinates
        contour_points = largest_contour.astype(np.float32)
        real_points = np.zeros_like(contour_points)
        real_points[:, 1] = precision * distance * (contour_points[:, 1] - pic_height / 2)
        real_points[:, 0] = precision * distance * (contour_points[:, 0] - pic_width / 2)

        huh = np.zeros_like(map)
        huh[
            largest_contour[:, 1],
            largest_contour[:, 0]
        ] = 255

        points3d = np.hstack(
            (real_points, np.ones((len(real_points), 1)) * 0.5),
            dtype=np.float32
        )
        self.pcd_pub.publish(point_cloud(points3d, 'chassis'))

        # extract main axis
        print(largest_contour.shape)
        center = np.mean(largest_contour, axis=0)
        # centered_contour


        img_contour = cv2.drawContours(map, contours, -1, color=155, thickness=1)
        scale = 10
        cv2.imshow('huh', cv2.resize(img_contour, dsize=None, fx=scale, fy=scale))
        cv2.waitKey(1)

    def extract_main_axis(self, centered_points:np.ndarray):

        pca = PCA(n_components=2)
        pca.fit(centered_points)
        main_axis = pca.components_[0]
        return main_axis

    def split_cloud(self, center, points_2d, n_iter=4):
        if n_iter == 0:
            return [center]
        
        # 1) center all points
        centered_points = points_2d - center

        # 2) extract main axis
        main_axis = self.extract_main_axis(centered_points)

        # 3) project all points onto that axis
        projections = np.dot(centered_points, main_axis)
        argsorted_proj = np.argsort(projections)

        projections = projections[argsorted_proj]
        argsorted_points = points_2d[argsorted_proj]
        # check that no points are behind the car
        if projections[-1] < 0:
            return np.asarray([np.zeros_like(points_2d[0])])
        # check that segment is longer that the width of the road
        if projections[-1] - projections[0] < ROAD_WIDTH:
            return np.asarray([center, center + main_axis * ROAD_WIDTH/2])
        

        # 4) get hal along selected axis
        half_split = int(len(projections) / 2)
        forward_pt1 = argsorted_points[half_split]

        forward_pt2 = argsorted_points[half_split + 1]
        i = 1
        while i < len(argsorted_proj)-1 and np.linalg.norm((forward_pt2 - forward_pt1)) < 1:
            i += 1
            forward_pt2 = argsorted_points[half_split + 1]

        #  5) select centers for next splits
        next_fwd_center = (forward_pt1 + forward_pt2) / 2
        next_bak_center = center

        # 6) call recursive and get their centers 
        centers_bak = self.split_cloud(next_bak_center, argsorted_points[:half_split], n_iter-1)
        centers_fwd = self.split_cloud(next_fwd_center, argsorted_points[half_split:], n_iter-1)
        # print(centers_bak, flush=True)
        # print(centers_fwd, flush=True)
        return np.concatenate((centers_bak, centers_fwd))

    def order_points(self, points2d):
        closest_idx = 0
        ordered_points = []
        while len(points2d) > 1:
            cur_center = points2d[closest_idx]
            ordered_points.append(copy(cur_center))
            points2d = np.delete(points2d, closest_idx, axis=0)
            closest_idx = np.argmin(np.linalg.norm(points2d - cur_center, axis=1))
        
        return np.asarray(ordered_points)

        

    def lidar_callback(self, msg:PointCloud2):
        cur_time = time_to_sec(msg.header.stamp)
        pos, rot, vel, time = self.find_closest_time(cur_time)
        points = read_points_numpy(msg)

        # extract 2d values
        points_2d = points[:, :2]
        points_2d = points_2d[points_2d[:, 0] >= -0.5]
        # points_2d = np.insert(points_2d, 0, [0, 0], axis=0)

        # extract road center by splitting cloud along its main axis
        centers = self.split_cloud(
            center=[0, -0.5], 
            points_2d=points_2d,
            n_iter=3
            )

        # centers = centers[np.linalg.norm(centers - centers[0], axis=1)<6]
        centers = self.order_points(centers)[1:]
        # make it smoother
        if len(centers) > 3:
            centers = interpolate_points(centers, 10, 20)
            # centers = interpolate_points(centers, 30, 20)

        # add colors for debug
        colors = np.linspace(0xFFFFFF, 0xFF0000, centers.shape[0], dtype=np.uint32)

        # make the z_axis 1
        points3d = np.hstack(
            (centers, np.ones((len(centers), 1)) * 0.5),
            dtype=np.float32
        )

        # transform to global
        global_points = R.from_euler('xyz', rot).apply(points3d) + pos
        
        # publish points
        # self.pcd_pub.publish(point_cloud(global_points, 'odom'))
        self.pcd_pub.publish(point_cloud(global_points, 'odom', colors))

    def odom_callback(self, msg:Odometry):
        time = self.time()
        pos, rot, vel = parse_odometry(msg)

        self.pos_hist[1:] = self.pos_hist[:-1]
        self.pos_hist[0] = pos

        self.rot_hist[1:] = self.rot_hist[:-1]
        self.rot_hist[0] = rot

        self.vel_hist[1:] = self.vel_hist[:-1]
        self.vel_hist[0] = vel
        
        self.time_hist[1:] = self.vel_hist[:-1]
        self.time_hist[0] = time


def main():
    ros.init()
    centerer = Centerer()

    ros.spin(centerer)
    centerer.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()

    
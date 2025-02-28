import rclpy as ros
from rclpy.node import Node


import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import Odometry

import numpy as np
import cv2
from scipy.spatial import Delaunay
from sklearn.decomposition import PCA
from utils import parse_odometry, time_to_sec, point_cloud

class Mapper(Node):
    def __init__(self):

        super().__init__('mapper')

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
            topic='/contour',
            qos_profile=10
        )

        # history of poses
        self.history_len = 200
        self.pose_hist = np.zeros((self.history_len, 3), np.float32)
        self.vel_hist = np.zeros((self.history_len, 3), np.float32)
        self.time_hist = np.zeros((self.history_len, 3), np.float32)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def find_closest_time(self, time):
        min_idx = np.argmin(np.abs(self.time_hist - time))
        return self.pose_hist[min_idx] + 0., self.vel_hist[min_idx]+0., self.time_hist[min_idx]+0.

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

        # 4) get hal along selected axis
        half_split = int(len(projections) / 2)
        forward_pt1 = argsorted_points[half_split]

        # FIXME make sure that pt1 and pt2 lie on opposite sides
        if projections[half_split + 1] > projections[half_split + 2]:
            forward_pt2 = argsorted_points[half_split + 1]
        else:
            forward_pt2 = argsorted_points[half_split + 2]
        
        #  5) select centers for next splits
        next_fwd_center = (forward_pt1 + forward_pt2) / 2
        next_bak_center = center

        # 6) call recursive and get their centers 
        centers_bak = self.split_cloud(next_bak_center, argsorted_points[:half_split], n_iter-1)
        centers_fwd = self.split_cloud(next_fwd_center, argsorted_points[half_split:], n_iter-1)
        # print(centers_bak, flush=True)
        # print(centers_fwd, flush=True)
        return np.concatenate((centers_bak, centers_fwd))


    def lidar_callback(self, msg:PointCloud2):
        time = time_to_sec(msg.header.stamp)
        points = read_points_numpy(msg)

        # extract 2d values
        points_2d = points[:, :2]

        # get first forseeable center
        # real_points = self.extract_directions([-1, 0], points_2d)

        centers = self.split_cloud(
            center=[-1, 0], 
            points_2d=points_2d,
            n_iter=3
            )

        points3d = np.hstack(
            (centers, np.ones((len(centers), 1)) * 0.5),
            dtype=np.float32
        )

        self.pcd_pub.publish(point_cloud(points3d, 'chassis'))


    def odom_callback(self, msg:Odometry):
        time = self.time()
        pose, vel = parse_odometry(msg)

        self.pose_hist[1:] = self.pose_hist[:-1]
        self.pose_hist[0] = pose

        self.vel_hist[1:] = self.vel_hist[:-1]
        self.vel_hist[0] = vel
        
        self.time_hist[1:] = self.vel_hist[:-1]
        self.time_hist[0] = time


def main():
    ros.init()
    mapper = Mapper()

    ros.spin(mapper)
    mapper.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()

    
#!/usr/bin/env python3
import rclpy as ros
import cv2
from rclpy.node import Node
import numpy as np

from utils import interpolate_points, parse_odometry, time_to_sec, ROAD_WIDTH
from utils import occup_grid, points_to_map, points_from_map
from history_keeper import HistoryKeeper

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import OccupancyGrid

from scipy.spatial.transform import Rotation as R

class Mapper(Node):

    def __init__(self):
        super().__init__('mapper')

        # listen for road center
        self.road_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic='/road_points',
            qos_profile=1,
            callback=self.center_callback
        )

        # listen for odom
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.odom_callback,
            qos_profile=10
        )

        # publisher for roi
        self.roi_pub = self.create_publisher(
            msg_type=OccupancyGrid,
            topic='/car_roi',
            qos_profile=10
        )

        self.hist_keeper = HistoryKeeper(100)


        self.grid_size = 0.1 # m
        self.car_size = 4 # m
        self.car_radius = int(self.car_size / self.grid_size)
        self.map_height = 400
        self.map_width = 400
        self.offset_x = -self.map_height/2
        self.offset_y = -self.map_width/2
        self.roi = 40 # m
        self.roi = int(self.roi / self.grid_size)
        self.map_data = np.zeros(
            (int(self.map_height / self.grid_size), int(self.map_width / self.grid_size))
        )

        self.visited = np.ones_like(self.map_data) * 255

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9


    def update_map(self, map_points):
        map_points = map_points[map_points[:, 0] >= 0]
        map_points = map_points[map_points[:, 0] < self.map_data.shape[0]]
        map_points = map_points[map_points[:, 1] >= 0]
        map_points = map_points[map_points[:, 1] < self.map_data.shape[1]]
        for point in map_points:
            if self.map_data[point[0], point[1]] < 255:
                self.map_data[point[0], point[1]] += 1

    def color_known_points(self, pos):
        car_pos = points_to_map([pos], self.grid_size, [self.offset_x, self.offset_y])[0]
        self.visited = cv2.circle(self.visited, car_pos[::-1], self.car_radius, color=0, thickness=-1)
        # cv2.imshow('huh', cv2.resize(mask, (800, 800)))
        # cv2.waitKey(1)


    def erode_map(self):
        ksize = int(ROAD_WIDTH * 0.75 / self.grid_size)
        if ksize % 2 == 0:
            ksize -= 1
        eroded_map = cv2.erode(self.map_data, kernel=np.ones((ksize,ksize)))
        return eroded_map
    
    def draw_car(self, zoom=False):
        time = self.time()
        pos, _, _, _ = self.hist_keeper.find_closest_time(time)
        mapped_pos = points_to_map([pos], self.grid_size, [self.offset_x, self.offset_y])[0]
        car_map = self.erode_map()
        car_radius = 40
        padding = 50
        car_map = cv2.circle(car_map, [mapped_pos[1], mapped_pos[0]], radius=car_radius, color=0, thickness=2)
        if zoom:
            max_x = mapped_pos[0] + car_radius + padding
            min_x = mapped_pos[0] - car_radius - padding
            max_y = mapped_pos[1] + car_radius + padding
            min_y = mapped_pos[1] - car_radius - padding
            return car_map[min_x:max_x, min_y:max_y]
        return car_map

    def get_roi(self, time=None):
        if time is None:
            time = self.time()
        pos, _, _, _ = self.hist_keeper.find_closest_time(time)
        mapped_pos = points_to_map([pos], self.grid_size, [self.offset_x, self.offset_y])[0]

        max_x = mapped_pos[0] + self.roi
        min_x = mapped_pos[0] - self.roi
        max_y = mapped_pos[1] + self.roi
        min_y = mapped_pos[1] - self.roi

        max_x, min_x = np.clip((max_x, min_x), 0, int(self.map_height / self.grid_size))
        max_y, min_y = np.clip((max_y, min_y), 0, int(self.map_width / self.grid_size))

        return cv2.bitwise_and(self.map_data, self.visited)[min_x:max_x, min_y:max_y], pos, (min_x, min_y)

    def center_callback(self, msg:PointCloud2):
        # print('called')
        time = time_to_sec(msg.header.stamp)
        points = read_points_numpy(msg, skip_nans=True)
        pos, rot, _, _ = self.hist_keeper.find_closest_time(time)
        points = R.from_euler('xyz', rot).apply(points) + pos

        # cut off colors if they are present
        points2d = points[:, :3]
        mapped = points_to_map(points2d, self.grid_size, [self.offset_x, self.offset_y])
        self.update_map(mapped)
        self.color_known_points(pos)
        # car = self.draw_car(True)
        car = self.get_roi()[0]
        # cv2.imshow('map', cv2.resize(car, (800,800)))
        # cv2.waitKey(1)
        self.publish_roi(time)

    def publish_roi(self, time=None):
        if time is None:
            time = self.time()
        roi, pos, (min_x, min_y) = self.get_roi()
        origin = points_from_map([[min_x, min_y]], self.grid_size, [self.offset_x, self.offset_y])[0]
        msg = occup_grid(
            map=roi,
            resolution=self.grid_size,
            origin_pos=origin,
            time=time
        )
        self.roi_pub.publish(msg)

    def odom_callback(self, msg:Odometry):
        time = self.time()
        pos, rot, vel = parse_odometry(msg)
        self.hist_keeper.update(pos, rot, vel, time)

def main():
    ros.init()
    mapper = Mapper()

    ros.spin(mapper)
    mapper.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
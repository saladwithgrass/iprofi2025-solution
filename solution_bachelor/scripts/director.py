import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import rclpy as ros
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from utils import load_grid, parse_odometry, points_from_map, points_to_map, ROAD_WIDTH, normal_2d, marker_msg
from history_keeper import HistoryKeeper
from visualization_msgs.msg import Marker

class Director(Node):

    def __init__(self):
        super().__init__('director')

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

        self.map_resolution = 0.1
        self.car_pos = [0, 0]
        self.lookahead = 10
        self.map_size = 800
        self.lookahead_thickness = 10
        self.cur_map = np.zeros((self.map_size, self.map_size))
        self.lookahead_circle_mask = None
        self.update_lookahead_circle()

        self.hist_keeper = HistoryKeeper()

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def update_lookahead_circle(self):
        self.lookahead_circle_mask = np.zeros_like(self.cur_map, np.uint8)
        cv2.circle(
            img=self.lookahead_circle_mask,
            center=(int(self.lookahead_circle_mask.shape[0]/2), int(self.lookahead_circle_mask.shape[1]/2)),
            radius=int(self.lookahead/self.map_resolution),
            color=255,
            thickness=self.lookahead_thickness
        )

    def get_possible_targets(self, map):
        return cv2.bitwise_and(map, self.lookahead_circle_mask)

    def select_road_ahead(self, map, vel, rot):
        loc_vel = R.from_euler('xyz', rot).apply([1, 0, 0])[:2][::-1]
        center = [
            int(map.shape[0] / 2),
            int(map.shape[1] / 2)
        ]

        width = map.shape[0]
        normal_1, normal_2 = normal_2d(loc_vel)

        normal_1 = (normal_1 * width).astype(int)
        normal_2 = (normal_2 * width).astype(int)
        close_points = np.array([normal_1 + center, normal_2 + center])

        far_points = close_points + (loc_vel*2*width).astype(int)

        all_points = np.concatenate((close_points, far_points[::-1]))

        mask = np.zeros_like(map)
        cv2.fillPoly(mask, [all_points], color=255)
        return cv2.bitwise_and(mask, map)
    
    def select_point_ahead(self, map, resolution, origin):
        x_indeces, y_indeces = np.where(map == 255)
        map_points = np.vstack((x_indeces, y_indeces)).T
        avg_index = np.mean(map_points, axis=0, dtype=int)
        return points_from_map([avg_index], resolution, origin)[0]


    def roi_callback(self, msg:OccupancyGrid):
        time = self.time()
        pos, rot, vel, _ = self.hist_keeper.find_closest_time(time) 
        data, resolution, origin = load_grid(msg)

        ksize = 5

        _, data = cv2.threshold(
            data,
            thresh=4,
            maxval=255,
            type=cv2.THRESH_BINARY
        )
        data = cv2.dilate(data, kernel=np.ones((ksize, ksize)))
        ksize = int(ROAD_WIDTH * 0.6 / self.map_resolution)
        if ksize % 2 == 0:
            ksize -= 1
        data = cv2.erode(data, kernel=np.ones((ksize,ksize)))
        data = self.select_road_ahead(data, vel, rot)
        cv2.imshow('huh', data)
        cv2.waitKey(1)


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
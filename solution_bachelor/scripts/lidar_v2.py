import rclpy as ros
import cv2
import numpy as np
import open3d as o3d

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

from skimage.measure import ransac
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree

from utils import point_cloud

class Lidar(Node):

    def __init__(self):

        super().__init__('lidar')

        # listen for point clouds
        self.rgbd_sub = self.create_subscription(
            msg_type=PointCloud2,
            callback=self.rgbd_callback,
            topic='/camera/points',
            qos_profile=10
        )

        # publisher for processed points
        self.groud_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic='/processed_points',
            qos_profile=10
        )

        # camera position parameters
        self.rotation = R.from_quat([-0.60816, 0.60816, -0.36075, 0.36075])
        self.position = np.array([-3, 0, 3])

        # crop car
        self.car_mask_pts = np.array([
            [375, 190],
            [259, 190],
            [205, 390],
            [425, 390],
        ], dtype=np.int32)
        self.imsize = (480, 640)
        self.car_poly = np.zeros(self.imsize, dtype=np.uint8) * 255
        cv2.fillPoly(self.car_poly, [self.car_mask_pts], color=255)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def convert_points_to_camera(self, points):
        return self.rotation.apply(points) + self.position

    def estimate_ground_open3d(self, pcd:o3d.geometry.PointCloud, distance_threshold=0.07):
        # Estimate plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                               ransac_n=3,
                                               num_iterations=1000)

        [a, b, c, d] = plane_model
        return plane_model, inliers
   
    def downsample_pointcloud(self, pcd:o3d.geometry.PointCloud, voxel_size=0.1):
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        return downsampled_pcd

    def estimate_ground_normals(self, pcd, angle_threshold=30):
        # Estimate normals
        pcd.estimate_normals()
        normals = np.asarray(pcd.normals)

        # Find points with normals close to vertical
        angles = np.arccos(np.abs(normals[:, 2])) * 180 / np.pi
        ground_mask = angles < angle_threshold

        return ground_mask

    def extract_road_by_color(self, colors):
        """
        Return image where the parts that are most probable to be the road are white.
        """
        colors = colors.tobytes()
        colors = np.frombuffer(colors, dtype=np.uint8)
        blue = colors[0::4]
        green = colors[1::4]
        red = colors[2::4]
        color_data = np.stack((blue, green, red), axis=1, dtype=np.uint8).reshape((480, 640, 3))
        hue_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2HSV)[:, :, 0]
        hue_data = cv2.bitwise_or(hue_data, self.car_poly)
        dilation_kernel = np.ones((3,3))
        dilated_hue = cv2.dilate(hue_data, dilation_kernel, iterations=4)
        _, dilated_hue = cv2.threshold(
            dilated_hue,
            thresh=20,
            maxval=255,
            type=cv2.THRESH_TOZERO
        )
        dilated_hue = cv2.bitwise_not(dilated_hue)
        return dilated_hue


    def rgbd_callback(self, msg:PointCloud2):

        # convert point to numpy
        point_data = read_points_numpy(msg, skip_nans=True)
        # extract colors
        colors = point_data[:, -1]
        # extract 3d points
        points_3d = point_data[:, :3]

        # convert to camera pos
        points_3d[:, :3] = self.convert_points_to_camera(points_3d[:, :3])

        road_image = self.extract_road_by_color(colors)
        cv2.imshow('road', road_image)
        cv2.waitKey(1)
        # convert to rgb for point3d
        road_image = cv2.cvtColor(road_image, cv2.COLOR_GRAY2RGB)
        # flatten to align with points
        road_image = road_image.reshape((road_image.shape[0] * road_image.shape[1], 3)) / 255.

        # crop by height
        height_mask = points_3d[: , 2] < 3
        points_3d = points_3d[height_mask]
        road_image = road_image[height_mask]

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(points_3d)
        o3d_pcd.colors = o3d.utility.Vector3dVector(road_image)
        o3d_pcd = self.downsample_pointcloud(
            pcd=o3d_pcd,
            voxel_size=0.5
            )

        # estimate ground with normals
        ground_mask = self.estimate_ground_normals(o3d_pcd)
        downsampled_points = np.asarray(o3d_pcd.points)[ground_mask]
        downsampled_colors = np.asarray(o3d_pcd.colors)[ground_mask]
        # color represents sureness by color
        downsampled_colors = downsampled_colors[:, 0]

        downsampled_points = downsampled_points[downsampled_colors > 0.5]

        self.groud_pub.publish(point_cloud(np.array(downsampled_points), 'chassis'))


def main():
    ros.init()
    lidar = Lidar()

    ros.spin(lidar)
    ros.shutdown()

if __name__ == '__main__':
    main()
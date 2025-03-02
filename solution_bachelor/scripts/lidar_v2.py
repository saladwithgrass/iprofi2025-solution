import rclpy as ros
import matplotlib.pyplot as plt
import cv2
import numpy as np
import open3d as o3d

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

from skimage.measure import ransac
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree

from utils import point_cloud, sec_to_time, time_to_sec, ROAD_WIDTH

class Lidar(Node):

    def __init__(self):

        super().__init__('lidar')

        # print(self._parameters)
        # # self.use_sim_time = True
        # param = self.get_parameter('use_sim_time')
        # print(param.value)

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
        self.car_poly = np.ones(self.imsize, dtype=np.uint8) * 255
        cv2.fillPoly(self.car_poly, [self.car_mask_pts], color=0)
        
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
            topic='/road_points',
            qos_profile=10
        )

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def remove_nan_vecs(self, points):
        good_points = np.prod(np.isfinite(points), axis=1, dtype=bool)
        return points[good_points]

    def create_pcd(self, points_3d):
        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(self.remove_nan_vecs(points_3d))
        return o3d_pcd

    def convert_points_to_camera(self, points):
        return self.rotation.apply(points) + self.position

    def estimate_ground_open3d(self, pcd:o3d.geometry.PointCloud, distance_threshold=0.07):
        # Estimate plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                               ransac_n=3,
                                               num_iterations=1000)

        [a, b, c, d] = plane_model
        return plane_model, inliers
   
    def downsample_pointcloud(self, pcd, voxel_size=0.1):
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

    def get_colored_image(self, colors):
        colors = colors.tobytes()
        colors = np.frombuffer(colors, dtype=np.uint8)
        return np.stack(
            (
                colors[0::4], 
                colors[1::4], 
                colors[2::4]
            ), 
            axis=1, 
            dtype=np.uint8
            ).reshape((480, 640, 3))

    def extract_road_image_by_color(self, color_data):
        """
        Return image where the parts that are most probable to be the road are white.
        """
        hue_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2HSV)[:, :, 0]
        hue_data = cv2.bitwise_and(hue_data, self.car_poly)
        hue_data = hue_data.astype(np.uint8)

        dilation_kernel = np.ones((5,5))
        dilated_hue = cv2.dilate(hue_data, dilation_kernel, iterations=5)

        _, dilated_hue = cv2.threshold(
            dilated_hue,
            thresh=50,
            maxval=255,
            type=cv2.THRESH_BINARY,
            dst=hue_data
        )
        # dilated_hue = cv2.GaussianBlur(dilated_hue, ksize=(155,155),sigmaX=10)
        dilated_hue = cv2.bitwise_not(dilated_hue)
        return dilated_hue

    def extract_road_by_color(self, colors, points_3d):
        road_image = self.extract_road_image_by_color(colors)
        road_image = road_image.flatten()
 
        points_3d = points_3d[road_image > 0]
        return points_3d
        pcd = self.create_pcd(points_3d)
        pcd = self.downsample_pointcloud(pcd, 1)
        return np.asarray(pcd.points)

    def extract_road_contours(self, image):
        thresholded = cv2.adaptiveThreshold(
            image, 
            maxValue=255,
            adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            thresholdType=cv2.THRESH_BINARY_INV,
            blockSize=5,
            C=1
            )

        cont_color = 155
        thresholded[:, 0] = cont_color
        thresholded[:, -1] = cont_color
        thresholded[0, :] = cont_color
        thresholded[-1, :] = cont_color

        car_point = (320, 300)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        found_contour = None
        contoured = np.zeros_like(thresholded, dtype=np.uint8)
        for contour in contours:
            # contoured *= 0
            # cv2.drawContours(contoured, [contour], -1, color=255, thickness=1)
            # cv2.imshow('huh', contoured)
            # cv2.waitKey(1000)
            dist = cv2.pointPolygonTest(contour=contour, pt=car_point, measureDist=True)
            
            if dist >= 0: # and dist < prev_dist:
                found_contour = contour
                break
        
        if found_contour is not None:
            contoured = np.zeros_like(thresholded, dtype=np.uint8)
            cv2.drawContours(contoured, [found_contour], -1, color=255, thickness=5)
            return contoured

        return thresholded
    
    def extract_road_contour_by_color(self, points_3d, colors):
        road_image = self.extract_road_by_color(colors)
        road_image = self.extract_road_contours(road_image)

        # cv2.imshow('road', road_image)
        # cv2.waitKey(1)

        # convert to rgb for point3d
        road_image = cv2.cvtColor(road_image, cv2.COLOR_GRAY2RGB)
        # flatten to align with points
        road_image = road_image.reshape((road_image.shape[0] * road_image.shape[1], 3)) / 255.

        # crop by height
        # height_mask = points_3d[:, 2] < 3
        road_image = road_image[points_3d[:, 2] < 3]
        points_3d = points_3d[points_3d[:, 2] < 3]

        # crop by color selection
        return points_3d[road_image[: , 0] > 0.5]
        # road_image = road_image[road_image[:, 0] > 0.5]

    def get_road_by_normals(self, points_3d):
        # init o3d objects
        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(points_3d)

        # downsample point cloud
        o3d_pcd = self.downsample_pointcloud(
            pcd=o3d_pcd,
            voxel_size=1
            )
        ground_mask = self.estimate_ground_normals(o3d_pcd, 30)
        return np.asarray(o3d_pcd.points)[ground_mask]

    def rgbd_callback(self, msg:PointCloud2):
        time = self.time()
        # convert point to numpy
        point_data = read_points_numpy(msg, skip_nans=False)
        # good_points = np.prod(np.isfinite(point_data), axis=1, dtype=bool)    
        # point_data = point_data[good_points]


        # extract colors
        colors = point_data[:, -1]
        colors = self.get_colored_image(colors)
        # extract 3d points
    
        points_3d = point_data[:, :3]
        # convert to camera pos
        points_3d[:, :3] = self.convert_points_to_camera(points_3d[:, :3])

        selected_points = self.extract_road_by_color(colors, points_3d)
        # selected_points = self.get_road_by_normals(points_3d)

        cloud_msg = point_cloud(selected_points, 'chassis')

        cloud_msg.header.stamp = sec_to_time(time)
        self.groud_pub.publish(cloud_msg)

def main():
    ros.init()
    lidar = Lidar()

    try:
        ros.spin(lidar)
    except KeyboardInterrupt:
        pass
    lidar.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()
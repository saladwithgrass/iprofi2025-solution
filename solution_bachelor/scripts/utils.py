import sensor_msgs.msg as sensor_msgs
import cv2
import std_msgs.msg as std_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import UnivariateSpline
from visualization_msgs.msg import Marker

ROAD_WIDTH = 4.9

def points_to_map(points, resolution, origin=np.zeros(2, float)):
        points = np.asarray(points)
        # scale points
        mapped_points = (points[:, :2] - origin) / resolution
        return mapped_points.astype(int)

def points_from_map(points, resolution, origin=np.zeros(2, float)):
        points = np.asarray(points)
        result = np.array(points, dtype=np.float64)
        result[:, 0] = (result[:, 0] + origin[0] / resolution) * resolution
        result[:, 1] = (result[:, 1] + origin[1] / resolution) * resolution
        return result

def cross_2d(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    return v1[0] * v2[1] - v1[1] * v2[0]

def normal_2d(pt1, pt2=np.zeros(2)):
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    normal_1 = np.array([-dy, dx], float) 
    normal_1 /= np.linalg.norm(normal_1)
    normal_2 = np.array([dy, -dx], float)
    normal_2 /= np.linalg.norm(normal_2)
    return normal_1, normal_2

def angle_from_vec_to_vec(v1, v2):
    return np.arctan2(
        cross_2d(v1, v2),
        np.dot(v1, v2)
    )

def quat_to_array(q:Quaternion):
    return q.x, q.y, q.z, q.w

def time_to_sec(time:Time):
    return time.sec + float(time.nanosec) / 1e9

def sec_to_time(sec:float):
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int((sec % 1) * 1e9)
    return msg

def parametric_interp(points, smoothing=1.):
    point_diffs = np.sum(np.diff(points, axis=0)**2, axis=1)
    curve_len = np.cumsum(point_diffs, axis=0)
    curve_len = np.insert(curve_len, 0, 0.)

    spline_x = UnivariateSpline(curve_len, points[:, 0], s=smoothing)
    spline_y = UnivariateSpline(curve_len, points[:, 1], s=smoothing)
    if points.shape[1] == 2:
        return spline_x, spline_y, curve_len

    spline_z = UnivariateSpline(curve_len, points[:, 2], s=smoothing)
    return spline_x, spline_y, spline_z, curve_len

def interpolate_points(points, n_steps=100, smoothing=3.):
    splines = parametric_interp(points, smoothing)

    curve_len = splines[-1]
    smooth_len = np.linspace(curve_len[0], curve_len[-1], n_steps)

    interp_x = splines[0](smooth_len)
    interp_y = splines[1](smooth_len)
    if points.shape[1] == 2:
        return np.vstack((interp_x, interp_y)).T

    interp_z = splines[2](smooth_len)
    return np.vstack((interp_x, interp_y, interp_z)).T

def parse_odometry(msg:Odometry):
    """Returns Position as Pose2d, Rotation in RPY as Pose2d and Velocity as Pose2d
    """
    pos = np.zeros(3, dtype=np.float32)
    rot = np.zeros(3, dtype=np.float32)
    velocity = np.zeros(3, dtype=np.float32)
    time = 0

    # get position
    pos[0] = msg.pose.pose.position.x
    pos[1] = msg.pose.pose.position.y
    pos[2] = msg.pose.pose.position.z

    # get rotation
    rot_quat = quat_to_array(msg.pose.pose.orientation)
    rot = R.from_quat(rot_quat).as_euler('xyz')

    # get velocity
    velocity[0] = msg.twist.twist.linear.x
    velocity[1] = msg.twist.twist.linear.y
    velocity[2] = msg.twist.twist.angular.z

    return pos, rot, velocity

def point_cloud(points, parent_frame, colors = None):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud1 message
    """
    # In a PointCloud1 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    bytesize = 3
    if colors is not None:
        buffer = []
        for i in range(points.shape[0]):
            buffer.extend(points[i].astype(np.float32).tobytes()) # Convert to float32 for consistency
            buffer.extend(colors[i].tobytes()) 
        data = bytes(buffer)
        bytesize = 4
    else:
        data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 3 bytes 
    # represents the x-coordinate, the next 3 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')
        ]
    
    if colors is not None:
        fields += [
            sensor_msgs.PointField(
                name='rgb', offset=3*itemsize, datatype=ros_dtype, count = 1
            )
        ]

    # The PointCloud1 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * bytesize), # Every point consists of three float32s.
        row_step=(itemsize * bytesize * points.shape[0]),
        data=data
    )

def occup_grid(map, resolution, origin_pos, time=0, frame_id='odom'):
    # print(origin_pos)
    map = np.array(map, dtype=np.int8)
    map -= 128
    msg = OccupancyGrid()
    
    msg.header.frame_id = frame_id
    msg.header.stamp = sec_to_time(time)

    msg.info.origin.position.x = origin_pos[0]
    msg.info.origin.position.y = origin_pos[1]

    msg.info.origin.orientation.w = -1.
    msg.info.origin.orientation.z = 0.

    msg.info.height = map.shape[0]
    msg.info.width = map.shape[1]
    msg.info.resolution = resolution

    msg.data = map.flatten().tolist()

    return msg

def load_grid(msg:OccupancyGrid):
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = np.zeros(3)
    origin[0] = msg.info.origin.position.x
    origin[1] = msg.info.origin.position.y
    origin[2] = msg.info.origin.position.z

    data = np.array(msg.data, int)
    data += 128
    data = data.astype(np.uint8)
    data.resize((height, width))
    return data, resolution, origin

def marker_msg(coord):
    msg = Marker()

    msg.type = 1

    msg.color.a = 1.
    msg.color.r = 1.
    msg.color.b = 1.
    msg.color.g = 1.

    msg.scale.x = 1.
    msg.scale.y = 1.
    msg.scale.z = 1.

    msg.pose.position.x = coord[0]
    msg.pose.position.y = coord[1]
    msg.pose.position.z = 1.

    msg.header.frame_id = 'odom'
    
    return msg

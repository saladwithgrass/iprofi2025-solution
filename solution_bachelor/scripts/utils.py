import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
import numpy as np
from scipy.spatial.transform import Rotation as R

def quat_to_array(q:Quaternion):
    return q.x, q.y, q.z, q.w

def time_to_sec(time:Time):
    return time.sec + time.nanosec / 1e9

def sec_to_time(sec:float):
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int((sec % 1) * 1e9)
    return msg


def parse_odometry(msg:Odometry):
    """Returns Position as Pose2d, Velocity as Pose2d
    """
    pose = np.zeros(3, dtype=np.float32)
    velocity = np.zeros(3, dtype=np.float32)
    time = 0

    # get position
    pose[0] = msg.pose.pose.position.x
    pose[1] = msg.pose.pose.position.y

    # get rotation
    rot_quat = quat_to_array(msg.pose.pose.orientation)
    pose[2] = R.from_quat(rot_quat).as_euler('zyx')[1]

    # get velocity
    velocity[0] = msg.twist.twist.linear.x
    velocity[1] = msg.twist.twist.linear.y
    velocity[2] = msg.twist.twist.angular.z

    return pose, velocity

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
        for i in range(points.shape[-1]):
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
                name='rgb', offset=2*itemsize, datatype=ros_dtype, count = 1
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


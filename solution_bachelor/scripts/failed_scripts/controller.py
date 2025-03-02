from shutil import rmtree

import sys, signal
from os import mkdir

import rclpy as ros
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Pose2D, Pose, Point
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
from solution_bachelor.msg import TwistStampedArray

import numpy as np
from numpy import sin, cos
from scipy.interpolate import pchip_interpolate
from scipy.spatial.transform import Rotation as R
from copy import copy
import pickle
from scipy.optimize import minimize

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # state variables
        self.pose = np.zeros((3,), dtype=np.float32)
        self.pose_callback_time = 0
        self.velocity = np.zeros((3,), dtype=np.float32)
        self.fwd_vel:float = 0
        self.vel_callback_time = 0
        self.to_local_tf = None

        # MPC PARAMETERS
        self.n_steps = 30 # mpc prediction steps
        self.desired_velocity = 0.5 # velocity that car should be moving with
        self.lookahead = 4 # distance that mpc optimizes on
        self.interp_dx = 0.25

        # error costs
        self.Q = np.array([
            [1., 0],
            [0,   1.2]
        ]) * 70
        # control costs
        self.R = np.array([
                [0.5, 0],
                [0, 0.2]

            ])

        self.max_torque_percent = 20
        self.max_wheel_angle = 0.6
        self.u1_bounds = np.array([-self.max_torque_percent, self.max_torque_percent]) # torque bounds
        self.u2_bounds = np.array([-self.max_wheel_angle, self.max_wheel_angle]) # steer bounds

        # car distance between front and mass center
        self.lf = 4.3 / 2

        # target trajectory
        self.target_trajectory = np.zeros((1,2), dtype=np.float32)
        self.immediate_trajectory = None
        self.immediate_len = 0
        self.immediate_timestamps = None

        # planner publisher
        self.plan_publisher = self.create_publisher(
            msg_type=MarkerArray,
            topic='/immediate_path',
            qos_profile=10
        )

        self.prediction_pub = self.create_publisher(
            msg_type=MarkerArray,
            topic='/pred_path',
            qos_profile=10
        )

        # state subscribers
        self.pos_subscriber = self.create_subscription(
            msg_type=Pose2D,
            topic='/car_pose/pose_2d',
            callback=self.pos_callback,
            qos_profile=10
        )
        self.vel_subscriber = self.create_subscription(
            msg_type=Pose2D,
            topic='/car_pose/velocity',
            callback=self.vel_callback,
            qos_profile=10
        )

        # publisher for commands
        self.control_publisher = self.create_publisher(
            msg_type=TwistStampedArray,
            topic='/scheduled_controls',
            qos_profile=10
        )

        return 
        self.init_time = self.get_clock().now().nanoseconds / 1e9
        self.is_accel_initialized = False
        self.is_vel_intialized = False
        self.is_initialized = False
        t_wait = 3
        print(f'----waiting {t_wait} seconds to init----')
        while self.time() < t_wait or self.to_local_tf is None:
            ros.spin_once(self, timeout_sec=0.5)
        
        self.is_initialized = True
        print('---------IM READY------------')

        base_trajectory = np.array([
            [0, 0],
            [3, 0],
            [6, -2],
            [9, 2],
        ]) * 2

        self.set_target_trajectory_local(base_trajectory)

        self.init_time = self.get_clock().now().nanoseconds / 1e9

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.init_time

    def smooth_trajectory(self, trajectory_2d):
        trajectory_len = np.sum(np.linalg.norm(trajectory_2d, axis=1))
        interp_steps = int(trajectory_len / self.interp_dx)

        t_param = trajectory_2d.shape[0]
        base_t = np.arange(0, t_param)
        interp_t = np.linspace(0, t_param-1, interp_steps)

        smooth_x = pchip_interpolate(base_t, trajectory_2d[:, 0], interp_t)
        smooth_y = pchip_interpolate(base_t, trajectory_2d[:, 1], interp_t)

        return np.stack((smooth_x, smooth_y), axis=1)

    def create_immediate_trajectory(self, trajectory_2d):
        # select points that are around target distance
        found_distance = 0
        last_point = 0
        while found_distance <= self.lookahead and last_point < trajectory_2d.shape[0]-1:
            last_point += 1
            found_distance += np.linalg.norm(trajectory_2d[last_point] - trajectory_2d[last_point-1])

        # find diff between desired and found
        distance_diff = found_distance - self.lookahead
        immediate_segment = trajectory_2d[:last_point]
        self.immediate_len = found_distance
        if distance_diff > 0.5:

            # find new point where trajectory will be split
            last_segment_vec = trajectory_2d[last_point] - trajectory_2d[last_point - 1]
            last_segment_len = np.linalg.norm(last_segment_vec)

            # find new segment len
            new_segment_len = last_segment_len - distance_diff
            if last_segment_len != 0:
                new_segment_vec = last_segment_vec / last_segment_len * new_segment_len

                # calculate new point
                new_point = np.array([trajectory_2d[last_point - 1] + new_segment_vec])

                # immediate segment
                immediate_segment = np.vstack((immediate_segment, new_point))
            else:
                immediate_segment = trajectory_2d
            self.immediate_len = self.lookahead

        # create base t 
        base_t = np.arange(0, immediate_segment.shape[0])

        # create new t for interp
        new_t = np.linspace(0, base_t[-1], self.n_steps)

        # interpolate coordinates
        new_x = pchip_interpolate(base_t, immediate_segment[:, 0], new_t)
        new_y = pchip_interpolate(base_t, immediate_segment[:, 1], new_t)
        return np.stack((new_x, new_y), axis=1)

    def extract_angles_from_traj(self, trajectory_points):
        angles = np.zeros((trajectory_points.shape[0]), dtype=np.float32)
        for angle_idx in range(trajectory_points.shape[0] - 1):
            cur_p = trajectory_points[angle_idx]
            next_p = trajectory_points[angle_idx+1]
            derivative_vector = next_p - cur_p
            angles[angle_idx] = np.arctan2(derivative_vector[1], derivative_vector[0]) % (2 * np.pi)

        angles[-1] = angles[-2]
        return np.array([angles]).T

    def generate_time_grid(self):
        self.immediate_timestamps = np.linspace(0, self.immediate_len/self.desired_velocity, self.n_steps) + self.time()

    def set_target_trajectory(self, trajectory_2d):
        # make a smooth trajectory 
        self.target_trajectory = self.smooth_trajectory(trajectory_2d)

        # create immediate trajectory
        self.immediate_trajectory = self.create_immediate_trajectory(self.target_trajectory)
        
        # add angles to immediate trajectory
        trajectory_angles = self.extract_angles_from_traj(self.immediate_trajectory)
        self.immediate_trajectory = np.hstack((self.immediate_trajectory, trajectory_angles))

        # generate time grid
        self.generate_time_grid()

    def set_target_trajectory_local(self, local_traj_2d):

        # apply transform to rotate this trajectory
        fake_angles = np.zeros((local_traj_2d.shape[0], 1), dtype=np.float32)
        local_traj_2d[:, 1] *= -1
        global_traj = self.to_local_tf.apply(np.hstack((local_traj_2d, fake_angles)))[:, :2]

        # add shift related to pose
        global_traj += self.pose[:2]

        # set trajectory
        self.set_target_trajectory(global_traj)

    def remove_old_points(self):

        # points are removed based on distance to the car
        # i will find distances from each point to current car
        # select the closest one
        # and remove all points before it
        distances = np.linalg.norm(self.target_trajectory - self.pose[:2], axis=1)
        min_idx = np.argmin(distances)
        print('removed point')
        self.target_trajectory = self.target_trajectory[min_idx:]

    def update_trajectory(self):
        self.remove_old_points()
        if self.target_trajectory.shape[0] == 0:
            self.target_trajectory = np.ones((self.n_steps * 5, 2), dtype=np.float32) * self.pose[:2]

        # create immediate trajectory
        self.immediate_trajectory = self.create_immediate_trajectory(self.target_trajectory)
        
        # add angles to immediate trajectory
        trajectory_angles = self.extract_angles_from_traj(self.immediate_trajectory)
        self.immediate_trajectory = np.hstack((self.immediate_trajectory, trajectory_angles))

        # generate time grid
        self.generate_time_grid()

    def vec_to_local(self, vec):
        return self.to_local_tf.apply(vec)

    def pos_callback(self, msg:Pose2D):
        prev_pos_time = self.pose_callback_time
        cur_time = self.time()

        self.pose_callback_time = cur_time

        # update values
        self.pose[0] = msg.x
        self.pose[1] = msg.y
        self.pose[2] = msg.theta

        self.to_local_tf = R.from_euler('xyz', [0, 0, self.pose[2]])
        if not self.is_initialized:
            return 
        

        ### 1) update trajectory and remove old points
        self.update_trajectory()

        ### 2) run control

        controls = self.optimize_control_MPC(self.pose, self.fwd_vel, self.immediate_trajectory)

        ### 3) create control msg

        vel_cmd = Twist()
        vel_cmd.linear.x = float(controls[0])
        vel_cmd.angular.z = float(controls[1])
        # self.control_publisher.publish(vel_cmd)

    def vel_callback(self, msg:Pose2D):
        self.is_vel_intialized = True
        prev_time = self.vel_callback_time
        cur_time = self.time()
        self.vel_callback_time = cur_time

        self.velocity[0] = msg.x
        self.velocity[1] = msg.y
        self.velocity[2] = msg.theta
        self.fwd_vel = self.vec_to_local(self.velocity)[0]

    def control_to_accel(self, u):
        m_c = 1500
        r = 0.3
        m_w = 11
        J_w = 1 /2 * m_w * r**2

        max_torque = 19.78
        a_c = max_torque * u / (
            (m_c + 4 * m_w) * r +
            4 * J_w / r
        )

        return a_c

    def next_model_step(self, cur_pos:np.ndarray, cur_velocity:float, control_inputs:np.ndarray, dt:float):
        x_i, y_i, phi_i = cur_pos
        v_i = cur_velocity # R.from_euler('xyz', [0, 0, phi_i]).apply(cur_velocity)[0]
        u1 = control_inputs[0] # max torque percentage. 
        u2 = control_inputs[1] # steering wheel angle
        u2 = np.clip(u2, -0.6, 0.6)

        # friction losses
        k_friction = 0.05  # losses due to turns
        drag_coefficient = 0.00  # linear drag losses

        # 1) update velocity
        accel = self.control_to_accel(u1)
        lateral_loss = k_friction * abs(np.tan(u2)) * v_i**2  
        drag_loss = np.abs(drag_coefficient * v_i)  

        # make sure losses are always against velocity
        lateral_loss *= -np.sign(accel)
        drag_loss *= -np.sign(accel)

        next_vel = v_i + (accel + lateral_loss + drag_loss) * dt

        # 2) update angle
        lf = 2.3 # distance from center mass to front axle

        next_angular_vel = v_i * np.tan(u2) / lf * 0.25

        next_phi = phi_i + next_angular_vel * dt

        # make sure angle is between 0 and 2 pi
        next_phi = next_phi % (np.pi * 2)

        # 3) update GLOBAL position
        next_x = x_i + v_i * cos(phi_i) * dt
        next_y = y_i + v_i * sin(phi_i) * dt

        return np.array([next_x, next_y, next_phi]), next_vel, next_angular_vel

    def xte(self, point_a, point_b, pos):
        ab = point_b - point_a
        ap = pos - point_a
        if ab[0] == 0 and ab[1] == 0:
            return np.linalg.norm(ap)
        return np.abs(ab[0] * ap[1] - ab[1] * ap[0]) / np.linalg.norm(ab)
    
    def angle_error(self, phi_i, phi_d):
        d1 = np.abs(phi_d - phi_i)
        d2 = np.pi * 2 - d1
        return min(d1, d2)
    
    def normalize_control(self, u):
        return np.array([
            u[0] / self.max_torque_percent,
            u[1] / self.max_wheel_angle
            ]
        )

    def predict_and_cost(self, u, x0, v0, ref_traj, n_steps, dt, return_pred=False):
        """Predicts trajectory based on initial state step by step and
           calculates the cost of given control u.

        Args:
            u (np.ndarray): FLATTENED Controls to be evaluated. 
            x0 (np.ndarray): initial x, y, phi.
            v0 (np.ndarray): intial forward velocity
            ref_traj (np.ndarray): Array of points of desired x, y, phi
            n_steps (int): Prediction horizon in steps.
            dt (float): Prediction discrete time.
        Returns:
            float: cost of given trajectory
        """
        cur_pose = x0
        cur_vel = v0
        cost = 0
        time_elapsed = 0

        if return_pred:
            predictions = []

        for step_i in range(n_steps):
            # get current controls
            u_i = u[step_i*2:(step_i + 1) * 2]

            # get next step
            cur_pose, cur_vel, _ = self.next_model_step(
                cur_pos=cur_pose,
                cur_velocity=cur_vel,
                control_inputs=u_i,
                dt = dt
            )

            if return_pred:
                predictions.append(copy(cur_pose))
            
            time_elapsed += dt
            falloff = 1 # np.exp(-time_elapsed/3)
            
            # split pose into position and orientation
            pos = cur_pose[:2]
            phi = cur_pose[2]

            # get ref points
            point_a = ref_traj[step_i]
            point_b = ref_traj[min(step_i+1, len(ref_traj) - 1)]

            # calculate errors
            xte_error = self.xte(point_a[:2], point_b[:2], pos)
            angle_error = self.angle_error(phi, ref_traj[step_i, 2])
            # normalize xte
            xte_error /= self.lookahead
            # normalize angle error
            angle_error = angle_error / np.pi


            # calculate error cost
            state_error = np.array([xte_error, angle_error])
            cost += state_error.T @ self.Q @ state_error * falloff

            # calculate control cost
            u_i = u[step_i*2:(step_i+1)*2]
            norm_u_i = self.normalize_control(u_i)
            cost += norm_u_i.T @ self.R @ norm_u_i * falloff

            # punish for not going with desired speed
            # calculate diff to desired vel
            vel_error = cur_vel - self.desired_velocity
            cost += vel_error**2 * 5 * falloff

        if return_pred:
            return cost, predictions

        return cost

    def optimize_control_MPC(self, x0, v0, ref_traj):

        self.publish_traj(ref_traj)
        
        u0 = np.ones(2 * self.n_steps) 
        # u0[::2] = 10
        u0[1::2] = 0.3 * np.sign(ref_traj[0,2] - x0[2])

        # create constraints 
        bounds = np.ones((self.n_steps * 2, 2), dtype=np.float32) 
        # and fill them with current bounds
        bounds[::2] = self.u1_bounds
        bounds[1::2] = self.u2_bounds
        
        # minimize cost
        opt_start = self.time()
        result = minimize(
                fun=lambda u: self.predict_and_cost(
                u=u,
                x0=x0,
                v0=v0,
                ref_traj=ref_traj,
                n_steps=self.n_steps,
                dt=(self.immediate_timestamps[1] - self.immediate_timestamps[0])
            ),
            x0=u0,
            # method='Nelder-Mead',
            method='SLSQP',
            bounds=bounds.reshape((self.n_steps * 2, 2)),
        )
        opt_end = self.time()
        print('optimization time: ', opt_end - opt_start)
        print(result.x[:2])
        # print('traj avg angle: ', np.average(ref_traj[:, 2]))
        # print('cur angle: ', x0[2])
        print('cost: ', result.fun)
        print('----------')
        if result['success'] or result.status == 1 or result.status == 9:
            self.schedule_controls(controls=result.x, times=self.immediate_timestamps)
            cost, pred = self.predict_and_cost(
                u=result.x,
                x0=x0, 
                v0=v0, 
                ref_traj=ref_traj,
                n_steps=self.n_steps, 
                dt=(self.immediate_timestamps[1] - self.immediate_timestamps[0]),
                return_pred=True
            )
            self.publish_traj(pred, self.prediction_pub, color=[0,1,0])
            return result.x[:2]
        else:
            print(result)
            return [0, 0]
        
    def schedule_controls(self, controls, times=None):
        """Publishes optimized control for the neares future.

        Args:
            controls (np.ndarray): FLATTENED array of control inputs.
            times (np.ndarray, optional): Times for controls to be dispatched. Defaults to None.
        """

        # check times
        if times is None:
            times = self.immediate_timestamps
        
        # check dimensions
        if controls.shape[0] != times.shape[0] * 2:
            self.get_logger().error(
                'CANT SCHEDULE CONTROL: TIME AND CONTROL HAVE DIFFERENT DIMS'
            )

        # create array to be published
        control_array:list[TwistStamped] = []

        for idx in range(times.shape[0]):
            cur_msg = TwistStamped()
            cur_msg.header.stamp.sec = int(times[idx])
            cur_msg.header.stamp.nanosec = int((times[idx] - int(times[idx])) * 1e9)
            cur_msg.twist.linear.x = float(controls[idx*2])
            cur_msg.twist.angular.z = float(controls[idx*2 + 1])
            control_array.append(cur_msg)

        # create schedule msg
        scheduled_msg = TwistStampedArray()
        scheduled_msg.twists = control_array
        # publish it
        self.control_publisher.publish(scheduled_msg)

    def publish_traj(self, ref_traj, pub=None, color=[1.,0., 1.]):

        ref_traj = np.array(ref_traj)
        color = np.array(np.array(color), dtype=float)

        if pub is None:
            pub = self.plan_publisher        
        
        markers = []

        for point_idx in range(ref_traj.shape[0]):

            marker = Marker()

            point = ref_traj[point_idx]

            cur_point = Point()
            cur_point.x = point[0]
            cur_point.y = point[1]
            cur_point.z = 1.

            phi = point[2]
            tmp_quat = R.from_euler('xyz', [0, 0, phi]).as_quat()
            cur_quat = Quaternion()

            cur_quat.x = tmp_quat[0]
            cur_quat.y = tmp_quat[1]
            cur_quat.z = tmp_quat[2]
            cur_quat.w = tmp_quat[3]

            cur_pose = Pose()
            # cur_pose.header.stamp = timestamp
            # cur_pose.header.stamp = self.immediate_timestamps
            cur_pose.position = cur_point
            cur_pose.orientation = cur_quat

            marker.pose = cur_pose
            marker.type = 0
            marker.id = point_idx
            marker.header.frame_id = 'odom'

            min_color = 0.5
            color_percent = min_color + (1 - min_color) * (point_idx / ref_traj.shape[0])
            marker.color.r = color[0] * color_percent
            marker.color.g = color[1] * color_percent
            marker.color.b = color[2] * color_percent
            marker.color.a = 1.

            scale = 0.1

            marker.scale.x = scale 
            marker.scale.y = scale / 2
            marker.scale.z = scale / 2

            markers.append(marker)

        path_msg = MarkerArray()
        path_msg.markers = markers
        # path_msg.header.frame_id = '<Fixed Frame>'
        # path_msg.header.stamp = self.time()
        pub.publish(path_msg)

    def save_data(self):
        with open(f'trajectories/data_{self.time()}.pkl', 'wb') as log_out:
            pickle.dump({
                'pos' : self.pos_data,
                'vel' : self.vel_data,
                'control' : self.control_data,
                'time' : self.timestamps
            },
            log_out
            )
    
def main():
    rmtree('trajectories')
    mkdir('trajectories')
    ros.init()
    control = Controller()

    try:
        ros.spin(control)
    except KeyboardInterrupt:
        print('AHA')
    finally:
        pass
    control.destroy_node()
    ros.shutdown()

if __name__ == '__main__':
    main()

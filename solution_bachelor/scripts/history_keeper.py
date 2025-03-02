import numpy as np
from scipy.spatial.transform import Rotation as R

class HistoryKeeper():
    def __init__(self, hist_len=200):
        # history of poses
        self.history_len = hist_len
        self.pos_hist = np.zeros((self.history_len, 3), np.float32)
        self.vel_hist = np.zeros((self.history_len, 3), np.float32)
        self.rot_hist = np.zeros((self.history_len, 3), np.float32)
        self.time_hist = np.zeros((self.history_len,), np.float64)
    
    def points_to_global(self, points, time):
        pos, rot, _, _ = self.find_closest_time(time)
        return R.from_euler('xyz', rot).apply(points) + pos

    def find_closest_time(self, time):
        min_idx = np.argmin(np.abs(self.time_hist - time))
        return ( 
            np.array(self.pos_hist[min_idx]), 
            np.array(self.rot_hist[min_idx]),
            np.array(self.vel_hist[min_idx]),
            np.array(self.time_hist[min_idx]),
        )

    def update(self, pos, rot, vel, time):
        self.pos_hist[1:] = self.pos_hist[:-1]
        self.pos_hist[0] = pos

        self.rot_hist[1:] = self.rot_hist[:-1]
        self.rot_hist[0] = rot

        self.vel_hist[1:] = self.vel_hist[:-1]
        self.vel_hist[0] = vel
        
        self.time_hist[1:] = self.time_hist[:-1]
        self.time_hist[0] = time
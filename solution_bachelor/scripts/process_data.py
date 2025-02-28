from scipy.stats import linregress
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from tqdm import tqdm
import numpy as np
import argparse
import pickle

def unpack_data(data):
    values = []
    times = []
    for value, time in tqdm(data):
        values.append(value)
        times.append(time)

    return np.array(values), np.array(times)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file')
    args = parser.parse_args()

    file_name:str = args.input_file

    ### LOAD DATA
    data_name = file_name
    with open(data_name, 'rb') as in_file:
        data = pickle.load(in_file)

    pos_data = data['pos']
    vel_data = data['vel']
    acc_data = data['acc']

    # extract torque
    torque_str = file_name.removeprefix('data_collection/data_').removesuffix('.pkl')
   
    ### CALCULALTE CAR MODEL
    m_c = 1500
    r = 0.3
    m_w = 11
    J_w = 1 /2 * m_w * r**2
    T = float(torque_str)

    a_c = T / (
        (m_c + 4 * m_w) * r +
        4 * J_w / r
    )

    ### LOAD ANGLES
    pos_values, pos_time = unpack_data(pos_data)
    ### LOAD VELOCITY
    vel_values, vel_time = unpack_data(vel_data)
    ### TRANSFORM VELOCITY
    for vel, pos in zip(vel_values, pos_values):
        vel[:2] = R.from_euler('xyz', [0, 0, -pos[2]]).apply(vel)[:2]
    
    ### PLOT VELOCITY

    selected_time = (vel_time > 3) * (vel_time < 7)
    slope, intercept, _, _, error = linregress(vel_time[selected_time], vel_values[selected_time, 0])
    plt.plot(vel_time, vel_values[:, 0])
    plt.plot(vel_time[selected_time], vel_time[selected_time] * slope + intercept)
    plt.plot(vel_time[selected_time], vel_time[selected_time] * a_c + intercept)
    plt.legend(['real', 'line fit', 'model'])
    print('##################')
    print('torq: ', T)
    print('model acceleration: ', a_c)
    print('real acceleration: ', slope)
    print('coeff: ', slope/a_c)
    print('##################')
    plt.show()



if __name__ == '__main__':
    main()
#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np


def crop_data(data, time, t0, t1):
    tmp = np.abs(time - t0)
    a = tmp.argmin()
    tmp = np.abs(time - t1)
    b = tmp.argmin()
    return data[a:b], time[a:b]


def plot_depth_vs_setpoint(reader: Reader):
    t_offset = 1699443392.7
    t_start = 1732104400.0
    t_end = 1732104480.0
    setpoint_data = reader.get_data('/bluerov00/depth_setpoint')
    n_messages = len(setpoint_data)
    depth_setpoints = np.zeros([n_messages])
    t_setpoints = np.zeros([n_messages])

    i = 0
    for msg, time_received in setpoint_data:
        depth_setpoints[i] = msg.data
        t_setpoints[i] = time_received * 1e-9
        i += 1
    t_setpoints -= t_offset
    depth_setpoints, t_setpoints = crop_data(depth_setpoints, t_setpoints,
                                             t_start, t_end)
    data = np.hstack(
        [t_setpoints.reshape(-1, 1),
         depth_setpoints.reshape(-1, 1)])
    np.savetxt('export/depth_setpoint.csv',
                data,
                delimiter=',',
                header='t, depth_setpoint',
                comments='')

    depth_data = reader.get_data('/bluerov00/depth')
    n_messages = len(depth_data)
    depth = np.zeros([n_messages])
    t_depth = np.zeros([n_messages])

    i = 0
    for msg, time_received in depth_data:
        depth[i] = msg.depth
        t_depth[i] = time_received * 1e-9
        i += 1
    t_depth -= t_offset
    depth, t_depth = crop_data(depth, t_depth, t_start, t_end)

    data = np.hstack([t_depth.reshape(-1, 1), depth.reshape(-1, 1)])
    np.savetxt('export/depth.csv',
                data,
                delimiter=',',
                header='t, depth',
                comments='')

    plt.figure()
    plt.plot(t_setpoints, depth_setpoints, label='Depth Setpoint')
    plt.plot(t_depth, depth, label='Current Depth')
    plt.legend()
    plt.show()


def main():
    reader = Reader('my_bag_file')
    plot_depth_vs_setpoint(reader)


if __name__ == '__main__':
    main()

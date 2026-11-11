"""Offline analysis of a recorded bag for the report.

    ros2 run csen934_project_pkg analyze_bag <bag_dir> [--out results] [--storage sqlite3]

Reads /Car_Odom_GT, /Car_Odom_Noisy, /Car_Odom_Filtered, matches messages on
identical header stamps, and writes:
    metrics.txt        RMSE / mean / std of noisy and filtered error vs GT
                       (the noisy std doubles as a check of the noise node)
    xy_yaw_vs_time.png x, y, yaw over time for all three streams
    trajectory.png     XY trajectories
    error_vs_time.png  position and yaw error over time
"""

import argparse
import math
import os

import numpy as np

TOPICS = {'/Car_Odom_GT': 'GT', '/Car_Odom_Noisy': 'Noisy', '/Car_Odom_Filtered': 'Filtered'}


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(uri, storage):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    data = {n: {} for n in TOPICS.values()}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        m = deserialize_message(raw, Odometry)
        key = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        p = m.pose.pose.position
        data[TOPICS[topic]][key] = (p.x, p.y, _yaw(m.pose.pose.orientation),
                                    m.twist.twist.linear.x, m.twist.twist.angular.z)
    return data


def analyse(data, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    os.makedirs(out, exist_ok=True)
    keys = sorted(set(data['GT']) & set(data['Noisy']) & set(data['Filtered']))
    if not keys:
        raise SystemExit('No messages with matching stamps across the three topics.')
    t = (np.array(keys) - keys[0]) * 1e-9
    arr = {n: np.array([data[n][k] for k in keys]) for n in data}
    labels = ['x [m]', 'y [m]', 'yaw [rad]', 'v [m/s]', 'w [rad/s]']

    lines = ['Samples matched: %d, duration %.1f s' % (len(keys), t[-1]), '']
    for n in ('Noisy', 'Filtered'):
        e = arr[n] - arr['GT']
        e[:, 2] = (e[:, 2] + math.pi) % (2 * math.pi) - math.pi
        lines.append(n)
        for i, lab in enumerate(labels):
            lines.append('  %-10s RMSE=%.4f  mean=%+.4f  std=%.4f' % (
                lab, np.sqrt(np.mean(e[:, i] ** 2)), e[:, i].mean(), e[:, i].std()))
        pos = np.hypot(e[:, 0], e[:, 1])
        lines.append('  position   RMSE=%.4f  max=%.4f' % (np.sqrt(np.mean(pos ** 2)), pos.max()))
        lines.append('')
    rn = np.sqrt(np.mean((arr['Noisy'][:, :2] - arr['GT'][:, :2]) ** 2))
    rf = np.sqrt(np.mean((arr['Filtered'][:, :2] - arr['GT'][:, :2]) ** 2))
    lines.append('Position RMSE reduction by EKF: %.1f %%' % (100.0 * (1 - rf / rn)))
    text = '\n'.join(lines)
    with open(os.path.join(out, 'metrics.txt'), 'w') as f:
        f.write(text + '\n')
    print(text)

    style = {'GT': {'c': 'k', 'lw': 2},
             'Noisy': {'c': 'tab:red', 'lw': 0.6, 'alpha': 0.6},
             'Filtered': {'c': 'tab:blue', 'lw': 1.2}}
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for i in range(3):
        for n in ('Noisy', 'Filtered', 'GT'):
            ax[i].plot(t, arr[n][:, i], label=n, **style[n])
        ax[i].set_ylabel(labels[i])
        ax[i].grid(alpha=0.3)
    ax[0].legend()
    ax[-1].set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'xy_yaw_vs_time.png'), dpi=150)

    fig, ax = plt.subplots(figsize=(8, 8))
    for n in ('Noisy', 'Filtered', 'GT'):
        ax.plot(arr[n][:, 0], arr[n][:, 1], label=n, **style[n])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'trajectory.png'), dpi=150)

    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for n in ('Noisy', 'Filtered'):
        e = arr[n] - arr['GT']
        ax[0].plot(t, np.hypot(e[:, 0], e[:, 1]), label=n, **style[n])
        ax[1].plot(t, np.degrees((e[:, 2] + math.pi) % (2 * math.pi) - math.pi),
                   label=n, **style[n])
    ax[0].set_ylabel('position error [m]')
    ax[1].set_ylabel('yaw error [deg]')
    ax[1].set_xlabel('time [s]')
    for a in ax:
        a.grid(alpha=0.3)
        a.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'error_vs_time.png'), dpi=150)
    print('Figures written to', os.path.abspath(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', default='results')
    ap.add_argument('--storage', default='sqlite3', help='sqlite3 (Humble default) or mcap')
    a = ap.parse_args()
    analyse(read_bag(a.bag, a.storage), a.out)


if __name__ == '__main__':
    main()

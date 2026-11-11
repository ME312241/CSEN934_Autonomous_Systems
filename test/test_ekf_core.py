"""Offline check of the EKF: simulate a car, add the milestone noise, filter.

Runs without ROS:  python3 -m pytest test/test_ekf_core.py -q
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from csen934_project_pkg.ekf_core import AckermannEKF  # noqa: E402, I100
import numpy as np  # noqa: E402

SIG = np.array([0.3, 0.3, math.radians(1.0), 0.3, math.radians(1.0)])


def _simulate(steer_fn, T=40.0, dt=0.016, v_cmd=10.0, accel=3.0, L=2.94, seed=0):
    """Ground truth uses a *different* speed model (constant accel) than the EKF."""
    rng = np.random.default_rng(seed)
    n = int(T / dt)
    gt = np.zeros((n, 5))
    s = np.zeros(5)
    u = []
    for k in range(n):
        delta = steer_fn(k * dt)
        v = min(s[3] + accel * dt, v_cmd)
        w = v * math.tan(delta) / L
        s = np.array([s[0] + s[3] * math.cos(s[2]) * dt,
                      s[1] + s[3] * math.sin(s[2]) * dt,
                      s[2] + s[4] * dt, v, w])
        gt[k] = s
        u.append((v_cmd, delta))
    z = gt + rng.normal(0.0, 1.0, gt.shape) * SIG
    return gt, z, u, dt


def _run(gt, z, u, dt):
    ekf = AckermannEKF()
    R = np.diag(SIG ** 2)
    est = np.zeros_like(gt)
    for k in range(len(z)):
        if not ekf.initialized:
            ekf.initialize(z[k], R)
        else:
            ekf.predict(dt, u[k - 1])
            ekf.update(z[k], R)
        est[k] = ekf.x
    return est


def _rmse(a, b):
    e = a - b
    e[:, 2] = (e[:, 2] + math.pi) % (2 * math.pi) - math.pi
    return np.sqrt(np.mean(e ** 2, axis=0))


def test_straight_line_noise_suppression():
    gt, z, u, dt = _simulate(lambda t: 0.0)
    est = _run(gt, z, u, dt)
    r_noisy, r_filt = _rmse(z, gt), _rmse(est, gt)
    # Filtered x, y, yaw, v must all beat the raw noisy measurement clearly.
    assert np.all(r_filt[:4] < 0.6 * r_noisy[:4]), (r_noisy, r_filt)


def test_turning_noise_suppression():
    gt, z, u, dt = _simulate(lambda t: 0.05 * math.sin(0.2 * t))
    est = _run(gt, z, u, dt)
    r_noisy, r_filt = _rmse(z, gt), _rmse(est, gt)
    assert np.all(r_filt[:3] < 0.7 * r_noisy[:3]), (r_noisy, r_filt)


def test_yaw_wraparound():
    ekf = AckermannEKF()
    R = np.diag(SIG ** 2)
    ekf.initialize([0, 0, math.pi - 0.01, 0, 0], R)
    ekf.update([0, 0, -math.pi + 0.01, 0, 0], R)
    assert abs(abs(ekf.x[2]) - math.pi) < 0.02


if __name__ == '__main__':
    for name, fn in [('straight', lambda t: 0.0), ('turning', lambda t: 0.05 * math.sin(0.2 * t))]:
        gt, z, u, dt = _simulate(fn)
        est = _run(gt, z, u, dt)
        print(name, 'noisy RMSE', np.round(_rmse(z, gt), 4))
        print(name, 'filt  RMSE', np.round(_rmse(est, gt), 4))

"""ROS-independent Extended Kalman Filter for an Ackermann (bicycle) vehicle.

State  s = [x, y, yaw, v, w]   (odom frame position, heading, body speed, yaw rate)
Input  u = [v_cmd, delta]      (from /Car_Ackrm: speed, steering_angle)
Meas.  z = [x, y, yaw, v, w]   (from /Car_Odom_Noisy, H = I)

Process model (Euler, step dt):
    x'   = x + v cos(yaw) dt
    y'   = y + v sin(yaw) dt
    yaw' = yaw + w dt
    v'   = v + a (v_cmd - v),         a = min(dt / tau, 1)   (first-order speed lag)
    w'   = v' tan(delta) / L                                   (kinematic bicycle)

Before any control message arrives the filter falls back to a constant
velocity / constant yaw-rate model (v' = v, w' = w).

Kept separate from the ROS node so it can be unit-tested without ROS.
"""

import math

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AckermannEKF:

    N = 5

    def __init__(self, wheelbase=2.94, tau=1.0,
                 q_diag=(0.02, 0.02, 0.0005, 2.0, 0.05), min_variance=1e-6):
        self.L = float(wheelbase)
        self.tau = float(tau)
        self.q = np.asarray(q_diag, dtype=float)     # continuous-time, per second
        self.min_var = float(min_variance)
        self.x = np.zeros(self.N)
        self.P = np.eye(self.N)
        self.initialized = False

    # ------------------------------------------------------------------ init
    def initialize(self, z, R):
        self.x = np.asarray(z, dtype=float).copy()
        self.x[2] = _wrap(self.x[2])
        self.P = self._sanitize(R).copy()
        self.initialized = True

    # --------------------------------------------------------------- predict
    def predict(self, dt, u=None):
        """u = (v_cmd, delta) or None if no command has been received yet."""
        if dt <= 0.0:
            return
        x, y, th, v, w = self.x
        c, s = math.cos(th), math.sin(th)

        F = np.eye(self.N)
        F[0, 2] = -v * s * dt
        F[0, 3] = c * dt
        F[1, 2] = v * c * dt
        F[1, 3] = s * dt
        F[2, 4] = dt

        if u is None:
            v_new, w_new = v, w
        else:
            v_cmd, delta = u
            a = min(dt / self.tau, 1.0)
            k = math.tan(delta) / self.L
            v_new = v + a * (v_cmd - v)
            w_new = v_new * k
            F[3, 3] = 1.0 - a
            F[4, 3] = (1.0 - a) * k
            F[4, 4] = 0.0

        self.x = np.array([x + v * c * dt, y + v * s * dt, _wrap(th + w * dt), v_new, w_new])
        Q = np.diag(self.q * dt)
        self.P = F @ self.P @ F.T + Q

    # ---------------------------------------------------------------- update
    def update(self, z, R):
        z = np.asarray(z, dtype=float)
        R = self._sanitize(R)
        innov = z - self.x
        innov[2] = _wrap(innov[2])
        S = self.P + R                                  # H = I
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2] = _wrap(self.x[2])
        I_K = np.eye(self.N) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T     # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        return innov, S

    # --------------------------------------------------------------- helpers
    def _sanitize(self, R):
        R = np.array(R, dtype=float)
        d = np.maximum(np.diag(R), self.min_var)
        np.fill_diagonal(R, d)
        return R

"""Constant-velocity Kalman filter in the odometry plane."""
import numpy as np


class PositionKalman:
    def __init__(self, xy, timestamp):
        self.state = np.array([xy[0], xy[1], 0., 0.], dtype=float)
        self.covariance = np.diag([.08, .08, .50, .50])
        self.timestamp = float(timestamp)

    def predict(self, timestamp):
        dt = max(0., min(float(timestamp) - self.timestamp, 2.0))
        if dt == 0:
            return self.state[:2].copy()
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q = np.array([[dt**4/4, 0, dt**3/2, 0],
                      [0, dt**4/4, 0, dt**3/2],
                      [dt**3/2, 0, dt**2, 0],
                      [0, dt**3/2, 0, dt**2]]) * .7
        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + q
        self.timestamp = float(timestamp)
        return self.state[:2].copy()

    def update(self, xy, timestamp):
        self.predict(timestamp)
        h = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])
        r = np.diag([.045, .045])
        residual = np.asarray(xy, dtype=float) - h @ self.state
        gain = self.covariance @ h.T @ np.linalg.inv(h @ self.covariance @ h.T + r)
        self.state += gain @ residual
        self.covariance = (np.eye(4) - gain @ h) @ self.covariance
        return self.state[:2].copy()

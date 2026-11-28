"""Simulated leg odometry: true kinematic body velocity corrupted by drift. SIMULATION ONLY.

The estimator sees only this drifting odometry. The true pose stays with the simulator
and the evaluation topic. Error model (per integration step, while moving):
  v_meas = v * (1 + linear_scale_error + N(0, linear_noise_std))
  w_meas = w * (1 + N(0, angular_noise_std)) + yaw_rate_bias_rad_s
Noise terms are fractional (dimensionless). No drift accumulates while stationary.
"""
import math

import numpy as np


class OdometrySimulator:
    def __init__(self, params, start=(0.0, 0.0, 0.0)):
        self.scale = params['linear_scale_error']
        self.bias = params['yaw_rate_bias_rad_s']
        self.lin_sigma = params['linear_noise_std']
        self.ang_sigma = params['angular_noise_std']
        self.rng = np.random.default_rng(params['seed'])
        self.x, self.y, self.yaw = map(float, start)
        self.vx = self.wz = 0.0
        self.distance = 0.0
        self.var_xy = 0.0
        self.var_yaw = 0.0

    def step(self, vx_true, wz_true, dt):
        if dt <= 0:
            raise ValueError('dt must be positive')
        moving = abs(vx_true) > 1e-6 or abs(wz_true) > 1e-6
        if moving:
            vx = vx_true * (1.0 + self.scale + self.rng.normal(0.0, self.lin_sigma))
            wz = wz_true * (1.0 + self.rng.normal(0.0, self.ang_sigma)) + self.bias
        else:
            vx = wz = 0.0
        mid = self.yaw + 0.5 * wz * dt
        self.x += vx * math.cos(mid) * dt
        self.y += vx * math.sin(mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self.vx, self.wz = vx, wz
        self.distance += abs(vx) * dt
        # Covariance growth for consumers; slam_toolbox does not use it.
        self.var_xy += (abs(vx) * dt * (abs(self.scale) + self.lin_sigma)) ** 2
        self.var_yaw += (abs(wz) * dt * self.ang_sigma + abs(self.bias) * dt * moving) ** 2
        return self.x, self.y, self.yaw

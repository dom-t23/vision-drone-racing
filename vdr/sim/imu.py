"""IMU model: white noise plus per-run biases that random-walk."""

from __future__ import annotations

import math

import numpy as np

from vdr.sim.config import SimConfig


class ImuModel:
    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.dt = cfg.control_dt
        self.gyro_bias = rng.normal(0.0, cfg.gyro_bias_sigma, 3)
        self.accel_bias = rng.normal(0.0, cfg.accel_bias_sigma, 3)
        # Discrete-time standard deviations for one sample at the control rate.
        self.gyro_sd = cfg.gyro_noise / math.sqrt(self.dt)
        self.accel_sd = cfg.accel_noise / math.sqrt(self.dt)
        self.gyro_walk_sd = cfg.gyro_bias_walk * math.sqrt(self.dt)
        self.accel_walk_sd = cfg.accel_bias_walk * math.sqrt(self.dt)

    def sample(self, specific_force: np.ndarray, omega: np.ndarray):
        """Return (accel, gyro) measurements for true body-frame values."""
        self.gyro_bias = self.gyro_bias + self.rng.normal(0.0, self.gyro_walk_sd, 3)
        self.accel_bias = self.accel_bias + self.rng.normal(0.0, self.accel_walk_sd, 3)
        accel = specific_force + self.accel_bias + self.rng.normal(0.0, self.accel_sd, 3)
        gyro = omega + self.gyro_bias + self.rng.normal(0.0, self.gyro_sd, 3)
        return accel, gyro

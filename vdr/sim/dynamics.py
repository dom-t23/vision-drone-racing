"""Quadrotor rigid-body model driven by acro-mode commands.

The vehicle is modelled the way an FPV flight controller presents it: the
pilot (here, the autonomy) commands mass-normalised collective thrust and
body rates, and an inner loop tracks them with first-order lags. Rotor drag
and a small quadratic airframe drag act on the translational dynamics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from vdr.geometry import exp_so3, orthonormalize
from vdr.sim.config import SimConfig

GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass
class QuadState:
    p: np.ndarray                      # world position (m)
    v: np.ndarray                      # world velocity (m/s)
    R: np.ndarray                      # body -> world rotation
    omega: np.ndarray                  # body angular rate (rad/s)
    thrust: float                      # current mass-normalised thrust (m/s^2)
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))  # last world acceleration

    def copy(self) -> "QuadState":
        return QuadState(self.p.copy(), self.v.copy(), self.R.copy(), self.omega.copy(),
                         self.thrust, self.acc.copy())


class QuadDynamics:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.dt = cfg.physics_dt
        self.alpha_thrust = 1.0 - math.exp(-self.dt / cfg.tau_thrust)
        self.alpha_rate = 1.0 - math.exp(-self.dt / cfg.tau_rate)
        self.rate_max = np.array([cfg.rate_max_rp, cfg.rate_max_rp, cfg.rate_max_yaw])
        self.drag_lin = np.array([cfg.drag_xy, cfg.drag_xy, cfg.drag_z])
        self._steps = 0

    def step(self, s: QuadState, thrust_cmd: float, rate_cmd: np.ndarray) -> None:
        """Advance the state in place by one physics step."""
        cfg = self.cfg
        c_cmd = min(max(float(thrust_cmd), 0.0), cfg.thrust_max)
        w_cmd = np.clip(rate_cmd, -self.rate_max, self.rate_max)

        # Inner loops (motor lag, rate controller) as first-order responses.
        s.thrust += (c_cmd - s.thrust) * self.alpha_thrust
        s.omega = s.omega + (w_cmd - s.omega) * self.alpha_rate

        # Translational dynamics.
        R = s.R
        v_body = R.T @ s.v
        drag_body = -self.drag_lin * v_body
        speed = math.sqrt(float(s.v @ s.v))
        acc = R @ (np.array([0.0, 0.0, s.thrust]) + drag_body) + GRAVITY - cfg.drag_quad * speed * s.v

        # Semi-implicit Euler.
        s.v = s.v + acc * self.dt
        s.p = s.p + s.v * self.dt
        s.R = R @ exp_so3(s.omega * self.dt)
        s.acc = acc

        self._steps += 1
        if self._steps % 200 == 0:
            s.R = orthonormalize(s.R)

    @staticmethod
    def specific_force_body(s: QuadState) -> np.ndarray:
        """What an ideal accelerometer measures: (a - g) in the body frame."""
        return s.R.T @ (s.acc - GRAVITY)

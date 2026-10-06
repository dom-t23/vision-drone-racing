"""Error-state Kalman filter (ESKF) for the drone's state relative to the target gate.

Navigation frame: the target gate's frame G (origin at the gate centre,
x = pass direction, z = up). Gates are upright and static, so G is a valid
gravity-aligned inertial frame for IMU mechanisation. When the drone passes a
gate the filter is re-expressed in the next gate's frame (`change_frame`).

Nominal state:  p (3), v (3), R = R_Gb (3x3), accel bias b_a (3), gyro bias b_g (3)
Error state:    dx = [dp, dv, dtheta, db_a, db_g]  (15), with R_true = R @ Exp(dtheta)

Measurements (from one gate detection):
  * gate centre in the camera frame, t_co (3)       -> `update_gate_position`
  * drone attitude relative to the gate, R_Gb (3)   -> `update_attitude`
Each update is chi-squared gated: an innovation whose Mahalanobis distance is
improbable under the predicted covariance is rejected as an outlier.

Reference: J. Sola, "Quaternion kinematics for the error-state Kalman filter", 2017.
"""

from __future__ import annotations

import math

import numpy as np

from vdr.geometry import exp_so3, log_so3, skew

GRAVITY_G = np.array([0.0, 0.0, -9.81])
CHI2_3DOF_99 = 11.345

_P, _V, _TH, _BA, _BG = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15)


class ESKF:
    def __init__(self, accel_noise: float, gyro_noise: float, accel_bias_walk: float, gyro_bias_walk: float):
        self.sa2 = accel_noise ** 2
        self.sg2 = gyro_noise ** 2
        self.sba2 = accel_bias_walk ** 2
        self.sbg2 = gyro_bias_walk ** 2
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.P = np.eye(15)
        self.initialized = False

    # ------------------------------------------------------------------ setup
    def initialize(self, p, v, R, P0: np.ndarray, ba=None, bg=None) -> None:
        self.p = np.array(p, float)
        self.v = np.array(v, float)
        self.R = np.array(R, float)
        if ba is not None:
            self.ba = np.array(ba, float)
        if bg is not None:
            self.bg = np.array(bg, float)
        self.P = np.array(P0, float)
        self.initialized = True

    # ------------------------------------------------------------- prediction
    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float) -> None:
        a_b = accel - self.ba
        w_b = gyro - self.bg
        R = self.R
        a_G = R @ a_b + GRAVITY_G

        self.p = self.p + self.v * dt + 0.5 * a_G * dt * dt
        self.v = self.v + a_G * dt
        dR = exp_so3(w_b * dt)
        self.R = R @ dR

        F = np.eye(15)
        F[_P, _V] = np.eye(3) * dt
        F[_V, _TH] = -R @ skew(a_b) * dt
        F[_V, _BA] = -R * dt
        F[_TH, _TH] = dR.T
        F[_TH, _BG] = -np.eye(3) * dt
        P = F @ self.P @ F.T
        P[_V, _V] += np.eye(3) * (self.sa2 * dt)
        P[_TH, _TH] += np.eye(3) * (self.sg2 * dt)
        P[_BA, _BA] += np.eye(3) * (self.sba2 * dt)
        P[_BG, _BG] += np.eye(3) * (self.sbg2 * dt)
        self.P = 0.5 * (P + P.T)

    # ----------------------------------------------------------------- updates
    def _apply(self, H: np.ndarray, r: np.ndarray, Rn: np.ndarray, gate: bool, thresh: float):
        P = self.P
        PHt = P @ H.T
        S = H @ PHt + Rn
        Sinv = np.linalg.inv(S)
        chi2 = float(r @ Sinv @ r)
        if gate and chi2 > thresh:
            return False, chi2
        K = PHt @ Sinv
        dx = K @ r
        IKH = np.eye(15) - K @ H
        self.P = IKH @ P @ IKH.T + K @ Rn @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.p = self.p + dx[_P]
        self.v = self.v + dx[_V]
        self.R = self.R @ exp_so3(dx[_TH])
        self.ba = self.ba + dx[_BA]
        self.bg = self.bg + dx[_BG]
        return True, chi2

    def predict_gate_in_camera(self, R_bc: np.ndarray, t_bc: np.ndarray) -> np.ndarray:
        """Expected gate-centre position in the camera frame, t_co = R_cb (R^T (0 - p) - t_bc)."""
        return R_bc.T @ (self.R.T @ (-self.p) - t_bc)

    def gate_position_innovation(self, t_co, Rn, R_bc, t_bc):
        """Residual, Jacobian and Mahalanobis distance for a gate-centre measurement (no update)."""
        w = self.R.T @ (-self.p)
        z_hat = R_bc.T @ (w - t_bc)
        r = t_co - z_hat
        H = np.zeros((3, 15))
        H[:, _P] = -R_bc.T @ self.R.T
        H[:, _TH] = R_bc.T @ skew(w)
        S = H @ self.P @ H.T + Rn
        return r, H, float(r @ np.linalg.solve(S, r))

    def update_gate_position(self, t_co, Rn, R_bc, t_bc, gate=True, thresh=CHI2_3DOF_99):
        r, H, _ = self.gate_position_innovation(t_co, Rn, R_bc, t_bc)
        return self._apply(H, r, Rn, gate, thresh)

    def attitude_innovation(self, R_meas, Rn):
        r = log_so3(self.R.T @ R_meas)
        S = self.P[_TH, _TH] + Rn
        return r, float(r @ np.linalg.solve(S, r))

    def update_attitude(self, R_meas, Rn, gate=True, thresh=CHI2_3DOF_99):
        r, _ = self.attitude_innovation(R_meas, Rn)
        H = np.zeros((3, 15))
        H[:, _TH] = np.eye(3)
        return self._apply(H, r, Rn, gate, thresh)

    # --------------------------------------------------------- frame change
    def change_frame(self, yaw: float, t: np.ndarray, pos_cov: np.ndarray, yaw_sigma: float) -> None:
        """Re-express the state in a new gate frame G2 given G2's pose in the current frame G.

        G2 = (rot_z(yaw), t) expressed in G. Uncertainty of that relative pose is
        added to the covariance (position block + yaw about the vertical).
        """
        c, s = math.cos(yaw), math.sin(yaw)
        Rt = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])   # rot_z(yaw)^T
        self.p = Rt @ (self.p - t)
        self.v = Rt @ self.v
        self.R = Rt @ self.R
        J = np.eye(15)
        J[_P, _P] = Rt
        J[_V, _V] = Rt
        P = J @ self.P @ J.T
        ez_b = self.R.T @ np.array([0.0, 0.0, 1.0])      # vertical axis in body frame
        P[_TH, _TH] += (yaw_sigma ** 2) * np.outer(ez_b, ez_b)
        dp = np.array([-self.p[1], self.p[0], 0.0])      # d(p)/d(yaw) of the new frame
        P[_P, _P] += pos_cov + (yaw_sigma ** 2) * np.outer(dp, dp)
        dv = np.array([-self.v[1], self.v[0], 0.0])
        P[_V, _V] += (yaw_sigma ** 2) * np.outer(dv, dv)
        self.P = 0.5 * (P + P.T)

    def position_std(self) -> np.ndarray:
        return np.sqrt(np.diag(self.P)[_P])

    def yaw_std(self) -> float:
        ez_b = self.R.T @ np.array([0.0, 0.0, 1.0])
        return math.sqrt(max(float(ez_b @ self.P[_TH, _TH] @ ez_b), 0.0))

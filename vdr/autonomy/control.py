"""Guidance (where to accelerate) and attitude control (how to get that acceleration).

Guidance works in the target gate's frame G (gate centre at the origin, pass
direction +x). Far from the gate it uses a receding-horizon cubic: the unique
cubic polynomial that starts at the current position/velocity and reaches the
gate centre at time-to-go T with velocity V along +x. Its initial
acceleration has a closed form,

    a = (6 (p_f - p) - 2 T (2 v + v_f)) / T^2,

re-solved every tick (a guidance law in the spirit of proportional navigation,
but with a terminal velocity constraint so the drone arrives square to the gate).
Close to the gate it simply holds the line through the opening.

Control turns the desired acceleration into acro-mode commands: collective
thrust along the desired body z axis, and body rates from a geometric
attitude error (Lee et al. 2010).
"""

from __future__ import annotations

import math

import numpy as np

from vdr.autonomy.config import AutonomyConfig
from vdr.geometry import vee
from vdr.interface import Command

G = 9.81
_EZ = np.array([0.0, 0.0, 1.0])


def gate_guidance(p: np.ndarray, v: np.ndarray, cfg: AutonomyConfig) -> tuple:
    """Desired acceleration (in G) and desired yaw to fly through the gate at the origin."""
    V = cfg.cruise_speed
    vf = np.array([V, 0.0, 0.0])
    if p[0] < cfg.terminal_x:
        dist = float(np.linalg.norm(p))
        # time-to-go if speed changes linearly from the current closing speed to V
        closing = max(float(-(v @ p)) / max(dist, 1e-6), 0.0)
        T = max(2.0 * dist / (closing + V), cfg.t_go_min)
        a = (6.0 * (-p) - 2.0 * T * (2.0 * v + vf)) / (T * T)
    else:
        # Terminal phase: hold the line through the opening at cruise speed.
        a = cfg.k_vel * (vf - v)
        a[1] -= cfg.k_lat * p[1]
        a[2] -= cfg.k_lat * p[2]
    if cfg.lateral_accel_priority:
        a = _prioritise_lateral(a, cfg.max_accel)
    yaw = math.atan2(-p[1], cfg.look_ahead_x - p[0])
    return a, yaw


def _prioritise_lateral(a: np.ndarray, a_max: float) -> np.ndarray:
    """Saturate an acceleration (gate frame) so that missing the opening costs more than
    speed: the lateral part (y, z) gets up to a_max, the along-track x part what is left."""
    n = float(np.linalg.norm(a))
    if n <= a_max:
        return a
    a = np.array(a, float)
    lat = float(math.hypot(a[1], a[2]))
    if lat >= a_max:
        a[1:] *= a_max / lat
        a[0] = 0.0
    else:
        a[0] = math.copysign(math.sqrt(a_max * a_max - lat * lat), a[0])
    return a


def hold_guidance(p: np.ndarray, v: np.ndarray, target: np.ndarray, cfg: AutonomyConfig) -> np.ndarray:
    """Critically-damped move to a point (used while searching for the next gate)."""
    k = cfg.k_vel
    return (k * k / 4.0) * (target - p) - k * v


def attitude_command(a_des: np.ndarray, yaw_des: float, R: np.ndarray, cfg: AutonomyConfig,
                     thrust_max: float, rate_max) -> Command:
    """Acceleration + heading -> collective thrust and body rates."""
    a = np.array(a_des, float)
    n = float(np.linalg.norm(a))
    if n > cfg.max_accel:
        a *= cfg.max_accel / n
    f = a + G * _EZ                          # required specific force (thrust direction)
    f[2] = max(f[2], 0.3 * G)
    # Tilt limit: scale the horizontal part down if it would tilt too far.
    fh = math.hypot(f[0], f[1])
    max_h = f[2] * math.tan(math.radians(cfg.max_tilt_deg))
    if fh > max_h:
        f[0] *= max_h / fh
        f[1] *= max_h / fh
    z_d = f / np.linalg.norm(f)
    x_c = np.array([math.cos(yaw_des), math.sin(yaw_des), 0.0])
    y_d = np.cross(z_d, x_c)
    y_d /= np.linalg.norm(y_d)
    x_d = np.cross(y_d, z_d)
    R_d = np.column_stack([x_d, y_d, z_d])

    e_R = 0.5 * vee(R_d.T @ R - R.T @ R_d)
    k = np.array([cfg.k_att_rp, cfg.k_att_rp, cfg.k_att_yaw])
    rates = np.clip(-k * e_R, -np.asarray(rate_max), np.asarray(rate_max))
    thrust = float(np.clip(f @ (R @ _EZ), 0.0, thrust_max))
    return Command(thrust=thrust, rates=rates)

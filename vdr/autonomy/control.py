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
        if cfg.along_track_tgo and cfg.feasible_lateral:
            a = _cubic_feasible_lateral(p, v, vf, cfg)
        elif cfg.along_track_tgo:
            a = _cubic_split_tgo(p, v, vf, cfg.t_go_min)
        else:
            dist = float(np.linalg.norm(p))
            # time-to-go if speed changes linearly from the current closing speed to V
            closing = max(float(-(v @ p)) / max(dist, 1e-6), 0.0)
            T = max(2.0 * dist / (closing + V), cfg.t_go_min)
            a = _cubic_accel(p, v, vf, T)
    else:
        # Terminal phase: hold the line through the opening at cruise speed.
        a = cfg.k_vel * (vf - v)
        a[1] -= cfg.k_lat * p[1]
        a[2] -= cfg.k_lat * p[2]
    if cfg.lateral_accel_priority:
        a = _prioritise_lateral(a, cfg.max_accel)
    yaw = math.atan2(-p[1], cfg.look_ahead_x - p[0])
    return a, yaw


def _cubic_accel(p: np.ndarray, v: np.ndarray, vf: np.ndarray, T) -> np.ndarray:
    """Initial acceleration of the cubic reaching the origin with velocity vf after T
    (T may be a scalar or a per-axis array)."""
    return (6.0 * (-p) - 2.0 * T * (2.0 * v + vf)) / (T * T)


def _cubic_split_tgo(p: np.ndarray, v: np.ndarray, vf: np.ndarray, t_go_min: float) -> np.ndarray:
    """Cubic guidance with time-to-go taken from the along-track motion alone.

    The arrival time is when x reaches 0, assuming x-speed changes linearly from v_x to V:
    T = 2 (-p_x) / (v_x + V). The along-track axis uses that T unfloored, so at cruise speed
    on the line the along-track demand is zero instead of a large spurious braking term.
    The lateral axes keep the t_go_min floor (their gain grows as 1/T^2)."""
    V = float(vf[0])
    T_x = max(2.0 * max(-float(p[0]), 0.0) / (max(float(v[0]), 0.0) + V), 0.05)
    T_lat = max(T_x, t_go_min)
    return _cubic_accel(p, v, vf, np.array([T_x, T_lat, T_lat]))


_T_GRID = 0.05 * 1.04 ** np.arange(120)     # 0.05 s .. ~5.3 s


def lateral_min_tgo(y: np.ndarray, vy: np.ndarray, a_lat: float, T_min: float) -> float:
    """Shortest T >= T_min for which the cubic taking the lateral state (y, vy) to rest on the
    axis (0, 0) never needs more than a_lat. Its acceleration is linear in time, so the peak is
    at one of the ends: a(0) = (-6 y - 4 T vy) / T^2, a(T) = (6 y + 2 T vy) / T^2."""
    T = np.concatenate(([T_min], _T_GRID[_T_GRID > T_min]))[:, None]
    a0 = np.linalg.norm(-6.0 * y - 4.0 * T * vy, axis=1) / T[:, 0] ** 2
    aT = np.linalg.norm(6.0 * y + 2.0 * T * vy, axis=1) / T[:, 0] ** 2
    ok = np.nonzero(np.maximum(a0, aT) <= a_lat)[0]
    return float(T[ok[0], 0]) if len(ok) else float(T[-1, 0])


def _cubic_feasible_lateral(p: np.ndarray, v: np.ndarray, vf: np.ndarray, cfg: AutonomyConfig) -> np.ndarray:
    """Along-track time-to-go guidance whose lateral part is planned within cfg.feasible_lat_accel.

    If the lateral move cannot be done by the along-track arrival time T_x without exceeding
    the budget, the lateral cubic uses the (longer) feasible T_lat and the along-track axis
    brakes at constant deceleration so that x reaches 0 at T_lat (arrival speed is left free,
    but T_lat is capped so it stays >= cfg.feasible_min_speed and the braking within what
    max_accel leaves after the lateral budget)."""
    a = _cubic_split_tgo(p, v, vf, cfg.t_go_min)
    V = float(vf[0])
    dx = max(-float(p[0]), 0.0)
    vx = max(float(v[0]), 0.0)
    T_x = max(2.0 * dx / (vx + V), 0.05)
    T_lat0 = max(T_x, cfg.t_go_min)
    T_req = lateral_min_tgo(np.asarray(p[1:], float), np.asarray(v[1:], float), cfg.feasible_lat_accel, T_lat0)
    if T_req <= T_lat0 + 1e-9:
        return a
    # Longest T we accept: arrival speed 2 dx / T - vx >= v_min.
    T_cap = 2.0 * dx / (vx + cfg.feasible_min_speed) if vx + cfg.feasible_min_speed > 0 else T_req
    T = max(min(T_req, T_cap), T_lat0)
    if T <= T_lat0 + 1e-9:
        return a
    a_lat = _cubic_accel(p[1:], v[1:], np.zeros(2), T)
    ax = 2.0 * (dx - vx * T) / (T * T)        # constant acceleration reaching x = 0 at T
    ax_budget = math.sqrt(max(cfg.max_accel ** 2 - float(a_lat @ a_lat), 0.0))
    ax = max(min(ax, float(a[0])), -ax_budget)
    return np.array([ax, a_lat[0], a_lat[1]])


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

"""Which square the stack trusts when a gate's outline was RANSAC-fitted (merged gates)."""

import math

import numpy as np

from vdr.autonomy import Autonomy, AutonomyConfig
from vdr.autonomy.detector import OUTER, GateDetection, order_clockwise
from vdr.sim import SimConfig, Simulator


def _setup(cfg):
    info = Simulator(SimConfig(), 0).vehicle_info()
    auto = Autonomy(info, cfg)
    up_c = info.R_bc.T @ np.array([0.0, 0.0, 1.0])     # level drone
    return info, auto, up_c


def _square(info, d, size, scale=1.0):
    """Image corners of an upright gate d metres straight ahead of a level drone."""
    sq = np.array([[0, -1, 1], [0, 1, 1], [0, 1, -1], [0, -1, -1]], float) * [0, size / 2, size / 2]
    P_b = sq + [d, 0.0, 0.0]
    P_c = (info.R_bc.T @ (P_b - info.t_bc).T).T
    uv = (P_c[:, :2] / P_c[:, 2:3]) * [info.K[0, 0], info.K[1, 1]] + [info.K[0, 2], info.K[1, 2]]
    c = uv.mean(axis=0)
    return order_clockwise(c + scale * (uv - c))


def test_overshooting_robust_outline_falls_back_to_opening():
    info, auto, up_c = _setup(AutonomyConfig(robust_outer_use_inner=True))
    d = 10.0
    det = GateDetection(_square(info, d, info.gate_outer_size, scale=1.2), OUTER, 1.0,
                        _square(info, d, info.gate_inner_size), robust=True)
    sols = auto._detection_solutions(det, up_c)
    assert sols and math.isclose(sols[0].size, info.gate_inner_size)
    assert abs(sols[0].distance - d) < 0.3


def test_consistent_or_clean_outline_keeps_outer():
    info, auto, up_c = _setup(AutonomyConfig(robust_outer_use_inner=True))
    d = 8.0
    inner = _square(info, d, info.gate_inner_size)
    agree = GateDetection(_square(info, d, info.gate_outer_size), OUTER, 1.0, inner, robust=True)
    clean = GateDetection(_square(info, d, info.gate_outer_size, scale=1.2), OUTER, 1.0, inner, robust=False)
    for det in (agree, clean):
        sols = auto._detection_solutions(det, up_c)
        assert sols and math.isclose(sols[0].size, info.gate_outer_size)


def test_flag_off_keeps_old_behaviour():
    info, auto, up_c = _setup(AutonomyConfig(robust_outer_use_inner=False))
    det = GateDetection(_square(info, 10.0, info.gate_outer_size, scale=1.2), OUTER, 1.0,
                        _square(info, 10.0, info.gate_inner_size), robust=True)
    assert math.isclose(auto._detection_solutions(det, up_c)[0].size, info.gate_outer_size)

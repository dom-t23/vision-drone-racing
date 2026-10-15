"""Guidance: lateral-priority clipping, split time-to-go (along_track_tgo), feasible lateral planning."""

import dataclasses

import numpy as np

from vdr.autonomy import AutonomyConfig
from vdr.autonomy.control import _prioritise_lateral, gate_guidance


def test_prioritise_lateral_keeps_small_requests():
    a = np.array([3.0, -4.0, 1.0])
    assert np.array_equal(_prioritise_lateral(a, 14.0), a)


def test_prioritise_lateral_shortens_along_track_first():
    a = _prioritise_lateral(np.array([-36.8, -10.0, 2.0]), 14.0)
    assert np.isclose(np.linalg.norm(a), 14.0)
    assert np.allclose(a[1:], [-10.0, 2.0])      # lateral request kept in full
    assert a[0] < 0.0                             # along-track sign kept


def test_prioritise_lateral_when_lateral_alone_saturates():
    a = _prioritise_lateral(np.array([5.0, -30.0, 40.0]), 14.0)
    assert a[0] == 0.0
    assert np.isclose(np.linalg.norm(a), 14.0)
    assert np.isclose(a[2] / a[1], 40.0 / -30.0)


def test_flag_off_leaves_guidance_unchanged():
    p, v = np.array([-1.5, 0.7, 0.0]), np.array([5.4, 0.8, -0.1])
    off = AutonomyConfig()
    on = dataclasses.replace(off, lateral_accel_priority=True)
    a_off, _ = gate_guidance(p, v, off)
    a_on, _ = gate_guidance(p, v, on)
    assert np.linalg.norm(a_off) > off.max_accel          # a saturating request
    assert np.isclose(np.linalg.norm(a_on), off.max_accel)
    # the lateral share of the commanded acceleration grows
    lat = lambda a: np.hypot(a[1], a[2]) / np.linalg.norm(a)
    assert lat(a_on) > lat(a_off)


def test_along_track_tgo_removes_spurious_braking():
    """On the line at cruise speed the split time-to-go asks for no along-track acceleration;
    the default floored T asks for heavy braking close to the gate."""
    off = dataclasses.replace(AutonomyConfig(), along_track_tgo=False)
    on = dataclasses.replace(off, along_track_tgo=True)
    p, v = np.array([-1.5, 0.6, -0.2]), np.array([off.cruise_speed, 0.0, 0.0])
    a_off, _ = gate_guidance(p, v, off)
    a_on, _ = gate_guidance(p, v, on)
    assert a_off[0] < -20.0
    assert abs(a_on[0]) < 1e-9
    assert np.allclose(a_on[1:], a_off[1:])      # lateral part unchanged (both floored at t_go_min)


def test_along_track_tgo_far_matches_cubic():
    """Far out on the axis, with no lateral offset, both time-to-go rules agree."""
    off = dataclasses.replace(AutonomyConfig(), along_track_tgo=False)
    on = dataclasses.replace(off, along_track_tgo=True)
    p, v = np.array([-8.0, 0.0, 0.0]), np.array([6.0, 0.0, 0.0])
    assert np.allclose(gate_guidance(p, v, off)[0], gate_guidance(p, v, on)[0])


def test_lateral_min_tgo_respects_budget():
    from vdr.autonomy.control import lateral_min_tgo
    y, vy = np.array([1.0, 0.0]), np.zeros(2)
    T = lateral_min_tgo(y, vy, 11.0, 0.3)
    assert 6.0 * 1.0 / T ** 2 <= 11.0                 # rest-to-rest peak within budget
    assert T < 1.05 * np.sqrt(6.0 / 11.0)             # and not much longer than needed
    assert lateral_min_tgo(np.zeros(2), np.zeros(2), 11.0, 0.4) == 0.4   # nothing to do


def test_feasible_lateral_brakes_only_when_needed():
    off = AutonomyConfig()
    on = dataclasses.replace(off, feasible_lateral=True)
    v = np.array([off.cruise_speed, 0.0, 0.0])
    # small offset far out: identical to the default law
    p = np.array([-8.0, 0.5, 0.0])
    assert np.allclose(gate_guidance(p, v, off)[0], gate_guidance(p, v, on)[0])
    # 1 m off at 4 m: default asks 24 m/s^2 sideways (saturates); flag brakes and stays in budget
    p = np.array([-4.0, 1.0, 0.0])
    a_off, a_on = gate_guidance(p, v, off)[0], gate_guidance(p, v, on)[0]
    assert abs(a_off[1]) > off.max_accel and a_off[0] == 0.0
    assert a_on[0] < -3.0
    assert np.linalg.norm(a_on) <= off.max_accel + 1e-9
    # arrival speed implied by the braking stays >= feasible_min_speed
    T = 2.0 * 4.0 / (off.cruise_speed + on.feasible_min_speed)
    assert a_on[0] >= 2.0 * (4.0 - off.cruise_speed * T) / T ** 2 - 1e-9

"""Guidance: lateral-priority clipping (lateral_accel_priority) and split time-to-go (along_track_tgo)."""

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
    off = AutonomyConfig()
    on = dataclasses.replace(off, along_track_tgo=True)
    p, v = np.array([-1.5, 0.6, -0.2]), np.array([off.cruise_speed, 0.0, 0.0])
    a_off, _ = gate_guidance(p, v, off)
    a_on, _ = gate_guidance(p, v, on)
    assert a_off[0] < -20.0
    assert abs(a_on[0]) < 1e-9
    assert np.allclose(a_on[1:], a_off[1:])      # lateral part unchanged (both floored at t_go_min)


def test_along_track_tgo_far_matches_cubic():
    """Far out on the axis, with no lateral offset, both time-to-go rules agree."""
    off = AutonomyConfig()
    on = dataclasses.replace(off, along_track_tgo=True)
    p, v = np.array([-8.0, 0.0, 0.0]), np.array([6.0, 0.0, 0.0])
    assert np.allclose(gate_guidance(p, v, off)[0], gate_guidance(p, v, on)[0])

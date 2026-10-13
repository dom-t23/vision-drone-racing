"""Guidance saturation: lateral-priority clipping (flag lateral_accel_priority)."""

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

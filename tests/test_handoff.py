"""Handoff to the next gate: how much the filter trusts the next gate's measured yaw."""

import math

import numpy as np

from vdr.autonomy import Autonomy, AutonomyConfig
from vdr.autonomy.stack import _NextGateEstimate
from vdr.sim import SimConfig, Simulator


def _yaw_std_after_switch(cfg, yaw_meas=math.radians(10.0)):
    info = Simulator(SimConfig(), 0).vehicle_info()
    auto = Autonomy(info, cfg)
    e = auto.eskf
    e.initialize(np.array([0.3, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]), np.eye(3), np.eye(15) * 1e-4)
    auto._prev_gate = np.zeros(3)
    h = _NextGateEstimate()
    for _ in range(60):     # many agreeing sightings: the averaged sigma alone would be tiny
        h.add(np.array([8.0, 1.0, 0.0]), 0.2, yaw_meas, math.radians(5.0))
    auto._switch_to(h, passed=True)
    return math.degrees(e.yaw_std())


def test_flag_off_trusts_averaged_yaw():
    # Old behaviour: sigma ~5/sqrt(60) deg, blended with the 20 deg bearing prior, floored at 5.
    assert _yaw_std_after_switch(AutonomyConfig(handoff_yaw_floor=False)) < 5.5


def test_floor_keeps_yaw_uncertainty_realistic():
    cfg = AutonomyConfig(handoff_yaw_floor=True, handoff_yaw_sigma_floor_deg=12.0)
    s = _yaw_std_after_switch(cfg)
    expect = 1.0 / math.sqrt(1.0 / 12.0 ** 2 + 1.0 / cfg.bearing_yaw_sigma_deg ** 2)   # ~10.3 deg
    assert abs(s - expect) < 0.5


def _yaw_after_switch(cfg, sightings):
    """Filter yaw (deg) right after switching to a gate built from (yaw_deg, ambiguous) sightings."""
    info = Simulator(SimConfig(), 0).vehicle_info()
    auto = Autonomy(info, cfg)
    e = auto.eskf
    e.initialize(np.array([0.3, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]), np.eye(3), np.eye(15) * 1e-4)
    auto._prev_gate = np.zeros(3)
    h = _NextGateEstimate()
    for yaw_deg, amb in sightings:      # gate straight ahead: bearing prior says yaw 0
        h.add(np.array([8.0, 0.0, 0.0]), 0.2, math.radians(yaw_deg), math.radians(5.0), ambiguous=amb)
    auto._switch_to(h, passed=True)
    # the new frame is rotated by +yaw, so the drone's heading in it is -yaw
    return -math.degrees(math.atan2(e.R[1, 0], e.R[0, 0]))


def test_unambiguous_yaw_votes_ignore_two_solution_sightings():
    sightings = [(30.0, True)] * 40 + [(4.0, False)] * 5
    off = _yaw_after_switch(AutonomyConfig(unambiguous_yaw_votes=False), sightings)
    on = _yaw_after_switch(AutonomyConfig(unambiguous_yaw_votes=True), sightings)
    assert off > 15.0                 # the 40 ambiguous 30-deg votes dominate
    assert 0.0 < on < 5.0             # only the 4-deg votes, blended with the 0-deg bearing prior


def test_unambiguous_yaw_votes_fall_back_when_all_ambiguous():
    sightings = [(10.0, True)] * 20
    cfg = AutonomyConfig(unambiguous_yaw_votes=True)
    assert abs(_yaw_after_switch(cfg, sightings) - _yaw_after_switch(AutonomyConfig(unambiguous_yaw_votes=False), sightings)) < 1e-9
    cfg_b = AutonomyConfig(unambiguous_yaw_votes=True, unambiguous_yaw_bearing_only=True)
    assert abs(_yaw_after_switch(cfg_b, sightings)) < 1e-6      # bearing prior alone

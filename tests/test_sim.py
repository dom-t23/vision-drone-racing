import math

import numpy as np

from vdr.interface import Command
from vdr.sim import SimConfig, Simulator
from vdr.sim.course import generate_course
from vdr.sim.dynamics import QuadDynamics, QuadState
from vdr.geometry import yaw_of


def _state():
    return QuadState(p=np.array([0.0, 0.0, 2.0]), v=np.zeros(3), R=np.eye(3), omega=np.zeros(3), thrust=9.81)


def test_hover_is_an_equilibrium():
    dyn = QuadDynamics(SimConfig())
    s = _state()
    for _ in range(400):
        dyn.step(s, 9.81, np.zeros(3))
    assert np.linalg.norm(s.p - [0, 0, 2]) < 1e-9


def test_zero_thrust_falls_at_g():
    cfg = SimConfig(tau_thrust=1e-4)
    dyn = QuadDynamics(cfg)
    s = _state()
    for _ in range(40):                                   # 0.1 s
        dyn.step(s, 0.0, np.zeros(3))
    assert math.isclose(s.v[2], -9.81 * 0.1, rel_tol=0.05)


def test_yaw_rate_command_turns_the_drone():
    dyn = QuadDynamics(SimConfig())
    s = _state()
    for _ in range(400):                                  # 1 s at 1 rad/s (minus the rate lag)
        dyn.step(s, 9.81, np.array([0.0, 0.0, 1.0]))
    assert 0.9 < yaw_of(s.R) < 1.0


def test_imu_reads_gravity_at_hover():
    sim = Simulator(SimConfig(gyro_noise=0.0, accel_noise=0.0, gyro_bias_sigma=0.0,
                              accel_bias_sigma=0.0, gyro_bias_walk=0.0, accel_bias_walk=0.0), seed=0)
    obs = sim.reset()
    for _ in range(5):
        obs = sim.step(Command.hover())
    assert np.allclose(obs.accel, [0, 0, 9.81], atol=1e-6)
    assert np.allclose(obs.gyro, 0, atol=1e-9)


def test_courses_are_deterministic_and_valid():
    cfg = SimConfig()
    for seed in range(30):
        a = generate_course(cfg, np.random.default_rng(seed))
        b = generate_course(cfg, np.random.default_rng(seed))
        assert np.allclose(a.centers(), b.centers())
        C = a.centers()
        # spacing and heights
        legs = np.linalg.norm(np.diff(C, axis=0), axis=1)
        assert np.all(legs > cfg.spacing_min - 1.0)       # (height change adds a little)
        assert np.all((C[:, 2] >= cfg.z_min) & (C[:, 2] <= cfg.z_max))
        # next gate is the nearest gate in the forward cone of each gate
        for i in range(len(C) - 1):
            n = a.gates[i].normal
            ahead = []
            for j in range(len(C)):
                d = C[j] - C[i]
                if j != i and d @ n / np.linalg.norm(d) > math.cos(math.radians(cfg.next_gate_cone_deg)):
                    ahead.append((np.linalg.norm(d), j))
            assert min(ahead)[1] == i + 1


def test_scoring_pass_and_crash():
    cfg = SimConfig()
    sim = Simulator(cfg, seed=0)
    g = sim.course.gates[0]
    # straight through the centre -> pass
    sim._check_events(g.center - 0.1 * g.normal, g.center + 0.1 * g.normal)
    assert sim.next_gate == 1 and not sim.done
    # through the frame of gate 1 -> crash
    g1 = sim.course.gates[1]
    off = g1.R @ np.array([0.0, (cfg.gate_inner + cfg.gate_outer) / 4.0, 0.0])
    sim._check_events(g1.center + off - 0.1 * g1.normal, g1.center + off + 0.1 * g1.normal)
    assert sim.outcome == "crash_gate"


def test_race_clock_scales_with_course_length():
    sim = Simulator(SimConfig(), seed=3)
    assert sim.time_limit < SimConfig().time_limit
    assert math.isclose(sim.time_limit, 2.0 + sim.path_length / 5.0)

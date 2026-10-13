"""Camera-only mode must never read the IMU."""

import numpy as np

from vdr.autonomy import Autonomy, AutonomyConfig
from vdr.interface import Observation
from vdr.sim import SimConfig, Simulator


def test_camera_only_ignores_imu():
    sim = Simulator(SimConfig(), 3)
    cfg = AutonomyConfig(camera_only=True)
    a, b = Autonomy(sim.vehicle_info(), cfg), Autonomy(sim.vehicle_info(), cfg)
    obs = sim.reset()
    for _ in range(800):                       # 2 s: initialises on a gate and starts flying
        ca = a.step(obs)
        junk = Observation(t=obs.t, accel=np.full(3, np.nan), gyro=np.full(3, 1e6), image=obs.image)
        cb = b.step(junk)
        assert ca.thrust == cb.thrust and np.array_equal(ca.rates, cb.rates)
        obs = sim.step(ca)
        if sim.done:
            break
    assert a.eskf.initialized and a.mode == "track"

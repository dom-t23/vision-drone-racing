"""Headless simulator: quadrotor dynamics, IMU, FPV renderer, course and scoring.

The autonomy stack must never import from here (enforced by tests/test_separation.py).
"""

from vdr.sim.config import SimConfig
from vdr.sim.world import Simulator, RunResult

__all__ = ["SimConfig", "Simulator", "RunResult"]

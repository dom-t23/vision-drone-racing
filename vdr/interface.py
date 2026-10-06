"""The data contract between the simulator and the autonomy stack.

This module is the ONLY thing the two sides share (plus pure maths in
vdr.geometry). It is deliberately small:

    sim -> autonomy:  VehicleInfo once at start-up, then an Observation every
                      control tick (IMU sample, plus a camera image when one
                      was captured).
    autonomy -> sim:  a Command every control tick (collective thrust and
                      body rates, like an FPV flight controller in acro mode).

HARD SEPARATION RULE: nothing here may carry ground truth -- no pose, no gate
positions, no course layout, no projected corner coordinates. Gate detection
must come from image pixels. tests/test_separation.py pins the exact field
lists below; changing them means changing that test, which should only ever
happen to ADD legitimate sensor data or vehicle specs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class VehicleInfo:
    """Static calibration and limits -- what a real team would know about its own drone."""

    # Camera intrinsics (pinhole, no distortion) and image size.
    K: np.ndarray                 # 3x3 intrinsic matrix
    image_width: int
    image_height: int
    # Camera extrinsics: camera -> body rotation and camera position in body frame.
    R_bc: np.ndarray
    t_bc: np.ndarray
    # Known gate geometry (all gates are identical, square).
    gate_outer_size: float        # m, outer edge length of the square frame
    gate_inner_size: float        # m, inner opening edge length
    # Vehicle limits.
    thrust_max: float             # maximum mass-normalised collective thrust, m/s^2
    rate_max: tuple               # (roll, pitch, yaw) body-rate command limits, rad/s
    # Timing.
    control_dt: float             # s between Observations / Commands
    gravity: float = 9.81


@dataclass
class Observation:
    """Everything the autonomy receives each control tick."""

    t: float                       # s since start
    accel: np.ndarray              # IMU specific force, body frame, m/s^2 (noisy, biased)
    gyro: np.ndarray               # IMU angular rate, body frame, rad/s (noisy, biased)
    image: Optional[np.ndarray] = None   # HxWx3 uint8 BGR frame, or None if no frame this tick


@dataclass
class Command:
    """Acro-mode command: mass-normalised collective thrust plus body rates."""

    thrust: float                  # m/s^2, along body +z
    rates: np.ndarray              # rad/s, body frame (roll, pitch, yaw rates about x, y, z)

    @staticmethod
    def hover(g: float = 9.81) -> "Command":
        return Command(thrust=g, rates=np.zeros(3))

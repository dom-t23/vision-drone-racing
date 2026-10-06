"""Simulator configuration.

Everything that defines how hard the world is lives here. Making the sim
harder (blur, latency, clutter, ...) means adding a field here with a default
that preserves old behaviour, then re-baselining (see CLAUDE.md).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

from vdr.cfgutil import override


@dataclass
class SimConfig:
    # --- timing -----------------------------------------------------------
    physics_dt: float = 0.0025        # s (400 Hz rigid-body integration)
    control_substeps: int = 2         # physics steps per control tick -> 200 Hz IMU / commands
    camera_every: int = 5             # control ticks per camera frame -> 40 Hz camera
    time_limit: float = 60.0          # s per run (hard cap)
    race_min_speed: float = 5.0       # m/s: race clock = start allowance + path length / this (0 disables)
    race_start_allowance: float = 2.0 # s added to the race clock for take-off

    # --- camera -------------------------------------------------------------
    image_width: int = 320
    image_height: int = 240
    hfov_deg: float = 100.0
    cam_uptilt_deg: float = 20.0      # FPV-style upward camera tilt
    cam_offset_x: float = 0.08        # camera position in body frame (m, forward)
    cam_offset_z: float = 0.02        # (m, up)
    pixel_noise_sigma: float = 3.0    # additive Gaussian image noise (0-255 scale)

    # --- gates & course -------------------------------------------------------
    gate_outer: float = 1.7           # outer edge of square frame (m)
    gate_inner: float = 1.4           # inner opening (m)
    n_gates: int = 8
    spacing_min: float = 6.0          # distance between consecutive gate centres (m)
    spacing_max: float = 11.0
    turn_max_deg: float = 35.0        # heading change after each gate
    gate_yaw_offset_deg: float = 15.0 # gate normal vs. approach bearing
    z_min: float = 1.4                # gate centre height range (m)
    z_max: float = 3.0
    dz_max: float = 0.8               # max height change between consecutive gates
    min_gate_separation: float = 4.0  # min distance between any two gate centres
    path_clearance: float = 2.5       # other gates must be this far from each straight leg
    next_gate_cone_deg: float = 60.0  # next gate must be the nearest gate in this cone after a pass

    # --- start ----------------------------------------------------------------
    start_dist_min: float = 5.0       # hover start, this far before gate 0
    start_dist_max: float = 7.0
    start_lateral: float = 1.0
    start_vertical: float = 0.3
    start_yaw_deg: float = 10.0

    # --- vehicle ----------------------------------------------------------------
    thrust_max: float = 4.0 * 9.81    # mass-normalised (thrust-to-weight 4)
    rate_max_rp: float = 12.0         # rad/s roll/pitch rate limit
    rate_max_yaw: float = 6.0         # rad/s yaw rate limit
    tau_thrust: float = 0.03          # s, motor spin-up lag
    tau_rate: float = 0.04            # s, inner rate-loop response
    drag_xy: float = 0.35             # 1/s, rotor drag (body x/y)
    drag_z: float = 0.10              # 1/s, rotor drag (body z)
    drag_quad: float = 0.015          # 1/m, quadratic airframe drag
    drone_radius: float = 0.15        # m, collision radius

    # --- IMU ----------------------------------------------------------------------
    gyro_noise: float = 0.002         # rad/s/sqrt(Hz), white noise density (incl. vibration)
    accel_noise: float = 0.05         # m/s^2/sqrt(Hz)
    gyro_bias_sigma: float = 0.01     # rad/s, per-run constant bias (1 sigma)
    accel_bias_sigma: float = 0.10    # m/s^2
    gyro_bias_walk: float = 1e-4      # rad/s^2/sqrt(Hz)
    accel_bias_walk: float = 1e-3     # m/s^3/sqrt(Hz)

    # --- run termination ------------------------------------------------------------
    flyaway_dist: float = 30.0        # m from nearest gate centre

    @property
    def control_dt(self) -> float:
        return self.physics_dt * self.control_substeps

    def to_dict(self) -> dict:
        return asdict(self)

    def override(self, **kwargs) -> "SimConfig":
        """Copy with fields overridden (strings are cast to each field's type)."""
        return override(self, **kwargs)

"""Autonomy configuration.

Every tunable of the flight stack lives here, and the harness can override any
field from the command line (`--flag name=value`). New ideas go in behind a
boolean field (an "experiment flag") so they can be A/B tested with the
harness and switched off cleanly if they don't help.

NOTE: the IMU noise values below are the autonomy's own *assumptions* (filter
tuning). They are deliberately not read from the simulator config.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

from vdr.cfgutil import override


@dataclass
class AutonomyConfig:
    # --- detector -------------------------------------------------------------
    hue_lo: int = 0                   # OpenCV hue (0-179) range of the gate paint
    hue_hi: int = 25
    sat_min: int = 120
    val_min: int = 100
    min_area_outer: float = 40.0      # px^2
    min_area_inner: float = 30.0
    approx_eps: float = 0.035         # approxPolyDP tolerance as a fraction of perimeter
    robust_quad_fit: bool = True      # RANSAC edge fit when gates overlap and the outline isn't a 4-gon
    min_edge_support: float = 0.6     # robust fits: fraction of the outline that must lie on the quad
    border_margin: float = 1.5        # px; reject quads touching the image border (truncated)

    # --- PnP / measurement model --------------------------------------------------
    corner_sigma_px: float = 1.5      # assumed corner noise (px)
    depth_factor: float = 0.7         # depth-noise scale for a 4-point square
    pos_sigma_floor: float = 0.03     # m
    att_sigma_floor_deg: float = 1.0
    att_factor: float = 4.0           # attitude-noise scale (single squares give poor rotation)
    max_reproj_px: float = 2.5        # reject PnP fits worse than this (not a square)
    max_gate_tilt_deg: float = 45.0   # gates are upright; reject poses tilted more than this

    # --- ESKF ------------------------------------------------------------------------
    accel_noise: float = 0.08         # m/s^2/sqrt(Hz)   (assumed, includes vibration)
    gyro_noise: float = 0.003         # rad/s/sqrt(Hz)
    accel_bias_walk: float = 0.005    # m/s^3/sqrt(Hz)
    gyro_bias_walk: float = 0.0005    # rad/s^2/sqrt(Hz)
    init_accel_bias_sigma: float = 0.2
    init_min_imu_samples: int = 40    # average this many accel samples (hovering) before initialising level
    init_gyro_bias_sigma: float = 0.03
    chi2_gating: bool = True          # reject measurements whose innovation fails a chi-squared test
    chi2_threshold: float = 11.345    # 3-DoF updates (gate position, attitude): 99% quantile
    max_consecutive_rejects: int = 8  # then re-initialise from the measurement (lock-out recovery)

    # --- gate tracking & handoff ---------------------------------------------------
    assoc_radius: float = 2.0         # m (+ assoc_radius_per_m * range, capped) to call a detection "the target gate"
    assoc_radius_per_m: float = 0.1   #   -- generous on purpose: the chi-squared gate decides if it's used
    assoc_radius_max: float = 4.0
    next_gate_min_x: float = 3.0      # a detection must be this far beyond the target plane to be "next gate"
    pass_x: float = 0.3               # m past the target plane -> switch to the next gate
    next_gate_cluster_radius: float = 1.5
    next_gate_min_sightings: int = 3  # a next-gate hypothesis needs this many sightings to be trusted
    retarget_margin: float = 2.0      # a gate this much nearer than the target, straight ahead, ...
    retarget_min_sightings: int = 4   # ... seen this many times, becomes the target instead
    retarget_min_range: float = 3.0   # only retarget while the current target is still this far away
    next_gate_cone_deg: float = 60.0  # next-gate candidates must lie within this cone of the pass direction
    yaw_prior_from_bearing: bool = True   # assume a new gate roughly faces the line from the previous gate
    bearing_yaw_sigma_deg: float = 20.0   # confidence in that prior
    init_yaw_sigma_deg: float = 12.0      # yaw uncertainty when (re)initialising on a gate
    switch_pos_sigma_per_m: float = 0.08  # floor on next-gate position uncertainty, per metre of range

    # --- guidance & control -------------------------------------------------------
    cruise_speed: float = 8.0         # m/s through the gates (race pace; see CLAUDE.md before changing)
    t_go_min: float = 0.3             # s, floor on time-to-go in the cubic guidance law
    terminal_x: float = -1.2          # m, inside this distance just hold the line through the gate
    look_ahead_x: float = 1.5         # m beyond the gate the camera is pointed at
    max_tilt_deg: float = 55.0
    max_accel: float = 14.0           # m/s^2, commanded acceleration limit
    k_vel: float = 3.0                # 1/s, velocity loop gain (terminal / search)
    k_lat: float = 3.0                # 1/s^2, lateral position gain (terminal)
    k_att_rp: float = 10.0            # 1/s, attitude -> body-rate gain (roll, pitch)
    k_att_yaw: float = 4.0            # 1/s (yaw)
    search_yaw_rate: float = 0.6      # rad/s while searching for the next gate

    def to_dict(self) -> dict:
        return asdict(self)

    def override(self, **kwargs) -> "AutonomyConfig":
        """Copy with fields overridden (strings are cast to each field's type)."""
        return override(self, **kwargs)

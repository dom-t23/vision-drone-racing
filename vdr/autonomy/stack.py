"""The flight stack: camera + IMU in, acro commands out.

    image --> GateDetector --> IPPE_SQUARE PnP --> ESKF updates (chi-squared gated)
    IMU   --> ESKF predict  ----------------------^        |
                                                           v
                     gate tracking / handoff --> guidance --> attitude control --> Command

Modes
-----
WAIT    hovering at the start until the first gate is seen; initialise the filter on it.
TRACK   flying to the target gate. Detections near the target's centre update the filter;
        detections well beyond it build an estimate of the NEXT gate (all gates look
        identical, so "next" = nearest gate ahead of the one we're about to pass).
        Two sanity checks re-anchor the target: a gate seen straight ahead NEARER than the
        target (retarget), and a gate seen straight ahead just BEYOND it while the target
        itself goes unseen (lost-target recovery, e.g. after a depth-biased handoff).
SEARCH  passed a gate without having seen the next one: stop and turn slowly until a
        gate appears ahead.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from vdr.autonomy import pnp
from vdr.autonomy.config import AutonomyConfig
from vdr.autonomy.control import attitude_command, gate_guidance, hold_guidance
from vdr.autonomy.detector import OUTER, GateDetector
from vdr.autonomy.eskf import ESKF
from vdr.geometry import rotation_between, rot_z, wrap_angle, yaw_of
from vdr.interface import Command, Observation, VehicleInfo

WAIT, TRACK, SEARCH = "wait", "track", "search"
_EZ = np.array([0.0, 0.0, 1.0])


class _NextGateEstimate:
    """Running weighted average of where the next gate is, in the current gate frame."""

    def __init__(self):
        self.sw = 0.0
        self.swt = np.zeros(3)
        self.yaw_s = 0.0          # weighted sum of sin(yaw)
        self.yaw_c = 0.0          # weighted sum of cos(yaw)
        self.yaw_w = 0.0
        self.n = 0

    @property
    def mean(self) -> np.ndarray:
        return self.swt / self.sw

    def add(self, c: np.ndarray, sigma: float, yaw: float, yaw_sigma: float) -> None:
        w = 1.0 / (sigma * sigma)
        self.sw += w
        self.swt += w * c
        wy = 1.0 / (yaw_sigma * yaw_sigma)
        self.yaw_s += wy * math.sin(yaw)
        self.yaw_c += wy * math.cos(yaw)
        self.yaw_w += wy
        self.n += 1

    def position_sigma(self) -> float:
        return math.sqrt(1.0 / self.sw)

    def measured_yaw(self):
        """Weighted circular mean of measured yaws and its approximate sigma."""
        yaw = math.atan2(self.yaw_s, self.yaw_c)
        R = math.hypot(self.yaw_s, self.yaw_c) / self.yaw_w     # resultant length, 1 = all agree
        spread = math.sqrt(max(-2.0 * math.log(max(R, 1e-6)), 0.0))
        sigma = max(1.0 / math.sqrt(self.yaw_w), spread / math.sqrt(self.n))
        return yaw, sigma


class Autonomy:
    def __init__(self, info: VehicleInfo, cfg: Optional[AutonomyConfig] = None):
        self.info = info
        self.cfg = cfg or AutonomyConfig()
        c = self.cfg
        self.detector = GateDetector(c, info.image_width, info.image_height)
        self.eskf = ESKF(c.accel_noise, c.gyro_noise, c.accel_bias_walk, c.gyro_bias_walk)
        self.mode = WAIT
        self.t = 0.0
        self._last_t: Optional[float] = None
        self._acc_sum = np.zeros(3)
        self._acc_n = 0
        self._ahead_hyps: list = []       # gates seen beyond the target (next-gate candidates)
        self._closer_hyps: list = []      # gates seen between us and the target (wrong-target check)
        self._beyond_hyps: list = []      # gates seen straight ahead just beyond the target (lost-target check)
        self._target_miss = 0             # consecutive frames with detections but none associated to the target
        self._prev_gate: Optional[np.ndarray] = None   # last passed gate, in the current frame
        self._consec_rejects = 0
        self._search_target = None
        self._search_yaw = 0.0
        self.passes = 0
        self.stats = dict(frames=0, detections=0, pos_updates=0, pos_rejects=0, att_updates=0,
                          att_rejects=0, reinits=0, handoffs=0, switches=0, retargets=0, searches=0,
                          target_frames=0, recoveries=0)

    # ================================================================== API
    def step(self, obs: Observation) -> Command:
        dt = self.info.control_dt
        if self._last_t is not None:
            dt = obs.t - self._last_t
        self._last_t = obs.t
        self.t = obs.t

        if self.eskf.initialized:
            if dt > 0:
                self.eskf.predict(obs.accel, obs.gyro, dt)
        else:
            self._acc_sum += obs.accel
            self._acc_n += 1

        if obs.image is not None:
            self._process_image(obs.image)

        if self.mode == TRACK and self.eskf.p[0] >= self.cfg.pass_x:
            self._handoff()
        return self._command()

    def debug(self) -> dict:
        e = self.eskf
        return dict(mode=self.mode, initialized=e.initialized, p=e.p.copy(), v=e.v.copy(),
                    R=e.R.copy(), passes=self.passes)

    # ========================================================== perception
    def _up_body(self) -> np.ndarray:
        if self.eskf.initialized:
            return self.eskf.R.T @ _EZ
        a = self._acc_sum / max(self._acc_n, 1)
        return a / np.linalg.norm(a)

    def _process_image(self, image: np.ndarray) -> None:
        info, cfg = self.info, self.cfg
        self.stats["frames"] += 1
        dets = self.detector.detect(image)
        self.stats["detections"] += len(dets)
        if not dets:
            return
        up_c = info.R_bc.T @ self._up_body()
        cands = []
        for d in dets:
            sols = self._detection_solutions(d, up_c)
            if sols:
                cands.append((d, sols))
        if not cands:
            return

        if self.mode == WAIT:
            if self._acc_n < cfg.init_min_imu_samples:
                return          # keep hovering until the accelerometer gives a clean gravity vector
            d, sols = max(cands, key=lambda c: c[0].area)
            self._init_on_gate(sols[0], keep_velocity=False)
            self.mode = TRACK
            return

        e = self.eskf
        target, ahead, closer, beyond = None, [], [], []
        cone = math.cos(math.radians(cfg.next_gate_cone_deg))
        fwd = e.R[:, 0].copy()
        fwd[2] = 0.0
        fwd /= max(np.linalg.norm(fwd), 1e-6)
        d_target = float(np.linalg.norm(e.p))
        for d, sols in cands:
            s0 = sols[0]
            c_G = e.p + e.R @ (info.t_bc + info.R_bc @ s0.t_co)     # gate centre in current frame
            radius = min(cfg.assoc_radius + cfg.assoc_radius_per_m * s0.distance, cfg.assoc_radius_max)
            if np.linalg.norm(c_G) <= radius:
                if target is None or d.area > target[0].area:
                    target = (d, sols, c_G)
            elif c_G[0] >= cfg.next_gate_min_x and c_G[0] / np.linalg.norm(c_G) >= cone:
                ahead.append((d, sols, c_G))
            elif self.mode == TRACK and s0.distance < d_target - cfg.retarget_margin:
                r = c_G - e.p
                if float(r @ fwd) >= cone * float(np.linalg.norm(r)):
                    closer.append((d, sols, c_G))
            elif self.mode == TRACK and cfg.recover_lost_target and s0.distance > d_target:
                r = c_G - e.p
                if float(r @ fwd) >= cone * float(np.linalg.norm(r)):
                    beyond.append((d, sols, c_G))

        if self.mode == TRACK:
            if target is not None:
                self.stats["target_frames"] += 1
                self._target_miss = 0
                self._update_target(target[1])
            else:
                self._target_miss += 1
            for a in ahead:
                self._add_sighting(self._ahead_hyps, a)
            for c in closer:
                h = self._add_sighting(self._closer_hyps, c)
                if h.n >= cfg.retarget_min_sightings and d_target > cfg.retarget_min_range:
                    # "Next gate = nearest gate ahead": a gate in front of us that is
                    # nearer than the one we're flying to means we picked the wrong one.
                    self.stats["retargets"] += 1
                    self._switch_to(h, passed=False)
                    return
            for b in beyond:
                h = self._add_sighting(self._beyond_hyps, b)
                if (h.n >= cfg.lost_target_min_sightings and self._target_miss >= cfg.lost_target_frames
                        and d_target > cfg.retarget_min_range):
                    # The target should be in view but never matches a detection, while a gate
                    # straight ahead just beyond it is seen again and again: the target estimate
                    # (usually a next-gate handoff with a depth bias) is wrong -- that gate is it.
                    self.stats["recoveries"] += 1
                    self._switch_to(h, passed=False)
                    return
        elif self.mode == SEARCH and ahead:
            a = min(ahead, key=lambda a: np.linalg.norm(a[2]))
            self._switch_to(self._add_sighting([], a), passed=False)

    def _detection_solutions(self, d, up_c: np.ndarray) -> list:
        """Valid PnP solutions for one detection (best first), choosing which square to trust."""
        info, cfg = self.info, self.cfg
        size = info.gate_outer_size if d.kind == OUTER else info.gate_inner_size
        sols = self._valid_solutions(d.corners, size, up_c)
        if sols and cfg.robust_outer_use_inner and d.kind == OUTER and d.robust and d.inner is not None:
            # A RANSAC outline usually means this gate's frame merged with another gate in
            # the image, and the fitted corners can overshoot (gate ~20% too near at 7-16 m).
            # The outline and the opening are two independent size cues for the same gate:
            # if they disagree on range, trust the opening (a clean contour of its own).
            inner = self._valid_solutions(d.inner, info.gate_inner_size, up_c)
            if inner and abs(sols[0].distance / inner[0].distance - 1.0) > cfg.robust_inner_depth_tol:
                sols = inner
        if not sols and d.inner is not None:
            # the outline fit was bad (often two gates merged) -- try the gate's own opening
            sols = self._valid_solutions(d.inner, info.gate_inner_size, up_c)
        return sols

    def _valid_solutions(self, corners: np.ndarray, size: float, up_c: np.ndarray) -> list:
        cfg = self.cfg
        return [s for s in pnp.solve_gate_pose(corners, self.info.K, size, up_c)
                if s.reproj_err <= cfg.max_reproj_px and pnp.gate_tilt_deg(s, up_c) <= cfg.max_gate_tilt_deg]

    def _yaw_meas(self, s: pnp.GatePose) -> float:
        """Yaw of a detected gate's frame relative to the current navigation frame."""
        R_GG2 = self.eskf.R @ self.info.R_bc @ s.R_cG
        return yaw_of(R_GG2)

    def _att_sigma(self, s: pnp.GatePose) -> float:
        c = self.cfg
        return pnp.attitude_sigma(s, self.info.K, c.corner_sigma_px, c.att_factor,
                                  math.radians(c.att_sigma_floor_deg))

    def _pos_cov(self, s: pnp.GatePose) -> np.ndarray:
        c = self.cfg
        return pnp.gate_position_cov(s, self.info.K, c.corner_sigma_px, c.depth_factor, c.pos_sigma_floor)

    # =============================================================== filter
    def _init_on_gate(self, s: pnp.GatePose, keep_velocity: bool) -> None:
        """(Re)initialise the filter in the frame of the gate seen in PnP solution `s`."""
        info, cfg, e = self.info, self.cfg, self.eskf
        R_meas, _ = pnp.body_pose_in_gate(s, info.R_bc, info.t_bc)
        if e.initialized:
            # keep roll/pitch from the filter, take yaw from the measurement
            R0 = rot_z(wrap_angle(yaw_of(R_meas) - yaw_of(e.R))) @ e.R
        else:
            up_b = self._up_body()
            R0 = rotation_between(R_meas @ up_b, _EZ) @ R_meas    # level from the accelerometer
        p0 = -R0 @ (info.t_bc + info.R_bc @ s.t_co)
        v0 = e.v.copy() if keep_velocity else np.zeros(3)

        yaw_sig = max(math.radians(cfg.init_yaw_sigma_deg), self._att_sigma(s))
        R_Gc = R0 @ info.R_bc
        P0 = np.zeros((15, 15))
        dp = np.array([-p0[1], p0[0], 0.0])
        P0[0:3, 0:3] = R_Gc @ self._pos_cov(s) @ R_Gc.T + yaw_sig ** 2 * np.outer(dp, dp)
        P0[3:6, 3:6] = np.eye(3) * (0.1 ** 2 if not keep_velocity else 0.5 ** 2)
        ez_b = R0.T @ _EZ
        rp = math.radians(2.0) if not e.initialized else math.radians(3.0)
        P0[6:9, 6:9] = rp ** 2 * np.eye(3) + (yaw_sig ** 2 - rp ** 2) * np.outer(ez_b, ez_b)
        if cfg.yaw_correlated_cov:
            # the same yaw error moves the position estimate (along dp) and the attitude (about ez_b)
            P0[0:3, 6:9] = yaw_sig ** 2 * np.outer(dp, ez_b)
            P0[6:9, 0:3] = P0[0:3, 6:9].T
        if e.initialized:
            P0[9:15, 9:15] = e.P[9:15, 9:15]
            e.initialize(p0, v0, R0, P0, e.ba, e.bg)
        else:
            P0[9:12, 9:12] = np.eye(3) * cfg.init_accel_bias_sigma ** 2
            P0[12:15, 12:15] = np.eye(3) * cfg.init_gyro_bias_sigma ** 2
            e.initialize(p0, v0, R0, P0)
        self._consec_rejects = 0

    def _update_target(self, sols: list) -> None:
        info, cfg, e = self.info, self.cfg, self.eskf
        s0 = sols[0]
        ok, _ = e.update_gate_position(s0.t_co, self._pos_cov(s0), info.R_bc, info.t_bc,
                                       gate=cfg.chi2_gating, thresh=cfg.chi2_threshold)
        if ok:
            self.stats["pos_updates"] += 1
            self._consec_rejects = 0
        else:
            self.stats["pos_rejects"] += 1
            self._consec_rejects += 1
            if self._consec_rejects >= cfg.max_consecutive_rejects:
                self.stats["reinits"] += 1
                self._init_on_gate(s0, keep_velocity=True)
                return

        # Attitude: of the (up to two) IPPE solutions, use the one the filter finds most plausible.
        best = None
        for s in sols:
            R_meas, _ = pnp.body_pose_in_gate(s, info.R_bc, info.t_bc)
            Rn = np.eye(3) * self._att_sigma(s) ** 2
            r, m = e.attitude_innovation(R_meas, Rn)
            if best is None or m < best[0]:
                best = (m, R_meas, Rn, r)
        ok, _ = e.update_attitude(best[1], best[2], gate=cfg.chi2_gating, thresh=cfg.chi2_threshold, r=best[3])
        self.stats["att_updates" if ok else "att_rejects"] += 1

    def _add_sighting(self, hyps: list, cand) -> _NextGateEstimate:
        """Add one sighting of a non-target gate to a list of gate hypotheses.

        Sightings are clustered by position; each cluster is one candidate gate.
        Keeping several hypotheses (instead of trusting the latest sighting)
        stops a single bad detection from redirecting the drone.
        """
        d, sols, c_G = cand
        s0 = sols[0]
        cfg = self.cfg
        cov = self._pos_cov(s0)
        sigma = math.sqrt(np.trace(cov) / 3.0) + 0.1 * float(np.linalg.norm(self.eskf.position_std()))
        yaw = self._yaw_meas(s0)
        ysig = max(self._att_sigma(s0), math.radians(5.0))
        radius = cfg.next_gate_cluster_radius + 0.1 * s0.distance
        best, best_d = None, 1e9
        for h in hyps:
            dist = float(np.linalg.norm(c_G - h.mean))
            if dist < best_d:
                best, best_d = h, dist
        if best is None or best_d > radius:
            best = _NextGateEstimate()
            hyps.append(best)
        best.add(c_G, sigma, yaw, ysig)
        return best

    def _choose_next(self) -> Optional[_NextGateEstimate]:
        """The next gate is the nearest well-supported hypothesis beyond the target gate."""
        if not self._ahead_hyps:
            return None
        support = max(h.n for h in self._ahead_hyps)
        need = min(self.cfg.next_gate_min_sightings, support)
        ok = [h for h in self._ahead_hyps if h.n >= need]
        return min(ok, key=lambda h: float(np.linalg.norm(h.mean)))

    def _handoff(self) -> None:
        """We've flown through the target gate: switch to the next one (or search for it)."""
        e = self.eskf
        self.passes += 1
        self._prev_gate = np.zeros(3)          # the gate just passed is the current origin
        nxt = self._choose_next()
        if nxt is None:
            self.mode = SEARCH
            self.stats["searches"] += 1
            self._search_target = np.array([e.p[0] + 1.5, e.p[1], e.p[2]])
            self._search_yaw = yaw_of(e.R)
            return
        self._switch_to(nxt, passed=True)

    def _switch_to(self, hyp: _NextGateEstimate, passed: bool) -> None:
        """Re-express the filter in the frame of the gate described by `hyp`."""
        cfg, e = self.cfg, self.eskf
        t = hyp.mean
        yaw_m, ysig_m = hyp.measured_yaw()
        yaw, ysig = yaw_m, ysig_m
        if cfg.yaw_prior_from_bearing:
            # Prior: the gate roughly faces along the line from the previous gate.
            origin = self._prev_gate if self._prev_gate is not None else e.p
            yaw_b = math.atan2(t[1] - origin[1], t[0] - origin[0])
            sb = math.radians(cfg.bearing_yaw_sigma_deg)
            if abs(wrap_angle(yaw_m - yaw_b)) < 2.0 * sb:
                w_m, w_b = 1.0 / ysig_m ** 2, 1.0 / sb ** 2
                yaw = yaw_b + wrap_angle(yaw_m - yaw_b) * w_m / (w_m + w_b)
                ysig = 1.0 / math.sqrt(w_m + w_b)
            else:
                yaw, ysig = yaw_b, sb
        ysig = max(ysig, math.radians(5.0))
        prev = self._prev_gate
        # Sightings of a distant gate share the same depth bias, so averaging many of
        # them doesn't make the estimate as good as 1/sqrt(n) suggests: floor it by range.
        pos_sigma = max(hyp.position_sigma(), cfg.switch_pos_sigma_per_m * float(np.linalg.norm(t - e.p)))
        e.change_frame(yaw, t, np.eye(3) * pos_sigma ** 2, ysig, correlate_yaw=cfg.yaw_correlated_cov)
        if prev is not None:
            c, s_ = math.cos(yaw), math.sin(yaw)
            d = prev - t
            self._prev_gate = np.array([c * d[0] + s_ * d[1], -s_ * d[0] + c * d[1], d[2]])
        self._ahead_hyps = []
        self._closer_hyps = []
        self._beyond_hyps = []
        self._target_miss = 0
        self._consec_rejects = 0
        self.mode = TRACK
        self.stats["handoffs" if passed else "switches"] += 1

    # ============================================================== control
    def _command(self) -> Command:
        info, cfg, e = self.info, self.cfg, self.eskf
        if self.mode == WAIT or not e.initialized:
            return Command.hover(info.gravity)
        if self.mode == TRACK:
            a, yaw = gate_guidance(e.p, e.v, cfg)
        else:
            a = hold_guidance(e.p, e.v, self._search_target, cfg)
            self._search_yaw = wrap_angle(self._search_yaw + cfg.search_yaw_rate * info.control_dt)
            yaw = self._search_yaw
        return attitude_command(a, yaw, e.R, cfg, info.thrust_max, info.rate_max)


"""The simulator: owns ground truth, steps physics in lockstep with the autonomy,
renders camera frames and scores gate passes.

Ground truth stays in here. The autonomy sees only `VehicleInfo` (once) and
`Observation`s (every control tick) -- see vdr/interface.py.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from vdr.geometry import rot_z
from vdr.interface import Command, Observation, VehicleInfo
from vdr.sim.camera import FpvCamera
from vdr.sim.config import SimConfig
from vdr.sim.course import Course, generate_course
from vdr.sim.dynamics import QuadDynamics, QuadState
from vdr.sim.imu import ImuModel

# Run outcomes
COMPLETED = "completed"
CRASH_GATE = "crash_gate"        # hit a gate frame
CRASH_GROUND = "crash_ground"
MISSED_GATE = "missed_gate"      # flew past the next gate outside its frame
WRONG_GATE = "wrong_gate"        # flew through a gate out of order
FLYAWAY = "flyaway"              # left the course area
TIMEOUT = "timeout"
OUTCOMES = (COMPLETED, CRASH_GATE, CRASH_GROUND, MISSED_GATE, WRONG_GATE, FLYAWAY, TIMEOUT)


@dataclass
class RunResult:
    seed: int
    n_gates: int
    gates_passed: int
    outcome: str
    time: float
    events: list = field(default_factory=list)   # (t, gate_index, kind)

    @property
    def completed(self) -> bool:
        return self.outcome == COMPLETED

    def to_dict(self) -> dict:
        return {
            "seed": self.seed,
            "n_gates": self.n_gates,
            "gates_passed": self.gates_passed,
            "outcome": self.outcome,
            "time": round(self.time, 3),
            "events": [[round(t, 3), g, k] for t, g, k in self.events],
        }


class Simulator:
    def __init__(self, cfg: SimConfig, seed: int, record: bool = False):
        self.cfg = cfg
        self.seed = seed
        ss = np.random.SeedSequence(seed)
        course_rng, imu_rng, cam_rng = (np.random.default_rng(s) for s in ss.spawn(3))
        self.course: Course = generate_course(cfg, course_rng)
        self.dyn = QuadDynamics(cfg)
        self.imu = ImuModel(cfg, imu_rng)
        self.camera = FpvCamera(cfg, cam_rng)
        self.camera.set_gates(self.course.gates)

        gates = self.course.gates
        self._centers = np.array([g.center for g in gates])
        self._normals = np.array([g.normal for g in gates])
        self._R_wg = np.array([g.R for g in gates])
        self._h_in = cfg.gate_inner / 2.0 - cfg.drone_radius
        self._h_out = cfg.gate_outer / 2.0 + cfg.drone_radius

        self.state = QuadState(
            p=self.course.start_position.copy(),
            v=np.zeros(3),
            R=rot_z(self.course.start_yaw),
            omega=np.zeros(3),
            thrust=9.81,
        )
        self.state.acc = np.zeros(3)
        # Race clock: the run times out unless the drone keeps up a minimum average
        # speed along the straight-line gate-to-gate path. Gates passed before the
        # clock runs out still count. This stops "fly slower" being a way to score.
        C = self._centers
        self.path_length = float(np.linalg.norm(C[0] - self.course.start_position)
                                 + np.sum(np.linalg.norm(np.diff(C, axis=0), axis=1)))
        self.time_limit = cfg.time_limit
        if cfg.race_min_speed > 0:
            self.time_limit = min(cfg.time_limit, cfg.race_start_allowance + self.path_length / cfg.race_min_speed)
        self.t = 0.0
        self.tick = 0
        self.next_gate = 0
        self.outcome: Optional[str] = None
        self.events: list = []
        self.record = record
        self.log: list = []          # (t, p, R, v) if record
        # Camera latency: frames are rendered at capture and handed over this many ticks later.
        self._latency_ticks = max(0, int(round(cfg.camera_latency / cfg.control_dt)))
        self._frame_queue: deque = deque()   # (delivery tick, image)

    # ------------------------------------------------------------ interface
    def vehicle_info(self) -> VehicleInfo:
        c = self.cfg
        return VehicleInfo(
            K=self.camera.K.copy(),
            image_width=c.image_width,
            image_height=c.image_height,
            R_bc=self.camera.R_bc.copy(),
            t_bc=self.camera.t_bc.copy(),
            gate_outer_size=c.gate_outer,
            gate_inner_size=c.gate_inner,
            thrust_max=c.thrust_max,
            rate_max=(c.rate_max_rp, c.rate_max_rp, c.rate_max_yaw),
            control_dt=c.control_dt,
        )

    @property
    def done(self) -> bool:
        return self.outcome is not None

    def reset(self) -> Observation:
        return self._observe(force_image=True)

    def step(self, cmd: Command) -> Observation:
        if self.done:
            raise RuntimeError("episode finished")
        f_sum = np.zeros(3)
        w_sum = np.zeros(3)
        n = self.cfg.control_substeps
        for _ in range(n):
            p0 = self.state.p.copy()
            self.dyn.step(self.state, cmd.thrust, np.asarray(cmd.rates, dtype=float))
            self.t += self.cfg.physics_dt
            f_sum += QuadDynamics.specific_force_body(self.state)
            w_sum += self.state.omega
            self._check_events(p0, self.state.p)
            if self.done:
                break
        self.tick += 1
        if self.record:
            self.log.append((self.t, self.state.p.copy(), self.state.R.copy(), self.state.v.copy()))
        if not self.done:
            self._check_termination()
        return self._observe(f_mean=f_sum / n, w_mean=w_sum / n)

    def result(self) -> RunResult:
        return RunResult(
            seed=self.seed,
            n_gates=len(self.course.gates),
            gates_passed=self.next_gate,
            outcome=self.outcome or TIMEOUT,
            time=self.t,
            events=list(self.events),
        )

    # ------------------------------------------------------------- internals
    def _observe(self, f_mean=None, w_mean=None, force_image=False) -> Observation:
        if f_mean is None:
            f_mean = QuadDynamics.specific_force_body(self.state)
            w_mean = self.state.omega
        accel, gyro = self.imu.sample(f_mean, w_mean)
        image = None
        if force_image or self.tick % self.cfg.camera_every == 0:
            image = self.camera.render(self.state.p, self.state.R)
            if self._latency_ticks:
                self._frame_queue.append((self.tick + self._latency_ticks, image))
                image = None
        if self._frame_queue and self._frame_queue[0][0] <= self.tick:
            image = self._frame_queue.popleft()[1]
        return Observation(t=self.t, accel=accel, gyro=gyro, image=image)

    def _finish(self, outcome: str, gate: int = -1) -> None:
        self.outcome = outcome
        if outcome != COMPLETED:
            self.events.append((self.t, gate, outcome))

    def _check_events(self, p0: np.ndarray, p1: np.ndarray) -> None:
        if p1[2] < 0.0:
            self._finish(CRASH_GROUND)
            return
        s0 = np.einsum("ij,ij->i", p0 - self._centers, self._normals)
        s1 = np.einsum("ij,ij->i", p1 - self._centers, self._normals)
        crossed = np.nonzero(np.sign(s0) != np.sign(s1))[0]
        for j in crossed:
            if s0[j] == s1[j]:
                continue
            t = s0[j] / (s0[j] - s1[j])
            x = p0 + t * (p1 - p0)
            local = self._R_wg[j].T @ (x - self._centers[j])
            ly, lz = abs(local[1]), abs(local[2])
            forward = s1[j] > s0[j]
            inside = ly <= self._h_in and lz <= self._h_in
            on_frame = (not inside) and ly <= self._h_out and lz <= self._h_out
            if on_frame:
                self._finish(CRASH_GATE, int(j))
                return
            if not forward:
                continue
            if inside:
                if j == self.next_gate:
                    self.events.append((self.t, int(j), "pass"))
                    self.next_gate += 1
                    if self.next_gate == len(self._centers):
                        self._finish(COMPLETED)
                        return
                else:
                    self._finish(WRONG_GATE, int(j))
                    return
            elif j == self.next_gate and math.hypot(local[1], local[2]) < 3.0:
                self._finish(MISSED_GATE, int(j))
                return

    def _check_termination(self) -> None:
        if self.t >= self.time_limit:
            self._finish(TIMEOUT, self.next_gate)
            return
        d = np.min(np.linalg.norm(self._centers - self.state.p, axis=1))
        if d > self.cfg.flyaway_dist:
            self._finish(FLYAWAY, self.next_gate)

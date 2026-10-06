"""Seeded random race courses made of identical, upright square gates.

Guarantees (checked by rejection sampling, see `_valid`):
  * consecutive gates are 6-11 m apart and the course turns at most ~35 deg per gate,
    so the next gate is inside the camera's field of view right after each pass;
  * from each gate, the next gate is the NEAREST gate in a forward cone, so
    "fly to the nearest gate ahead" is an unambiguous rule even though all
    gates look identical;
  * no other gate sits close to the straight line between consecutive gates
    (clear line of sight, no obstacles in the way);
  * the drone starts hovering a few metres in front of gate 0, facing it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vdr.geometry import rot_z
from vdr.sim.config import SimConfig


@dataclass
class Gate:
    center: np.ndarray      # world position of the gate centre (m)
    yaw: float              # heading of the gate normal = intended pass direction (rad)

    @property
    def R(self) -> np.ndarray:
        """Gate frame -> world. Gate frame: x = pass direction, y = left, z = up."""
        return rot_z(self.yaw)

    @property
    def normal(self) -> np.ndarray:
        return np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])

    def to_local(self, p_world: np.ndarray) -> np.ndarray:
        return self.R.T @ (p_world - self.center)


@dataclass
class Course:
    gates: list
    start_position: np.ndarray
    start_yaw: float

    def centers(self) -> np.ndarray:
        return np.array([g.center for g in self.gates])


def _segment_point_dist(a, b, p) -> float:
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - p))


def _valid(gates, start_p, start_yaw, cfg: SimConfig) -> bool:
    n = len(gates)
    C = np.array([g.center for g in gates])
    # 1. separation
    for i in range(n):
        for j in range(i + 1, n):
            if np.linalg.norm(C[i] - C[j]) < cfg.min_gate_separation:
                return False
    # 2. clear legs
    for i in range(n - 1):
        for j in range(n):
            if j in (i, i + 1):
                continue
            if _segment_point_dist(C[i], C[i + 1], C[j]) < cfg.path_clearance:
                return False
    # 3. next gate is the nearest gate in the forward cone after each pass
    cone = math.radians(cfg.next_gate_cone_deg)
    for i in range(n - 1):
        nrm = gates[i].normal
        best, best_d = None, 1e9
        for j in range(n):
            if j == i:
                continue
            d = C[j] - C[i]
            dist = np.linalg.norm(d)
            ang = math.acos(np.clip(d @ nrm / dist, -1.0, 1.0))
            if ang < cone and dist < best_d:
                best, best_d = j, dist
        if best != i + 1:
            return False
    # 4. from the start, gate 0 is the nearest gate in the forward cone, and the
    #    start point is well clear of every other gate.
    hdg = np.array([math.cos(start_yaw), math.sin(start_yaw), 0.0])
    best, best_d = None, 1e9
    for j in range(n):
        d = C[j] - start_p
        dist = np.linalg.norm(d)
        if j != 0 and dist < cfg.min_gate_separation:
            return False
        ang = math.acos(np.clip(d @ hdg / dist, -1.0, 1.0))
        if ang < cone and dist < best_d:
            best, best_d = j, dist
    return best == 0


def generate_course(cfg: SimConfig, rng: np.random.Generator, max_tries: int = 2000) -> Course:
    turn = math.radians(cfg.turn_max_deg)
    off = math.radians(cfg.gate_yaw_offset_deg)
    for _ in range(max_tries):
        z0 = rng.uniform(cfg.z_min + 0.2, cfg.z_max - 0.4)
        gates = [Gate(np.array([0.0, 0.0, z0]), 0.0)]
        for _i in range(1, cfg.n_gates):
            prev = gates[-1]
            bearing = prev.yaw + rng.uniform(-turn, turn)
            dist = rng.uniform(cfg.spacing_min, cfg.spacing_max)
            z = float(np.clip(prev.center[2] + rng.uniform(-cfg.dz_max, cfg.dz_max), cfg.z_min, cfg.z_max))
            c = prev.center + dist * np.array([math.cos(bearing), math.sin(bearing), 0.0])
            c[2] = z
            gates.append(Gate(c, bearing + rng.uniform(-off, off)))

        g0 = gates[0]
        d0 = rng.uniform(cfg.start_dist_min, cfg.start_dist_max)
        lat = rng.uniform(-cfg.start_lateral, cfg.start_lateral)
        vert = rng.uniform(-cfg.start_vertical, cfg.start_vertical)
        start_p = g0.center + g0.R @ np.array([-d0, lat, vert])
        start_yaw = g0.yaw + math.radians(rng.uniform(-cfg.start_yaw_deg, cfg.start_yaw_deg))
        if _valid(gates, start_p, start_yaw, cfg):
            return Course(gates, start_p, start_yaw)
    raise RuntimeError("could not generate a valid course; constraints too tight")

"""Run one episode: simulator and autonomy in lockstep.

The runner is the only place that sees both sides. It hands the autonomy
nothing but VehicleInfo and Observations; ground truth is used here purely to
score the run and to compute diagnostic statistics (estimation error), never
fed back into the flight stack.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from vdr.autonomy import Autonomy, AutonomyConfig
from vdr.geometry import log_so3
from vdr.sim import SimConfig, Simulator


def run_episode(seed: int, sim_cfg: Optional[SimConfig] = None, auto_cfg: Optional[AutonomyConfig] = None,
                record: bool = False, keep_frames: int = 0) -> dict:
    sim_cfg = sim_cfg or SimConfig()
    auto_cfg = auto_cfg or AutonomyConfig()
    sim = Simulator(sim_cfg, seed, record=record)
    auto = Autonomy(sim.vehicle_info(), auto_cfg)

    pos_err2, yaw_err2, n_err, wrong_gate_frames, track_frames = 0.0, 0.0, 0, 0, 0
    max_pos_err = 0.0
    frames = []
    trace = []
    gates = sim.course.gates
    obs = sim.reset()
    while True:
        cmd = auto.step(obs)
        if obs.image is not None:
            dbg = auto.debug()
            if dbg["initialized"]:
                # Diagnostics only: which real gate is the filter's frame closest to?
                best = None
                for j, g in enumerate(gates):
                    p_true = g.R.T @ (sim.state.p - g.center)
                    err = float(np.linalg.norm(dbg["p"] - p_true))
                    if best is None or err < best[0]:
                        best = (err, j, g)
                err, j, g = best
                R_true = g.R.T @ sim.state.R
                yaw_err = abs(log_so3(dbg["R"].T @ R_true)[2])
                if dbg["mode"] == "track":
                    track_frames += 1
                    if j != sim.next_gate:
                        wrong_gate_frames += 1
                pos_err2 += err * err
                yaw_err2 += yaw_err * yaw_err
                max_pos_err = max(max_pos_err, err)
                n_err += 1
            if record:
                trace.append(dict(t=obs.t, mode=dbg["mode"], p_est=dbg["p"].tolist(),
                                  p_true=sim.state.p.tolist(), next_gate=sim.next_gate))
            if keep_frames and len(frames) < keep_frames:
                frames.append(obs.image)
        obs = sim.step(cmd)
        if sim.done:
            break

    res = sim.result().to_dict()
    res["diag"] = {
        "pos_err_rms": round(math.sqrt(pos_err2 / n_err), 3) if n_err else None,
        "pos_err_max": round(max_pos_err, 3) if n_err else None,
        "yaw_err_rms_deg": round(math.degrees(math.sqrt(yaw_err2 / n_err)), 2) if n_err else None,
        "wrong_gate_frac": round(wrong_gate_frames / track_frames, 3) if track_frames else None,
        **auto.stats,
        "passes_counted_by_autonomy": auto.passes,
    }
    if record:
        res["log"] = sim.log
        res["trace"] = trace
        res["course"] = sim.course
    if keep_frames:
        res["frames"] = frames
    return res

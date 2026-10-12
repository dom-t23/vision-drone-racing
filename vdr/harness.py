"""Deterministic regression harness.

    python -m vdr.harness --runs 50 --seed 0                 # baseline, writes results/<date>_<commit>.json
    python -m vdr.harness --runs 50 --flag chi2_gating=false # A/B an autonomy flag
    python -m vdr.harness --runs 50 --sim pixel_noise_sigma=6 # harder world
    python -m vdr.harness --runs 8 --no-save                 # quick smoke test
    python -m vdr.harness --runs 8 --link udp                # autonomy in its own process over UDP

Run i uses seed (seed + i) for the course, sensor noise and everything else,
so a given (code, config, seed) always produces exactly the same result.
Runs execute in parallel worker processes; results don't depend on --workers.

Headline metric -- gate pass rate:
    (gates passed in order, summed over runs) / (gates on the course, summed over runs)
A run ends at the first crash / miss / timeout, so later gates count as not passed.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import subprocess
import sys
import time
from collections import Counter
from multiprocessing import Pool
from pathlib import Path
from statistics import median

from vdr.autonomy import AutonomyConfig
from vdr.runner import run_episode
from vdr.sim import SimConfig
from vdr.sim.world import OUTCOMES

REPO = Path(__file__).resolve().parent.parent


def _worker_init():
    try:
        import cv2
        cv2.setNumThreads(1)
    except Exception:
        pass


def _run(args):
    seed, sim_kw, auto_kw, link = args
    sim_cfg = SimConfig().override(**sim_kw)
    auto_cfg = AutonomyConfig().override(**auto_kw)
    t0 = time.time()
    r = run_episode(seed, sim_cfg, auto_cfg, link=link)
    r["wall_s"] = round(time.time() - t0, 2)
    return r


def _git(*args) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=REPO, stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


def summarize(runs: list) -> dict:
    total_gates = sum(r["n_gates"] for r in runs)
    passed = sum(r["gates_passed"] for r in runs)
    n_max = max(r["n_gates"] for r in runs)
    per_gate = [round(sum(1 for r in runs if r["gates_passed"] > k) / len(runs), 3) for k in range(n_max)]
    completed = [r for r in runs if r["outcome"] == "completed"]
    outcomes = Counter(r["outcome"] for r in runs)
    errs = [r["diag"]["pos_err_rms"] for r in runs if r["diag"].get("pos_err_rms") is not None]
    return {
        "pass_rate": round(passed / total_gates, 4),
        "completion_rate": round(len(completed) / len(runs), 4),
        "gates_passed": passed,
        "gates_total": total_gates,
        "per_gate_pass_rate": per_gate,
        "outcomes": {k: outcomes.get(k, 0) for k in OUTCOMES},
        "mean_lap_time_completed": round(sum(r["time"] for r in completed) / len(completed), 2) if completed else None,
        "median_pos_err_rms": round(median(errs), 3) if errs else None,
    }


def parse_kv(items) -> dict:
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"expected key=value, got {it!r}")
        k, v = it.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0, help="first seed; run i uses seed + i")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--flag", action="append", default=[], help="autonomy config override key=value")
    ap.add_argument("--sim", action="append", default=[], help="simulator config override key=value")
    ap.add_argument("--out", type=str, default=None, help="results JSON path (default results/<date>_<commit>[_tag].json)")
    ap.add_argument("--tag", type=str, default="", help="suffix for the default results filename")
    ap.add_argument("--link", choices=["inproc", "udp"], default="inproc",
                    help="udp: each run's autonomy is a separate process talking over localhost UDP")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    if a.runs < 1:
        ap.error("--runs must be at least 1")

    auto_kw = parse_kv(a.flag)
    sim_kw = parse_kv(a.sim)
    sim_cfg = SimConfig().override(**sim_kw)          # validate early
    auto_cfg = AutonomyConfig().override(**auto_kw)

    jobs = [(a.seed + i, sim_kw, auto_kw, a.link) for i in range(a.runs)]
    t0 = time.time()
    if a.workers > 1:
        with Pool(a.workers, initializer=_worker_init) as pool:
            runs = pool.map(_run, jobs, chunksize=1)
    else:
        _worker_init()
        runs = [_run(j) for j in jobs]
    wall = time.time() - t0

    summary = summarize(runs)
    commit = _git("rev-parse", "--short", "HEAD")
    # "dirty" = uncommitted changes to the code that produced these numbers (docs don't count)
    dirty = bool(_git("status", "--porcelain", "--", "vdr", "requirements.txt"))
    result = {
        "timestamp": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit": commit,
        "dirty": dirty,
        "runs": a.runs,
        "seed": a.seed,
        "flags": auto_kw,
        "sim_overrides": sim_kw,
        "link": a.link,
        "wall_s": round(wall, 1),
        **summary,
        "autonomy_config": auto_cfg.to_dict(),
        "sim_config": sim_cfg.to_dict(),
        "per_run": runs,
    }

    if not a.quiet:
        print(f"runs={a.runs} seed={a.seed} flags={auto_kw or '-'} sim={sim_kw or '-'} link={a.link}  ({wall:.0f}s)")
        print(f"  gate pass rate   {summary['pass_rate'] * 100:5.1f}%  ({summary['gates_passed']}/{summary['gates_total']})")
        print(f"  completion rate  {summary['completion_rate'] * 100:5.1f}%")
        print(f"  per-gate         {' '.join(f'{x * 100:.0f}' for x in summary['per_gate_pass_rate'])}")
        print(f"  outcomes         {', '.join(f'{k}={v}' for k, v in summary['outcomes'].items() if v)}")
        print(f"  lap time (done)  {summary['mean_lap_time_completed']}  s;  median est. error {summary['median_pos_err_rms']} m")

    if not a.no_save:
        out = Path(a.out) if a.out else None
        if out is None:
            date = _dt.date.today().isoformat()
            name = f"{date}_{commit}{'-dirty' if dirty else ''}{('_' + a.tag) if a.tag else ''}.json"
            out = REPO / "results" / name
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=1))
        if not a.quiet:
            print(f"  saved {out.relative_to(REPO) if out.is_relative_to(REPO) else out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

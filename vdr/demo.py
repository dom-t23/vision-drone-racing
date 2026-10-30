"""Export recorded flights for the browser demo (docs/demo/index.html).

    python -m vdr.demo --benchmark-out docs/demo/benchmark.js   # what docs/demo ships
    python -m vdr.demo --seeds 0 3 --methods default    # pick flights
    vdr-racing demo-data --out my_flights.js            # from the installed app

Each flight is simulated here, in Python, exactly as the benchmark flies it; the browser
page only plays the recording back. Per camera frame it stores the FPV image (JPEG), the
gate quads the detector found in that image (re-running the same detector on the same
pixels), the true position and the filter's estimate. Ground truth is used only to draw
the replay -- the flight itself never sees it (see vdr/runner.py).

The estimate lives in the current target gate's frame (there is no map). To draw it on a
world-frame top view we express it through the gate it is closest to, like the runner's
diagnostics do.
"""

from __future__ import annotations

import argparse
import base64
import json
import os

import cv2
import numpy as np

from vdr.autonomy import AutonomyConfig
from vdr.autonomy.detector import GateDetector
from vdr.runner import run_episode
from vdr.sim import SimConfig

METHODS = {
    "default": ("Camera + IMU", {}),
    "camera_only": ("Camera only (no IMU)", {"camera_only": True}),
}
DEFAULT_SEEDS = [0, 1017, 2034]

# Committed benchmark files shown on the page: (seed set, camera + IMU file, camera-only file).
# Current default (orange clutter world, init rescue on); the 059848c flag-on files fly the same code.
BENCHMARK = [
    (0, "2026-10-28_e65b78d_baseline.json", "2026-10-28_059848c_ab-init-rescue-camonly.json"),
] + [(s, f"2026-10-28_059848c_ab-init-rescue-seed{s}.json", f"2026-10-28_059848c_ab-init-rescue-camonly-seed{s}.json")
     for s in range(1000, 6000, 1000)]


def _r(x, n=3):
    return [round(float(v), n) for v in x]


def export_flight(seed: int, method: str, stride: int = 2, quality: int = 70) -> dict:
    label, flags = METHODS[method]
    sim_cfg = SimConfig()
    auto_cfg = AutonomyConfig().override(**flags)
    res = run_episode(seed, sim_cfg=sim_cfg, auto_cfg=auto_cfg, record=True,
                      keep_frames=100000, frame_stride=stride)
    course = res["course"]
    gates = course.gates
    det = GateDetector(auto_cfg, sim_cfg.image_width, sim_cfg.image_height)
    trace = {round(tr["t"], 6): tr for tr in res["trace"]}

    frames = []
    for t, im in res["frames"]:
        tr = trace.get(round(t, 6))
        if tr is None:
            continue
        p_true = np.array(tr["p_true"])
        est = None
        if tr["mode"] != "wait":
            p_est = np.array(tr["p_est"])
            # express the gate-frame estimate in the world through the gate that fits best
            j = min(range(len(gates)), key=lambda k: np.linalg.norm(p_est - gates[k].to_local(p_true)))
            est = _r(gates[j].R @ p_est + gates[j].center, 2)
        quads = [dict(c=_r(d.corners.ravel(), 1), k=d.kind[0]) for d in det.detect(im)]
        ok, jpg = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, quality])
        frames.append(dict(t=round(t, 3), p=_r(p_true, 2), e=est, m=tr["mode"][0],
                           g=int(tr["next_gate"]), q=quads,
                           img=base64.b64encode(jpg.tobytes()).decode("ascii")))
    return dict(
        seed=seed, method=method, label=label, outcome=res["outcome"],
        gates_passed=res["gates_passed"], n_gates=res["n_gates"], time=res.get("time"),
        gate_outer=sim_cfg.gate_outer, gate_inner=sim_cfg.gate_inner,
        image_size=[sim_cfg.image_width, sim_cfg.image_height],
        gates=[dict(c=_r(g.center, 2), yaw=round(float(g.yaw), 4)) for g in gates],
        start=_r(course.start_position, 2),
        frames=frames,
    )


def benchmark_table(results_dir: str = "results") -> dict:
    """Pass rates from the committed results files in BENCHMARK (nothing is re-run)."""
    rows, tot = [], [0, 0, 0]
    for seed, f_imu, f_cam in BENCHMARK:
        a = json.load(open(os.path.join(results_dir, f_imu)))
        b = json.load(open(os.path.join(results_dir, f_cam)))
        rows.append(dict(set=f"{seed}-{seed + a['runs'] - 1}", imu=f"{100 * a['pass_rate']:.1f}%",
                         cam=f"{100 * b['pass_rate']:.1f}%"))
        tot[0] += a["gates_passed"]; tot[1] += b["gates_passed"]; tot[2] += a["gates_total"]
    rows.append(dict(set="pooled", imu=f"{100 * tot[0] / tot[2]:.1f}%", cam=f"{100 * tot[1] / tot[2]:.1f}%"))
    note = ("Gate pass rate. Single sets of 50 courses move by a few points from noise alone, so compare the "
            "pooled row. Source: " + ", ".join(f"results/{f}" for row in BENCHMARK for f in row[1:]) + " in the repo.")
    return dict(rows=rows, note=note)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, nargs="+", default=DEFAULT_SEEDS)
    ap.add_argument("--methods", nargs="+", choices=sorted(METHODS), default=["default", "camera_only"])
    ap.add_argument("--stride", type=int, default=2, help="keep every Nth camera frame (40 Hz camera)")
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality of the stored frames")
    ap.add_argument("--out", default="docs/demo/flights.js")
    ap.add_argument("--benchmark-out", default="",
                    help="also write the benchmark table from committed results (e.g. docs/demo/benchmark.js)")
    a = ap.parse_args(argv)
    flights = []
    for seed in a.seeds:
        for m in a.methods:
            f = export_flight(seed, m, a.stride, a.quality)
            print(f"seed {seed:5d}  {f['label']:22s} {f['outcome']:12s} {f['gates_passed']}/{f['n_gates']} gates"
                  f"  {len(f['frames'])} frames")
            flights.append(f)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w") as fh:
        fh.write("// generated by `python -m vdr.demo` -- recorded flights for docs/demo/index.html\n")
        fh.write("window.FLIGHTS = ")
        json.dump(flights, fh, separators=(",", ":"))
        fh.write(";\n")
    print(f"wrote {a.out} ({os.path.getsize(a.out) / 1e6:.1f} MB)")
    if a.benchmark_out:
        with open(a.benchmark_out, "w") as fh:
            fh.write("// generated by `python -m vdr.demo` from committed results files\n")
            fh.write("window.BENCHMARK = " + json.dumps(benchmark_table()) + ";\n")
        print(f"wrote {a.benchmark_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

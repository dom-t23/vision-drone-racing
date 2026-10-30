"""Write-up figures, generated only from committed results files.

    python -m vdr.report            # writes docs/img/{history,ablations,clutter,sensitivity,paired_laps*}.png

Every number plotted is read from a `results/*.json` file listed below, so the
figures can be regenerated (and checked) by anyone with the repo. Nothing here
runs the simulator.
"""

from __future__ import annotations

import argparse
import textwrap
import json
import os
from statistics import mean, median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RESULTS = "results"

# Headline baselines (seed 0, default config) in the order they were set.
# (file, short label, simulator generation: 0 = legacy cv2 fill, 1 = area-coverage renderer,
#  2 = area-coverage renderer + 20 orange clutter objects, the default since 2026-10-27)
HISTORY = [
    ("2026-10-07_c25234a_baseline.json", "bootstrap", 0),
    ("2026-10-10_0181f50_baseline.json", "lost-target\nrecovery", 0),
    ("2026-10-11_68513f6_baseline.json", "robust-outline\ncheck", 0),
    ("2026-10-12_a1d6b65_rebaseline-coverage-aa.json", "renderer fix\n(re-baseline)", 1),
    ("2026-10-12_3b6cbbb_baseline.json", "sub-pixel\ncorners", 1),
    ("2026-10-13_0f1e256_baseline.json", "drop far lone\nRANSAC outlines", 1),
    ("2026-10-15_1d4bb7f_baseline.json", "along-track\ntime-to-go", 1),
    ("2026-10-16_3395fb0_baseline.json", "feasible lateral\nguidance", 1),
    ("2026-10-19_9acf025_baseline.json", "upright PnP", 1),
    ("2026-10-22_5143039_baseline.json", "unambiguous\nyaw votes", 1),
    ("2026-10-27_11f0132_rebaseline-clutter.json", "orange clutter\n(re-baseline)", 2),
    ("2026-10-28_e65b78d_baseline.json", "init rescue", 2),
]

# Component A/Bs: (label, [(off file, on file), ...] one pair per seed set).
ABLATIONS = [
    ("chi-squared gating", [("2026-10-07_c25234a_ablation-no-chi2.json", "2026-10-07_c25234a_baseline.json")]),
    ("lost-target recovery", [
        ("2026-10-10_0181f50_ablation-no-recovery.json", "2026-10-10_0181f50_baseline.json"),
        ("2026-10-10_0181f50_ablation-no-recovery-seed1000.json", "2026-10-10_0181f50_baseline-seed1000.json"),
        ("2026-10-10_0181f50_ablation-no-recovery-seed2000.json", "2026-10-10_0181f50_baseline-seed2000.json"),
    ]),
    ("robust-outline check", [
        ("2026-10-11_68513f6_ablation-no-robust-inner.json", "2026-10-11_68513f6_baseline.json"),
        ("2026-10-11_68513f6_ablation-no-robust-inner-seed1000.json", "2026-10-11_68513f6_baseline-seed1000.json"),
        ("2026-10-11_68513f6_ablation-no-robust-inner-seed2000.json", "2026-10-11_68513f6_baseline-seed2000.json"),
        ("2026-10-11_68513f6_ablation-no-robust-inner-seed3000.json", "2026-10-11_68513f6_baseline-seed3000.json"),
    ]),
    ("sub-pixel corners", [
        ("2026-10-12_06ceff9_baseline.json", "2026-10-12_06ceff9_ab-subpix.json"),
        ("2026-10-12_06ceff9_baseline-seed1000.json", "2026-10-12_06ceff9_ab-subpix-seed1000.json"),
        ("2026-10-12_06ceff9_baseline-seed2000.json", "2026-10-12_06ceff9_ab-subpix-seed2000.json"),
        ("2026-10-12_06ceff9_baseline-seed3000.json", "2026-10-12_06ceff9_ab-subpix-seed3000.json"),
    ]),
    ("drop far lone RANSAC outlines", [
        ("2026-10-12_b6a6a31_baseline.json", "2026-10-12_b6a6a31_ab-droplone.json"),
        ("2026-10-12_b6a6a31_baseline-seed1000.json", "2026-10-12_b6a6a31_ab-droplone-seed1000.json"),
        ("2026-10-13_b6a6a31_baseline-seed2000.json", "2026-10-13_b6a6a31_ab-droplone-seed2000.json"),
        ("2026-10-13_b6a6a31_baseline-seed3000.json", "2026-10-13_b6a6a31_ab-droplone-seed3000.json"),
        ("2026-10-13_b6a6a31_baseline-seed4000.json", "2026-10-13_b6a6a31_ab-droplone-seed4000.json"),
    ]),
    # Off: seed 0 re-flown at 131d93c (per-run identical to 0f1e256); other sets from 2d7e3f6
    # with the flag off (code path unchanged by 131d93c).
    ("along-track time-to-go", [
        ("2026-10-14_131d93c_baseline.json", "2026-10-14_131d93c_ab-attgo.json"),
        ("2026-10-14_2d7e3f6_baseline-seed1000.json", "2026-10-14_131d93c_ab-attgo-seed1000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed2000.json", "2026-10-14_131d93c_ab-attgo-seed2000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed3000.json", "2026-10-15_131d93c_ab-attgo-seed3000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed4000.json", "2026-10-15_131d93c_ab-attgo-seed4000.json"),
    ]),
    # Off: the along-track time-to-go "on" files (same code path at d37cf14 with the flag off),
    # plus seed set 5000 flown at d37cf14 with the flag off.
    ("feasible lateral guidance", [
        ("2026-10-15_1d4bb7f_baseline.json", "2026-10-15_d37cf14_ab-feaslat.json"),
        ("2026-10-14_131d93c_ab-attgo-seed1000.json", "2026-10-16_d37cf14_ab-feaslat-seed1000.json"),
        ("2026-10-14_131d93c_ab-attgo-seed2000.json", "2026-10-16_d37cf14_ab-feaslat-seed2000.json"),
        ("2026-10-15_131d93c_ab-attgo-seed3000.json", "2026-10-16_d37cf14_ab-feaslat-seed3000.json"),
        ("2026-10-15_131d93c_ab-attgo-seed4000.json", "2026-10-16_d37cf14_ab-feaslat-seed4000.json"),
        ("2026-10-16_d37cf14_baseline-seed5000.json", "2026-10-16_d37cf14_ab-feaslat-seed5000.json"),
    ]),
    # upright_pnp + upright_keep_tilt. Off: the feasible-lateral default (same code path at da2703a).
    ("upright PnP", [
        ("2026-10-16_3395fb0_baseline.json", "2026-10-19_da2703a_ab-upright-tilt.json"),
    ] + [(f"2026-10-16_d37cf14_ab-feaslat-seed{s}.json", f"2026-10-19_da2703a_ab-upright-tilt-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
    # unambiguous_yaw_votes (fallback to all sightings). Off and on flown at fe855c9 / 6b66b99.
    ("unambiguous next-gate yaw votes", [
        ("2026-10-22_fe855c9_ab-unamb-off.json", "2026-10-22_6b66b99_ab-unamb2-on.json"),
    ] + [(f"2026-10-22_fe855c9_ab-unamb-off-seed{s}.json", f"2026-10-22_6b66b99_ab-unamb2-on-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
    # init_rescue, on the clutter world. Off: the clutter re-baseline (same code path at 059848c,
    # per run identical: 2026-10-28_059848c_ab-init-rescue-off.json).
    ("init rescue (lone far outline, 3-corner fit)", [
        ("2026-10-27_11f0132_rebaseline-clutter.json", "2026-10-28_059848c_ab-init-rescue.json"),
    ] + [(f"2026-10-27_11f0132_rebaseline-clutter-seed{s}.json", f"2026-10-28_059848c_ab-init-rescue-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
]

# Camera-only mode (no IMU) A/B of the same change, shown in the write-up table.
CAMONLY = [
    ("init rescue, camera only", [
        ("2026-10-27_11f0132_rebaseline-clutter-camonly.json", "2026-10-28_059848c_ab-init-rescue-camonly.json"),
    ] + [(f"2026-10-27_11f0132_rebaseline-clutter-camonly-seed{s}.json",
          f"2026-10-28_059848c_ab-init-rescue-camonly-seed{s}.json") for s in range(1000, 6000, 1000)]),
]

# A/Bs that were tried and left off (shown in the write-up, not in the ablation figure).
REJECTED = [
    ("lateral-first saturation", [
        ("2026-10-13_2d7e3f6_baseline.json", "2026-10-13_2d7e3f6_ab-latprio.json"),
        ("2026-10-14_2d7e3f6_baseline-seed1000.json", "2026-10-14_2d7e3f6_ab-latprio-seed1000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed2000.json", "2026-10-14_2d7e3f6_ab-latprio-seed2000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed3000.json", "2026-10-14_2d7e3f6_ab-latprio-seed3000.json"),
        ("2026-10-14_2d7e3f6_baseline-seed4000.json", "2026-10-14_2d7e3f6_ab-latprio-seed4000.json"),
    ]),
    ("handoff yaw sigma floor", [
        ("2026-10-16_3395fb0_baseline.json", "2026-10-17_cbd0150_ab-yawfloor.json"),
        ("2026-10-16_d37cf14_ab-feaslat-seed1000.json", "2026-10-17_cbd0150_ab-yawfloor-seed1000.json"),
        ("2026-10-16_d37cf14_ab-feaslat-seed2000.json", "2026-10-17_cbd0150_ab-yawfloor-seed2000.json"),
        ("2026-10-16_d37cf14_ab-feaslat-seed3000.json", "2026-10-17_cbd0150_ab-yawfloor-seed3000.json"),
        ("2026-10-16_d37cf14_ab-feaslat-seed4000.json", "2026-10-17_cbd0150_ab-yawfloor-seed4000.json"),
        ("2026-10-16_d37cf14_ab-feaslat-seed5000.json", "2026-10-17_cbd0150_ab-yawfloor-seed5000.json"),
    ]),
    # upright_pnp alone (yaw-only attitude update, tilt from the filter): tilt drifted.
    ("upright PnP, yaw-only update", [
        ("2026-10-16_3395fb0_baseline.json", "2026-10-18_bc81c40_ab-upright.json"),
    ] + [(f"2026-10-16_d37cf14_ab-feaslat-seed{s}.json", f"2026-10-19_bc81c40_ab-upright-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
    # joint_squares on top of the upright PnP default.
    ("joint outline + opening fit", [
        ("2026-10-19_9acf025_baseline.json", "2026-10-19_42e68bf_ab-joint.json"),
    ] + [(f"2026-10-19_da2703a_ab-upright-tilt-seed{s}.json", f"2026-10-20_42e68bf_ab-joint-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
    # unambiguous_yaw_votes with the bearing prior alone when no sighting is unambiguous.
    ("yaw votes, bearing-only fallback", [
        ("2026-10-22_fe855c9_ab-unamb-off.json", "2026-10-22_fe855c9_ab-unamb-on.json"),
    ] + [(f"2026-10-22_fe855c9_ab-unamb-off-seed{s}.json", f"2026-10-22_fe855c9_ab-unamb-on-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
    # Gate hypotheses after a filter re-initialisation (off: the unambiguous-votes default).
] + [(f"hypotheses after reinit: {kind}", [
        ("2026-10-22_6b66b99_ab-unamb2-on.json", f"2026-10-24_418da48_ab-reinit-{kind}.json"),
    ] + [(f"2026-10-22_6b66b99_ab-unamb2-on-seed{s}.json",
          f"2026-10-2{3 if s == 5000 else 4}_418da48_ab-reinit-{kind}-seed{s}.json")
         for s in range(1000, 6000, 1000)]) for kind in ("reexpress", "clear")] + [
    # reinit_unambiguous_only (no reinit from a two-solution measurement), clutter-free world.
    ("no reinit from an ambiguous measurement", [
        ("2026-10-22_6b66b99_ab-unamb2-on.json", "2026-10-26_f3542f3_ab-reinit-unamb.json"),
    ] + [(f"2026-10-22_6b66b99_ab-unamb2-on-seed{s}.json", f"2026-10-26_f3542f3_ab-reinit-unamb-seed{s}.json")
         for s in range(1000, 6000, 1000)]),
]

# Paired lap-time comparisons (same seeds, flag off vs on): (label, pairs, output file name).
PAIRED = [
    ("sub-pixel corners", ABLATIONS[3][1], "paired_laps.png"),
    ("along-track time-to-go", ABLATIONS[5][1], "paired_laps_tgo.png"),
    ("feasible lateral guidance", ABLATIONS[6][1], "paired_laps_feaslat.png"),
    ("upright PnP", ABLATIONS[7][1], "paired_laps_upright.png"),
    ("unambiguous yaw votes", ABLATIONS[8][1], "paired_laps_unamb.png"),
]

# Sensor sensitivity sweeps (seed set 0, one SimConfig knob each; the autonomy is not told about
# either). (label, default the sweep was flown on, [(value in ms, file), ...]).
SENSITIVITY = [
    ("camera latency (not compensated)", "upright-PnP default", [
        (0, "2026-10-19_9acf025_baseline.json"),
        (10, "2026-10-21_1077d61_latency-10ms.json"),
        (15, "2026-10-21_1077d61_latency-15ms.json"),
        (20, "2026-10-20_1077d61_latency-20ms.json"),
        (30, "2026-10-21_1077d61_latency-30ms.json"),
        (40, "2026-10-20_1077d61_latency-40ms.json"),
        (50, "2026-10-20_1077d61_latency-50ms.json"),
    ]),
    ("motion blur (exposure time)", "unambiguous-votes default", [
        (0, "2026-10-22_5143039_baseline.json"),
        (5, "2026-10-23_f58b856_exposure-5ms.json"),
        (10, "2026-10-23_f58b856_exposure-10ms.json"),
        (20, "2026-10-23_f58b856_exposure-20ms.json"),
    ]),
]

# Orange clutter sweep (seed set 0, SimConfig.n_distractors; 20 became the default).
CLUTTER = [
    (0, "2026-10-27_c76b992_knob-clutter0.json"),
    (10, "2026-10-27_c76b992_sweep-clutter10.json"),
    (20, "2026-10-27_c76b992_sweep-clutter20.json"),
    (40, "2026-10-27_c76b992_sweep-clutter40.json"),
]

C_OFF, C_ON, C_INK, C_GRID = "#9aa5b1", "#d9480f", "#212529", "#e9ecef"


def load(name: str, root: str = RESULTS) -> dict:
    with open(os.path.join(root, name)) as f:
        return json.load(f)


def pooled_pass(files, root=RESULTS) -> tuple:
    ds = [load(f, root) for f in files]
    return sum(d["gates_passed"] for d in ds), sum(d["gates_total"] for d in ds)


def paired_laps(pairs, root=RESULTS) -> list:
    """Lap-time differences (on - off) for runs completed both ways, matched by seed."""
    diffs = []
    for off_f, on_f in pairs:
        off = {r["seed"]: r for r in load(off_f, root)["per_run"]}
        for r in load(on_f, root)["per_run"]:
            o = off.get(r["seed"])
            if o and o["outcome"] == "completed" and r["outcome"] == "completed":
                diffs.append(r["time"] - o["time"])
    return diffs


def _style(ax):
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color=C_GRID, lw=0.8)
    ax.set_axisbelow(True)


def plot_history(path, root=RESULTS) -> list:
    rows = []
    for f, label, gen in HISTORY:
        d = load(f, root)
        rows.append((label, gen, 100 * d["pass_rate"], 100 * d["completion_rate"], f))
    fig, ax = plt.subplots(figsize=(11, 4))
    xs = range(len(rows))
    ax.plot(xs, [r[2] for r in rows], "-o", color=C_ON, lw=2, label="gate pass rate")
    ax.plot(xs, [r[3] for r in rows], "--s", color=C_OFF, lw=1.5, label="courses completed")
    for x, r in zip(xs, rows):
        ax.annotate(f"{r[2]:.1f}%", (x, r[2]), textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=9, color=C_INK)
    notes = {1: "renderer fixed", 2: "20 orange clutter\nobjects added"}
    for gen, note in notes.items():
        split = next(i for i, r in enumerate(rows) if r[1] == gen) - 0.5
        ax.axvline(split, color=C_INK, lw=0.8, ls=":")
        ax.text(split + 0.05, 77, f"{note}:\nnot comparable\nacross this line", fontsize=8, color=C_INK,
                va="bottom")
    ax.set_xticks(list(xs), [r[0] for r in rows], fontsize=8)
    ax.set_ylim(75, 103)
    ax.set_ylabel("% (50 courses, seed set 0)")
    ax.set_title("Headline benchmark after each kept change", fontsize=11)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return rows


def plot_ablations(path, root=RESULTS) -> list:
    rows = []
    for label, pairs in ABLATIONS:
        off = pooled_pass([p[0] for p in pairs], root)
        on = pooled_pass([p[1] for p in pairs], root)
        per_set = [(100 * load(a, root)["pass_rate"], 100 * load(b, root)["pass_rate"]) for a, b in pairs]
        rows.append((label, len(pairs), 100 * off[0] / off[1], 100 * on[0] / on[1], per_set))
    fig, ax = plt.subplots(figsize=(13, 4.6))
    w = 0.36
    for i, (label, n, off, on, per_set) in enumerate(rows):
        ax.bar(i - w / 2, off, w, color=C_OFF, label="off" if i == 0 else None)
        ax.bar(i + w / 2, on, w, color=C_ON, label="on" if i == 0 else None)
        for a, b in per_set:
            ax.plot([i - w / 2, i + w / 2], [a, b], color=C_INK, lw=0.6, alpha=0.6, marker=".", ms=4)
        ax.text(i - w / 2, 71, f"{off:.1f}", ha="center", fontsize=9, color="white", weight="bold")
        ax.text(i + w / 2, 71, f"{on:.1f}", ha="center", fontsize=9, color="white", weight="bold")
    sim = ["legacy renderer" if pairs[0][0] < "2026-10-12" else
           "clutter-free" if pairs[0][0] < "2026-10-27" else "clutter world" for _, pairs in ABLATIONS]
    ax.set_xticks(range(len(rows)),
                  [f"{textwrap.fill(r[0], 16)}\n{r[1]} set{'s' if r[1] > 1 else ''},\n{sim[i]}"
                   for i, r in enumerate(rows)],
                  fontsize=8)
    ax.set_ylim(70, 101)
    ax.set_ylabel("gate pass rate, % (pooled)")
    ax.set_title("Each component switched off vs on (lines: individual 50-course seed sets)", fontsize=10)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return rows


def plot_paired(path, root=RESULTS, which: int = 0) -> dict:
    label, pairs, _ = PAIRED[which]
    diffs = paired_laps(pairs, root)
    stats = {"n": len(diffs), "mean": mean(diffs), "median": median(diffs),
             "faster": sum(d < 0 for d in diffs)}
    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.hist(diffs, bins=30, color=C_ON, edgecolor="white")
    ax.axvline(0, color=C_INK, lw=0.8)
    ax.axvline(stats["mean"], color=C_INK, lw=1.2, ls="--")
    ax.set_xlabel(f"lap time with {label} minus without (s)")
    ax.set_ylabel("runs")
    ax.set_title(f"Paired lap times: {stats['faster']} of {stats['n']} runs faster, "
                 f"mean {stats['mean']:+.3f} s", fontsize=10)
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return stats


def plot_sensitivity(path, root=RESULTS) -> list:
    rows = []
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.6))
    for (label, base, pts), c, m in zip(SENSITIVITY, (C_ON, C_OFF), ("o", "s")):
        xs, pr, ee = [], [], []
        for ms, f in pts:
            d = load(f, root)
            xs.append(ms); pr.append(100 * d["pass_rate"]); ee.append(d["median_pos_err_rms"])
            rows.append((label, ms, pr[-1], ee[-1], f))
        ax1.plot(xs, pr, "-" + m, color=c, lw=2, label=f"{label}\n({base})")
        ax2.plot(xs, ee, "-" + m, color=c, lw=2)
    ax1.axvline(25, color=C_INK, lw=0.8, ls=":")
    ax1.text(25.5, 60, "frame period\n(40 Hz)", fontsize=8, color=C_INK)
    ax1.set_xlabel("ms"); ax1.set_ylabel("gate pass rate, % (seed set 0)")
    ax1.set_ylim(0, 103)
    ax1.legend(loc="lower left", frameon=False, fontsize=8)
    ax2.set_xlabel("ms"); ax2.set_ylabel("median per-run est. error (m)")
    ax2.set_ylim(0, None)
    for ax in (ax1, ax2):
        _style(ax)
    fig.suptitle("Sensor sensitivity: when a frame was taken matters, how sharp it is does not", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return rows


def plot_clutter(path, root=RESULTS) -> list:
    rows = []
    for n, f in CLUTTER:
        d = load(f, root)
        rows.append((n, 100 * d["pass_rate"], 100 * d["completion_rate"], d["median_pos_err_rms"], f))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.4))
    xs = [r[0] for r in rows]
    ax1.plot(xs, [r[1] for r in rows], "-o", color=C_ON, lw=2, label="gate pass rate")
    ax1.plot(xs, [r[2] for r in rows], "--s", color=C_OFF, lw=1.5, label="courses completed")
    ax1.axvline(20, color=C_INK, lw=0.8, ls=":")
    ax1.text(20.5, 81, "default", fontsize=8, color=C_INK)
    ax1.set_ylim(80, 102); ax1.set_xlabel("orange clutter objects per course")
    ax1.set_ylabel("% (seed set 0)")
    ax1.legend(loc="lower left", frameon=False, fontsize=8)
    ax2.plot(xs, [r[3] for r in rows], "-o", color=C_ON, lw=2)
    ax2.set_ylim(0, None); ax2.set_xlabel("orange clutter objects per course")
    ax2.set_ylabel("median per-run est. error (m)")
    for ax in (ax1, ax2):
        _style(ax)
    fig.suptitle("Gate-coloured clutter beside the course (before init rescue)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--results", default=RESULTS)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    for r in plot_history(os.path.join(a.out, "history.png"), a.results):
        print(f"history   {r[0]!r:28} pass {r[2]:.1f}%  completion {r[3]:.0f}%  {r[4]}")
    for r in plot_ablations(os.path.join(a.out, "ablations.png"), a.results):
        print(f"ablation  {r[0]:22} {r[1]} set(s)  off {r[2]:.1f}%  on {r[3]:.1f}%")
    for label, pairs in REJECTED:
        off = pooled_pass([p[0] for p in pairs], a.results)
        on = pooled_pass([p[1] for p in pairs], a.results)
        print(f"rejected  {label:22} {len(pairs)} set(s)  off {100 * off[0] / off[1]:.2f}%  "
              f"on {100 * on[0] / on[1]:.2f}%")
    for label, pairs in CAMONLY:
        off = pooled_pass([p[0] for p in pairs], a.results)
        on = pooled_pass([p[1] for p in pairs], a.results)
        print(f"camonly   {label:22} {len(pairs)} set(s)  off {100 * off[0] / off[1]:.2f}%  "
              f"on {100 * on[0] / on[1]:.2f}%")
    for r in plot_clutter(os.path.join(a.out, "clutter.png"), a.results):
        print(f"clutter   {r[0]:3d} objects  pass {r[1]:.2f}%  completed {r[2]:.0f}%  est. error {r[3]:.3f} m  {r[4]}")
    for r in plot_sensitivity(os.path.join(a.out, "sensitivity.png"), a.results):
        print(f"sensitivity {r[0]:34} {r[1]:3d} ms  pass {r[2]:.2f}%  est. error {r[3]:.3f} m  {r[4]}")
    for i, (label, _, fname) in enumerate(PAIRED):
        s = plot_paired(os.path.join(a.out, fname), a.results, i)
        print(f"paired    {label:22} n={s['n']} mean {s['mean']:+.3f} s  median {s['median']:+.3f} s  "
              f"faster {s['faster']}")


if __name__ == "__main__":
    main()

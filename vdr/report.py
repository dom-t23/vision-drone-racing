"""Write-up figures, generated only from committed results files.

    python -m vdr.report            # writes docs/img/{history,ablations,paired_laps}.png

Every number plotted is read from a `results/*.json` file listed below, so the
figures can be regenerated (and checked) by anyone with the repo. Nothing here
runs the simulator.
"""

from __future__ import annotations

import argparse
import json
import os
from statistics import mean, median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RESULTS = "results"

# Headline baselines (seed 0, default config) in the order they were set.
# (file, short label, simulator generation: 0 = legacy cv2 fill, 1 = area-coverage renderer)
HISTORY = [
    ("2026-10-07_c25234a_baseline.json", "bootstrap", 0),
    ("2026-10-10_0181f50_baseline.json", "lost-target\nrecovery", 0),
    ("2026-10-11_68513f6_baseline.json", "robust-outline\ncheck", 0),
    ("2026-10-12_a1d6b65_rebaseline-coverage-aa.json", "renderer fix\n(re-baseline)", 1),
    ("2026-10-12_3b6cbbb_baseline.json", "sub-pixel\ncorners", 1),
    ("2026-10-13_0f1e256_baseline.json", "drop far lone\nRANSAC outlines", 1),
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
]

# Paired lap-time comparison (same seeds, flag off vs on).
PAIRED = ("sub-pixel corners", ABLATIONS[3][1])

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
    fig, ax = plt.subplots(figsize=(8, 4))
    xs = range(len(rows))
    ax.plot(xs, [r[2] for r in rows], "-o", color=C_ON, lw=2, label="gate pass rate")
    ax.plot(xs, [r[3] for r in rows], "--s", color=C_OFF, lw=1.5, label="courses completed")
    for x, r in zip(xs, rows):
        ax.annotate(f"{r[2]:.1f}%", (x, r[2]), textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=9, color=C_INK)
    split = next(i for i, r in enumerate(rows) if r[1] == 1) - 0.5
    ax.axvline(split, color=C_INK, lw=0.8, ls=":")
    ax.text(split + 0.05, 77, "simulator renderer fixed:\nnot comparable across this line",
            fontsize=8, color=C_INK, va="bottom")
    ax.set_xticks(list(xs), [r[0] for r in rows], fontsize=8)
    ax.set_ylim(75, 101)
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
    fig, ax = plt.subplots(figsize=(8, 4))
    w = 0.36
    for i, (label, n, off, on, per_set) in enumerate(rows):
        ax.bar(i - w / 2, off, w, color=C_OFF, label="off" if i == 0 else None)
        ax.bar(i + w / 2, on, w, color=C_ON, label="on" if i == 0 else None)
        for a, b in per_set:
            ax.plot([i - w / 2, i + w / 2], [a, b], color=C_INK, lw=0.6, alpha=0.6, marker=".", ms=4)
        ax.text(i - w / 2, 71, f"{off:.1f}", ha="center", fontsize=9, color="white", weight="bold")
        ax.text(i + w / 2, 71, f"{on:.1f}", ha="center", fontsize=9, color="white", weight="bold")
    sim = ["legacy renderer" if pairs[0][0] < "2026-10-12" else "current renderer" for _, pairs in ABLATIONS]
    ax.set_xticks(range(len(rows)),
                  [f"{r[0]}\n{r[1]} seed set{'s' if r[1] > 1 else ''}, {sim[i]}" for i, r in enumerate(rows)],
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


def plot_paired(path, root=RESULTS) -> dict:
    label, pairs = PAIRED
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
    s = plot_paired(os.path.join(a.out, "paired_laps.png"), a.results)
    print(f"paired    n={s['n']} mean {s['mean']:+.3f} s  median {s['median']:+.3f} s  faster {s['faster']}")


if __name__ == "__main__":
    main()

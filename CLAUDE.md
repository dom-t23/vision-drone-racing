# CLAUDE.md — working rules for this repo

Vision-based autonomous drone racing in a self-contained Python simulator. A racing
quadrotor flies a course of identical square gates using only its monocular FPV camera
and IMU. This file is read at the start of every overnight run. The scheduled task's
prompt is the top-level authority; nothing here may weaken it.

## Non-negotiables (summary — the task prompt has the full text)

- **Separation rule.** The autonomy (`vdr/autonomy/`) receives only what is in
  `vdr/interface.py`: camera images, IMU samples, camera intrinsics/extrinsics, gate size,
  vehicle limits. Never pass ground truth, gate positions, course layout or projected
  corners. `tests/test_separation.py` enforces this — never weaken it.
- **Standalone project.** No references to any competition, its simulator, rules or branding.
- **Honest results.** Every number in docs comes from a committed `results/*.json`.
- **Design decisions fixed by Dom:** thrust + body-rate (acro) commands; all gates identical
  with no IDs (the course guarantees the next gate is visible after each pass); first-week
  extras: README flight GIF, optional UDP link mode, GitHub Actions CI.
- Git: work on `main`, small commits, never force-push, never delete Dom's work.

## Architecture

```
vdr/interface.py      VehicleInfo / Observation / Command  — the ONLY sim<->autonomy contract
vdr/geometry.py       SO(3) helpers + camera mount (pure maths, shared)
vdr/sim/              ground truth lives here
  config.py           SimConfig — every "how hard is the world" knob
  dynamics.py         rigid body; acro inner loops as first-order lags; rotor + quadratic drag
  imu.py              white noise + random-walk biases
  camera.py           pinhole renderer: sky/ground, orange gate frames (outer+inner contour fill), noise
  course.py           seeded courses with visibility / nearest-next-gate guarantees
  world.py            Simulator: lockstep stepping, gate pass/crash scoring, race clock
vdr/autonomy/
  config.py           AutonomyConfig — every tunable + experiment flags
  detector.py         HSV threshold -> contours (outer + holes) -> quad fit (approxPolyDP, RANSAC fallback)
  pnp.py              IPPE_SQUARE, gravity-resolved corner labelling, gate frame maths, noise models
  eskf.py             15-state error-state KF in the target gate's frame; chi2-gated updates; frame change
  control.py          cubic terminal-velocity guidance + geometric attitude -> thrust/body rates
  stack.py            Autonomy: modes WAIT/TRACK/SEARCH, association, next-gate hypotheses, handoff, retarget
vdr/runner.py         one episode in lockstep (+ diagnostics from truth, never fed back)
vdr/harness.py        seeded multi-process regression harness -> results/*.json
tests/                pytest suite (~10 s)
```

Frames: world/gate frames gravity-aligned z-up; gate frame G has x = pass direction, y = left.
Body FLU. Camera OpenCV (x right, y down, z forward), pitched up 20 deg. `R_ab` maps b -> a.

Key ideas: the filter lives in the *current target gate's* frame (no map needed); a pass is
declared when the estimate crosses x = +0.3 m, then the state is re-expressed in the next
gate's frame, whose pose was estimated from sightings made while approaching. The next gate
is "the nearest gate ahead"; if a nearer gate shows up in front of us after a handoff we
retarget to it.

## How to run

```
pip install --break-system-packages -r requirements.txt
python -m pytest -q                                   # ~10 s, must pass before every push
python -m vdr.harness --runs 50 --seed 0              # benchmark, ~65 s on 2 cores, writes results/
python -m vdr.harness --runs 50 --flag NAME=VALUE     # A/B an AutonomyConfig field
python -m vdr.harness --runs 50 --sim NAME=VALUE      # change the world (SimConfig field)
python -m vdr.harness --runs 8 --no-save              # quick smoke test
python -m vdr.harness --runs 50 --seed 1000           # second seed set to confirm a small gain
```

`from vdr.runner import run_episode; run_episode(seed, record=True)` returns the trajectory,
estimate trace and course for one seed — use it to debug a specific failure.

## Benchmark rules

- **Headline metric:** gate pass rate on `--runs 50 --seed 0` with default configs
  (gates passed in order / gates on course; a run ends at the first crash, miss or timeout).
  Also report completion rate, outcomes, and mean lap time of completed runs.
- **Race clock:** each run must average >= `race_min_speed` (5 m/s) along the gate-to-gate
  path or it times out. Slowing down is not a way to score (4 m/s gives 57% — see results).
- **Pace rule:** do not lower `cruise_speed` (or otherwise slow the drone) to gain pass rate.
  A change only counts as an improvement if pass rate rises without mean lap time getting
  more than 5% worse. Going faster at equal pass rate is an improvement.
- **Noise:** with 50 runs, differences under ~8 percentage points are noise. Confirm smaller
  gains on a second seed set (`--seed 1000`) before keeping them.
- **A/B procedure:** add the idea behind an `AutonomyConfig` bool (default False), run the
  harness with it on and off on the same seeds, keep it only if it clearly helps (then flip
  the default to True, keep the flag for ablation), otherwise revert/leave off and log why.
- **Making the sim harder:** add a `SimConfig` field whose default keeps old behaviour, then
  flip the default in a separate commit, re-baseline with `--tag rebaseline-<what>`, and say
  in NIGHTLY_LOG that numbers before/after aren't comparable.
- Results files: `results/<date>_<commit>[-dirty]_<tag>.json`. Commit code first so the
  benchmark file records a clean commit hash; commit every results file you cite.

## Each run

1. Read this file, NIGHTLY_LOG.md (latest entries), ROADMAP.md, the latest baseline results.
2. Run tests and the 50-run harness on current main to confirm the baseline.
3. Do ONE chunk (A pass rate / B robustness & realism / C write-up), verify, commit.
4. Append to NIGHTLY_LOG.md; update ROADMAP.md; lessons learned below if durable.

## Lessons learned

- 2026-10-07 — Initialising roll/pitch from a single accelerometer sample tilted the filter
  by up to ~10 deg (IMU noise is ~0.7 m/s^2 per sample), which showed up as a 1 m height error
  at 6 m and a slow yaw divergence (seed 7 crashed). Averaging 40 samples while hovering
  fixed it (`init_min_imu_samples`).
- 2026-10-07 — Identical gates overlap in the image all the time (the next gate is seen
  through or touching the current one), so 4-vertex `approxPolyDP` alone fails at the start
  of most runs. RANSAC edge fitting + using hole contours as INNER detections fixed it; reject
  RANSAC fits whose edges don't cover >= 60% of the outline (merged distant gates).
- 2026-10-07 — Single-square PnP gives good gate *position* (cm at 6 m) but poor *rotation*
  (5-30 deg, plus IPPE flips at range). Hence the filter uses the gate centre in the camera
  frame as its main measurement and treats attitude as a separate, weaker, gated update.
- 2026-10-07 — Next-gate estimates from far sightings share a depth bias; trusting them too
  much caused phantom targets ~1.5 m short of the real gate. Floor the handoff position
  uncertainty by range (`switch_pos_sigma_per_m`) and keep the target association radius
  generous (the chi2 gate decides), with "next gate" requiring >= 3 m beyond the target.
- 2026-10-07 — A time-to-go floor in the cubic guidance made the drone brake to a stop 0.7 m
  before a gate. Switch to a terminal "hold the line" mode at x > -1.2 m instead.
- 2026-10-07 — When a gate's outline merges with another gate, its RANSAC quad can fail the
  uprightness check; fall back to the gate's own inner opening rather than dropping it
  (seed 25 never initialised before; baseline 82.2% -> 89.2%).
- 2026-10-07 — Chi-squared gating is worth ~14 pp of pass rate at race pace
  (results/2026-10-07_c25234a_ablation-no-chi2.json 75.5% vs _baseline.json 89.2%).

- 2026-10-10 — A depth-biased handoff estimate (2-5 m short) leaves the real gate outside
  the association radius but short of "next gate" range, so it was silently ignored and the
  filter flew blind with a falsely small covariance. Every detection must land in *some*
  category that can correct the target: a gate straight ahead beyond an unmatched target now
  becomes the target (`recover_lost_target`; 89.2% -> 94.5%). Diagnose handoffs by hooking
  `_handoff` with truth (estimate minus true position right after the switch).

## Instruction changes

(none yet)

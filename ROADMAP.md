# Roadmap

Prioritised. Each overnight run takes ONE item (or a well-argued new one), and updates this
file: move finished items to "Done" with the result, re-rank the rest on new evidence.

Current baseline (since 2026-10-15, along-track time-to-go):
`results/2026-10-15_1d4bb7f_baseline.json` — 100.0% pass rate, 100% completion, 11.26 s lap,
median est. error 0.401 m (pooled over seeds 0/1000/2000/3000/4000: 97.75%; seed 0 is a lucky set).
Previous: `results/2026-10-13_0f1e256_baseline.json` — 96.8%, 12.27 s (pooled 97.50%).
Evidence older than 2026-10-12 was measured on
the legacy renderer (gates painted ~0.7 px too wide) and is not directly comparable:
`2026-10-11_8d3e44c_baseline.json` (95.2%), `2026-10-10_0181f50_baseline.json` (94.5%),
`2026-10-07_c25234a_baseline.json` (89.2%).

## (A) Pass rate

0. **Guidance that plans within its acceleration limit.** Crash analysis (run 10, 10 failed
   runs over 5 seed sets): every crash hits the target gate's frame 0.5-0.8 m off centre, and in
   8/10 the estimate was within 6 cm at impact — the drone runs out of lateral authority (often
   after a late post-handoff correction, 5/10). Clipping lateral-first (`lateral_accel_priority`)
   rescued 8 and broke 8 by bang-bang overshoot (pooled 97.50% -> 98.25%, completed 240 -> 240;
   left off). Run 11 removed the spurious along-track braking (the t_go_min floor;
   `along_track_tgo`, kept: pass flat, laps -1.03 s); the 7 runs it broke are again lateral
   saturation 0.5-1.2 m off centre with < 0.12 m estimate error. Next: a lateral profile whose peak acceleration respects `max_accel` (choose T or
   brake x only when needed), keep vertical demand from starving the tilt budget, and maybe
   replace the weak terminal hold-the-line law with a miss-distance law.
1. **Remaining next-gate range bias.** Re-measured 2026-10-13 (run 9): clean outlines and
   openings are within ~1% to 13 m; beyond 13 m outlines read 1-4% near and openings 1-4% far
   (opposite signs -> item 4). Far lone RANSAC outlines are now dropped. Older notes: the worst source (RANSAC outlines of merged gates,
   ~18% short at 7-16 m) is fixed (robust-outline check, done); handoffs > 1.5 m off along x
   fell from 17.8% to 10.1%. What is left: inner openings read +5-10% far beyond 9 m and clean
   outlines 1-2% near, both from the anti-aliased mask growing the frame by a fraction of a
   pixel on each side (see run 4 in NIGHTLY_LOG). Sub-pixel corners (now default)
   and item 4 (both squares, whose biases have opposite signs) attack this directly.
   Also consider requiring a close-range accepted sighting of the target before a pass.
2. **Gravity-constrained ("upright") PnP.** Gates are vertical, so solve 4 DoF (position + yaw)
   by minimising reprojection error with roll/pitch fixed from the filter, seeded from the
   IPPE solutions. Removes most IPPE flips; replace the 3-DoF attitude update with a yaw-only
   update. Evidence: 7% of attitude updates rejected, median yaw error 4.6 deg, up to ~12 deg
   in failing runs; single-square rotation errors of 5-30 deg at 6-11 m (see Lessons learned).
4. **Use both squares.** When outer and inner corners are both found, solve with all 8 points
   (IPPE start, then `solvePnPRefineLM` / SQPNP) for lower noise.
5. **Two-gate lookahead guidance.** At 8 m/s the drone has ~1 s after a pass to line up for
   the next gate. Plan through the current AND next gate (e.g. a cubic through both with the
   exit velocity chosen for the turn) so it starts the turn on the right line.
6. **Measurement covariance from the PnP Jacobian** instead of the heuristic distance model
   (`pnp.gate_position_cov`). Proper anisotropy should cut rejects and phantom offsets.
7. **Speed policy.** Once accuracy improves: slow for sharp turns, faster on straights
   (lap time is reported with every result; see the pace rule in CLAUDE.md).

## (B) Robustness, realism, tooling

- **Failure analysis tool** (the run-10 scratch analysis — estimate vs truth in the target
  frame and logged guidance accelerations before each crash — is what it should automate): `python -m vdr.analyze results/<file>.json` -> failure modes by
  gate index, outcome, pre-crash estimation error; `--seed N` re-runs one seed with a
  trajectory/estimate plot saved to `docs/img/`.
- **Stack unit tests:** handoff, retarget and search logic with scripted detections.
- **Realism, one knob at a time (each: add SimConfig field defaulting to old behaviour ->
  flip default -> re-baseline):**
  - camera latency (20-40 ms) and IMU/camera timestamp jitter
  - motion blur proportional to angular rate and speed
  - lens distortion (+ give the autonomy calibrated distortion coefficients via VehicleInfo)
  - lighting/exposure changes, gate colour variation, shadows
  - orange distractor objects (false positives) and partial occlusion
  - wind gusts; unknown thrust scaling (battery sag)
  - higher-resolution camera option (and runtime budget check)

- **Camera-only follow-ups:** test with a deliberately wrong vehicle model (drag, lags); estimate
  thrust scale/drag online; a vision-only attitude cue (horizon line) to replace the accelerometer.

## (C) Write-up

- Keep `docs/WRITEUP.md` and the README current after each kept change (add new results files
  to `vdr/report.py`'s lists and re-run `python -m vdr.report`). Next due by 2026-10-19.
- Possible additions: a per-stage figure (detection -> PnP error vs range, from a committed
  diagnostic results file), and a failure-mode breakdown once `vdr.analyze` exists.

## Done

- 2026-10-15 — Along-track time-to-go (`along_track_tgo`, default on): the 0.3 s T floor made the
  cubic brake at 20-70 m/s² inside x ≈ -2.4 m. Pooled pass 97.50% -> 97.75% (flat), completed
  240 -> 241, paired laps -1.03 s (233 runs); headline seed 0 96.8% -> 100.0%, 12.27 -> 11.26 s.

- 2026-10-14 — Crash analysis (A0): crashes are guidance saturation, not estimation error at
  impact. `lateral_accel_priority` A/B on 5 seed sets: pooled 97.50% -> 98.25%, completed runs
  240 -> 240 (8 rescued / 8 broken), laps -0.04 s; left off (see NIGHTLY_LOG run 10).

- 2026-10-13 — Camera-only mode (`camera_only`, no IMU, command-driven prediction): pooled
  98.0% -> 94.1% on 4 seed sets, left off. Standalone app (`vdr-racing`, PyInstaller release
  workflow for Windows/macOS/Linux, pip/pipx install) and browser replay (`docs/demo/`, Pages).

- 2026-10-13 — Drop far lone RANSAC outlines (`drop_lone_robust_far`, > 6 m with no valid
  opening; they read ~12% near). Pooled 5 seed sets 96.25% -> 97.50%, missed_gate 4 -> 0,
  laps unchanged (paired -0.007 s), est. error down on all sets; seed 0 alone 97.5% -> 96.8%.
  Re-tuning `robust_inner_depth_tol` was measured to be pointless (see NIGHTLY_LOG run 9).

- 2026-10-12 — Write-up (C): `docs/WRITEUP.md`, three figures from committed results
  (`python -m vdr.report`: history, ablations, paired lap times), README links. No flight change.

- 2026-10-12 — Sub-pixel corners on by default (`subpixel_corners`), refinement vectorised
  (1.17 -> 0.50 ms per quad). Four seed sets: pass rate unchanged (pooled 96.9% -> 96.7%),
  completed laps 0.155 s faster (paired, 145/179 faster). Headline 97.5% / 12.33 s.

- 2026-10-12 — Renderer realism (B): area-coverage gate anti-aliasing
  (`SimConfig.gate_supersample=8`); cv2 LINE_AA had painted every gate edge ~0.7 px too wide.
  Re-baselined: 96.2% (seed 0), 95.2% (seed 1000), lap 12.5 s; not comparable with earlier runs.

- 2026-10-11 — UDP link mode (`--link udp`, `vdr/link/`), the last first-week extra: the
  autonomy runs as a separate process over localhost UDP in lockstep, with fragmentation and
  retransmission. 50-run seed 0 per-run results identical to in-process (95.2% both);
  3 new separation tests cover the autonomy process.

- 2026-10-11 — Robust-outline sanity check (`robust_outer_use_inner`): a RANSAC-fitted outline
  whose range disagrees (> 8%) with the gate's own opening is replaced by the opening's PnP.
  Seed 0 94.5% -> 95.2%; 1000 93.5% -> 96.5%; 2000 95.0% -> 96.0%; 3000 (held out)
  87.8% -> 95.2%; pooled 92.7% -> 95.8%, lap time within 1.4%.

- 2026-10-10 — First-week extras: README flight GIF (`python -m vdr.flightviz`) and GitHub
  Actions CI (pytest + 8-run harness smoke test). No flight change (94.5% before and after).

- 2026-10-10 — Lost-target recovery (`recover_lost_target`): a gate seen straight ahead just
  beyond an unmatched target becomes the target. 89.2% -> 94.5% (seed 0), 91.2% -> 93.5%
  (seed 1000), 86.8% -> 95.0% (seed 2000, held out); lap time within 0.6%.

- 2026-10-07 — Bootstrap: simulator, autonomy stack, harness, 40 tests.
  Baseline 89.2% pass rate / 82% completion / 11.9 s lap (`2026-10-07_c25234a_baseline.json`).
  Ablations: no chi2 gating 75.5%; 4 m/s cruise 57.0% (race clock).

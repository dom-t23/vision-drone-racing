# Roadmap

Prioritised. Each overnight run takes ONE item (or a well-argued new one), and updates this
file: move finished items to "Done" with the result, re-rank the rest on new evidence.

Current baseline (since 2026-10-19, upright PnP):
`results/2026-10-19_9acf025_baseline.json` — 100.0% pass rate, 100% completion, 11.60 s lap,
median est. error 0.299 m (pooled over seeds 0/1000/.../5000: 99.54%; quote the pooled number).
Previous: `results/2026-10-16_3395fb0_baseline.json` — 97.8%, 11.59 s (pooled 99.08%).
Before that: `results/2026-10-15_1d4bb7f_baseline.json` — 100.0%, 11.26 s (pooled 6 sets 97.92%).
Before that: `results/2026-10-13_0f1e256_baseline.json` — 96.8%, 12.27 s (pooled 97.50%).
Evidence older than 2026-10-12 was measured on
the legacy renderer (gates painted ~0.7 px too wide) and is not directly comparable:
`2026-10-11_8d3e44c_baseline.json` (95.2%), `2026-10-10_0181f50_baseline.json` (94.5%),
`2026-10-07_c25234a_baseline.json` (89.2%).

## (A) Pass rate

0. **Next-gate yaw at handoff (was "phantom handoffs").** Run 14 found the 2-4 m post-handoff
   errors are a wrong next-gate *yaw*: a frame rotated about the gate centre leaves the
   gate-centre measurement with zero innovation, so only the weak PnP attitude update fixes it
   (~0.5-0.7 s). Averaged sighting yaw: RMS 9-10.5 deg, 4-11% of handoffs > 15 deg, worse
   under 7 m, claimed sigma 2-3x too small. Flooring that sigma (`handoff_yaw_floor`) alone was
   flat (pooled 99.08% -> 99.17%, left off). The fix has to improve the yaw measurement itself
   -> item 2 first, then re-try the floor. Also: replace the weak terminal hold-the-line law
   with a miss-distance law.
1. **Remaining next-gate range bias.** Re-measured 2026-10-13 (run 9): clean outlines and
   openings are within ~1% to 13 m; beyond 13 m outlines read 1-4% near and openings 1-4% far
   (opposite signs -> item 4). Far lone RANSAC outlines are now dropped. Older notes: the worst source (RANSAC outlines of merged gates,
   ~18% short at 7-16 m) is fixed (robust-outline check, done); handoffs > 1.5 m off along x
   fell from 17.8% to 10.1%. What is left: inner openings read +5-10% far beyond 9 m and clean
   outlines 1-2% near, both from the anti-aliased mask growing the frame by a fraction of a
   pixel on each side (see run 4 in NIGHTLY_LOG). Sub-pixel corners (now default)
   and item 4 (both squares, whose biases have opposite signs) attack this directly.
   Also consider requiring a close-range accepted sighting of the target before a pass.
2. **(next) Follow-ups to upright PnP (done 2026-10-19, see Done).** Diagnose the 2 runs it broke
   (1005 crash at gate 4, 5007 wrong_gate at gate 1); re-try `handoff_yaw_floor` on top of it;
   speed up `refine_upright` (harness 90 s -> 140 s per 50 runs).
4. **Error of merged / overlapping-gate detections.** Run 17 measured clean single-gate
   detections on rendered frames: yaw RMS < 1.6 deg and range bias < 0.1% out to 12 m. The
   remaining yaw/handoff error must come from RANSAC outlines, openings seen through other
   gates, and same-viewpoint averaging. Measure per-detection error by type on rendered
   multi-gate scenes before the next perception change.
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
  - **camera latency: knob added (run 19, `SimConfig.camera_latency`, default 0)**. Seed 0:
    10 ms 100%, 15 ms 92%, 20 ms 52%, 30 ms 18.75%, 40 ms 4.5%. The stale *attitude* is the
    whole effect (stale position alone: 100% at 20 ms). Blocked on a contract decision (Dom):
    frame capture timestamp in `Observation` (or latency in `VehicleInfo`); then (A) attitude
    rewind / delayed ESKF update behind a flag, then flip the default to ~25 ms and re-baseline.
  - IMU/camera timestamp jitter
  - motion blur proportional to angular rate and speed
  - lens distortion (+ give the autonomy calibrated distortion coefficients via VehicleInfo)
  - lighting/exposure changes, gate colour variation, shadows
  - orange distractor objects (false positives) and partial occlusion
  - wind gusts; unknown thrust scaling (battery sag)
  - higher-resolution camera option (and runtime budget check)

- **Camera-only follow-ups:** test with a deliberately wrong vehicle model (drag, lags); estimate
  thrust scale/drag online; a vision-only attitude cue (horizon line) to replace the accelerometer.

## (C) Write-up

- Current as of d356d7a (2026-10-20): upright PnP default (pooled 6 sets 99.54%), joint fit as
  "tried", camera-only re-run (98.62%). Next due by 2026-10-27.
- Keep `docs/WRITEUP.md` and the README current after each kept change (add new results files
  to `vdr/report.py`'s lists and re-run `python -m vdr.report`).
- When a flight-changing default flips, also re-run the camera-only A/B on the 6 sets (from a
  clean worktree so doc edits don't make the files `-dirty`), update `vdr/demo.py`'s BENCHMARK
  and regenerate `docs/demo/` and `docs/img/flight.gif`.
- Possible additions: a per-stage figure (detection -> PnP error vs range, from a committed
  diagnostic results file), a handoff-yaw-error figure, and a failure-mode breakdown once
  `vdr.analyze` exists.

## Done

- 2026-10-21 — Camera latency knob (B, run 19): `SimConfig.camera_latency` (default 0, old
  behaviour); seed-0 sweep 0-50 ms shows a cliff at 15-20 ms (100% -> 52%), caused by stale
  attitude, not stale position. Default not flipped (needs a frame-timestamp contract field).

- 2026-10-20 — Write-up refresh (run 18): README/WRITEUP/figures report the 9acf025 default
  (pooled 6 sets 99.54%, seed 0 100.0%, 11.60 s), upright PnP in history/ablations/paired laps,
  stories 6.9 (tilt drift) and 6.10 (joint fit ceiling). Camera-only re-run on 6 sets: pooled
  98.62% vs 99.54% (was 92.96% vs 99.08%); replay and GIF regenerated.

- 2026-10-20 **Both squares (A4), tried**: `joint_squares` (upright 8-point fit of outline +
  opening) flat — pooled 6 sets 99.54% -> 99.54% (1 rescued / 1 broken), laps +0.007 s; left
  off. Clean single-gate outlines are already accurate to 12 m; it only helps beyond that.

- 2026-10-19 **Upright PnP** (`upright_pnp` + `upright_keep_tilt`): 4-DoF fit (position + yaw)
  with tilt from the filter; attitude update keeps IPPE roll/pitch. Pooled 99.08% -> 99.54%,
  completed 295 -> 298, est. error lower on all 6 sets, laps flat. Yaw-only update alone was
  worse (97.71%: tilt drifted).

- 2026-10-17 — Write-up refresh (run 15): README/WRITEUP/figures report the 3395fb0 default
  (pooled 6 sets 99.08%, seed 0 97.8%, 11.59 s), feasible lateral in history/ablations/paired
  laps, stories 6.7 (plan around saturation) and 6.8 (handoff yaw). Camera-only re-run on 6 sets
  with the current default: pooled 99.08% vs 92.96% (completed 295 vs 268); replay and GIF
  regenerated.

- 2026-10-17 — Handoff yaw diagnosis + `handoff_yaw_floor` A/B (12 deg floor on the averaged
  next-gate yaw sigma): pooled 6 sets 99.08% -> 99.17% (1 rescued, 0 broken), laps +0.02 s;
  left off. Phantom handoffs traced to next-gate yaw errors (see item A0, NIGHTLY_LOG run 14).

- 2026-10-16 — Feasible lateral guidance (`feasible_lateral`, default on): shortest lateral
  time-to-go whose cubic peak stays within 11 m/s², braking along x when that is later than the
  along-track arrival. Pooled 6 sets 97.92% -> 99.08%, completed 290 -> 295 (9 rescued, 4 broken),
  paired laps +0.34 s (+3%); headline seed 0 100.0% -> 97.8%.

- 2026-10-15 — Write-up refresh (run 12): README/WRITEUP report the 1d4bb7f baseline (seed 0
  100.0%, pooled over 5 sets 97.75%, 11.26 s), along-track time-to-go in the history, ablation
  and a second paired-laps figure (`docs/img/paired_laps_tgo.png`), the lateral-first A/B as a
  "tried, left off" row, guidance section rewritten, debugging story 6.6 (crashes were control).
  Flight GIF re-recorded with the current default.
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

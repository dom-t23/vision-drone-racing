# Roadmap

Prioritised. Each overnight run takes ONE item (or a well-argued new one), and updates this
file: move finished items to "Done" with the result, re-rank the rest on new evidence.

Current baseline: `results/2026-10-10_0181f50_baseline.json` (94.5% pass rate, 90% completion,
11.97 s lap, 5 failures: 2 crash_gate, 3 missed_gate). Older evidence below cites
`2026-10-07_c25234a_baseline.json` (89.2%).

## (A) Pass rate

1. **Fix depth-biased next-gate estimates (root cause of early handoffs).** Lost-target
   recovery (done) catches most of them, but 9 of 19 remaining failures over seeds 0/1000/2000
   still have a handoff 2-5 m short, noticed too late (< 3 m) to recover. Ideas: weight far
   sightings by a range-dependent *bias* model, refine the handoff estimate with close-range
   sightings made right before the pass, or require a close-range (< ~4 m) accepted sighting
   of the target before a pass can be declared. Items 2, 3 and 6 below attack the same bias.
2. **Gravity-constrained ("upright") PnP.** Gates are vertical, so solve 4 DoF (position + yaw)
   by minimising reprojection error with roll/pitch fixed from the filter, seeded from the
   IPPE solutions. Removes most IPPE flips; replace the 3-DoF attitude update with a yaw-only
   update. Evidence: 7% of attitude updates rejected, median yaw error 4.6 deg, up to ~12 deg
   in failing runs; single-square rotation errors of 5-30 deg at 6-11 m (see Lessons learned).
3. **Sub-pixel corners.** Contour corners are integer pixels and biased ~0.5 px inwards
   (outer) / outwards (holes): ~5-7% range bias on far inner-opening detections. Fit lines
   to edge points (or `cv2.cornerSubPix` on the grey image) and intersect.
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

- **First-week extras (fixed design decisions):**
  - [ ] Flight GIF for the README: onboard camera + top-down course view, rendered from
        `run_episode(seed, record=True)` with matplotlib/Pillow.
  - [ ] Optional UDP link mode: sim and autonomy as separate processes exchanging
        Observation/Command messages over localhost UDP in lockstep (`--link udp`), with a test
        that it gives identical results to in-process mode.
  - [ ] GitHub Actions CI running `pytest` on push. Note: pushing `.github/workflows/*` may need
        the `workflow` permission; if the push is rejected, keep the file out of the commit,
        say so in the final message, and leave it for Dom to add.
- **Failure analysis tool:** `python -m vdr.analyze results/<file>.json` -> failure modes by
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

## (C) Write-up

- README results table and a results-history plot generated from committed results files.
- `docs/WRITEUP.md`: problem, pipeline diagram, ESKF in the gate frame, chi-squared gating
  (with the ablation: -14 pp without it), the identical-gates problem and the
  nearest-gate-ahead rule, harness methodology (seeds, race clock, noise threshold),
  debugging stories from Lessons learned.

## Done

- 2026-10-10 — Lost-target recovery (`recover_lost_target`): a gate seen straight ahead just
  beyond an unmatched target becomes the target. 89.2% -> 94.5% (seed 0), 91.2% -> 93.5%
  (seed 1000), 86.8% -> 95.0% (seed 2000, held out); lap time within 0.6%.

- 2026-10-07 — Bootstrap: simulator, autonomy stack, harness, 40 tests.
  Baseline 89.2% pass rate / 82% completion / 11.9 s lap (`2026-10-07_c25234a_baseline.json`).
  Ablations: no chi2 gating 75.5%; 4 m/s cruise 57.0% (race clock).

# Nightly log

Newest entries at the bottom. One entry per run: date and time, what, why, how verified,
pass rate before -> after, results files.

---

## 2026-10-07 ~01:30 BST — Run 1: bootstrap (started interactively with Dom)

**What:** built the whole project from an empty repo.
- `vdr/sim`: quadrotor dynamics driven by thrust + body-rate commands (acro), IMU with
  noise and random-walk biases, 320x240 FPV renderer (20 deg uptilt, 100 deg HFOV),
  seeded 8-gate courses of identical gates with guarantees (next gate visible and nearest
  ahead after each pass; clear legs), scoring (pass / crash_gate / crash_ground /
  missed_gate / wrong_gate / flyaway / timeout) and a race clock (>= 5 m/s average).
- `vdr/autonomy`: HSV + contour gate detector (outer outlines and inner openings, RANSAC
  quad fit for overlapping gates), IPPE_SQUARE PnP with gravity-resolved corner labelling,
  15-state ESKF in the target gate's frame with chi-squared gated gate-position and attitude
  updates, next-gate hypotheses, handoff on passing, retarget to a nearer gate ahead,
  cubic terminal-velocity guidance and geometric attitude control.
- `vdr/harness`: 50-run seeded parallel harness writing `results/*.json`.
- 40 tests including `tests/test_separation.py` (contract field lists + no `vdr.sim` imports
  in the autonomy).

**Design decisions (Dom):** acro thrust + body-rate commands; identical gates with no IDs;
first-week extras = README flight GIF, UDP link mode, GitHub Actions CI.

**Why these choices:** see CLAUDE.md "Lessons learned" — each was forced by a concrete
failure during bring-up (overlapping gates, accelerometer-only initial tilt, guidance stall,
phantom next-gate estimates, merged outlines).

**Benchmark choice:** default cruise speed 8 m/s plus the race clock, so the benchmark is at
race pace and can't be gamed by flying slowly (at 4 m/s the clock expires: 57.0%). At 4 m/s
with the clock switched off the stack passes 97.8% of gates, so race pace is where the
headroom is.

**Verified:** `pytest` 40 passed; episodes are bit-for-bit deterministic (test); harness
runs 50 episodes in ~65 s on 2 cores.

**Results (seed 0, 50 runs, commit c25234a):**
| config | pass rate | completion | lap (s) | file |
|---|---|---|---|---|
| baseline | **89.2%** | 82% | 11.9 | `2026-10-07_c25234a_baseline.json` |
| chi2 gating off | 75.5% | 60% | 12.0 | `2026-10-07_c25234a_ablation-no-chi2.json` |
| cruise 4 m/s | 57.0% | 0% (clock) | — | `2026-10-07_c25234a_ablation-4mps.json` |
| cruise 4 m/s, no race clock | 97.8% | 96% | 23.5 | `2026-10-07_0423930_ablation-4mps-noclock.json` |

(0423930 only changed the harness's dirty-flag logic; the flight code is identical to c25234a.)

**Next:** ROADMAP (A1) early handoffs are the main failure mode; (B) first-week extras.

---

## 2026-10-10 — Run 2: (A) lost-target recovery

**Baseline confirmed on main (865c062):** pytest 40 passed; `--runs 50 --seed 0` = 89.2%
pass rate, 82% completion, 11.94 s lap — identical to `2026-10-07_c25234a_baseline.json`.

**Diagnosis (ROADMAP A1, "early handoffs"):** hooked `_handoff`/`_switch_to` with ground truth
(scratch script, diagnostics only). In 6 of the 9 seed-0 failures (10, 15, 25, 36, 37, 42) the
handoff to the next gate put its estimate 2-5 m *short* (depth-biased far sightings). The real
gate then lands just outside the association radius: it is not "target", not "next gate"
(needs x >= 3 m beyond the target) and not "closer" (it is farther), so it is ignored. The
filter flies ~1 s on IMU alone with a falsely small covariance (x sigma ~0.1 m), declares a
pass 2-3 m before the real gate, and clips the frame. Seed 15: estimate x = -6.5 m vs true -9.3 m
at the handoff; pass declared at true x = -2.7 m.

**Change (`recover_lost_target`, default now True):** a third sanity check next to retarget.
While tracking, a detection in the drone's forward cone, *beyond* the target and not otherwise
classified, is clustered into a "beyond" hypothesis. If the target has gone unmatched for
`lost_target_frames` = 3 consecutive frames with detections, the beyond hypothesis has
`lost_target_min_sightings` = 3 sightings and the target is still > 3 m away, switch the target
to it (same `_switch_to` as a retarget). New stat `recoveries`; new end-to-end test (seed 15).

**Tried and dropped:** (1) 6 frames / 4 sightings: seed 0 94.0%, seed 1000 91.2% (no gain;
recovery came too late in seed 1010). (2) Re-initialising from the sighting's own PnP yaw
instead of the averaged hypothesis: seed 1000 90.0%, worse. The 3/3 thresholds were chosen after
looking at seeds 0 and 1000, so seed 2000 was run as a held-out confirmation.

**Results (50 runs each, commit 0181f50):**
| seeds | recovery off | recovery on | completion off -> on | lap off -> on (s) |
|---|---|---|---|---|
| 0 (headline) | 89.2% | **94.5%** | 82% -> 90% | 11.94 -> 11.97 |
| 1000 | 91.2% | 93.5% | 80% -> 84% | 12.03 -> 12.10 |
| 2000 (held out) | 86.8% | 95.0% | 74% -> 88% | 12.08 -> 12.04 |

Files: `results/2026-10-10_0181f50_{baseline,ablation-no-recovery}{,-seed1000,-seed2000}.json`.
Lap time within 0.6% (pace rule OK). Pooled over 150 runs: 89.1% -> 94.3%.

**Still failing (seed 0):** 3, 11 (missed gate 3 with correct handoffs: large lateral/height
handoff errors), 5 (crash at gate 5, handoffs correct), 37 and 42 (still an early handoff). Across the three seed sets 9 of 19 remaining failures still have an early handoff. The root
cause — depth-biased next-gate estimates — is untouched; recovery only limits the damage.

**Next:** fix the handoff estimate itself (ROADMAP A2 upright PnP / A3 sub-pixel corners /
A6 PnP-Jacobian covariance), or B (first-week extras) since the last two runs were build + A.

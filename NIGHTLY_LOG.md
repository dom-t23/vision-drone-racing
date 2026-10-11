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

---

## 2026-10-10 — Run 3: (B) first-week extras — flight GIF + CI

**Baseline confirmed on main (d33c241):** pytest 41 passed; `--runs 50 --seed 0` = 94.5% pass
rate, 90% completion, 11.97 s lap — identical to `2026-10-10_0181f50_baseline.json`.

**Chunk B (last run was A):**
- `python -m vdr.flightviz --seed N --out docs/img/flight.gif`: onboard camera (with HUD:
  time, true speed, gates passed, tracker mode) beside a top-down view of the course and the
  true trajectory. Truth is used for the picture only, outside the autonomy. README now shows
  seed 0 (completes 8/8). Real-time at 8 fps (every 5th camera frame). The camera's pixel noise
  makes every frame unique, so per-frame palettes gave 9 MB; one shared 48-colour palette from
  a 6-frame montage, no dithering, gives 3.3 MB.
- `run_episode(..., keep_frames, frame_stride)`: frames are now `(t, image)` pairs, one every
  `frame_stride` camera frames (no previous callers; flight unaffected).
- `.github/workflows/tests.yml`: pytest + 8-run harness smoke test on push to main and PRs.
  The workflow push was accepted (no `workflow` permission problem).
- Tests: +2 (`tests/test_flightviz.py`), 43 total. README results table updated to the
  current baseline and the recovery ablation.

**Verified:** pytest 43 passed; harness on fe6114d = 94.5% / 90% / 11.97 s, identical to before
(`results/2026-10-10_fe6114d_baseline.json`) — the change touches no flight code.

**Next:** (A) ROADMAP A1 depth-biased handoff estimates; last remaining extra is UDP link mode.

---

## 2026-10-11 — Run 4: (A) robust-outline sanity check

**Baseline confirmed on main (d9209f9):** pytest 43 passed; `--runs 50 --seed 0` = 94.5% pass
rate, 90% completion, 11.97 s lap — identical to `2026-10-10_fe6114d_baseline.json`.

**Diagnosis (ROADMAP A1, depth-biased next-gate estimates):** scratch diagnostic (truth used
offline only) comparing every PnP solution's range with the true range of the nearest gate,
seeds 0-9, split by detection type. Median estimated/true range:
- clean outer outlines: 0.98-1.00 at all ranges (gate looks ~1-2% near)
- inner openings: 1.01 at < 3 m rising to 1.10 at 12-30 m (looks far; the AA mask grows the frame)
- **RANSAC-fitted ("robust") outer outlines at 7-16 m: 0.80-0.84** — 2-3 m short at 10-15 m,
  while the *same detections'* inner openings read 1.03.
A robust outline means the gate's frame merged with another gate in the image — typically the
next gate seen overlapping the current one — which is exactly when next-gate sightings are
made. The RANSAC quad overshoots onto the other gate, so the gate looks too big, i.e. too near.

**Change (`robust_outer_use_inner`, default now True; `robust_inner_depth_tol` = 0.08):** for a
robust outer detection with a valid inner-opening fit, if the two ranges disagree by > 8%, use
the opening's PnP. Refactored into `Autonomy._detection_solutions`; +3 unit tests
(`tests/test_stack_selection.py`), 46 total.

**Tried first and changed:** "always use the opening for robust outlines" (tolerance 0):
seed 0 94.8%, 1000 93.5%, 2000 95.8% vs 94.5/93.5/95.0 off — within noise, and it broke 4 seed-0
runs that the old code completed (14, 16, 18, 40). Seed 16: near the gate both fits were good
and the outer was more accurate; only when they *disagreed* was the outer badly wrong (gate 3:
outer 3.4 m off, opening 0.25 m). Hence the agreement test. 0.08 was the only tolerance tried.
Because the refinement was motivated by a seed-0 failure, seed 3000 was run as a held-out set.

**Results (50 runs each, commit 68513f6):**
| seeds | check off | check on | completion off -> on | lap off -> on (s) |
|---|---|---|---|---|
| 0 (headline) | 94.5% | **95.2%** | 90% -> 94% | 11.97 -> 12.10 |
| 1000 | 93.5% | 96.5% | 84% -> 92% | 12.10 -> 12.27 |
| 2000 | 95.0% | 96.0% | 88% -> 90% | 12.04 -> 12.14 |
| 3000 (held out) | 87.8% | 95.2% | 76% -> 94% | 12.08 -> 12.11 |

Files: `results/2026-10-11_68513f6_{baseline,ablation-no-robust-inner}{,-seed1000,-seed2000,-seed3000}.json`.
Pooled over 200 runs: 92.7% -> 95.8% pass, 84.5% -> 92.5% completion; lap time +0.2% to +1.4%
(pace rule OK). Each single set is under the 8 pp noise threshold, but all four move the same
way and the held-out set moves most. Scratch handoff diagnostic (seeds 0-49, estimate minus
truth right after each pass handoff): handoffs with |x error| > 1.5 m 17.8% -> 10.1%.

**Still failing (seed 0):** 3 runs (1 crash_gate, 2 missed_gate). 10% of handoffs are still
> 1.5 m off along x; the remaining bias sources are the inner openings' +5-10% range at > 9 m
(mask growth) and the filter's own position error at the moment of the sighting.

**Next:** (A) correct the mask-growth edge bias (ROADMAP A3 sub-pixel corners) or use both
squares together (A4); or (B) the UDP link mode, the last first-week extra.

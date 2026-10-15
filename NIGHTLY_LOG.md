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

---

## 2026-10-11 — Run 5: (B) UDP link mode (last first-week extra)

**Baseline confirmed on main (06008fb):** pytest 46 passed; `--runs 50 --seed 0` = 95.2% pass
rate, 94% completion, 12.1 s lap — identical to `2026-10-11_68513f6_baseline.json`.

**Chunk B (last run was A; pass rate is at 95.2% where single-set gains are now below the
8 pp noise threshold, so the remaining extra was the better use of a run):**
- `vdr/link/`: `python -m vdr.harness --link udp` (or `run_episode(seed, link="udp")`) spawns
  the autonomy as its own process (`python -m vdr.link.autonomy_node`), which talks to the
  simulator only over localhost UDP: HELLO -> VehicleInfo -> (Observation n -> Command n) per
  tick -> BYE -> counters.
- Wire format (`wire.py`): JSON header + raw numpy bytes, built from the interface
  dataclasses' field lists; decoding calls the dataclass constructor, so an extra field
  (e.g. a gate position) raises instead of passing through.
- Images (230 kB) exceed one datagram: `udp.py` fragments into 60 kB datagrams and reassembles.
- Lockstep with loss handling: if Command n doesn't arrive in 2 s the sim resends Observation n;
  the autonomy answers a repeated n from its cache and never steps twice (tested by dropping
  observations and commands on purpose).
- The autonomy process returns its filter estimate as telemetry on camera ticks, used only for
  the runner's diagnostics (estimation error), the same as `auto.debug()` in-process.
- Separation tests +3: the autonomy-side link modules import nothing outside autonomy /
  interface / geometry / cfgutil / link (AST), importing the node loads no `vdr.sim*` module
  (subprocess check), and the wire messages carry exactly the contract's fields.
  `tests/test_link.py` +5 (round trip bit-exact, extra field rejected, fragmentation,
  UDP == in-process for seed 5, lossy lockstep). 54 tests total.

**Verified (commit 8d3e44c):** pytest 54 passed. `--runs 50 --seed 0`: in-process 95.2% / 94% /
12.1 s (`results/2026-10-11_8d3e44c_baseline.json`); UDP 95.2% / 94% / 12.1 s
(`results/2026-10-11_8d3e44c_link-udp.json`). All 50 per-run results (outcome, time, events,
diagnostics) are identical between the two and to `68513f6_baseline.json`; 0 retransmissions.
Wall time 82 s in-process vs 125 s over UDP (process start-up + serialisation).

**Next:** (A) ROADMAP A1/A3 (inner-opening mask-growth range bias) or A4 (both squares). (C) is
due by 2026-10-14 (7 days since the last write-up on 2026-10-07): docs/WRITEUP.md.

---

## 2026-10-12 — Run 6: (B) renderer realism — area-coverage anti-aliasing (re-baseline)

**Baseline confirmed on main (8c96b4b):** pytest 54 passed; `--runs 50 --seed 0` = 95.2% pass
rate, 94% completion, 12.1 s lap — identical to `2026-10-11_8d3e44c_baseline.json`.

**Why (started as ROADMAP A3, sub-pixel corners):** scratch diagnostic (truth used offline
only): 1500 random single-gate views at 2-20 m, detected corners vs true projected corners.
Contour corners: outer squares +0.3% to +1.7% too big, openings -2.7% to -10.4% too small,
RMS 0.6 / 1.6 px. A first sub-pixel refinement (half-contrast edge crossings) made it *worse*
(outer +10%, openings -12% at 14-20 m, RMS exactly 1.0 px at every range) — a constant pixel
offset, i.e. the edges themselves were drawn in the wrong place. Rendering a 20 x 20 px square
with cv2 `fillPoly(LINE_AA)` paints 456 px² instead of 400: **the simulator's renderer painted
every gate ~0.7 px too wide on every edge** (squares too big, openings too small). That is a
simulator defect, not something the autonomy should be tuned to, so this run became (B).

**Change:** `SimConfig.gate_supersample` (commit dfb2535, default 0 = old renderer): each gate
is rasterised into an N x N supersampled mask, box-filtered to per-pixel coverage and
alpha-blended — like a pixel integrating light over its area. cv2's fill also includes
sub-pixels whose centres lie on/just outside the edge, so the outline is shrunk and the
opening grown by half a sub-pixel first; with N = 8 a 20 px frame with a 10 px opening paints
300.0 px² (exact 300)
and a half-covered edge pixel gets 0.50. Render cost 0.6-1.0 ms -> 1.1-2.4 ms per frame.
Default flipped to 8 in a separate commit (a1d6b65). Tests: +1 renderer test (coverage and
area; legacy fill shown too fat). `test_lost_target_recovery_rescues_a_phantom_handoff`
(seed 15) no longer has a phantom handoff on the new renderer, so it is pinned to
`gate_supersample=0`, where the scenario was found.

**Corner accuracy on the new renderer (same diagnostic):**
| range (m) | contour: outer / opening size, RMS | sub-pixel: outer / opening size, RMS |
|---|---|---|
| 2-4 | -1.1% / -0.5%, 0.8 / 0.9 px | -0.0% / +0.0%, 0.10 / 0.12 px |
| 7-10 | -3.3% / -1.2%, 0.8 / 0.9 px | +0.0% / -0.1%, 0.16 / 0.15 px |
| 14-20 | -6.3% / -1.6%, 0.8 / 0.9 px | +1.7% / -2.5%, 0.32 / 0.33 px |
(The contour tracer follows the centres of boundary mask pixels, so it now reads gates far.)

**Re-baseline (commit a1d6b65; numbers before/after are NOT comparable):**
| seeds | pass | completion | outcomes | lap (s) | file |
|---|---|---|---|---|---|
| 0 (headline) | **96.2%** | 94% | 47 done, 3 crash_gate | 12.5 | `2026-10-12_a1d6b65_rebaseline-coverage-aa.json` |
| 1000 | 95.2% | 90% | 45 done, 5 crash_gate | 12.53 | `..._rebaseline-coverage-aa-seed1000.json` |
Before the flip, the same code with the default renderer reproduced 95.2% exactly. Completed
runs are on average 0.37 s slower (median 0.30 s, 45 runs completed in both); unverified
hypothesis: contour corners now read gates 1-6% *far* (before ~1% near), which changes when
the guidance turns and hands off.

**Preliminary (not a decision): sub-pixel corners** (`subpixel_corners`, commit b6ec22d,
default False; +1 detector test): seed 0 96.2% -> 96.8% (lap 12.5 -> 12.3 s), seed 1000
95.2% -> 96.0% (lap 12.53 -> 12.33 s); median estimation error 0.58 -> 0.57 m and
0.62 -> 0.55 m. Same direction on both sets, but each well under the 8 pp noise threshold.
Files `2026-10-12_a1d6b65_ab-subpix{,-seed1000}.json`. Harness wall time 73 s -> 107 s (the
per-profile Python loop; vectorise before flipping).

**Next:** (A) finish the sub-pixel A/B: vectorise, run seeds 2000/3000, decide; then revisit
`robust_inner_depth_tol` (its 8% was set on the old renderer's biases). (C) write-up is due
by 2026-10-14.

---

## 2026-10-12 — Run 7: (A) sub-pixel corners — finish the A/B, now default

**Baseline confirmed on main (66a635a):** pytest 56 passed; `--runs 50 --seed 0` = 96.2% pass
rate, 94% completion, 12.5 s lap, 3 crash_gate — identical to
`2026-10-12_a1d6b65_rebaseline-coverage-aa.json`.

**Speed-up first (commit 06ceff9):** `refine_quad_subpix` now samples all four edges' profiles
with one `cv2.remap`, finds every half-level crossing at once (argmax over the "above half"
mask instead of a per-profile Python loop) and fits each edge line in closed form (2x2 scatter
angle) instead of an SVD. On 386 calls recorded from seed 3 it returns the same corners to
1e-13 px, 1.17 -> 0.50 ms per quad. Whole episodes: sub-pixel overhead 9.1 s -> 4.8 s per 4
episodes; harness wall time with the flag on is now ~+20% (114 -> 137 s) instead of ~+50%.
The flag-off path is bit-identical to a1d6b65 (all 50 `per_run` entries equal).

**A/B (commit 06ceff9, same seeds, flag off vs on):**
| seeds | pass off -> on | completion | lap (s) off -> on | median est. error (m) |
|---|---|---|---|---|
| 0 | 96.2% -> 97.5% | 94% -> 96% | 12.50 -> 12.33 | 0.583 -> 0.558 |
| 1000 | 95.2% -> 94.8% | 90% -> 88% | 12.53 -> 12.34 | 0.622 -> 0.564 |
| 2000 | 98.5% -> 98.0% | 96% -> 96% | 12.46 -> 12.29 | 0.536 -> 0.424 |
| 3000 | 97.5% -> 96.5% | 96% -> 94% | 12.30 -> 12.15 | 0.571 -> 0.645 |
| pooled (1600 gates) | 96.9% -> 96.7% | 94% -> 93.5% | 12.44 -> 12.28 | |
Files `results/2026-10-12_06ceff9_{baseline,ab-subpix}{,-seed1000,-seed2000,-seed3000}.json`.
Pass rate: no change (all four differences within noise, two up, two down). Lap time: faster
on all four sets; paired over the 179 runs completed both ways, -0.155 s mean (median -0.147,
s.e. 0.016), 145 of 179 faster. Per the pace rule, faster at equal pass rate is an improvement,
so **`subpixel_corners` is now True by default** (commit 3b6cbbb; the flag stays for ablation).
Why the laps get faster is not verified; hypothesis: contour corners read gates 1-6% far, so
the guidance was planning against a gate further away than it is.

Outcome churn is large: 8 runs rescued and 10 broken across the four sets, including 3 new
`missed_gate` outcomes (seeds 1005, 1006, 3004; none with the flag off) — worth a look.

**Noise lesson:** the earlier preliminary A/B (a1d6b65, SVD line fit) gave 96.8% / 96.0% on
seeds 0 / 1000; the same algorithm with the closed-form fit (corners equal to 1e-13 px) gives
97.5% / 94.8%. 40+ of 50 runs have different traces and 1-2 outcomes flip per set. Per-seed
outcomes are chaotic, so only pooled multi-set numbers mean anything below ~8 pp.

**New headline (commit 3b6cbbb, default config):** `--runs 50 --seed 0` = **97.5%** pass rate,
96% completion, 48 completed / 2 crash_gate, 12.33 s lap
(`results/2026-10-12_3b6cbbb_baseline.json`; per-run identical to `06ceff9_ab-subpix.json`).
Tests: 56 passed. The seed-15 phantom-handoff scenario test now also pins
`subpixel_corners=False` (it was found with contour corners and the legacy renderer).

**Next:** (C) the write-up is due by 2026-10-14 (docs/WRITEUP.md). Then (A): look at the three
new `missed_gate` runs; re-tune `robust_inner_depth_tol` (8%, set on the legacy renderer's
biases) and the range-noise model now that corners are within ~1-2% at range.

---

## 2026-10-12 — Run 8: (C) write-up

**Baseline confirmed on main (8872bfa):** pytest 56 passed; `--runs 50 --seed 0` = 97.5% pass
rate, 96% completion, 12.33 s lap, 48 completed / 2 crash_gate — identical to
`2026-10-12_3b6cbbb_baseline.json`.

**Chunk C (due by 2026-10-14; no write-up since the bootstrap on 2026-10-07):**
- `docs/WRITEUP.md`: problem and rules, results, pipeline (mermaid diagram), detection, PnP
  ambiguities, the ESKF in the target gate's frame and the handoff frame change, chi-squared
  gating, the gate-category table (target / next / nearer / beyond), cubic guidance and
  geometric control, the simulator, evaluation method (seeds, flags, noise, pace rule, paired
  lap times, re-baselines), four debugging stories, limitations.
- `python -m vdr.report` (commit 861bf57) draws three figures into `docs/img/` from a fixed
  list of committed results files only: headline pass rate after each kept change (with the
  renderer re-baseline marked), each component off vs on pooled over its seed sets, and the
  paired lap-time histogram for sub-pixel corners. Numbers it prints match the log: 89.2 /
  94.5 / 95.2 / 96.2 / 97.5%; chi2 75.5 -> 89.2, recovery 89.1 -> 94.3, robust-outline
  92.7 -> 95.8, sub-pixel 96.9 -> 96.7; paired laps n = 179, mean -0.155 s, 145 faster.
- `tests/test_report.py` (+1): every listed file exists and the figures render. 57 tests.
- README links the write-up and shows the history figure; test count and harness time
  updated (README and CLAUDE.md said 54 tests / ~65 s; it is 57 / ~130 s since sub-pixel
  corners).

**Verified:** pytest 57 passed; 8-run harness smoke on 861bf57 100% (64/64). No flight code
changed, so no new results file.

**Pass rate:** no change (97.5%).

**Next:** (A) ROADMAP item 0 — re-tune `robust_inner_depth_tol` and the range-noise model for
the new corners, and look at the three `missed_gate` runs (seeds 1005, 1006, 3004). Next (C) due
by 2026-10-19.

## 2026-10-12 — Interactive session (not a scheduled run): code review fixes

Requested by Dom: review the codebase, fix what was found.

- **bb0d16f** `flightviz --live`: tkinter window that replays a flight at real-time pace.
- **63ac0b6** cleanups with identical flight results — ESKF computes P Hᵀ and S once per
  update; detector skips re-fitting a hole whose quad already failed in pass 1; harness
  reports a true median (was the upper median for even run counts) and rejects `--runs 0`;
  flightviz handles 0/1 frames. Verified: 50 runs, seed 0, per-run results identical.
- **49d3f9a** small behaviour changes + two opt-in flags:
  - `ESKF.change_frame` rotates `pos_cov` (old-frame covariance of t) into the new frame
    (rounding-only today: callers pass an isotropic covariance);
  - OUTER detection area measured on the sub-pixel refined quad (INNER already was);
  - `AutonomyConfig.yaw_correlated_cov` (default False): add a frame's yaw uncertainty to
    p, v, attitude as one rank-1 term with cross-covariances (init and handoff);
  - `SimConfig.pixel_noise_fresh` (default False): independent image noise every frame
    instead of a bank of 8 patterns. Not benchmarked; flipping it needs a re-baseline.

**Defaults after 49d3f9a** (`*_49d3f9a_baseline*.json`): 97.5 / 95.75 / 97.5 / 94.0% on seed
sets 0/1000/2000/3000, pooled 96.19% (vs 96.69% for `*_06ceff9_ab-subpix*.json`; within
noise — chaotic per-seed flips). Headline unchanged: 97.5%, 96% completion, 12.33 s.

**A/B `yaw_correlated_cov`** (`*_49d3f9a_ab-yawcorr*.json`): 97.0 / 93.25 / 96.5 / 98.75%,
pooled 96.38% vs 96.19% off; paired laps n = 179, mean -0.003 s, 87 faster. No clear effect:
left off (flag kept for later, e.g. under harder sim settings).

**Pass rate:** no change (97.5%).

---

## 2026-10-13 — Run 9: (A) drop far RANSAC outlines that have no opening to cross-check

**Baseline confirmed on main (41a2fe0):** pytest 58 passed; `--runs 50 --seed 0` = 97.5% pass
rate, 96% completion, 12.33 s lap, 48 completed / 2 crash_gate — identical to
`2026-10-12_3b6cbbb_baseline.json`.

**Measure first (ROADMAP item 0).** Scratch script (not committed): wrap
`Autonomy._detection_solutions` and compare every PnP range with the true range to the gate
nearest the estimate, seeds 0-19, default config, ~20 000 detections:
| type | 7-10 m | 10-13 m | 13-20 m | 20-40 m |
|---|---|---|---|---|
| clean outline | -0.1% | -0.3% | -1.3% | -4.2% |
| opening (of an outline) | -0.1% | +0.1% | +1.3% | +3.5% |
| RANSAC outline, any | -17.3% | -10.6% | -12.5% | -9.6% |
(medians.) RANSAC outlines with no valid opening of their own, binned by true range: 4-6 m
+0.0%, 6-7 m -0.5% (p10 -15%), 7-8 m -7.1%, 8-10 m -11.1%, 10-15 m -13.2%, 15-40 m -12.4%;
72-75% of them > 5% near beyond 8 m. With sub-pixel corners and the fixed renderer, clean
squares are good to ~1% out to 13 m. Re-tuning `robust_inner_depth_tol` would not help: over
RANSAC outlines beyond 7 m the chosen range is -2.8% median at tol 0 vs -3.5% at 8%, ~half
still > 5% off either way, because 455 of 819 of them have no valid opening to compare with.

**Change (commit b6a6a31, flag `drop_lone_robust_far`, default False; +1 test):** a RANSAC
outline with no valid opening of its own, measured beyond `lone_robust_max_range` = 6 m, is
dropped instead of being used as a target update or next-gate sighting.

**A/B (commit b6a6a31, same seeds, off -> on):**
| seeds | pass | completion | lap (s) | median est. error (m) |
|---|---|---|---|---|
| 0 | 97.5% -> 96.8% | 96 -> 94% | 12.33 -> 12.27 | 0.556 -> 0.394 |
| 1000 | 94.8% -> 100.0% | 88 -> 100% | 12.35 -> 12.38 | 0.545 -> 0.459 |
| 2000 | 98.0% -> 97.2% | 96 -> 96% | 12.29 -> 12.32 | 0.414 -> 0.402 |
| 3000 | 96.5% -> 98.0% | 94 -> 96% | 12.15 -> 12.18 | 0.643 -> 0.538 |
| 4000 (held out, run after the first four) | 94.5% -> 95.5% | 92 -> 94% | 12.29 -> 12.26 | 0.625 -> 0.482 |
| pooled (2000 gates) | **96.25% -> 97.50%** | | | |
Files `results/2026-10-1{2,3}_b6a6a31_{baseline,ab-droplone}{,-seed1000,...,-seed4000}.json`.
Paired laps (229 runs completed both ways): mean -0.007 s, median 0.000, s.e. 0.008 — unchanged.
11 runs rescued, 4 broken (seeds 4, 2015, 3036, 4010, all crash_gate). `missed_gate`
outcomes 4 -> 0 (incl. seeds 1005, 1006, 3004 from run 7). Estimation error lower on all five
sets. Every per-set difference is under the 8 pp noise threshold (3 up, 2 down), so the
decision rests on the pooled number, the held-out set agreeing, and the consistent error drop.
**Kept: default True** (commit 0f1e256). The seed-15 recovery scenario test now pins the flag
off: with it on, that seed completes all 8 gates with no recovery needed.

**New headline (commit 0f1e256):** `--runs 50 --seed 0` = **96.8%** pass rate, 94% completion,
47 completed / 3 crash_gate, 12.27 s lap, median est. error 0.394 m
(`results/2026-10-13_0f1e256_baseline.json`; per-run identical to `b6a6a31_ab-droplone.json`).
The seed-0 set drops one run (seed 4) — noise against a pooled +1.25 pp; reported as is.
README, WRITEUP (results table, ablation row, section 6.5) and figures updated.
Tests: 59 passed.

**Pass rate:** headline 97.5% -> 96.8% (seed 0); pooled over five seed sets 96.25% -> 97.50%.

**Next:** (A) the remaining failures are all `crash_gate`; look at what the estimate was doing
before each (a failure-analysis tool, ROADMAP (B), would make this quick). Then ROADMAP items
2/4 (upright PnP, both squares). (C) due by 2026-10-19.

## 2026-10-13 — Interactive session (not a scheduled run): camera-only mode, standalone app, browser replay

Requested by Dom: a fully vision-based option with no map and no IMU, and a version viewers can
install or open easily. He chose: camera-only as an A/B'd option (default only if it holds up);
standalone executables and a browser demo.

**Camera-only (commit d26fe46, flag `camera_only`, default False).** The IMU fields of the
observation are never read (`tests/test_camera_only.py` feeds NaN accel / 1e6 gyro and checks
identical commands). Between frames the ESKF is propagated with a predicted specific force and
body rate from the drone's own last command: first-order rate (40 ms) and thrust (30 ms) lags
and linear drag (0.35 / 0.1 1/s), model-error noise 1.0 m/s²/√Hz and 0.05 rad/s/√Hz. The
filter assumes it starts level. These are the autonomy's own model assumptions, set to the
nominal airframe (the sim also has quadratic drag the model omits); no sensitivity test with a
deliberately wrong model was run.

A/B (commit db88370 — d26fe46 plus the app entry point, no flight-code change), off -> on:
| seeds | pass | completion | lap (s) | median est. error (m) |
|---|---|---|---|---|
| 0 | 96.8% -> 98.0% | 94 -> 96% | 12.27 -> 12.28 | 0.394 -> 0.441 |
| 1000 | 100.0% -> 96.8% | 100 -> 92% | 12.38 -> 12.47 | 0.459 -> 0.535 |
| 2000 | 97.2% -> 89.5% | 96 -> 82% | 12.32 -> 12.47 | 0.402 -> 0.453 |
| 3000 | 98.0% -> 92.2% | 96 -> 86% | 12.18 -> 12.25 | 0.538 -> 0.545 |
| pooled (1600 gates) | **98.00% -> 94.12%** | | | |
Files `results/2026-10-13_db88370_{baseline,ab-camonly}{,-seed1000,-seed2000,-seed3000}.json`.
Paired laps (173 runs): +0.064 s mean. 5 runs rescued, 20 broken; estimation error worse on all
four sets. **Left off** — camera + IMU stays the default. Seed set 0 alone pointed the other way.

Baseline check at db88370: seed 0 96.75% (= `2026-10-13_0f1e256_baseline.json`, per-run).

**Standalone app.** `vdr/app.py` is one entry point (`vdr-racing`: launcher; `fly`, `bench`,
`demo-data` sub-commands). The launcher re-runs its own executable with a sub-command when
frozen; harness workers call `freeze_support`. `pyproject.toml` (pip/pipx install, console
script `vdr-racing`); `packaging/build.py` (PyInstaller one-folder build + zip);
`.github/workflows/release.yml` builds Windows/macOS/Linux on a `v*` tag, smoke-tests each
executable with a 1-course benchmark and attaches the zips to a release. Linux build verified
here: 185 MB zip; `bench`, `fly --out x.gif` and `demo-data` run from the executable (the tk
launcher window could not be opened here — no display/tkinter in this container). Harness writes
`results/` into the current folder and records commit "unknown" when not run from a git checkout.

**Browser replay** (`docs/demo/`, published by `.github/workflows/pages.yml` once Pages is set to
"GitHub Actions"): `python -m vdr.demo` records flights (JPEG frames, detector quads re-run on the
same pixels, true and estimated positions); the page plays them back with a top view and the
camera-only benchmark table read from the committed results. Seeds 0, 1017, 2034, both modes
(camera-only crashes on 1017 and 2034, matching the results files).

Tests: 63 passed. **Pass rate:** no change to the default (headline 96.8%).

## 2026-10-14 — Run 10: (A) what the remaining crashes are; lateral-priority saturation (left off)

**Baseline check** at ef43188: tests 63 passed; `--runs 50 --seed 0` 96.8% (387/400), 47 completed /
3 crash_gate, 12.27 s — same as `2026-10-13_0f1e256_baseline.json`.

**Crash analysis (ROADMAP A0).** Re-flew the 10 failed runs of the current default over seed sets
0/1000/2000/3000/4000 (seeds 4, 14, 47, 2015, 2049, 3018, 3036, 4008, 4010, 4042) with
`run_episode(record=True)`, comparing the estimate with truth in the target gate's frame, and
logged the guidance acceleration with a monkeypatched `gate_guidance` (scratch scripts, not committed):
- All 10 hit the frame of the gate they were aiming at, 0.5-0.8 m off centre laterally.
- In 8 of 10 the estimate was within 6 cm of truth at impact (exceptions: 47 and 2015, 0.4-0.5 m
  along x). So it is not estimation error at the moment of the crash.
- In 5 of 10 (4, 2015, 3018, 3036, 4010) the estimate had been 1.7-3.5 m off about 0.8-1.0 s
  before impact and was corrected with ~0.6 s to go — late after a handoff.
- Either way the drone then needs more lateral acceleration than it has: close to `terminal_x`
  the cubic asks for 30-75 m/s² and `attitude_command` scales *all* axes down to 14 m/s², so a
  large spurious along-track braking term (-20 to -39 m/s² at x ≈ -1.3 m) eats most of the
  budget. The terminal "hold the line" law (k_lat = 3 1/s² for ~0.15 s) barely moves the drone.

**A/B: `lateral_accel_priority`** (commit 2d7e3f6, default False). When the gate guidance asks for
more than `max_accel`, keep the lateral (gate y/z) part and shorten the along-track part first.
| seeds | pass off -> on | completion | lap (s) | median est. error (m) |
|---|---|---|---|---|
| 0 | 96.8% -> 96.2% | 94 -> 94% | 12.27 -> 12.29 | 0.394 -> 0.434 |
| 1000 | 100.0% -> 100.0% | 100 -> 100% | 12.38 -> 12.32 | 0.459 -> 0.445 |
| 2000 | 97.2% -> 95.2% | 96 -> 88% | 12.32 -> 12.25 | 0.402 -> 0.362 |
| 3000 | 98.0% -> 99.8% | 96 -> 98% | 12.18 -> 12.19 | 0.538 -> 0.515 |
| 4000 | 95.5% -> 100.0% | 94 -> 100% | 12.26 -> 12.20 | 0.482 -> 0.478 |
| pooled (2000 gates) | **97.50% -> 98.25%** | 240 -> 240 runs | | |
Files `results/2026-10-1{3,4}_2d7e3f6_{baseline,ab-latprio}{,-seed1000,...,-seed4000}.json`.
Paired laps (232 runs completed both ways): mean -0.040 s. 8 runs rescued (4, 14, 2015, 3018,
3036, 4008, 4010, 4042 — 8 of the 10 analysed crashes; 47 and 2049 still crash) and 8 broken
(5, 11, 2003, 2010, 2030, 2042, 2046, 3019; 5 of them on set 2000), so completed runs are
unchanged (240/250 both ways); the gate-count gain is broken runs failing later. Not a clear
win -> **left off**.

Why it breaks runs (guidance traces of 5, 2003, 2030, 3019 with the flag on): with lateral
priority the saturated cubic becomes bang-bang — full 14 m/s² sideways until ~1.8 m out builds
4-5 m/s of lateral speed that can't be removed in time, so the drone overshoots through the
centre line into the other side of the frame. In 3019 the guidance asked for -9 to -12 m/s²
vertically; `f_z` then nearly vanishes and the tilt limit in `attitude_command` clips the
horizontal part anyway (commanded -13 m/s² laterally, achieved ~3). Reordering the clip is not
enough: the guidance must *plan* within its acceleration limit (braking-aware / time-optimal
lateral profile, or start the correction earlier), and vertical demand must not starve the tilt
budget.

Added `tests/test_control.py` (4 tests on the saturation helper and flag-off equivalence).
Tests: 67 passed. **Pass rate:** no change to the default (headline 96.8%, pooled 97.50%).

**Next:** (A) a guidance law that respects the acceleration limit — e.g. lateral cubic with T
chosen so its peak acceleration stays under `max_accel` (slowing x only when needed), or a
min-time bang-bang with a braking switch; A/B pooled over 5 sets. Also: the 5 late corrections
point at handoff accuracy (ROADMAP A1/A4). (C) write-up due by 2026-10-19.

## 2026-10-15 — Run 11: (A) along-track time-to-go (kept: pass rate flat, laps -1.03 s)

**Baseline check** at 1e0fdb1: tests 67 passed; `--runs 50 --seed 0` 96.8% (387/400), 47 completed /
3 crash_gate, 12.27 s — same as `2026-10-13_0f1e256_baseline.json`.

**Where the spurious braking came from.** Run 10 found the cubic asking for -20 to -39 m/s²
along-track near x ≈ -1.3 m. The cause is the `t_go_min` = 0.3 s floor on time-to-go: at 8 m/s
the floor is active from x ≈ -2.4 m to `terminal_x` = -1.2 m, where the gate is really
0.15-0.3 s away, so the cubic "arrives too early" and brakes hard (at 8 m/s on the line with
0.6 m lateral offset: a_x = -27 m/s² at x = -2 m, -73 m/s² at -1.3 m, against a lateral demand of
-40). Uniform clipping to 14 m/s² then left 12 and 7 m/s² for the lateral correction.
Range-based T (it includes the lateral offset) adds a smaller braking term further out.

**A/B: `along_track_tgo`** (commit 131d93c). Time-to-go from the along-track motion only,
T_x = 2(-p_x)/(v_x + V); the x axis uses T_x unfloored (zero demand at cruise on the line), the
lateral axes keep the 0.3 s floor.
| seeds | pass off -> on | completion | lap (s) |
|---|---|---|---|
| 0 | 96.8% -> 100.0% | 94 -> 100% | 12.27 -> 11.26 |
| 1000 | 100.0% -> 96.5% | 100 -> 96% | 12.38 -> 11.31 |
| 2000 | 97.2% -> 98.8% | 96 -> 96% | 12.32 -> 11.28 |
| 3000 | 98.0% -> 96.5% | 96 -> 94% | 12.18 -> 11.14 |
| 4000 | 95.5% -> 97.0% | 94 -> 96% | 12.26 -> 11.21 |
| pooled (2000 gates) | **97.50% -> 97.75%** | 240 -> 241 runs | |
Off: `results/2026-10-14_131d93c_baseline.json` (seed 0, identical per-run to the 0f1e256 file) and
`results/2026-10-1{3,4}_2d7e3f6_baseline-seed{1000..4000}.json` (flag-off code path unchanged).
On: `results/2026-10-1{4,5}_131d93c_ab-attgo{,-seed1000,...,-seed4000}.json`.
Paired laps (233 runs completed both ways): mean -1.03 s (-8.4%), median -1.03 s, worst +0.085 s.
Pass rate is flat (within noise), so by the pace rule (faster at equal pass rate) it is an
improvement -> **kept**, default flipped in 1d4bb7f. Post-flip headline:
`results/2026-10-15_1d4bb7f_baseline.json` — 100.0% (400/400), 50/50 completed, 11.26 s,
median est. error 0.401 m. UDP smoke test (8 runs) passes.

Rescued 8 runs (4, 14, 47, 3018, 3036, 4008, 4010, 4042), broke 7 (1020, 1033, 3001, 3004, 3010,
4004, 4011). The broken ones (re-flown with `run_episode(record=True)`) are the same failure as
run 10, now at a higher average speed: all hit / miss the target gate 0.54-1.19 m off centre
laterally; estimate error at impact <= 0.12 m in all 7; in 1020 and 1033 it was 3.5-3.9 m off
0.8 s before (late post-handoff correction). So lateral authority close to the gate is still
the limit — the cubic's lateral demand saturates.

Tests: `tests/test_control.py` +2 (no along-track demand on the line at cruise; agrees with
the old law far out on the axis). `test_lost_target_recovery_rescues_a_phantom_handoff` now pins
`along_track_tgo=False` (the new flight path no longer produces that phantom handoff). 69 passed.

**Pass rate (headline, seed 0):** 96.8% -> 100.0% (pooled 5 sets 97.50% -> 97.75%, i.e. flat);
lap 12.27 -> 11.26 s.

**Next:** (A) lateral demand that respects `max_accel`: e.g. pick lateral T so the cubic's peak
lateral acceleration (6|Δy|/T² for a rest-to-rest move) stays under the limit and slow along x
only when that T exceeds the arrival time; pooled over 5 sets, paired laps. (C) write-up is due
by 2026-10-19 — README/WRITEUP still quote the 96.8% / 12.27 s baseline (true for its file).

## 2026-10-15 — Run 12: (C) write-up refresh

Chunk choice: runs 10 and 11 were both (A), and README/WRITEUP still quoted the pre-run-11
baseline (96.8% / 12.27 s), so this run is (C).

**Baseline check** at 1c03a85 (local `main` had been 35 commits behind `origin/main`;
fast-forwarded before starting): tests 69 passed; `--runs 50 --seed 0` 100.0% (400/400),
50/50 completed, 11.26 s, median est. error 0.401 m — identical to
`results/2026-10-15_1d4bb7f_baseline.json` (run not saved; no code change since).

**What changed (docs and figures only; no autonomy or sim code touched):**
- `vdr/report.py`: 1d4bb7f baseline added to the history; along-track time-to-go added to the
  ablation figure (5 sets: 97.50% -> 97.75%); new `REJECTED` list (lateral-first saturation,
  97.50% -> 98.25%, printed, not plotted); `PAIRED` is now a list, giving a second figure
  `docs/img/paired_laps_tgo.png` (233 paired runs, 232 faster, mean -1.025 s, median -1.033 s).
  Ablation tick labels wrapped (they overlapped with 6 bars). `tests/test_report.py` covers the
  new lists and both paired figures.
- README: results table leads with seed set 0 (100.0%) *and* the pooled 5-set number
  (97.75%, 241/250 completed, from `results/2026-10-1{4,5}_131d93c_ab-attgo[-seedN].json`,
  same code as the default), says plainly that seed 0 is a lucky set; camera-only table
  labelled as measured before along-track time-to-go; test count 63 -> 69.
- WRITEUP: section 2 (pooled number, failure outcomes 8 crash_gate + 1 missed_gate), ablation
  table rows for along-track time-to-go and the left-off lateral-first A/B, section 3.6
  describes the current time-to-go law, section 5 adds the second paired-laps figure, new
  debugging story 6.6 (crashes were control, not perception; phantom braking from the T floor),
  section 7 names lateral saturation as the main remaining failure.
- `docs/img/flight.gif` re-recorded (seed 0, current default: completed 8/8).

All numbers checked against `python -m vdr.report` output and the results files.
Tests: 69 passed; 8-run smoke test 100.0%. **Pass rate:** no change (headline 100.0%, pooled
97.75%).

Not done: the browser replay recordings (`docs/demo/flights.js`) and the camera-only A/B
predate along-track time-to-go; re-running camera-only on the current default should come
first so the replay and its table match (ROADMAP C).

**Next:** (A) lateral guidance that plans within `max_accel` (ROADMAP A0), pooled over 5 sets
with paired laps.


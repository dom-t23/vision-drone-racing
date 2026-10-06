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

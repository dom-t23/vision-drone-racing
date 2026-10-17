# Vision-Based Autonomous Drone Racing

A simulated racing quadrotor flies a course of identical square gates at 8 m/s using **only
its monocular FPV camera and IMU** — no GPS, no depth sensor, no map of where the gates are.
Everything is self-contained Python: the simulator renders real camera images, and the
flight stack has to find the gates in the pixels.

![Seed 0: onboard camera (left) and top-down course view (right)](docs/img/flight.gif)

*Course seed 0, default configuration, real-time playback at 8 fps. Left: the 320×240 camera
image the flight stack actually receives, with a HUD (time, true speed, gates passed, tracker
mode). Right: true trajectory over the course (for viewing only — the drone never gets the
map). Regenerate with `python -m vdr.flightviz --seed 0`.*

[![tests](https://github.com/dom-t23/vision-drone-racing/actions/workflows/tests.yml/badge.svg)](https://github.com/dom-t23/vision-drone-racing/actions/workflows/tests.yml)

## Try it

- **In the browser, nothing to install:** [flight replay](https://dom-t23.github.io/vision-drone-racing/)
  — recorded flights with the onboard camera, the detector's gate outlines and the drone's
  own position estimate, for both sensing modes (source: [`docs/demo/`](docs/demo/)).
- **Desktop app, no Python needed:** download the zip for Windows, macOS or Linux from
  [Releases](https://github.com/dom-t23/vision-drone-racing/releases), unzip, and run
  `vdr-racing` (`vdr-racing.exe` on Windows) inside the `vdr-racing` folder. It opens the
  launcher: fly a course and watch it, save a GIF, or run a benchmark. The builds are not
  code-signed: on macOS right-click → Open the first time; on Windows choose
  "More info → Run anyway".
- **With Python 3.10+:** `pipx install git+https://github.com/dom-t23/vision-drone-racing`
  (or `pip install` the same URL), then `vdr-racing`, `vdr-racing fly --seed 3 --live`,
  `vdr-racing bench --runs 8`.

## Results

Benchmark: 50 seeded random courses × 8 gates, default configuration, race clock on
(each run must average ≥ 5 m/s or it times out). Every number links to a committed results
file; see [NIGHTLY_LOG.md](NIGHTLY_LOG.md) for the history.

Current simulator (area-coverage gate rendering, since 2026-10-12):

| configuration | gate pass rate | courses completed | mean lap |
|---|---|---|---|
| **baseline, pooled over 6 seed sets (300 courses)** | **99.08%** | 98.3% | 11.5–11.7 s per set |
| **baseline (8 m/s)**, seed set 0 | **97.8%** | 96% | 11.59 s |
| same, feasible lateral guidance off | 100.0% | 100% | 11.26 s |
| same, also along-track time-to-go off | 96.8% | 94% | 12.27 s |
| same, also far lone RANSAC outlines kept | 97.5% | 96% | 12.33 s |
| same, also sub-pixel corners off | 96.2% | 94% | 12.5 s |

Files: [`baseline`](results/2026-10-16_3395fb0_baseline.json),
pooled: the baseline plus `results/2026-10-16_d37cf14_ab-feaslat-seed{1000..5000}.json` (same
code as the baseline), [`feasible lateral guidance off`](results/2026-10-15_1d4bb7f_baseline.json),
[`along-track time-to-go off`](results/2026-10-13_0f1e256_baseline.json),
[`far lone RANSAC outlines kept`](results/2026-10-12_3b6cbbb_baseline.json),
[`sub-pixel corners off`](results/2026-10-12_06ceff9_baseline.json).
Single 50-run sets differ by one or two runs from noise alone, so the pooled row is the number
to quote and components are judged pooled over several seed sets. The latest change makes the
guidance plan its sideways correction within the acceleration limit (braking along the track
when a correction would not fit) instead of clipping an impossible request: pooled pass rate
97.92% → 99.08% and 5 more completed courses, for laps 3% slower (+0.34 s on 286 paired runs).
On seed set 0 alone it went 100.0% → 97.8%, because that set had been the lucky one. Before it,
taking the time-to-go from along-track motion made laps 1.03 s faster at an unchanged pass
rate. Most remaining failures start at a handoff, when the next gate's estimated *yaw* is off
by 16–46° in the failing runs (details in the write-up).

![Headline benchmark after each kept change](docs/img/history.png)

The full write-up — pipeline, filter design, evaluation method, ablations and debugging
stories — is in [docs/WRITEUP.md](docs/WRITEUP.md). Figures are regenerated from the
committed results files with `python -m vdr.report`.

Earlier simulator (cv2 anti-aliased fill, which painted gates ~0.7 px too wide on every
edge; numbers are not comparable with the table above):

| configuration | gate pass rate | courses completed | mean lap |
|---|---|---|---|
| baseline (8 m/s) | 95.2% | 94% | 12.1 s |
| same, robust-outline check off | 94.5% | 90% | 12.0 s |
| earlier baseline, lost-target recovery off | 89.2% | 82% | 11.9 s |
| earlier baseline, chi-squared gating off | 75.5% | 60% | 12.0 s |
| cruise 4 m/s (race clock expires) | 57.0% | 0% | — |

Files: [`baseline`](results/2026-10-11_68513f6_baseline.json),
[`no robust-outline check`](results/2026-10-11_68513f6_ablation-no-robust-inner.json),
[`no recovery`](results/2026-10-10_0181f50_ablation-no-recovery.json),
[`no chi2`](results/2026-10-07_c25234a_ablation-no-chi2.json),
[`4 m/s`](results/2026-10-07_c25234a_ablation-4mps.json).

## How it works

```
 camera 40 Hz ─► gate detector ─► IPPE_SQUARE PnP ─► chi-squared gate ─┐
                 (HSV, contours,   (gravity resolves                   │
                  RANSAC quads)     the square's symmetry)              ▼
 IMU 200 Hz ───────────────────────────────────────────────► ESKF in the target gate's frame
                                                                        │
            gate tracking: target / next-gate hypotheses / handoff ◄────┤
                                                                        ▼
                     cubic terminal-velocity guidance ─► attitude control ─► thrust + body rates
```

- **Detection.** Gates are found by colour, then each contour is fitted with a
  quadrilateral. Because every gate is identical, the next gate is usually seen *through* or
  *touching* the one being approached; merged outlines are handled with a RANSAC fit of the
  four dominant edges, and each gate's inner opening is used as a second, independent square.
- **Pose from one square.** OpenCV's `solvePnP` with `SOLVEPNP_IPPE_SQUARE` gives the gate's
  pose from its four corners. A square has 4-fold symmetry, so the corner labelling is fixed
  using gravity (gates are upright); the planar "flip" ambiguity is left to the filter.
- **State estimation without a map.** An error-state Kalman filter (15 states: position,
  velocity, attitude, accelerometer and gyro biases) runs in the frame of the gate the drone
  is flying to. The IMU drives the prediction at 200 Hz; each detection gives a measurement
  of the gate centre in the camera frame and a weaker attitude measurement. Both pass a
  chi-squared test on the innovation, so misdetections and wrong PnP solutions are rejected.
- **Which gate is next?** With no IDs, the rule is "the nearest gate ahead". Gates seen beyond
  the target are clustered into hypotheses while approaching; on passing, the filter is
  re-expressed in the next gate's frame. If a nearer gate appears in front, the drone
  retargets.
- **Guidance and control.** A receding-horizon cubic brings the drone to the gate centre with
  its velocity along the gate normal, choosing its time-to-go so the sideways acceleration it
  needs stays within the drone's limit (braking along the track when it must); a geometric
  attitude controller turns the desired
  acceleration into collective thrust and body-rate commands, as a pilot would in acro mode.

## Simulator

`vdr/sim`: rigid-body quadrotor (thrust + body-rate inputs with motor and rate-loop lags,
rotor and air drag), IMU with white noise and drifting biases, a 320×240 pinhole FPV camera
pitched up 20°, and seeded courses whose layout guarantees that after each gate the next one
is in view and is the nearest gate ahead. The simulator and the flight stack only share the
small contract in [`vdr/interface.py`](vdr/interface.py); a test fails if the autonomy ever
imports the simulator or the contract grows a ground-truth field.

**UDP link mode** (`--link udp`) makes that separation physical: the autonomy runs as its
own process (`vdr/link/autonomy_node.py`, which imports no simulator code) and exchanges
Observations and Commands with the simulator over localhost UDP in lockstep. Arrays travel
as raw bytes, so the 50-run benchmark gives per-run results identical to in-process mode
(`results/2026-10-11_8d3e44c_link-udp.json` vs `_baseline.json`).

## Run it

**Desktop launcher (easiest).** On Windows, double-click `run_ui.bat` (the first run sets up
`.venv` and installs the requirements). Anywhere else, run `python -m vdr.ui` after
`pip install -r requirements.txt`. From the window you can fly a course and watch it live,
save a flight as a GIF, or run a quick benchmark, choosing the course seed, playback speed and
camera noise.

**Command line.**

```bash
pip install -r requirements.txt
python -m pytest -q                         # 73 tests, ~45 s
python -m vdr.harness --runs 50 --seed 0    # the benchmark (~2 min on 2 cores)
python -m vdr.harness --runs 50 --flag chi2_gating=false      # an ablation
python -m vdr.harness --runs 50 --sim pixel_noise_sigma=8     # a harder world
python -m vdr.harness --runs 8 --link udp                     # autonomy in its own process
python -m vdr.flightviz --seed 0 --live                       # watch one flight in a window
```

Every run is deterministic for a given seed and configuration, so a change can be evaluated
by re-running the same 50 courses with it switched on and off.

**Camera-only mode.** `--flag camera_only=true` drops the IMU entirely: between camera
frames the filter predicts motion from the drone's own thrust and body-rate commands through
an assumed vehicle model (rate and thrust lags, linear drag). It flies, but less reliably:

| seed set | camera + IMU (default) | camera only |
|---|---|---|
| 0 | 96.8% | 98.0% |
| 1000 | 100.0% | 96.8% |
| 2000 | 97.2% | 89.5% |
| 3000 | 98.0% | 92.2% |
| pooled (1600 gates) | **98.0%** | **94.1%** |

Files: `results/2026-10-13_db88370_{baseline,ab-camonly}[-seedN].json` (measured before
along-track time-to-go, so the default column is the older 96.8% baseline). Seed set 0 alone
would suggest the opposite; pooled, camera-only loses 20 runs and rescues 5. Its vehicle model
uses the airframe's nominal values; a real drone's model would be less exact.

**Building the app.** `pip install . pyinstaller && python packaging/build.py` makes
`dist/vdr-racing/` for the OS you run it on. Pushing a `v*` tag runs
`.github/workflows/release.yml`, which builds Windows, macOS and Linux and attaches them to a
GitHub release. `python -m vdr.demo --benchmark-out docs/demo/benchmark.js` regenerates the
browser replay's recordings.

## Layout

```
vdr/interface.py   the sim <-> autonomy contract
vdr/sim/           dynamics, IMU, camera renderer, courses, scoring
vdr/autonomy/      detector, PnP, ESKF, guidance/control, the flight stack
vdr/link/          optional UDP link mode (wire format, autonomy process)
vdr/harness.py     regression harness -> results/*.json
vdr/report.py      write-up figures from committed results
vdr/ui.py          desktop launcher (python -m vdr.ui, or run_ui.bat on Windows)
vdr/app.py         `vdr-racing` entry point: launcher, fly, bench, demo-data
vdr/demo.py        records flights for the browser replay (docs/demo/)
packaging/         PyInstaller build of the standalone app
tests/             unit and end-to-end tests
```

[ROADMAP.md](ROADMAP.md) lists what's next.

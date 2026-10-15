"""End-to-end smoke tests: the full stack flies, deterministically."""

from vdr.harness import summarize
from vdr.runner import run_episode
from vdr.sim import SimConfig
from vdr.autonomy.config import AutonomyConfig


def test_episode_is_deterministic():
    a = run_episode(5)
    b = run_episode(5)
    assert a["outcome"] == b["outcome"]
    assert a["gates_passed"] == b["gates_passed"]
    assert a["time"] == b["time"]
    assert a["diag"] == b["diag"]


def test_stack_passes_gates():
    r = run_episode(5)
    assert r["gates_passed"] >= 3, r


def test_summary_arithmetic():
    runs = [
        {"n_gates": 8, "gates_passed": 8, "outcome": "completed", "time": 10.0, "diag": {"pos_err_rms": 0.5}},
        {"n_gates": 8, "gates_passed": 2, "outcome": "crash_gate", "time": 4.0, "diag": {"pos_err_rms": 1.5}},
    ]
    s = summarize(runs)
    assert s["pass_rate"] == 10 / 16
    assert s["completion_rate"] == 0.5
    assert s["per_gate_pass_rate"][:3] == [1.0, 1.0, 0.5]
    assert s["outcomes"]["crash_gate"] == 1
    assert s["mean_lap_time_completed"] == 10.0


def test_lost_target_recovery_rescues_a_phantom_handoff():
    # Seed 15: the next-gate estimate handed over at gate 1 is ~2.8 m too short, so the
    # real gate 2 never matches the target and (without recovery) the drone declares a
    # pass 2.7 m early and clips the frame. Recovery re-anchors on the gate it does see.
    # The scenario was found with the legacy renderer (gate_supersample=0), whose
    # over-wide paint contributed to the short handoff, so the test pins that renderer
    # (and the contour-only corners it was found with; sub-pixel corners became default later),
    # and keeps far lone RANSAC outlines (with them dropped, the default since 2026-10-13,
    # this seed completes all 8 gates without needing a recovery), and the floored
    # time-to-go guidance it was found with (with along_track_tgo, the default since
    # 2026-10-15, the flight path changes and no recovery is needed either).
    r = run_episode(15, sim_cfg=SimConfig(gate_supersample=0),
                    auto_cfg=AutonomyConfig(subpixel_corners=False, drop_lone_robust_far=False,
                                            along_track_tgo=False))
    assert r["diag"]["recoveries"] >= 1, r["diag"]
    assert r["gates_passed"] >= 3, r

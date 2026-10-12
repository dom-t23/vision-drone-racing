"""Desktop launcher: the commands its buttons run are valid for the CLI tools (no window opened)."""

from vdr import flightviz, harness
from vdr.ui import bench_cmd, fly_cmd, gif_cmd


def _cli_args(cmd):
    assert cmd[1] == "-m"
    return cmd[3:]


def test_buttons_build_commands_the_tools_accept(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(flightviz, "render", lambda *a, **k: calls.append((a, k)) or {})
    flightviz.main(_cli_args(fly_cmd(7, 0.5, 2, 4.0, True)))
    flightviz.main(_cli_args(gif_cmd(7, 3, 0.0, False, str(tmp_path / "x.gif"))))
    (a1, k1), (a2, k2) = calls
    assert a1 == (7, "") and k1["live"] and k1["speed"] == 0.5 and k1["stride"] == 2
    assert k1["sim_cfg"].pixel_noise_sigma == 4.0 and k1["sim_cfg"].pixel_noise_fresh
    assert a2 == (7, str(tmp_path / "x.gif")) and not k2["live"] and k2["sim_cfg"].pixel_noise_sigma == 0.0

    cmd = bench_cmd(3, 2, 3.0, False)
    assert "--no-save" in cmd and "--sim" not in cmd       # default settings pass no overrides
    monkeypatch.setattr(harness, "_run", lambda job: {"n_gates": 8, "gates_passed": 8, "outcome": "completed",
                                                      "time": 12.0, "diag": {"pos_err_rms": 0.5}})
    assert harness.main(_cli_args(cmd) + ["--workers", "1", "--quiet"]) == 0

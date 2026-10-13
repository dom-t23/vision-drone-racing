"""Packaged entry point (vdr.app) and the browser-demo exporter (vdr.demo)."""

import json
import sys

from vdr import app, demo


def test_self_command_from_source_and_frozen(monkeypatch):
    assert app.self_command("bench", "--runs", "1") == [sys.executable, "-m", "vdr.app", "bench", "--runs", "1"]
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    assert app.self_command("fly", "--seed", "2") == [sys.executable, "fly", "--seed", "2"]


def test_dispatch(monkeypatch, capsys):
    assert app.main(["nope"]) == 2
    assert app.main(["--help"]) == 0
    called = []
    from vdr import harness
    monkeypatch.setattr(harness, "main", lambda argv: called.append(argv) or 0)
    assert app.main(["bench", "--runs", "3"]) == 0 and called == [["--runs", "3"]]


def test_demo_export(tmp_path):
    out = tmp_path / "flights.js"
    assert demo.main(["--seeds", "1", "--methods", "default", "--stride", "8", "--out", str(out)]) == 0
    text = out.read_text()
    flights = json.loads(text[text.index("=") + 1:].strip().rstrip(";"))
    (f,) = flights
    assert f["seed"] == 1 and len(f["gates"]) == f["n_gates"] and f["frames"]
    fr = f["frames"][-1]
    assert {"t", "p", "e", "m", "g", "q", "img"} <= set(fr)

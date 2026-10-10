"""Flight GIF renderer: smoke test on the first second of one episode."""

from PIL import Image

from vdr.flightviz import render
from vdr.runner import run_episode


def test_frame_stride_keeps_every_nth_frame():
    res = run_episode(0, keep_frames=5, frame_stride=3)
    ts = [t for t, _ in res["frames"]]
    assert len(ts) == 5
    dts = [b - a for a, b in zip(ts, ts[1:])]
    assert max(dts) - min(dts) < 1e-9 and dts[0] > 0


def test_render_writes_animated_gif(tmp_path):
    out = tmp_path / "flight.gif"
    info = render(0, str(out), stride=5, max_seconds=1.0)
    assert out.exists() and info["frames"] >= 5
    im = Image.open(out)
    assert im.n_frames == info["frames"]
    assert im.size[0] > im.size[1]          # camera + top-down side by side

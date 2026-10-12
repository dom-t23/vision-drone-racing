"""Render a flight as an animated GIF: onboard camera (left) + top-down course view (right).

    python -m vdr.flightviz --seed 0 --out docs/img/flight.gif

Uses ground truth (gate positions, true trajectory) for the top-down panel only. This is a
visualisation run from outside the autonomy: nothing here is fed back into the flight stack.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from vdr.runner import run_episode  # noqa: E402
from vdr.sim import SimConfig  # noqa: E402

MODE_COLOURS = {"track": (80, 220, 80), "search": (0, 200, 255), "wait": (200, 200, 200)}


def _hud(img_bgr, text_lines, mode):
    img = img_bgr.copy()
    y = 16
    for line in text_lines:
        cv2.putText(img, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    MODE_COLOURS.get(mode, (255, 255, 255)) if line.startswith("mode") else (255, 255, 255),
                    1, cv2.LINE_AA)
        y += 16
    return img


def _topdown_figure(course, gate_half, height_px):
    C = course.centers()
    pts = np.vstack([C[:, :2], course.start_position[None, :2]])
    lo, hi = pts.min(0) - 3.0, pts.max(0) + 3.0
    # pad the shorter axis so the panel is between 1:1 and 2:1 (no empty bands)
    span = hi - lo
    aspect = float(np.clip(span[0] / span[1], 1.0, 2.0))
    mid = (lo + hi) / 2
    half = np.maximum(span, [span[1] * aspect, span[0] / aspect]) / 2
    lo, hi = mid - half, mid + half
    h_in = height_px / 100.0
    fig = plt.figure(figsize=(h_in * aspect, h_in), dpi=100)
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor("#e9efe6")
    for i, g in enumerate(course.gates):
        left = np.array([-math.sin(g.yaw), math.cos(g.yaw)])
        a = g.center[:2] + gate_half * left
        b = g.center[:2] - gate_half * left
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#f86e18", lw=3, solid_capstyle="butt")
        n = g.normal[:2]
        ax.annotate("", xy=g.center[:2] + 0.9 * n, xytext=g.center[:2],
                    arrowprops=dict(arrowstyle="->", color="#f86e18", lw=0.8))
        ax.text(g.center[0] - 0.9 * n[0], g.center[1] - 0.9 * n[1], str(i + 1), fontsize=7,
                ha="center", va="center", color="#7a3a10")
    (trail,) = ax.plot([], [], color="#1f4e9c", lw=1.4)
    (dot,) = ax.plot([], [], "o", color="#1f4e9c", ms=4)
    return fig, trail, dot


def _build_frames(seed: int, stride: int, max_seconds: float, sim_cfg: SimConfig | None):
    """Fly one episode and return (episode result, [(t, composite PIL image)])."""
    sim_cfg = sim_cfg or SimConfig()
    res = run_episode(seed, sim_cfg=sim_cfg, record=True, keep_frames=100000, frame_stride=stride)
    course = res["course"]
    trace = {round(tr["t"], 6): tr for tr in res["trace"]}
    log_t = np.array([e[0] for e in res["log"]])
    log_p = np.array([e[1] for e in res["log"]])
    log_v = np.array([e[3] for e in res["log"]])
    n_gates = len(course.gates)

    frames = [(t, im) for t, im in res["frames"] if t <= max_seconds]
    if not frames:
        raise RuntimeError(f"seed {seed}: no camera frames recorded within {max_seconds} s")
    h_px = frames[0][1].shape[0]
    fig, trail, dot = _topdown_figure(course, sim_cfg.gate_outer / 2.0, h_px)

    out_imgs = []
    for t, im in frames:
        k = int(np.searchsorted(log_t, t, side="right"))
        k = max(k, 1)
        tr = trace.get(round(t, 6), {})
        mode = tr.get("mode", "wait")
        speed = float(np.linalg.norm(log_v[k - 1]))
        passed = min(tr.get("next_gate", 0), n_gates)
        cam = _hud(im, [f"t {t:5.2f} s   {speed:4.1f} m/s", f"gates {passed}/{n_gates}",
                        f"mode {mode}"], mode)
        trail.set_data(log_p[:k, 0], log_p[:k, 1])
        dot.set_data([log_p[k - 1, 0]], [log_p[k - 1, 1]])
        fig.canvas.draw()
        top = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        cam_rgb = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
        if top.shape[0] != cam_rgb.shape[0]:
            s = cam_rgb.shape[0] / top.shape[0]
            top = cv2.resize(top, (int(round(top.shape[1] * s)), cam_rgb.shape[0]))
        out_imgs.append(Image.fromarray(np.hstack([cam_rgb, top])))
    plt.close(fig)
    return res, [(t, im) for (t, _), im in zip(frames, out_imgs)]


def play_live(frames, speed: float = 1.0, title: str = "vdr flight") -> None:
    """Show composite frames in a window at real-time pace (close the window to stop)."""
    import tkinter as tk

    from PIL import ImageTk

    root = tk.Tk()
    root.title(title)
    label = tk.Label(root)
    label.pack()
    t0 = frames[0][0]

    def show(i, start):
        if i >= len(frames):
            root.title(title + " (finished)")
            return
        t, im = frames[i]
        photo = ImageTk.PhotoImage(im)
        label.configure(image=photo)
        label.image = photo
        nxt = frames[i + 1][0] if i + 1 < len(frames) else t
        delay = max(1, int(1000 * (start + (nxt - t0) / speed - time.monotonic())))
        root.after(delay, show, i + 1, start)

    show(0, time.monotonic())
    root.mainloop()


def render(seed: int, out: str, stride: int = 5, max_seconds: float = 30.0,
           sim_cfg: SimConfig | None = None, live: bool = False, speed: float = 1.0) -> dict:
    res, frames = _build_frames(seed, stride, max_seconds, sim_cfg)
    out_imgs = [im for _, im in frames]
    summary = {"seed": seed, "outcome": res["outcome"], "gates_passed": res["gates_passed"],
               "n_gates": res["n_gates"], "frames": len(out_imgs), "file": out or None}
    if live:
        play_live(frames, speed=speed, title=f"vdr flight - seed {seed}")
        if not out:
            return summary

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    # real-time playback: frame spacing taken from the kept frames' timestamps
    if len(frames) > 1:
        duration_ms = int(round(1000 * float(np.median(np.diff([t for t, _ in frames])))))
    else:
        duration_ms = 1000
    # one shared palette, no dithering: the sensor noise would otherwise make every frame unique
    sample = [out_imgs[i] for i in np.linspace(0, len(out_imgs) - 1, 6).astype(int)]
    montage = Image.fromarray(np.vstack([np.asarray(im) for im in sample]))
    palette = montage.quantize(colors=48, method=Image.Quantize.MEDIANCUT)
    pal = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in out_imgs]
    pal[0].save(out, save_all=True, append_images=pal[1:], duration=duration_ms, loop=0,
                optimize=True)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="docs/img/flight.gif",
                    help="GIF path (with --live, a GIF is only written if --out is given explicitly)")
    ap.add_argument("--live", action="store_true", help="play the flight in a window")
    ap.add_argument("--speed", type=float, default=1.0, help="live playback speed multiplier")
    ap.add_argument("--stride", type=int, default=5, help="keep every Nth camera frame")
    args = ap.parse_args(argv)
    out = args.out
    if args.live and not any(a.startswith("--out") for a in (argv or sys.argv[1:])):
        out = ""
    print(render(args.seed, out, stride=args.stride, live=args.live, speed=args.speed))


if __name__ == "__main__":
    main()

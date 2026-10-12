"""Desktop launcher: fly, record and benchmark the simulator without the command line.

    python -m vdr.ui

A small tkinter window (tkinter ships with Python, no extra dependencies). Each button
runs the existing command-line tools (vdr.flightviz, vdr.harness) as a subprocess and
streams their output into the window, so the UI adds no new code paths to the sim or
the flight stack.
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def _sim_args(noise: float, fresh_noise: bool) -> list:
    """--sim overrides for settings that differ from the defaults (so runs read as standard)."""
    from vdr.sim import SimConfig
    args = []
    if noise != SimConfig().pixel_noise_sigma:
        args += ["--sim", f"pixel_noise_sigma={noise:g}"]
    if fresh_noise:
        args += ["--sim", "pixel_noise_fresh=true"]
    return args


def fly_cmd(seed: int, speed: float, stride: int, noise: float, fresh_noise: bool) -> list:
    """Command that flies one course and plays it in a window."""
    return [sys.executable, "-m", "vdr.flightviz", "--live", "--seed", str(seed),
            "--speed", f"{speed:g}", "--stride", str(stride), *_sim_args(noise, fresh_noise)]


def gif_cmd(seed: int, stride: int, noise: float, fresh_noise: bool, out: str) -> list:
    """Command that flies one course and saves it as an animated GIF."""
    return [sys.executable, "-m", "vdr.flightviz", "--seed", str(seed), "--stride", str(stride),
            "--out", out, *_sim_args(noise, fresh_noise)]


def bench_cmd(seed: int, runs: int, noise: float, fresh_noise: bool) -> list:
    """Command that runs the benchmark harness without writing a results file."""
    return [sys.executable, "-m", "vdr.harness", "--runs", str(runs), "--seed", str(seed),
            "--no-save", *_sim_args(noise, fresh_noise)]


class App:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.root = root
        self.proc = None
        self.lines: queue.Queue = queue.Queue()
        root.title("Vision drone racing")
        root.minsize(560, 420)

        frm = ttk.Frame(root, padding=12)
        frm.pack(fill="both", expand=True)

        opts = ttk.LabelFrame(frm, text="Settings", padding=8)
        opts.pack(fill="x")
        self.seed = tk.IntVar(value=0)
        self.speed = tk.DoubleVar(value=1.0)
        self.stride = tk.IntVar(value=2)
        self.runs = tk.IntVar(value=8)
        self.noise = tk.DoubleVar(value=3.0)
        self.fresh = tk.BooleanVar(value=False)
        rows = [
            ("Course seed", ttk.Spinbox(opts, from_=0, to=99999, textvariable=self.seed, width=8),
             "which random course to fly (same seed = same course)"),
            ("Playback speed", ttk.Spinbox(opts, from_=0.1, to=4.0, increment=0.25, textvariable=self.speed,
                                           width=8), "1 = real time, 0.5 = slow motion"),
            ("Frame stride", ttk.Spinbox(opts, from_=1, to=10, textvariable=self.stride, width=8),
             "show every Nth camera frame (1 = smoothest)"),
            ("Image noise", ttk.Spinbox(opts, from_=0, to=20, increment=1, textvariable=self.noise, width=8),
             "camera pixel noise, 0-255 scale (default 3)"),
            ("Benchmark runs", ttk.Spinbox(opts, from_=1, to=200, textvariable=self.runs, width=8),
             "courses per benchmark (50 = the headline benchmark)"),
        ]
        for r, (label, widget, hint) in enumerate(rows):
            ttk.Label(opts, text=label).grid(row=r, column=0, sticky="w", padx=(0, 8), pady=2)
            widget.grid(row=r, column=1, sticky="w", pady=2)
            ttk.Label(opts, text=hint, foreground="gray").grid(row=r, column=2, sticky="w", padx=8)
        ttk.Checkbutton(opts, text="Fresh noise every frame (slower, more realistic)",
                        variable=self.fresh).grid(row=len(rows), column=0, columnspan=3, sticky="w", pady=(4, 0))

        btns = ttk.Frame(frm, padding=(0, 10))
        btns.pack(fill="x")
        self.buttons = [
            ttk.Button(btns, text="Fly and watch", command=self.fly),
            ttk.Button(btns, text="Save GIF...", command=self.save_gif),
            ttk.Button(btns, text="Run benchmark", command=self.bench),
        ]
        for b in self.buttons:
            b.pack(side="left", padx=(0, 6))
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="right")

        self.status = tk.StringVar(value="Ready.")
        ttk.Label(frm, textvariable=self.status).pack(anchor="w")
        self.log = tk.Text(frm, height=12, wrap="word", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, pady=(4, 0))
        self.log.configure(state="disabled")

        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self._drain)

    # ------------------------------------------------------------- settings
    def _settings(self):
        """Current settings, or None (after showing an error) if a field isn't a valid number."""
        from tkinter import messagebox
        try:
            s = dict(seed=self.seed.get(), speed=self.speed.get(), stride=self.stride.get(),
                     runs=self.runs.get(), noise=self.noise.get(), fresh=self.fresh.get())
        except self.tk.TclError:
            messagebox.showerror("Invalid setting", "Every setting must be a number.")
            return None
        if s["speed"] <= 0 or s["stride"] < 1 or s["runs"] < 1 or s["noise"] < 0 or s["seed"] < 0:
            messagebox.showerror("Invalid setting", "Speed must be > 0, stride and runs >= 1, "
                                                    "seed and noise >= 0.")
            return None
        return s

    # -------------------------------------------------------------- actions
    def fly(self):
        s = self._settings()
        if s:
            self._start(fly_cmd(s["seed"], s["speed"], s["stride"], s["noise"], s["fresh"]),
                        f"Simulating course {s['seed']}... the window opens when the flight is ready.")

    def save_gif(self):
        from tkinter import filedialog
        s = self._settings()
        if not s:
            return
        out = filedialog.asksaveasfilename(defaultextension=".gif", filetypes=[("GIF", "*.gif")],
                                           initialfile=f"flight_seed{s['seed']}.gif")
        if out:
            self._start(gif_cmd(s["seed"], s["stride"], s["noise"], s["fresh"], out),
                        f"Rendering course {s['seed']} to {out}...")

    def bench(self):
        s = self._settings()
        if s:
            self._start(bench_cmd(s["seed"], s["runs"], s["noise"], s["fresh"]),
                        f"Benchmarking {s['runs']} courses from seed {s['seed']}...")

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()

    def close(self):
        self.stop()
        self.root.destroy()

    # ------------------------------------------------------------ subprocess
    def _start(self, cmd: list, status: str):
        if self.proc and self.proc.poll() is None:
            return
        self._write(f"\n$ {' '.join(cmd[1:])}\n")
        self.status.set(status)
        for b in self.buttons:
            b.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self.proc = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, env=env)
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

    def _pump(self, proc):
        for line in proc.stdout:
            self.lines.put(line)
        proc.wait()
        self.lines.put(None)                     # end-of-process marker

    def _drain(self):
        try:
            while True:
                line = self.lines.get_nowait()
                if line is None:
                    code = self.proc.returncode
                    self.status.set("Done." if code == 0 else f"Stopped (exit code {code}).")
                    for b in self.buttons:
                        b.configure(state="normal")
                    self.stop_btn.configure(state="disabled")
                else:
                    self._write(line)
        except queue.Empty:
            pass
        self.root.after(100, self._drain)

    def _write(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")


def main():
    import tkinter as tk
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()

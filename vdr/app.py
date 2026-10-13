"""Single entry point for the installed app and the standalone executables.

    vdr-racing                 open the desktop launcher (same as `python -m vdr.ui`)
    vdr-racing fly [...]       fly one course; `--live` plays it in a window (vdr.flightviz)
    vdr-racing bench [...]     run the benchmark harness (vdr.harness)
    vdr-racing demo-data [...] export recorded flights for the browser demo (vdr.demo)

`python -m vdr.app ...` does the same from a source checkout. In a frozen executable
(PyInstaller) there is no `python -m`, so the launcher re-runs this same executable with
a sub-command instead; see `self_command`.
"""

from __future__ import annotations

import multiprocessing
import sys

SUBCOMMANDS = {
    "fly": "vdr.flightviz",
    "bench": "vdr.harness",
    "demo-data": "vdr.demo",
}


def frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def self_command(sub: str, *args: str) -> list:
    """argv that runs a sub-command of this app, from source or from a frozen executable."""
    if frozen():
        return [sys.executable, sub, *args]
    return [sys.executable, "-m", "vdr.app", sub, *args]


def main(argv=None) -> int:
    multiprocessing.freeze_support()        # harness worker processes in frozen builds
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        from vdr import ui
        ui.main()
        return 0
    sub, rest = argv[0], argv[1:]
    if sub in ("-h", "--help"):
        print(__doc__)
        return 0
    if sub not in SUBCOMMANDS:
        print(f"unknown command {sub!r}; expected one of: {', '.join(SUBCOMMANDS)}", file=sys.stderr)
        return 2
    import importlib
    mod = importlib.import_module(SUBCOMMANDS[sub])
    return int(mod.main(rest) or 0)


if __name__ == "__main__":
    sys.exit(main())

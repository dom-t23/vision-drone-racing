"""Build the standalone app with PyInstaller: dist/vdr-racing/ (+ a zip for release).

    pip install pyinstaller
    python packaging/build.py

Runs on Windows, macOS and Linux; each OS builds its own executable
(.github/workflows/release.yml does all three on a version tag).
"""

from __future__ import annotations

import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NAME = "vdr-racing"


def main() -> int:
    cmd = [
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--onedir",
        "--name", NAME,
        "--distpath", str(ROOT / "dist"), "--workpath", str(ROOT / "build"),
        "--specpath", str(ROOT / "build"),
        "--paths", str(ROOT),
        # the app needs numpy, cv2, PIL, matplotlib (flight view) and tkinter; keep the rest out
        "--exclude-module", "scipy",
        "--exclude-module", "pytest", "--exclude-module", "IPython",
        "--hidden-import", "vdr.flightviz", "--hidden-import", "vdr.harness",
        "--hidden-import", "vdr.ui", "--hidden-import", "vdr.demo",
        str(ROOT / "packaging" / "vdr_racing.py"),
    ]
    subprocess.check_call(cmd, cwd=ROOT)
    system = {"Darwin": "macos", "Windows": "windows"}.get(platform.system(), "linux")
    archive = shutil.make_archive(str(ROOT / "dist" / f"{NAME}-{system}-{platform.machine().lower()}"),
                                  "zip", root_dir=ROOT / "dist", base_dir=NAME)
    print("built", archive)
    return 0


if __name__ == "__main__":
    sys.exit(main())

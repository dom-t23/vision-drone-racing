"""PyInstaller entry script for the standalone executable (see packaging/build.py)."""

import sys

from vdr.app import main

if __name__ == "__main__":
    sys.exit(main())

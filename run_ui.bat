@echo off
rem Double-click to open the simulator launcher (python -m vdr.ui).
rem First run: creates a virtual environment in .venv and installs the requirements.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Setting up .venv - first run only, this takes a minute...
    python -m venv .venv || (echo Python 3.10+ is required: https://www.python.org/downloads/ & pause & exit /b 1)
    ".venv\Scripts\python.exe" -m pip install -q -r requirements.txt || (pause & exit /b 1)
)
start "" ".venv\Scripts\pythonw.exe" -m vdr.ui

import re
from pathlib import Path

from vdr.harness import library_versions

REQUIREMENTS = Path(__file__).resolve().parent.parent / "requirements.txt"


def test_installed_versions_match_pins():
    pins = dict(re.findall(r"^(numpy|opencv-python-headless)==(\S+)$", REQUIREMENTS.read_text(), re.M))
    versions = library_versions()
    assert versions["numpy"] == pins["numpy"]
    # cv2 reports 5.0.0 for the wheel 5.0.0.93
    assert pins["opencv-python-headless"].startswith(versions["opencv"] + ".")

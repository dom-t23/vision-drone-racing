"""The autonomy stack. It may only use vdr.interface, vdr.geometry and vdr.cfgutil
from the rest of the project -- never vdr.sim (enforced by tests/test_separation.py)."""

from vdr.autonomy.config import AutonomyConfig
from vdr.autonomy.stack import Autonomy

__all__ = ["Autonomy", "AutonomyConfig"]

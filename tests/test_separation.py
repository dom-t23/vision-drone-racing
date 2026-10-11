"""Enforces the HARD SEPARATION RULE between simulator and autonomy.

The autonomy may receive ONLY camera images, IMU readings, the known gate
size, camera intrinsics/extrinsics and vehicle limits. These tests pin the
data contract and the import graph. If one of them fails, the fix is almost
never to edit the test -- it is to stop leaking ground truth.
"""

import ast
import dataclasses
import inspect
from pathlib import Path

import numpy as np

from vdr.interface import Command, Observation, VehicleInfo

AUTONOMY_DIR = Path(__file__).resolve().parents[1] / "vdr" / "autonomy"

# The complete, allowed contents of the contract. Adding a field here must be a
# deliberate decision that the new field is legitimate sensor data or a vehicle spec.
ALLOWED_OBSERVATION = {"t", "accel", "gyro", "image"}
ALLOWED_VEHICLE_INFO = {
    "K", "image_width", "image_height",          # camera intrinsics
    "R_bc", "t_bc",                              # camera extrinsics
    "gate_outer_size", "gate_inner_size",        # known gate geometry
    "thrust_max", "rate_max",                    # vehicle limits
    "control_dt", "gravity",                     # timing / physics constant
}
ALLOWED_COMMAND = {"thrust", "rates"}
ALLOWED_IMPORTS = ("vdr.autonomy", "vdr.interface", "vdr.geometry", "vdr.cfgutil")


def _field_names(cls):
    return {f.name for f in dataclasses.fields(cls)}


def test_observation_carries_only_sensor_data():
    assert _field_names(Observation) == ALLOWED_OBSERVATION


def test_vehicle_info_carries_only_calibration_and_limits():
    assert _field_names(VehicleInfo) == ALLOWED_VEHICLE_INFO


def test_command_is_thrust_and_body_rates():
    assert _field_names(Command) == ALLOWED_COMMAND


def test_autonomy_never_imports_the_simulator():
    offenders = []
    for path in AUTONOMY_DIR.rglob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:          # relative imports stay inside vdr.autonomy
                    continue
                names = [node.module or ""]
            for n in names:
                if n == "vdr" or (n.startswith("vdr") and not n.startswith(ALLOWED_IMPORTS)):
                    offenders.append(f"{path.name}: {n}")
    assert not offenders, f"autonomy imports outside its allowed set: {offenders}"


def test_autonomy_api_takes_only_the_contract():
    from vdr.autonomy import Autonomy

    init = inspect.signature(Autonomy.__init__)
    assert list(init.parameters) == ["self", "info", "cfg"]
    step = inspect.signature(Autonomy.step)
    assert list(step.parameters) == ["self", "obs"]


def test_simulator_observations_hold_no_extra_data():
    """What the sim actually hands over at runtime matches the contract."""
    from vdr.sim import SimConfig, Simulator
    from vdr.interface import Command

    sim = Simulator(SimConfig(), seed=0)
    obs = sim.reset()
    assert set(vars(obs)) == ALLOWED_OBSERVATION
    assert isinstance(obs.image, np.ndarray) and obs.image.dtype == np.uint8 and obs.image.ndim == 3
    for _ in range(3):
        obs = sim.step(Command.hover())
        assert set(vars(obs)) == ALLOWED_OBSERVATION
        assert obs.accel.shape == (3,) and obs.gyro.shape == (3,)


LINK_AUTONOMY_SIDE = ("autonomy_node.py", "wire.py", "udp.py", "__init__.py")


def test_link_autonomy_process_never_imports_the_simulator():
    """UDP link mode: the autonomy process's modules import nothing from vdr.sim/runner."""
    link_dir = AUTONOMY_DIR.parent / "link"
    allowed = ALLOWED_IMPORTS + ("vdr.link.wire", "vdr.link.udp", "vdr.link")
    offenders = []
    for name in LINK_AUTONOMY_SIDE:
        tree = ast.parse((link_dir / name).read_text())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for n in names:
                if n == "vdr" or (n.startswith("vdr") and not n.startswith(allowed)):
                    offenders.append(f"{name}: {n}")
    assert not offenders, f"autonomy-side link modules import outside the allowed set: {offenders}"


def test_link_autonomy_process_runtime_imports():
    """Importing the autonomy process's entry point pulls in no simulator module at all."""
    import subprocess
    import sys

    code = ("import sys, vdr.link.autonomy_node; "
            "bad = [m for m in sys.modules if m.startswith(('vdr.sim', 'vdr.runner', 'vdr.harness'))]; "
            "print(bad); sys.exit(1 if bad else 0)")
    r = subprocess.run([sys.executable, "-c", code], cwd=AUTONOMY_DIR.parents[1], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_link_wire_messages_are_built_from_the_contract():
    """The wire format encodes exactly the interface dataclasses' fields."""
    from vdr.link import wire
    from vdr.sim import SimConfig, Simulator

    sim = Simulator(SimConfig(), seed=0)
    assert set(wire.decode(wire.encode_info(sim.vehicle_info()))) == ALLOWED_VEHICLE_INFO
    assert set(wire.decode(wire.encode_obs(sim.reset()))) == ALLOWED_OBSERVATION
    assert set(wire.decode(wire.encode_cmd(Command.hover()))["cmd"]) == ALLOWED_COMMAND

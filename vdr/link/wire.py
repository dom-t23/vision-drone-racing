"""Byte encoding of the sim <-> autonomy contract.

A message is a small JSON header followed by the raw bytes of any numpy arrays,
so floats travel bit-exact (float64 in, float64 out) and a run over UDP is
identical to a run in-process. Contract messages are built from the
dataclasses' own field lists: decoding passes fields straight to the
dataclass constructor, so a message carrying anything outside vdr.interface
fails to decode rather than leaking through.

Only vdr.interface and the standard library / numpy are imported here, so both
processes can use it.
"""

from __future__ import annotations

import dataclasses
import json
import struct

import numpy as np

from vdr.interface import Command, Observation, VehicleInfo

# message types
HELLO, INFO, OBS, CMD, BYE, STATS = range(1, 7)
TYPE_NAMES = {HELLO: "hello", INFO: "info", OBS: "obs", CMD: "cmd", BYE: "bye", STATS: "stats"}

_LEN = struct.Struct("!I")


def _pack(obj, blobs: list):
    if isinstance(obj, np.ndarray):
        a = np.ascontiguousarray(obj)
        blobs.append(a.tobytes())
        return {"__nd__": [a.dtype.str, list(a.shape), len(blobs) - 1]}
    if isinstance(obj, dict):
        return {k: _pack(v, blobs) for k, v in obj.items()}
    if isinstance(obj, tuple):
        return {"__tuple__": [_pack(v, blobs) for v in obj]}
    if isinstance(obj, list):
        return [_pack(v, blobs) for v in obj]
    if isinstance(obj, (np.floating, np.integer, np.bool_)):
        return obj.item()
    return obj


def _unpack(obj, blobs: list):
    if isinstance(obj, dict):
        if "__nd__" in obj:
            dtype, shape, i = obj["__nd__"]
            return np.frombuffer(blobs[i], dtype=np.dtype(dtype)).reshape(shape).copy()
        if "__tuple__" in obj:
            return tuple(_unpack(v, blobs) for v in obj["__tuple__"])
        return {k: _unpack(v, blobs) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_unpack(v, blobs) for v in obj]
    return obj


def encode(payload: dict) -> bytes:
    """dict (JSON-able values, numpy arrays, tuples) -> bytes."""
    blobs: list = []
    head = json.dumps(_pack(payload, blobs), separators=(",", ":")).encode()
    parts = [_LEN.pack(len(head)), head, _LEN.pack(len(blobs))]
    for b in blobs:
        parts += [_LEN.pack(len(b)), b]
    return b"".join(parts)


def decode(data: bytes) -> dict:
    off = 0

    def take(n):
        nonlocal off
        out = data[off:off + n]
        if len(out) != n:
            raise ValueError("truncated message")
        off += n
        return out

    (n_head,) = _LEN.unpack(take(4))
    head = json.loads(take(n_head))
    (n_blobs,) = _LEN.unpack(take(4))
    blobs = []
    for _ in range(n_blobs):
        (n,) = _LEN.unpack(take(4))
        blobs.append(take(n))
    return _unpack(head, blobs)


def _fields(obj) -> dict:
    return {f.name: getattr(obj, f.name) for f in dataclasses.fields(obj)}


def encode_info(info: VehicleInfo) -> bytes:
    return encode(_fields(info))


def decode_info(data: bytes) -> VehicleInfo:
    return VehicleInfo(**decode(data))


def encode_obs(obs: Observation) -> bytes:
    return encode(_fields(obs))


def decode_obs(data: bytes) -> Observation:
    return Observation(**decode(data))


def encode_cmd(cmd: Command, telemetry: dict | None = None) -> bytes:
    """A Command plus optional autonomy telemetry (the filter's own estimate, for the
    runner's diagnostics only -- it flows autonomy -> sim and is never fed back)."""
    return encode({"cmd": _fields(cmd), "telemetry": telemetry})


def decode_cmd(data: bytes):
    d = decode(data)
    return Command(**d["cmd"]), d["telemetry"]

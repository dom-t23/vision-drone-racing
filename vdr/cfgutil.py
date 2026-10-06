"""Tiny helper to override dataclass configs from command-line strings."""

from __future__ import annotations

from dataclasses import asdict, fields, replace


def override(cfg, **kwargs):
    """Return a copy of dataclass `cfg` with fields replaced.

    String values are cast to the field's declared type, so the harness can
    pass `--flag chi2_gating=false` or `--sim pixel_noise_sigma=6`.
    """
    types = {f.name: f.type for f in fields(cfg)}
    changes = {}
    for k, v in kwargs.items():
        if k not in types:
            raise KeyError(f"unknown field for {type(cfg).__name__}: {k}")
        changes[k] = cast(v, types[k], getattr(cfg, k))
    return replace(cfg, **changes)


def cast(v, typ, current=None):
    if not isinstance(v, str):
        return v
    name = typ if isinstance(typ, str) else getattr(typ, "__name__", str(typ))
    if current is not None and not isinstance(current, str):
        name = type(current).__name__
    if name == "bool":
        low = v.lower()
        if low in ("1", "true", "yes", "on"):
            return True
        if low in ("0", "false", "no", "off"):
            return False
        raise ValueError(f"not a bool: {v}")
    if name == "int":
        return int(v)
    if name == "float":
        return float(v)
    return v


def as_dict(cfg) -> dict:
    return asdict(cfg)

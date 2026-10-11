"""Simulator side of UDP link mode: UdpAutonomy stands in for Autonomy in the runner.

It spawns the autonomy process, sends it the VehicleInfo, and then exchanges one
Observation for one Command per control tick. step()/debug()/stats/passes mirror
the in-process Autonomy so vdr.runner can use either.
"""

from __future__ import annotations

import dataclasses
import json
import subprocess
import sys
from pathlib import Path

from vdr.interface import Command, Observation, VehicleInfo
from vdr.link import wire
from vdr.link.udp import LinkTimeout, UdpEndpoint

REPO = Path(__file__).resolve().parents[2]


class LinkError(RuntimeError):
    pass


class UdpAutonomy:
    def __init__(self, info: VehicleInfo, cfg=None, reply_timeout: float = 2.0, max_retries: int = 5,
                 start_timeout: float = 30.0):
        self.reply_timeout = reply_timeout
        self.max_retries = max_retries
        self.ep = UdpEndpoint()
        host, port = self.ep.address
        cfg_json = json.dumps(dataclasses.asdict(cfg) if cfg is not None else {})
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "vdr.link.autonomy_node", "--sim", f"{host}:{port}", "--config", cfg_json],
            cwd=str(REPO), stdin=subprocess.DEVNULL)
        self.seq = -1
        self.retries = 0
        self._telemetry = dict(mode="wait", initialized=False, p=None, R=None)
        self.stats: dict = {}
        self.passes = 0
        try:
            self._handshake(info, start_timeout)
        except BaseException:
            self.close()
            raise

    def _handshake(self, info: VehicleInfo, timeout: float) -> None:
        try:
            _, _, _, addr = self._recv_type(wire.HELLO, timeout)
        except LinkTimeout:
            raise LinkError("autonomy process did not say hello") from None
        self.ep.peer = addr
        self._info_msg = wire.encode_info(info)
        self.ep.send(wire.INFO, 0, self._info_msg)

    def _recv_type(self, mtype: int, timeout: float, seq: int | None = None):
        while True:
            if self.proc.poll() is not None:
                raise LinkError(f"autonomy process exited with code {self.proc.returncode}")
            m, s, payload, addr = self.ep.recv(timeout)
            if m == wire.HELLO and mtype != wire.HELLO:
                self.ep.send(wire.INFO, 0, self._info_msg)   # our INFO was lost; resend
                continue
            if m == mtype and (seq is None or s == seq):
                return m, s, payload, addr

    def step(self, obs: Observation) -> Command:
        self.seq += 1
        payload = wire.encode_obs(obs)
        for attempt in range(self.max_retries + 1):
            self.ep.send(wire.OBS, self.seq, payload)
            try:
                _, _, reply, _ = self._recv_type(wire.CMD, self.reply_timeout, self.seq)
            except LinkTimeout:
                self.retries += 1
                continue
            cmd, tel = wire.decode_cmd(reply)
            if tel is not None:
                self._telemetry = tel
            return cmd
        raise LinkError(f"no command for observation {self.seq} after {self.max_retries} retries")

    def debug(self) -> dict:
        return dict(self._telemetry)

    def finish(self) -> None:
        """Collect the autonomy's counters (diagnostics) and shut it down."""
        for _ in range(self.max_retries + 1):
            self.ep.send(wire.BYE, self.seq + 1, b"")
            try:
                _, _, payload, _ = self._recv_type(wire.STATS, self.reply_timeout)
            except LinkTimeout:
                continue
            d = wire.decode(payload)
            self.stats, self.passes = d["stats"], d["passes"]
            break
        self.close()

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.ep.close()

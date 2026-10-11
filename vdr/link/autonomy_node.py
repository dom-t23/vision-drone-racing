"""The autonomy process for UDP link mode.

    python -m vdr.link.autonomy_node --sim 127.0.0.1:PORT [--config JSON]

Imports only the autonomy, the interface and the wire format -- never vdr.sim.
Protocol (lockstep):
    autonomy -> sim   HELLO
    sim -> autonomy   INFO   VehicleInfo
    sim -> autonomy   OBS n  Observation          (repeated if CMD n was lost)
    autonomy -> sim   CMD n  Command (+ telemetry) (cached; resent for a repeated OBS n)
    sim -> autonomy   BYE
    autonomy -> sim   STATS  counters for the run's diagnostics
"""

from __future__ import annotations

import argparse
import json
import sys

from vdr.autonomy import Autonomy, AutonomyConfig
from vdr.link import wire
from vdr.link.udp import LinkTimeout, UdpEndpoint


def _telemetry(auto: Autonomy) -> dict:
    d = auto.debug()
    return dict(mode=d["mode"], initialized=bool(d["initialized"]), p=d["p"], R=d["R"])


def serve(sim_addr: tuple, cfg: AutonomyConfig, idle_timeout: float = 30.0, hello_every: float = 0.2) -> int:
    ep = UdpEndpoint()
    ep.peer = sim_addr
    # handshake: say hello until the simulator sends the vehicle info
    auto = None
    waited = 0.0
    while auto is None:
        ep.send(wire.HELLO, 0, b"")
        try:
            mtype, _, payload, _ = ep.recv(hello_every)
        except LinkTimeout:
            waited += hello_every
            if waited > idle_timeout:
                return 2
            continue
        if mtype == wire.INFO:
            auto = Autonomy(wire.decode_info(payload), cfg)

    last_seq, last_reply = -1, None
    while True:
        try:
            mtype, seq, payload, _ = ep.recv(idle_timeout)
        except LinkTimeout:
            return 3                     # simulator went away
        if mtype == wire.OBS:
            if seq == last_seq and last_reply is not None:
                ep.send(wire.CMD, seq, last_reply)            # lost reply: resend, don't re-step
                continue
            if seq != last_seq + 1:
                continue                                      # stale duplicate
            obs = wire.decode_obs(payload)
            cmd = auto.step(obs)
            tel = _telemetry(auto) if obs.image is not None else None
            last_seq, last_reply = seq, wire.encode_cmd(cmd, tel)
            ep.send(wire.CMD, seq, last_reply)
        elif mtype == wire.BYE:
            ep.send(wire.STATS, seq, wire.encode(dict(stats=auto.stats, passes=auto.passes)))
            # stay briefly in case STATS is lost and BYE is repeated
            try:
                while True:
                    mtype, seq, _, _ = ep.recv(0.3)
                    if mtype == wire.BYE:
                        ep.send(wire.STATS, seq, wire.encode(dict(stats=auto.stats, passes=auto.passes)))
            except LinkTimeout:
                return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", required=True, help="simulator HOST:PORT")
    ap.add_argument("--config", default="{}", help="AutonomyConfig fields as JSON")
    a = ap.parse_args(argv)
    host, port = a.sim.rsplit(":", 1)
    try:
        import cv2
        cv2.setNumThreads(1)
    except Exception:
        pass
    cfg = AutonomyConfig().override(**json.loads(a.config))
    return serve((host, int(port)), cfg)


if __name__ == "__main__":
    sys.exit(main())

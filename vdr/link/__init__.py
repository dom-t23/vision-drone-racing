"""Optional UDP link mode: simulator and autonomy as separate processes.

    python -m vdr.harness --runs 8 --link udp

The simulator side (vdr.runner) spawns `python -m vdr.link.autonomy_node`, which
owns the Autonomy and talks to the simulator only through localhost UDP
datagrams carrying the vdr.interface contract (VehicleInfo once, then
Observation -> Command every control tick, in lockstep). The autonomy process
never imports vdr.sim (enforced by tests/test_separation.py), so this mode
demonstrates the separation physically: the only thing that crosses the
process boundary is what the wire format can encode.

Modules:
    wire.py           byte encoding of the contract (+ autonomy telemetry, diagnostics only)
    udp.py            fragmenting datagram endpoint (an image is ~230 kB, a datagram <= 64 kB)
    autonomy_node.py  the autonomy process
    sim_side.py       UdpAutonomy: a drop-in stand-in for Autonomy used by the runner
"""

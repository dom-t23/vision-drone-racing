"""UDP link mode (vdr.link): the wire format and the separate-process lockstep."""

import numpy as np
import pytest

from vdr.interface import Command, Observation
from vdr.link import wire
from vdr.link.sim_side import UdpAutonomy
from vdr.link.udp import FRAG_BYTES, UdpEndpoint
from vdr.runner import run_episode
from vdr.sim import SimConfig, Simulator


def test_contract_round_trips_bit_exact():
    sim = Simulator(SimConfig(), seed=3)
    info = sim.vehicle_info()
    info2 = wire.decode_info(wire.encode_info(info))
    assert np.array_equal(info2.K, info.K) and info2.K.dtype == info.K.dtype
    assert info2.rate_max == info.rate_max and isinstance(info2.rate_max, tuple)
    assert info2.control_dt == info.control_dt

    obs = sim.reset()
    obs2 = wire.decode_obs(wire.encode_obs(obs))
    assert obs2.t == obs.t
    assert np.array_equal(obs2.accel, obs.accel) and np.array_equal(obs2.gyro, obs.gyro)
    assert obs2.image.dtype == np.uint8 and np.array_equal(obs2.image, obs.image)
    no_img = wire.decode_obs(wire.encode_obs(Observation(t=0.1, accel=obs.accel, gyro=obs.gyro)))
    assert no_img.image is None

    cmd = Command(thrust=9.8100000000001, rates=np.array([0.1, -1e-17, 3.0]))
    cmd2, tel = wire.decode_cmd(wire.encode_cmd(cmd, {"mode": "track", "p": np.ones(3)}))
    assert cmd2.thrust == cmd.thrust and np.array_equal(cmd2.rates, cmd.rates)
    assert tel["mode"] == "track" and np.array_equal(tel["p"], np.ones(3))


def test_wire_rejects_fields_outside_the_contract():
    sim = Simulator(SimConfig(), seed=0)
    obs = sim.reset()
    leaky = wire.encode(dict(t=obs.t, accel=obs.accel, gyro=obs.gyro, image=None, gate_pos=np.zeros(3)))
    with pytest.raises(TypeError):
        wire.decode_obs(leaky)


def test_large_messages_are_fragmented_and_reassembled():
    a, b = UdpEndpoint(), UdpEndpoint()
    try:
        payload = np.random.default_rng(0).integers(0, 256, 3 * FRAG_BYTES + 123, dtype=np.uint8).tobytes()
        a.send(wire.OBS, 7, payload, b.address)
        mtype, seq, got, _ = b.recv(2.0)
        assert (mtype, seq, got) == (wire.OBS, 7, payload)
        assert a.datagrams_sent == 4
    finally:
        a.close()
        b.close()


def test_udp_episode_is_identical_to_in_process():
    a = run_episode(5)
    b = run_episode(5, link="udp")
    assert b["diag"].pop("link_retries") == 0
    assert a == b


class _LossyUdpAutonomy(UdpAutonomy):
    """Drops some outgoing observation datagrams and some incoming commands."""

    def __init__(self, *args, **kw):
        super().__init__(*args, reply_timeout=0.3, **kw)
        send, recv = self.ep.send, self.ep.recv
        self.dropped_obs = self.dropped_cmd = 0

        def lossy_send(mtype, seq, payload, addr=None):
            if mtype == wire.OBS and seq % 37 == 5 and seq not in self._obs_dropped:
                self._obs_dropped.add(seq)
                self.dropped_obs += 1
                return                              # the observation is lost (retransmit goes through)
            send(mtype, seq, payload, addr)

        def lossy_recv(timeout):
            m = recv(timeout)
            if m[0] == wire.CMD and m[1] % 41 == 9 and m[1] not in self._cmd_dropped:
                self._cmd_dropped.add(m[1])
                self.dropped_cmd += 1
                return lossy_recv(timeout)          # the reply is lost
            return m

        self._obs_dropped = set()
        self._cmd_dropped = set()
        self.ep.send, self.ep.recv = lossy_send, lossy_recv


def test_lockstep_survives_lost_datagrams_without_double_stepping():
    sim = Simulator(SimConfig(), seed=5)
    ref = Simulator(SimConfig(), seed=5)
    from vdr.autonomy import Autonomy
    auto_ref = Autonomy(ref.vehicle_info())
    lossy = _LossyUdpAutonomy(sim.vehicle_info())
    try:
        obs, obs_ref = sim.reset(), ref.reset()
        for _ in range(120):
            c, c_ref = lossy.step(obs), auto_ref.step(obs_ref)
            assert c.thrust == c_ref.thrust and np.array_equal(c.rates, c_ref.rates)
            obs, obs_ref = sim.step(c), ref.step(c_ref)
    finally:
        lossy.finish()
    assert lossy.dropped_obs >= 2 and lossy.dropped_cmd >= 2
    assert lossy.retries == lossy.dropped_obs + lossy.dropped_cmd
    assert lossy.stats["frames"] == auto_ref.stats["frames"]

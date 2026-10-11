"""A datagram endpoint that sends messages larger than one UDP datagram.

Each message (type, seq, payload) is split into fragments of at most
FRAG_BYTES; the receiver reassembles them and returns whole messages only.
Loss is handled one level up by the lockstep protocol: the simulator resends
an observation if its command doesn't arrive in time, and the autonomy answers
a repeated observation with its cached command (it never steps twice).
"""

from __future__ import annotations

import socket
import struct
import time
from typing import Optional

MAGIC = b"VDRL"
_HDR = struct.Struct("!4sBIHH")      # magic, msg type, seq, fragment index, fragment count
FRAG_BYTES = 60000                   # payload per datagram (< 65507 for IPv4 UDP)


class LinkTimeout(Exception):
    pass


class UdpEndpoint:
    def __init__(self, host: str = "127.0.0.1", port: int = 0, buf_bytes: int = 4 << 20):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for opt in (socket.SO_RCVBUF, socket.SO_SNDBUF):
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, opt, buf_bytes)
            except OSError:
                pass
        self.sock.bind((host, port))
        self.peer: Optional[tuple] = None
        self._partial: dict = {}          # (type, seq) -> {index: bytes}
        self.datagrams_sent = 0
        self.datagrams_dropped = 0        # malformed / stale fragments discarded

    @property
    def address(self) -> tuple:
        return self.sock.getsockname()

    def send(self, mtype: int, seq: int, payload: bytes, addr: Optional[tuple] = None) -> None:
        addr = addr or self.peer
        n = max(1, -(-len(payload) // FRAG_BYTES))
        if n > 0xFFFF:
            raise ValueError("message too large")
        for i in range(n):
            chunk = payload[i * FRAG_BYTES:(i + 1) * FRAG_BYTES]
            self.sock.sendto(_HDR.pack(MAGIC, mtype, seq, i, n) + chunk, addr)
            self.datagrams_sent += 1

    def recv(self, timeout: float):
        """Next whole message as (type, seq, payload, addr); raises LinkTimeout."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise LinkTimeout()
            self.sock.settimeout(left)
            try:
                data, addr = self.sock.recvfrom(FRAG_BYTES + _HDR.size + 64)
            except socket.timeout:
                raise LinkTimeout() from None
            if len(data) < _HDR.size:
                self.datagrams_dropped += 1
                continue
            magic, mtype, seq, i, n = _HDR.unpack_from(data)
            if magic != MAGIC or n == 0 or i >= n:
                self.datagrams_dropped += 1
                continue
            body = data[_HDR.size:]
            if n == 1:
                return mtype, seq, body, addr
            key = (mtype, seq)
            parts = self._partial.setdefault(key, {})
            parts[i] = body
            if len(parts) == n:
                del self._partial[key]
                # anything older than a completed message can never complete usefully
                for k in [k for k in self._partial if k[1] < seq]:
                    self.datagrams_dropped += len(self._partial.pop(k))
                return mtype, seq, b"".join(parts[j] for j in range(n)), addr

    def close(self) -> None:
        self.sock.close()

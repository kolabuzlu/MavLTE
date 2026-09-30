#!/usr/bin/env python3
"""mavrelay - the relay and command line tools of MavLTE, a MAVLink link over 4G/LTE.

    flight controller -UART- ESP32 + 4G modem ==UDP==> mavrelay server <==UDP== GCS agent <-- Mission Planner
                                                       (public server)          (MavLTE app or mavrelay gcs)

Subcommands:
    server   Public relay. Authenticates the vehicle and GCS agents and forwards MAVLink between them.
    gcs      Runs next to Mission Planner / QGroundControl and gives them a local UDP and TCP port.
    vehicle  Vehicle side in Python (serial port or SITL), for bench tests without the ESP32.
    genkey   Print a new random key.

Needs only the Python standard library (3.8+). `vehicle --serial` also needs pyserial.
The wire protocol is described in docs/PROTOCOL.md.
"""

from __future__ import annotations

import argparse
import asyncio
import configparser
import hashlib
import hmac
import ipaddress
import logging
import secrets
import socket
import struct
import sys
import threading
import time
from collections import Counter, deque
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

__version__ = "1.1.5"

log = logging.getLogger("mavrelay")
slog = log.getChild("relay")  # one logger per role, so combined logs (sitl_demo.py) stay readable
glog = log.getChild("gcs")
vlog = log.getChild("vehicle")

# ---------------------------------------------------------------------------------------------
# Wire protocol (docs/PROTOCOL.md)

MAGIC = 0xA5
VERSION = 1
HEADER = struct.Struct("<BBBBII")  # magic, version, type, role, session, seq
TAG_LEN = 16
NONCE_LEN = 8
INFO_MAX = 64
MAX_DATAGRAM = 1200  # IP packet stays below 1280 bytes, so it is never fragmented
MAX_PAYLOAD = MAX_DATAGRAM - HEADER.size - TAG_LEN

HELLO, WELCOME, DATA, PING, PONG, REJECT, STATUS = range(1, 8)
ROLE_SERVER, ROLE_VEHICLE, ROLE_GCS = 0, 1, 2
ROLE_NAMES = {ROLE_SERVER: "server", ROLE_VEHICLE: "vehicle", ROLE_GCS: "gcs"}

PING_BODY = struct.Struct("<IHHhBB")  # t_ms, rtt_ms, rx_loss_permille, rssi_dbm, rat, reserved
PONG_BODY = struct.Struct("<IB")  # t_ms (echoed), flags
STATUS_BODY = struct.Struct("<BBHHHhH")  # flags, rat, rtt_ms, up_loss, down_loss, rssi_dbm, idle_ms

U16_UNKNOWN = 0xFFFF
RSSI_UNKNOWN = 0x7FFF
RAT_UNKNOWN = 0xFF
PONG_GCS_PRESENT = 0x01
PING_FLAG_WATCHING = 0x01  # GCS agent: only watching the vehicle's link, send it STATUS but no telemetry
STATUS_VEHICLE_ONLINE = 0x01
REJECT_UNKNOWN_SESSION = 1
NO_VEHICLE_STATUS = STATUS_BODY.pack(0, RAT_UNKNOWN, U16_UNKNOWN, U16_UNKNOWN, U16_UNKNOWN, RSSI_UNKNOWN, U16_UNKNOWN)

# 3GPP TS 27.007 <AcT> values
RAT_NAMES = {0: "GSM", 1: "GSM", 2: "3G", 3: "EDGE", 4: "HSDPA", 5: "HSUPA", 6: "HSPA", 7: "LTE"}

HELLO_INTERVAL = 1.0
PING_INTERVAL = 1.0
LINK_TIMEOUT = 10.0  # client: no valid packet from the server for this long -> new session


class Packet(NamedTuple):
    type: int
    role: int
    session: int
    seq: int
    body: bytes


def compute_tag(key: bytes, msg: bytes) -> bytes:
    return hmac.new(key, msg, hashlib.sha256).digest()[:TAG_LEN]


def encode(key: bytes, ptype: int, role: int, session: int, seq: int, body: bytes = b"") -> bytes:
    msg = HEADER.pack(MAGIC, VERSION, ptype, role, session, seq) + bytes(body)
    return msg + compute_tag(key, msg)


def decode(data: bytes) -> Optional[Packet]:
    """Split a datagram into its fields, or return None if it is not ours. Does not check the tag."""
    if len(data) < HEADER.size + TAG_LEN:
        return None
    magic, version, ptype, role, session, seq = HEADER.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        return None
    return Packet(ptype, role, session, seq, bytes(data[HEADER.size:-TAG_LEN]))


def verify(key: bytes, data: bytes) -> bool:
    return hmac.compare_digest(compute_tag(key, data[:-TAG_LEN]), data[-TAG_LEN:])


class ReplayWindow:
    """Anti-replay window over the last 64 sequence numbers (as in IPsec, RFC 4303)."""

    SIZE = 64

    def __init__(self) -> None:
        self.top = 0  # highest sequence number accepted
        self.mask = 0  # bit i set: sequence number top - i has been accepted

    def accept(self, seq: int) -> bool:
        """Record seq. False for 0, for duplicates and for packets older than the window."""
        if seq == 0:
            return False
        if seq > self.top:
            shift = seq - self.top
            self.mask = ((self.mask << shift) | 1) & ((1 << self.SIZE) - 1) if shift < self.SIZE else 1
            self.top = seq
            return True
        age = self.top - seq
        if age >= self.SIZE or self.mask & (1 << age):
            return False
        self.mask |= 1 << age
        return True


class LossMeter:
    """Packet loss over the last few seconds, from gaps in the sequence numbers."""

    def __init__(self, seconds: int = 10) -> None:
        self.history: deque = deque(maxlen=seconds)
        self.top = 0
        self.expected = 0
        self.received = 0

    def packet(self, seq: int) -> None:
        if seq > self.top:
            self.expected += seq - self.top if self.top else 1
            self.top = seq
        self.received += 1

    def roll(self) -> None:
        """Close the current one-second bucket."""
        self.history.append((self.expected, self.received))
        self.expected = self.received = 0

    def permille(self) -> Optional[int]:
        expected = sum(e for e, _ in self.history)
        if not expected:
            return None
        received = sum(r for _, r in self.history)
        return max(0, min(1000, round(1000 * (expected - received) / expected)))


# ---------------------------------------------------------------------------------------------
# MAVLink framing

MAV_STX_V1 = 0xFE
MAV_STX_V2 = 0xFD
MAV_IFLAG_SIGNED = 0x01
MAV_SIGNATURE_LEN = 13


class MavFramer:
    """Tracks MAVLink v1/v2 frame boundaries in a byte stream.

    Only the frame structure is used (start byte, payload length, signed flag). CRCs are not
    checked, so no message definitions are needed; bytes are never changed or dropped.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.stx = 0  # start byte of the frame in progress
        self.got = 0  # bytes of the frame in progress so far (0 = between frames)
        self.need = 0  # total length of the frame in progress (0 = not known yet)
        self.plen = 0  # payload length

    @property
    def in_frame(self) -> bool:
        return self.got > 0

    def push(self, b: int) -> bool:
        """Feed one byte. True if the stream is on a frame boundary after it."""
        if self.got == 0:
            if b == MAV_STX_V2 or b == MAV_STX_V1:
                self.stx, self.got, self.need = b, 1, 0
                return False
            return True  # stray byte between frames
        self.got += 1
        if self.got == 2:
            self.plen = b
            if self.stx == MAV_STX_V1:
                self.need = 8 + b
        elif self.got == 3 and self.stx == MAV_STX_V2:
            if b & ~MAV_IFLAG_SIGNED:  # unknown incompatibility flag: this was not a frame start
                self.reset()
                return True
            self.need = 12 + self.plen + (MAV_SIGNATURE_LEN if b & MAV_IFLAG_SIGNED else 0)
        if self.got == self.need:
            self.reset()
            return True
        return False


class Batcher:
    """Collects a MAVLink byte stream and hands it out in chunks that end on frame boundaries.

    A frame start that does not complete within frame_timeout is passed on as plain bytes, so
    a stray start byte cannot hold the stream up.
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD, frame_timeout: float = 0.5) -> None:
        self.max_payload = max_payload
        self.frame_timeout = frame_timeout
        self.framer = MavFramer()
        self.buf = bytearray()
        self.ready = 0  # length of the prefix of buf that ends on a frame boundary
        self.t_first = 0.0  # arrival time of buf[0]
        self.t_frame = 0.0  # arrival time of the first byte of the frame in progress

    def feed(self, data: bytes, now: float) -> List[bytes]:
        """Add bytes. Returns chunks that had to be cut because the buffer was full."""
        out = []
        framer = self.framer
        for b in data:
            if len(self.buf) >= self.max_payload:
                out.append(self._cut_full())
            if not self.buf:
                self.t_first = now
            if not framer.in_frame:
                self.t_frame = now
            self.buf.append(b)
            if framer.push(b):
                self.ready = len(self.buf)
        return out

    def poll(self, now: float, max_age: float) -> Optional[bytes]:
        """Complete frames, once the oldest buffered byte is max_age seconds old (0: right away)."""
        if self.framer.in_frame and now - self.t_frame >= self.frame_timeout:
            self.framer.reset()
            self.ready = len(self.buf)
        if self.ready and now - self.t_first >= max_age:
            return self._take(self.ready)
        return None

    def _take(self, n: int) -> bytes:
        chunk = bytes(self.buf[:n])
        del self.buf[:n]
        self.ready = 0
        self.t_first = self.t_frame  # anything left is the start of the frame in progress
        return chunk

    def _cut_full(self) -> bytes:
        if self.ready:
            return self._take(self.ready)
        self.framer.reset()  # a "frame" longer than a datagram is not MAVLink
        return self._take(len(self.buf))


# ---------------------------------------------------------------------------------------------
# Helpers


def parse_key(text: str) -> bytes:
    text = text.strip()
    try:
        key = bytes.fromhex(text)
    except ValueError:
        raise ValueError("key must be hex (create one with: mavrelay.py genkey)") from None
    if len(key) < 16:
        raise ValueError("key is too short; use 32 bytes (64 hex characters) from: mavrelay.py genkey")
    return key


def parse_hostport(text: str, default_host: str = "") -> Tuple[str, int]:
    """'host:port', '[v6addr]:port' or ':port'."""
    text = text.strip()
    if text.startswith("["):
        host, _, port = text[1:].partition("]:")
    else:
        host, sep, port = text.rpartition(":")
        if not sep:
            raise ValueError(f"expected host:port, got {text!r}")
    return host or default_host, int(port)


def fmt_addr(addr) -> str:
    host, port = addr[0], addr[1]
    if host.startswith("::ffff:"):
        host = host[7:]
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}"


def fmt_permille(value: int) -> str:
    return "?" if value == U16_UNKNOWN else f"{value / 10:.1f}%"


def mono_ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


class LinkStatus(NamedTuple):
    """Vehicle link as reported by the server in STATUS packets."""

    online: bool
    rat: int
    rtt_ms: int
    up_loss: int
    down_loss: int
    rssi_dbm: int
    idle_ms: int

    @classmethod
    def unpack(cls, body: bytes) -> "LinkStatus":
        body = body[: STATUS_BODY.size] + NO_VEHICLE_STATUS[len(body):]  # missing fields: unknown
        flags, rat, rtt, up, down, rssi, idle = STATUS_BODY.unpack(body)
        return cls(bool(flags & STATUS_VEHICLE_ONLINE), rat, rtt, up, down, rssi, idle)

    def describe(self) -> str:
        if self.idle_ms == U16_UNKNOWN and not self.online:
            return "vehicle: not connected to the server"
        if not self.online:
            return f"vehicle: OFFLINE, last heard {self.idle_ms / 1000:.1f} s ago"
        parts = ["vehicle: online"]
        if self.rat != RAT_UNKNOWN or self.rssi_dbm != RSSI_UNKNOWN:
            radio = RAT_NAMES.get(self.rat, "") if self.rat != RAT_UNKNOWN else ""
            if self.rssi_dbm != RSSI_UNKNOWN:
                radio = f"{radio} {self.rssi_dbm} dBm".strip()
            parts.append(radio)
        if self.rtt_ms != U16_UNKNOWN:
            parts.append(f"rtt to server {self.rtt_ms} ms")
        parts.append(f"loss up {fmt_permille(self.up_loss)} down {fmt_permille(self.down_loss)}")
        return ", ".join(parts)


# ---------------------------------------------------------------------------------------------
# Server


class Session:
    def __init__(self, sid: int, role: int, key: bytes, addr, info: str, now: float, nonce: bytes = b"") -> None:
        self.sid = sid
        self.role = role
        self.key = key
        self.addr = addr
        self.info = info
        self.nonce = nonce  # from the HELLO that asked for this session
        self.active = False
        self.watching = False  # a GCS agent with no GCS software attached (PING_FLAG_WATCHING)
        self.created = now
        self.last_rx = now
        self.tx_seq = 0
        self.window = ReplayWindow()
        self.loss = LossMeter()
        self.rx_bytes = 0
        self.tx_bytes = 0
        # as reported by the client in its PINGs
        self.rtt_ms = U16_UNKNOWN
        self.peer_loss = U16_UNKNOWN
        self.rssi_dbm = RSSI_UNKNOWN
        self.rat = RAT_UNKNOWN

    def describe(self) -> str:
        info = f" ({self.info})" if self.info else ""
        return f"{ROLE_NAMES[self.role]} {fmt_addr(self.addr)}{info}"


class RelayServer(asyncio.DatagramProtocol):
    """Forwards MAVLink between the vehicle session and all GCS sessions (and plain TCP clients)."""

    MAX_PENDING = 32
    PENDING_TIMEOUT = 15.0
    ONLINE_TIMEOUT = 3.0  # vehicle counts as online if heard within this time
    GCS_PRESENT_TIMEOUT = 5.0
    SUMMARY_INTERVAL = 300.0

    def __init__(self, keys: Dict[int, bytes], session_timeout: float = 120.0, max_gcs: int = 8) -> None:
        self.keys = keys
        self.session_timeout = session_timeout
        self.max_gcs = max_gcs
        self.sessions: Dict[int, Session] = {}
        self.vehicle: Optional[Session] = None
        self.tcp: Optional[TcpGcsPort] = None
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.counters: Counter = Counter()
        self._last_reject: Dict[int, float] = {}
        self._last_bad_key_log = -1e9
        self._vehicle_online = False
        self._last_summary = time.monotonic()

    # -- asyncio.DatagramProtocol

    def connection_made(self, transport) -> None:
        self.transport = transport

    def error_received(self, exc) -> None:
        slog.debug("udp error: %s", exc)

    def datagram_received(self, data: bytes, addr) -> None:
        pkt = decode(data)
        if pkt is None:
            self.counters["junk"] += 1
            return
        key = self.keys.get(pkt.role)
        now = time.monotonic()
        if key is None or not verify(key, data):
            self.counters["bad_tag"] += 1
            if now - self._last_bad_key_log >= 60.0:
                self._last_bad_key_log = now
                slog.warning("packet from %s claiming to be %s has the wrong key: check the %s key there "
                            "matches %s_key here", fmt_addr(addr), ROLE_NAMES.get(pkt.role, "?"),
                            ROLE_NAMES.get(pkt.role, "?"), ROLE_NAMES.get(pkt.role, "?"))
            return
        if pkt.type == HELLO:
            self._on_hello(pkt, key, addr, now)
            return
        sess = self.sessions.get(pkt.session)
        if sess is None or sess.role != pkt.role:
            if pkt.type in (DATA, PING):
                self._reject(pkt, key, addr, now)
            return
        if not sess.window.accept(pkt.seq):
            self.counters["replayed"] += 1
            return
        sess.loss.packet(pkt.seq)
        sess.last_rx = now
        if sess.addr != addr:
            if sess.active:
                slog.info("%s moved to %s", sess.describe(), fmt_addr(addr))
            sess.addr = addr
        if not sess.active:
            self._activate(sess)
        if pkt.type == DATA:
            sess.rx_bytes += len(pkt.body)
            if sess.role == ROLE_VEHICLE:
                self._from_vehicle(pkt.body)
            else:
                self.to_vehicle(pkt.body)
        elif pkt.type == PING:
            self._on_ping(sess, pkt.body)

    # -- forwarding

    def to_vehicle(self, payload: bytes) -> None:
        if self.vehicle is None:
            self.counters["dropped_no_vehicle"] += 1
            return
        self._send(self.vehicle, DATA, payload)

    def _from_vehicle(self, payload: bytes) -> None:
        for sess in self.gcs_sessions():
            if not sess.watching:
                self._send(sess, DATA, payload)
        if self.tcp is not None:
            self.tcp.broadcast(payload)

    def gcs_sessions(self) -> List[Session]:
        return [s for s in self.sessions.values() if s.active and s.role == ROLE_GCS]

    def gcs_present(self, now: float) -> bool:
        if self.tcp is not None and self.tcp.clients:
            return True
        return any(now - s.last_rx < self.GCS_PRESENT_TIMEOUT for s in self.gcs_sessions() if not s.watching)

    def _send(self, sess: Session, ptype: int, body: bytes) -> None:
        if sess.tx_seq >= 0xFFFFFFFF:  # sequence numbers used up: the client will get REJECT and start over
            self.sessions.pop(sess.sid, None)
            if sess is self.vehicle:
                self.vehicle = None
            return
        sess.tx_seq += 1
        if ptype == DATA:
            sess.tx_bytes += len(body)
        self.transport.sendto(encode(sess.key, ptype, ROLE_SERVER, sess.sid, sess.tx_seq, body), sess.addr)

    # -- session handling

    def _on_hello(self, pkt: Packet, key: bytes, addr, now: float) -> None:
        if len(pkt.body) < NONCE_LEN:
            return
        nonce = pkt.body[:NONCE_LEN]
        # A HELLO carries no sequence number, so a captured one can be replayed at will. Clients use a
        # new nonce for every attempt: one nonce never gets a second session.
        known = next((s for s in self.sessions.values() if s.role == pkt.role and s.nonce == nonce), None)
        if known is not None:
            if not known.active:  # the client retrying before our WELCOME reached it
                self.transport.sendto(encode(key, WELCOME, ROLE_SERVER, known.sid, 0, nonce), addr)
            return  # else a replay of the HELLO that started an active session
        pending = [s for s in self.sessions.values() if not s.active]
        if len(pending) >= self.MAX_PENDING:
            # make room among this address's own attempts first, so that one sender cannot push out
            # everyone else's before they can answer
            own = [s for s in pending if s.addr[0] == addr[0]]
            del self.sessions[min(own or pending, key=lambda s: s.created).sid]
        sid = 0
        while sid == 0 or sid in self.sessions:
            sid = secrets.randbits(32)
        info = pkt.body[NONCE_LEN:NONCE_LEN + INFO_MAX].decode("utf-8", "replace")
        info = "".join(c for c in info if c.isprintable())
        self.sessions[sid] = Session(sid, pkt.role, key, addr, info, now, nonce)
        self.transport.sendto(encode(key, WELCOME, ROLE_SERVER, sid, 0, nonce), addr)
        slog.debug("HELLO from %s %s -> session %08x", ROLE_NAMES.get(pkt.role), fmt_addr(addr), sid)

    def _activate(self, sess: Session) -> None:
        sess.active = True
        if sess.role == ROLE_VEHICLE:
            old = self.vehicle
            if old is not None and self.sessions.get(old.sid) is old:
                del self.sessions[old.sid]
            self.vehicle = sess
            self._vehicle_online = True
            slog.info("%s connected%s", sess.describe(), " (new session)" if old is not None else "")
        else:
            gcs = self.gcs_sessions()
            while len(gcs) > self.max_gcs:
                victim = min((s for s in gcs if s is not sess), key=lambda s: s.last_rx)
                del self.sessions[victim.sid]
                gcs.remove(victim)
                slog.info("%s dropped: too many GCS sessions", victim.describe())
            slog.info("%s connected", sess.describe())

    def _on_ping(self, sess: Session, body: bytes) -> None:
        if len(body) < 4:
            return
        if len(body) >= PING_BODY.size:
            _, sess.rtt_ms, sess.peer_loss, sess.rssi_dbm, sess.rat, ping_flags = PING_BODY.unpack_from(body)
            watching = sess.role == ROLE_GCS and bool(ping_flags & PING_FLAG_WATCHING)
            if watching != sess.watching:
                sess.watching = watching
                slog.info("%s %s", sess.describe(), "is only watching" if watching else "wants telemetry")
        flags = PONG_GCS_PRESENT if self.gcs_present(time.monotonic()) else 0
        self._send(sess, PONG, body[:4] + bytes([flags]))

    def _reject(self, pkt: Packet, key: bytes, addr, now: float) -> None:
        if now - self._last_reject.get(pkt.session, -1e9) < 1.0:
            return
        self._last_reject[pkt.session] = now
        body = bytes([REJECT_UNKNOWN_SESSION])
        self.transport.sendto(encode(key, REJECT, ROLE_SERVER, pkt.session, 0, body), addr)
        slog.debug("REJECT unknown %s session %08x from %s", ROLE_NAMES.get(pkt.role), pkt.session, fmt_addr(addr))

    # -- periodic work (once a second)

    def tick(self, now: float) -> None:
        for sess in list(self.sessions.values()):
            if sess.active:
                expired = now - sess.last_rx > self.session_timeout
            else:
                expired = now - sess.created > self.PENDING_TIMEOUT
            if expired:
                del self.sessions[sess.sid]
                if sess is self.vehicle:
                    self.vehicle = None
                if sess.active:
                    slog.info("%s session expired", sess.describe())
            else:
                sess.loss.roll()

        v = self.vehicle
        online = v is not None and now - v.last_rx < self.ONLINE_TIMEOUT
        if online != self._vehicle_online and v is not None:
            if online:
                slog.info("vehicle link restored")
            else:
                slog.warning("vehicle link lost (nothing received for %.0f s)", self.ONLINE_TIMEOUT)
        self._vehicle_online = online

        status = self.status_body(now)
        for sess in self.gcs_sessions():
            self._send(sess, STATUS, status)

        for sid in [s for s, t in self._last_reject.items() if now - t > 10.0]:
            del self._last_reject[sid]

        if now - self._last_summary >= self.SUMMARY_INTERVAL:
            self._last_summary = now
            slog.info("%s", self.summary(now))

    def status_body(self, now: float) -> bytes:
        v = self.vehicle
        if v is None:
            return NO_VEHICLE_STATUS
        idle = now - v.last_rx
        up = v.loss.permille()
        return STATUS_BODY.pack(
            STATUS_VEHICLE_ONLINE if idle < self.ONLINE_TIMEOUT else 0,
            v.rat,
            v.rtt_ms,
            U16_UNKNOWN if up is None else up,
            v.peer_loss,
            v.rssi_dbm,
            min(U16_UNKNOWN - 1, int(idle * 1000)),
        )

    def summary(self, now: float) -> str:
        v = self.vehicle
        vs = "no vehicle" if v is None else (
            f"vehicle {fmt_addr(v.addr)} idle {now - v.last_rx:.1f} s, "
            f"{v.rx_bytes / 1e6:.2f} MB up / {v.tx_bytes / 1e6:.2f} MB down"
        )
        tcp = len(self.tcp.clients) if self.tcp is not None else 0
        extra = ", ".join(f"{k} {n}" for k, n in sorted(self.counters.items()))
        gcs = self.gcs_sessions()
        watching = sum(s.watching for s in gcs)
        return f"status: {vs}; {len(gcs)} GCS agent(s) ({watching} only watching), {tcp} TCP client(s)" + (
            f"; {extra}" if extra else ""
        )


class TcpGcsPort:
    """Plain TCP port on the server for GCS software that connects without the agent.

    Traffic on this port is not authenticated, so only addresses listed in tcp_allow may
    connect (default: this machine only, for use through an SSH tunnel).
    """

    MAX_BACKLOG = 256 * 1024

    def __init__(self, relay: RelayServer, allow: List) -> None:
        self.relay = relay
        self.allow = allow
        self.clients: set = set()
        self.server: Optional[asyncio.AbstractServer] = None

    async def start(self, host: str, port: int) -> None:
        self.server = await asyncio.start_server(self._client, host or None, port)

    def allowed(self, ip: str) -> bool:
        addr = ipaddress.ip_address(ip.split("%")[0])
        if addr.version == 6 and addr.ipv4_mapped is not None:
            addr = addr.ipv4_mapped
        return any(addr in net for net in self.allow)

    def broadcast(self, payload: bytes) -> None:
        for writer in list(self.clients):
            if writer.transport.get_write_buffer_size() > self.MAX_BACKLOG:
                self.relay.counters["dropped_tcp_backlog"] += 1
                continue
            writer.write(payload)

    async def _client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        if not self.allowed(peer[0]):
            slog.warning("tcp: refused %s (not in tcp_allow)", fmt_addr(peer))
            writer.close()
            return
        slog.info("tcp: GCS connected from %s", fmt_addr(peer))
        self.clients.add(writer)
        batcher = Batcher()
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                now = time.monotonic()
                for chunk in batcher.feed(data, now):
                    self.relay.to_vehicle(chunk)
                chunk = batcher.poll(now, 0)
                if chunk:
                    self.relay.to_vehicle(chunk)
        except (ConnectionError, OSError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()
            slog.info("tcp: GCS %s disconnected", fmt_addr(peer))


# ---------------------------------------------------------------------------------------------
# Client side (GCS agent and Python vehicle)


class TunnelClient(asyncio.DatagramProtocol):
    """Keeps a session with the relay server and carries MAVLink over it."""

    def __init__(
        self,
        role: int,
        key: bytes,
        host: str,
        port: int,
        on_data: Callable[[bytes], None],
        on_status: Optional[Callable[[LinkStatus], None]] = None,
        info: str = "",
        radio: Optional[Callable[[], Tuple[int, int]]] = None,
    ) -> None:
        self.role = role
        self.key = key
        self.host = host
        self.port = port
        self.on_data = on_data
        self.on_status = on_status
        self.info = info.encode()[:INFO_MAX]
        self.radio = radio  # returns (rssi_dbm, rat) for PINGs
        self.ping_flags = 0  # PING_FLAG_*
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.server_addr = None
        self.session = 0
        self.tx_seq = 0
        self.window = ReplayWindow()
        self.loss = LossMeter()
        self.nonce = b""
        self.rtt_ms = U16_UNKNOWN
        self.gcs_present = True  # until the server says otherwise
        self.last_rx = 0.0
        self.last_ping = 0.0
        self.last_hello = 0.0
        self.last_roll = 0.0
        self.hellos = 0  # HELLOs sent without an answer
        self.need_resolve = True
        self.next_resolve = 0.0
        self.counters: Counter = Counter()
        self.connected = asyncio.Event()
        self.log = log.getChild(ROLE_NAMES[role])

    @property
    def is_connected(self) -> bool:
        return self.session != 0

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        try:
            while True:
                now = time.monotonic()
                if self.need_resolve and now >= self.next_resolve:
                    await self._resolve(loop)
                self.tick(now)
                await asyncio.sleep(0.1)
        finally:
            if self.transport is not None:
                self.transport.close()

    async def _resolve(self, loop) -> None:
        """Look the server up again (its address may have changed while the link was down)."""
        try:
            infos = await loop.getaddrinfo(self.host, self.port, type=socket.SOCK_DGRAM)
            family, _, _, _, addr = infos[0]
            if self.transport is None or self.transport.get_extra_info("socket").family != family:
                if self.transport is not None:
                    self.transport.close()
                local = ("::", 0) if family == socket.AF_INET6 else ("0.0.0.0", 0)
                self.transport, _ = await loop.create_datagram_endpoint(lambda: self, local_addr=local)
        except (OSError, UnicodeError) as exc:  # UnicodeError: a name with an empty or overlong label
            self.log.warning("cannot reach %s: %s; retrying in 5 s", self.host, exc)
            self.next_resolve = time.monotonic() + 5.0
            return
        self.server_addr = addr[:2]
        self.need_resolve = False

    def tick(self, now: float) -> None:
        if now - self.last_roll >= 1.0:
            self.last_roll = now
            self.loss.roll()
        if self.transport is None or self.server_addr is None:
            return
        if not self.session:
            if now - self.last_hello >= HELLO_INTERVAL:
                self.last_hello = now
                self.hellos += 1
                if self.hellos % 30 == 0:
                    self.log.warning("no answer from %s:%d; looking its address up again", self.host, self.port)
                    self.need_resolve = True
                if not self.nonce:
                    self.nonce = secrets.token_bytes(NONCE_LEN)
                self._sendto(encode(self.key, HELLO, self.role, 0, 0, self.nonce + self.info))
        elif now - self.last_rx > LINK_TIMEOUT:
            self.log.warning("no answer from the server for %.0f s; reconnecting", LINK_TIMEOUT)
            self._drop_session()
            self.need_resolve = True
        elif now - self.last_ping >= PING_INTERVAL:
            self.last_ping = now
            self._send_ping()

    def send_data(self, payload: bytes) -> bool:
        if not self.session:
            self.counters["dropped_no_session"] += 1
            return False
        self._send(DATA, payload)
        return True

    def _send(self, ptype: int, body: bytes) -> None:
        if self.tx_seq >= 0xFFFFFFFF:  # sequence numbers used up: get a new session
            self._drop_session()
            return
        self.tx_seq += 1
        self._sendto(encode(self.key, ptype, self.role, self.session, self.tx_seq, body))

    def _sendto(self, data: bytes) -> None:
        try:
            self.transport.sendto(data, self.server_addr)
        except OSError as exc:
            self.log.debug("send failed: %s", exc)

    def ping_now(self) -> None:
        """PINGs at once instead of within the second, e.g. so the server learns new ping_flags."""
        if self.session:
            self.last_ping = time.monotonic()
            self._send_ping()

    def _send_ping(self) -> None:
        rssi, rat = self.radio() if self.radio else (RSSI_UNKNOWN, RAT_UNKNOWN)
        loss = self.loss.permille()
        body = PING_BODY.pack(mono_ms(), self.rtt_ms, U16_UNKNOWN if loss is None else loss, rssi, rat,
                              self.ping_flags)
        self._send(PING, body)

    def _drop_session(self) -> None:
        self.session = 0
        self.nonce = b""
        self.last_hello = 0.0
        self.rtt_ms = U16_UNKNOWN
        self.gcs_present = True
        self.connected.clear()

    # -- asyncio.DatagramProtocol

    def error_received(self, exc) -> None:
        self.log.debug("udp error: %s", exc)

    def datagram_received(self, data: bytes, addr) -> None:
        pkt = decode(data)
        if pkt is None or pkt.role != ROLE_SERVER or not verify(self.key, data):
            self.counters["bad"] += 1
            return
        now = time.monotonic()
        if pkt.type == WELCOME:
            if not self.session and self.nonce and pkt.session and pkt.body[:NONCE_LEN] == self.nonce:
                self.session = pkt.session
                self.tx_seq = 0
                self.window = ReplayWindow()
                self.loss = LossMeter()
                self.nonce = b""
                self.hellos = 0
                self.last_rx = now
                self.last_ping = now
                self._send_ping()  # activates the session on the server
                self.connected.set()
                self.log.info("connected to server %s (session %08x)", fmt_addr(self.server_addr), self.session)
            return
        if not self.session or pkt.session != self.session:
            return
        if pkt.type == REJECT:
            self.log.info("server does not know our session any more; reconnecting")
            self._drop_session()
            return
        if not self.window.accept(pkt.seq):
            self.counters["replayed"] += 1
            return
        self.last_rx = now
        self.loss.packet(pkt.seq)
        if pkt.type == DATA:
            self.on_data(pkt.body)
        elif pkt.type == PONG:
            if len(pkt.body) >= PONG_BODY.size:
                t_ms, flags = PONG_BODY.unpack_from(pkt.body)
                self.rtt_ms = min(U16_UNKNOWN - 1, (mono_ms() - t_ms) & 0xFFFFFFFF)
                self.gcs_present = bool(flags & PONG_GCS_PRESENT)
        elif pkt.type == STATUS and self.on_status is not None:
            self.on_status(LinkStatus.unpack(pkt.body))


# ---------------------------------------------------------------------------------------------
# Local endpoints of the GCS agent


class LocalUdp(asyncio.DatagramProtocol):
    """Sends to a GCS that listens on UDP (Mission Planner / QGC default: 127.0.0.1:14550)."""

    def __init__(self, target: Tuple[str, int], on_input: Callable[[bytes], None]) -> None:
        self.target = target
        self.on_input = on_input
        self.batcher = Batcher()
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.last_rx = 0.0  # when the GCS last sent something (Mission Planner: a heartbeat every second)

    def connection_made(self, transport) -> None:
        self.transport = transport

    def error_received(self, exc) -> None:
        pass  # nobody listening on the target yet (ICMP port unreachable)

    def close(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None

    def datagram_received(self, data: bytes, addr) -> None:
        now = time.monotonic()
        self.last_rx = now
        for chunk in self.batcher.feed(data, now):
            self.on_input(chunk)
        chunk = self.batcher.poll(now, 0)
        if chunk:
            self.on_input(chunk)

    def send(self, payload: bytes) -> None:
        if self.transport is not None:
            try:
                self.transport.sendto(payload, self.target)
            except OSError:
                pass


class LocalTcp:
    """TCP port for GCS software (Mission Planner: TCP, 127.0.0.1, 5760)."""

    MAX_BACKLOG = 256 * 1024

    def __init__(self, on_input: Callable[[bytes], None]) -> None:
        self.on_input = on_input
        self.clients: set = set()
        self.server: Optional[asyncio.AbstractServer] = None

    async def start(self, host: str, port: int) -> None:
        self.server = await asyncio.start_server(self._client, host, port)

    def stop(self) -> None:
        """Stops listening and disconnects all clients."""
        if self.server is not None:
            self.server.close()
            self.server = None
        for writer in list(self.clients):
            writer.close()

    def send(self, payload: bytes) -> None:
        for writer in list(self.clients):
            if writer.transport.get_write_buffer_size() <= self.MAX_BACKLOG:
                writer.write(payload)

    async def _client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        glog.info("local TCP client %s connected", fmt_addr(peer))
        self.clients.add(writer)
        batcher = Batcher()
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                now = time.monotonic()
                for chunk in batcher.feed(data, now):
                    self.on_input(chunk)
                chunk = batcher.poll(now, 0)
                if chunk:
                    self.on_input(chunk)
        except (ConnectionError, OSError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()
            glog.info("local TCP client %s disconnected", fmt_addr(peer))


# ---------------------------------------------------------------------------------------------
# Vehicle-side links (Python vehicle for SITL / bench tests)


class SerialLink:
    """Flight controller on a serial port (needs pyserial)."""

    def __init__(self, spec: str) -> None:
        port, _, baud = spec.rpartition(":")
        if not port:
            port, baud = spec, "115200"
        self.port, self.baud = port, int(baud)
        self.serial = None

    async def start(self, on_input: Callable[[bytes], None]) -> None:
        try:
            import serial  # type: ignore
        except ImportError:
            raise SystemExit("--serial needs pyserial: pip install pyserial") from None
        self.serial = serial.Serial(self.port, self.baud, timeout=0.05)
        loop = asyncio.get_running_loop()

        def reader() -> None:
            while True:
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    loop.call_soon_threadsafe(on_input, data)

        threading.Thread(target=reader, daemon=True).start()
        vlog.info("serial %s at %d baud", self.port, self.baud)

    def send(self, data: bytes) -> None:
        self.serial.write(data)


class TcpLink:
    """Connects to a TCP port, e.g. ArduPilot SITL (SERIAL1 is tcp:5762)."""

    def __init__(self, spec: str) -> None:
        self.host, self.port = parse_hostport(spec, "127.0.0.1")
        self.writer: Optional[asyncio.StreamWriter] = None

    async def start(self, on_input: Callable[[bytes], None]) -> None:
        asyncio.ensure_future(self._run(on_input))

    async def _run(self, on_input) -> None:
        while True:
            try:
                reader, self.writer = await asyncio.open_connection(self.host, self.port)
                vlog.info("connected to %s:%d", self.host, self.port)
                while True:
                    data = await reader.read(4096)
                    if not data:
                        break
                    on_input(data)
            except OSError as exc:
                vlog.debug("tcp %s:%d: %s", self.host, self.port, exc)
            if self.writer is not None:
                self.writer.close()
                self.writer = None
                vlog.warning("lost connection to %s:%d", self.host, self.port)
            await asyncio.sleep(2)

    def send(self, data: bytes) -> None:
        if self.writer is not None:
            self.writer.write(data)


class UdpLink(asyncio.DatagramProtocol):
    """Listens on a UDP port and answers whoever sent last (e.g. SITL --out udp:127.0.0.1:14560)."""

    def __init__(self, spec: str) -> None:
        self.bind = parse_hostport(spec, "0.0.0.0")
        self.peer = None
        self.transport = None
        self.on_input: Optional[Callable[[bytes], None]] = None

    async def start(self, on_input: Callable[[bytes], None]) -> None:
        self.on_input = on_input
        loop = asyncio.get_running_loop()
        await loop.create_datagram_endpoint(lambda: self, local_addr=self.bind)

    def connection_made(self, transport) -> None:
        self.transport = transport

    def error_received(self, exc) -> None:
        pass

    def datagram_received(self, data: bytes, addr) -> None:
        self.peer = addr
        self.on_input(data)

    def send(self, data: bytes) -> None:
        if self.peer is not None:
            self.transport.sendto(data, self.peer)


# ---------------------------------------------------------------------------------------------
# Roles


async def run_server(opts) -> None:
    keys = {ROLE_VEHICLE: opts.vehicle_key, ROLE_GCS: opts.gcs_key}
    relay = RelayServer(keys, session_timeout=opts.session_timeout)
    loop = asyncio.get_running_loop()
    host, port = opts.listen
    await loop.create_datagram_endpoint(lambda: relay, local_addr=(host or "0.0.0.0", port))
    slog.info("MavLTE relay %s listening on udp %s", __version__, fmt_addr((host or "0.0.0.0", port)))
    if opts.tcp_listen:
        relay.tcp = TcpGcsPort(relay, opts.tcp_allow)
        await relay.tcp.start(*opts.tcp_listen)
        allowed = ", ".join(str(n) for n in opts.tcp_allow)
        slog.info("plain TCP port for GCS software on %s, allowed: %s", fmt_addr(opts.tcp_listen), allowed)
    while True:
        await asyncio.sleep(1.0)
        relay.tick(time.monotonic())


class StatusPrinter:
    """Logs the vehicle link status: at once when it changes, otherwise every `interval` seconds."""

    def __init__(self, interval: float) -> None:
        self.interval = interval
        self.state: Optional[Tuple[bool, bool]] = None
        self.last = 0.0

    def __call__(self, status: LinkStatus) -> None:
        now = time.monotonic()
        state = (status.online, status.idle_ms == U16_UNKNOWN)
        if state != self.state or now - self.last >= self.interval:
            glog.info("%s", status.describe())
            self.state, self.last = state, now


def is_loopback(host: str) -> bool:
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host == "localhost"


class GcsAgent:
    """The GCS agent: a session with the relay, plus a UDP and a TCP port for GCS software.

    The two ports can be switched on and off while the agent runs (the MavLTE app does that).
    With neither on, the agent only watches: the relay keeps it posted on the aircraft's link
    but does not count it as a GCS, so the aircraft holds its telemetry back.
    """

    STATUS_FRESH = 3.0  # seconds a STATUS from the relay stays valid

    def __init__(self, server: Tuple[str, int], key: bytes, on_status=None, info: Optional[str] = None) -> None:
        self.client = TunnelClient(ROLE_GCS, key, server[0], server[1], on_data=self._to_gcs,
                                   on_status=self._got_status, info=info or f"mavlte-agent/{__version__}")
        self.client.ping_flags = PING_FLAG_WATCHING  # until a port is on
        self.on_status = on_status
        self.status: Optional[LinkStatus] = None
        self.status_time = 0.0
        self.udp: Optional[LocalUdp] = None
        self.tcp: Optional[LocalTcp] = None
        self.to_gcs_bytes = 0  # MAVLink from the aircraft, handed to GCS software
        self.from_gcs_bytes = 0  # MAVLink from GCS software, sent to the aircraft

    @property
    def watching(self) -> bool:
        return bool(self.client.ping_flags & PING_FLAG_WATCHING)

    def _update_watching(self) -> None:
        flags = PING_FLAG_WATCHING if self.udp is None and self.tcp is None else 0
        if flags != self.client.ping_flags:
            self.client.ping_flags = flags
            self.client.ping_now()  # telemetry starts (or stops) within a second, not two

    def vehicle_status(self) -> Optional[LinkStatus]:
        """The aircraft's link as last reported by the relay, or None if that is not recent."""
        if self.status is None or time.monotonic() - self.status_time > self.STATUS_FRESH:
            return None
        return self.status

    def _got_status(self, status: LinkStatus) -> None:
        self.status, self.status_time = status, time.monotonic()
        if self.on_status is not None:
            self.on_status(status)

    def _to_gcs(self, payload: bytes) -> None:
        self.to_gcs_bytes += len(payload)
        if self.udp is not None:
            self.udp.send(payload)
        if self.tcp is not None:
            self.tcp.send(payload)

    def _from_gcs(self, chunk: bytes) -> None:
        self.from_gcs_bytes += len(chunk)
        self.client.send_data(chunk)

    async def start_udp(self, target: Tuple[str, int]) -> None:
        self._close_udp()
        host = target[0] or "127.0.0.1"
        udp = LocalUdp((host, target[1]), self._from_gcs)
        try:
            await asyncio.get_running_loop().create_datagram_endpoint(
                lambda: udp, local_addr=("127.0.0.1" if is_loopback(host) else "0.0.0.0", 0))
            self.udp = udp
        finally:
            self._update_watching()
        glog.info("sending MAVLink to udp %s (Mission Planner: connect UDP, port %d)", fmt_addr((host, target[1])),
                  target[1])

    def stop_udp(self) -> None:
        if self._close_udp():
            self._update_watching()

    def _close_udp(self) -> bool:
        if self.udp is None:
            return False
        self.udp.close()
        self.udp = None
        glog.info("UDP port off")
        return True

    async def start_tcp(self, bind: Tuple[str, int]) -> None:
        """Raises OSError if the port is taken."""
        self._close_tcp()
        host = bind[0] or "127.0.0.1"
        tcp = LocalTcp(self._from_gcs)
        try:
            await tcp.start(host, bind[1])
            self.tcp = tcp
        finally:
            self._update_watching()
        glog.info("listening on tcp %s (Mission Planner: connect TCP, %s, port %d)", fmt_addr((host, bind[1])),
                  "127.0.0.1" if host in ("0.0.0.0", "::") else host, bind[1])

    def stop_tcp(self) -> None:
        if self._close_tcp():
            self._update_watching()

    def _close_tcp(self) -> bool:
        if self.tcp is None:
            return False
        self.tcp.stop()
        self.tcp = None
        glog.info("TCP port off")
        return True

    def close_ports(self) -> None:
        """Closes the UDP and TCP ports, without telling the relay (the agent is going away)."""
        self._close_udp()
        self._close_tcp()

    async def run(self) -> None:
        glog.info("connecting to relay %s:%d", self.client.host, self.client.port)
        try:
            await self.client.run()
        finally:
            self.close_ports()


async def run_gcs(opts) -> None:
    agent = GcsAgent(opts.server, opts.key, on_status=StatusPrinter(opts.status_interval))
    if opts.udp:
        await agent.start_udp(opts.udp)
    if opts.tcp:
        try:
            await agent.start_tcp(opts.tcp)
        except OSError as exc:
            glog.warning("cannot listen on tcp port %d: %s", opts.tcp[1], exc)
    await agent.run()


async def run_vehicle(opts) -> None:
    if opts.serial:
        link = SerialLink(opts.serial)
    elif opts.tcp:
        link = TcpLink(opts.tcp)
    else:
        link = UdpLink(opts.udp)
    batcher = Batcher()
    client = TunnelClient(ROLE_VEHICLE, opts.key, *opts.server, on_data=lambda data: link.send(data),
                          info=f"mavlte-pyvehicle/{__version__}")

    def send(chunk: bytes) -> None:
        if opts.always_send or client.gcs_present:
            client.send_data(chunk)

    def on_input(data: bytes) -> None:
        for chunk in batcher.feed(data, time.monotonic()):
            send(chunk)

    await link.start(on_input)

    async def flusher() -> None:
        while True:
            await asyncio.sleep(0.005)
            chunk = batcher.poll(time.monotonic(), opts.batch_ms / 1000)
            if chunk:
                send(chunk)

    asyncio.ensure_future(flusher())
    vlog.info("connecting to relay %s:%d", *opts.server)
    await client.run()


# ---------------------------------------------------------------------------------------------
# Command line


def load_config(path: Optional[str], section: str) -> Dict[str, str]:
    if not path:
        return {}
    # interpolation=None: values are read as written, so a '%' (in a vehicle name, say) is just a '%'
    parser = configparser.ConfigParser(inline_comment_prefixes=("#", ";"), interpolation=None)
    if not parser.read(path, encoding="utf-8"):
        raise SystemExit(f"cannot read config file {path}")
    return dict(parser[section]) if parser.has_section(section) else {}


def update_config(path: str, section: str, values: Dict[str, str], remove=()) -> None:
    """Sets keys in one section of an INI file (and drops those in `remove`), keeping everything else,
    comments included."""
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        lines = []
    header = f"[{section}]".lower()
    start = next((i for i, line in enumerate(lines) if line.strip().lower() == header), None)
    if start is None:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(f"[{section}]")
        start = len(lines) - 1
    end = next((i for i in range(start + 1, len(lines)) if lines[i].strip().startswith("[")), len(lines))
    pending = {k.lower(): v for k, v in values.items()}
    drop = {k.lower() for k in remove}
    for i in range(start + 1, end):
        text = lines[i].strip()
        if not text or text[0] in "#;" or "=" not in text:
            continue
        name = text.split("=", 1)[0].strip().lower()
        if name in pending:
            lines[i] = f"{name} = {pending.pop(name)}"
        elif name in drop:
            lines[i] = None
    kept = [line for line in lines[:end] if line is not None]
    end, lines = len(kept), kept + lines[end:]
    insert_at = end
    while insert_at > start + 1 and not lines[insert_at - 1].strip():
        insert_at -= 1
    for name, value in pending.items():
        lines.insert(insert_at, f"{name} = {value}")
        insert_at += 1
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="mavrelay", description="MavLTE relay and tools: authenticated MAVLink relay "
                                                             "for an aircraft on 4G/LTE.")
    p.add_argument("--version", action="version", version=__version__)
    sub = p.add_subparsers(dest="role", required=True)

    s = sub.add_parser("server", help="run the public relay")
    s.add_argument("--config", help="INI file with a [server] section")
    s.add_argument("--listen", help="UDP address for vehicle and GCS agents (default 0.0.0.0:14650)")
    s.add_argument("--vehicle-key", help="hex key shared with the vehicle")
    s.add_argument("--gcs-key", help="hex key shared with GCS agents")
    s.add_argument("--tcp-listen", help="optional plain TCP port for GCS software, e.g. 127.0.0.1:5760")
    s.add_argument("--tcp-allow", help="comma separated networks allowed on the TCP port (default: loopback)")
    s.add_argument("--session-timeout", type=float, help="seconds of silence before a session is dropped (120)")
    s.add_argument("--log-level", help="debug, info, warning (default info)")

    g = sub.add_parser("gcs", help="run next to Mission Planner / QGroundControl")
    g.add_argument("--config", help="INI file with a [gcs] section")
    g.add_argument("--server", help="relay address, host:port")
    g.add_argument("--key", help="hex GCS key (same as gcs_key on the server)")
    g.add_argument("--udp", help="send MAVLink to this UDP address (default 127.0.0.1:14550, 'off' to disable)")
    g.add_argument("--tcp", help="listen for GCS software on this TCP address (default 127.0.0.1:5760, 'off')")
    g.add_argument("--status-interval", type=float, help="seconds between vehicle link status lines (10)")
    g.add_argument("--log-level", help="debug, info, warning (default info)")

    v = sub.add_parser("vehicle", help="vehicle side in Python (serial port or SITL)")
    v.add_argument("--config", help="INI file with a [vehicle] section")
    v.add_argument("--server", help="relay address, host:port")
    v.add_argument("--key", help="hex vehicle key (same as vehicle_key on the server)")
    src = v.add_mutually_exclusive_group()
    src.add_argument("--serial", help="flight controller serial port, e.g. COM5:115200 or /dev/ttyACM0:115200")
    src.add_argument("--tcp", help="connect to TCP, e.g. SITL 127.0.0.1:5762")
    src.add_argument("--udp", help="listen on UDP, e.g. 0.0.0.0:14560")
    v.add_argument("--batch-ms", type=float, help="collect this long before sending (default 50)")
    v.add_argument("--always-send", action="store_true", default=None,
                   help="send telemetry even while no GCS is connected")
    v.add_argument("--log-level", help="debug, info, warning (default info)")

    sub.add_parser("genkey", help="print a new random key")
    return p


class Options:
    """Command line values, falling back to the config file, then to defaults."""

    def __init__(self, args: argparse.Namespace, conf: Dict[str, str]) -> None:
        self._args = args
        self._conf = conf

    def get(self, name: str, default=None):
        value = getattr(self._args, name, None)
        if value is None:
            value = self._conf.get(name, None)
            if value is not None and value.strip() == "":
                value = None
        return default if value is None else value


def resolve_options(args: argparse.Namespace) -> argparse.Namespace:
    o = Options(args, load_config(getattr(args, "config", None), args.role))
    out = argparse.Namespace(role=args.role, log_level=o.get("log_level", "info"))

    def key(name: str) -> bytes:
        value = o.get(name)
        if value is None:
            raise SystemExit(f"missing {name.replace('_', '-')} (in the config file or on the command line)")
        try:
            return parse_key(value)
        except ValueError as exc:
            raise SystemExit(f"{name}: {exc}") from None

    def endpoint(name: str, default: Optional[str]):
        value = o.get(name, default)
        if value is None or str(value).lower() in ("off", "no", "none", ""):
            return None
        return parse_hostport(str(value))

    if args.role == "server":
        out.listen = endpoint("listen", "0.0.0.0:14650")
        out.vehicle_key = key("vehicle_key")
        out.gcs_key = key("gcs_key")
        out.tcp_listen = endpoint("tcp_listen", None)
        allow = o.get("tcp_allow", "127.0.0.1/32, ::1/128")
        out.tcp_allow = [ipaddress.ip_network(n.strip(), strict=False) for n in allow.split(",") if n.strip()]
        out.session_timeout = float(o.get("session_timeout", 120))
    elif args.role == "gcs":
        out.server = endpoint("server", None)
        if out.server is None:
            raise SystemExit("missing --server host:port")
        out.key = key("key")
        out.udp = endpoint("udp", "127.0.0.1:14550")
        out.tcp = endpoint("tcp", "127.0.0.1:5760")
        out.status_interval = float(o.get("status_interval", 10))
    else:
        out.server = endpoint("server", None)
        if out.server is None:
            raise SystemExit("missing --server host:port")
        out.key = key("key")
        if args.serial or args.tcp or args.udp:  # one source: the command line's, if it names one
            out.serial, out.tcp, out.udp = args.serial, args.tcp, args.udp
        else:
            out.serial, out.tcp, out.udp = o.get("serial"), o.get("tcp"), o.get("udp")
        if not (out.serial or out.tcp or out.udp):
            raise SystemExit("give one of --serial, --tcp or --udp")
        out.batch_ms = float(o.get("batch_ms", 50))
        value = o.get("always_send", False)
        out.always_send = value if isinstance(value, bool) else str(value).lower() in ("1", "yes", "true", "on")
    return out


def run_async(coro) -> None:
    if sys.platform == "win32":
        # The default proactor loop on Windows handles UDP "port unreachable" errors badly.
        if sys.version_info >= (3, 12):
            asyncio.run(coro, loop_factory=asyncio.SelectorEventLoop)
            return
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(coro)


def main(argv: Optional[List[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    if args.role == "genkey":
        print(secrets.token_hex(32))
        return
    opts = resolve_options(args)
    logging.basicConfig(
        level=getattr(logging, str(opts.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    runner = {"server": run_server, "gcs": run_gcs, "vehicle": run_vehicle}[opts.role]
    try:
        run_async(runner(opts))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

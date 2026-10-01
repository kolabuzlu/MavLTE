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
import json
import logging
import os
import re
import secrets
import socket
import struct
import sys
import threading
import time
from collections import Counter, deque
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

__version__ = "1.5.3"

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
PONG_VOICE = 0x02  # to the vehicle: the locator voice is on, sound the speaker
PING_FLAG_WATCHING = 0x01  # GCS agent: only watching the vehicle's link, send it STATUS but no telemetry
PING_FLAG_SPEAKING = 0x02  # vehicle: its locator voice sounds
PING_FLAG_VOICE_FAILED = 0x04  # vehicle: asked to sound, but its modem does not play it
STATUS_VEHICLE_ONLINE = 0x01
IDLE_CAPPED = 0xFFFE  # STATUS idle_ms tops out here: the relay last heard the vehicle 65.5 s ago or more
STATUS_VOICE_ON = 0x02  # the relay has the locator voice switched on
STATUS_SPEAKING = 0x04  # the vehicle said in its last PING that it sounds
STATUS_VOICE_FAILED = 0x08  # ... that it cannot
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


def fmt_age(seconds: float) -> str:
    """12 s, 5 min, 3 h 20 min, 2 days."""
    s = max(0, int(seconds))
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60} min"
    if s < 86400:
        return f"{s // 3600} h {s % 3600 // 60} min" if s % 3600 >= 60 else f"{s // 3600} h"
    return f"{s // 86400} days" if s >= 2 * 86400 else "1 day"


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
    voice_on: bool = False  # the relay has the locator voice switched on
    speaking: bool = False  # the vehicle said its speaker sounds (when last heard)
    voice_failed: bool = False  # the vehicle said it cannot

    @classmethod
    def unpack(cls, body: bytes) -> "LinkStatus":
        body = body[: STATUS_BODY.size] + NO_VEHICLE_STATUS[len(body):]  # missing fields: unknown
        flags, rat, rtt, up, down, rssi, idle = STATUS_BODY.unpack(body)
        return cls(bool(flags & STATUS_VEHICLE_ONLINE), rat, rtt, up, down, rssi, idle, bool(flags & STATUS_VOICE_ON),
                   bool(flags & STATUS_SPEAKING), bool(flags & STATUS_VOICE_FAILED))

    @property
    def connected(self) -> bool:
        """The vehicle has a session with the server (online, or recently)."""
        return self.online or self.idle_ms != U16_UNKNOWN

    def voice_text(self) -> str:
        """The locator voice: 'off', or what the aircraft makes of it."""
        if not self.voice_on:
            return "off"
        if self.voice_failed:
            return "on, but the aircraft cannot play it"
        if self.speaking:
            return ("on, the aircraft's speaker sounds" if self.online
                    else "on, the aircraft's speaker was sounding when last heard")
        return "on, waiting for the aircraft"

    def describe(self) -> str:
        voice = f"; locator voice {self.voice_text()}" if self.voice_on else ""
        if not self.connected:
            return "vehicle: not connected to the server" + voice
        if not self.online:
            heard = "over a minute ago" if self.idle_ms >= IDLE_CAPPED else f"{self.idle_ms / 1000:.1f} s ago"
            return f"vehicle: OFFLINE, last heard {heard}" + voice
        parts = ["vehicle: online"]
        if self.rat != RAT_UNKNOWN or self.rssi_dbm != RSSI_UNKNOWN:
            radio = RAT_NAMES.get(self.rat, "") if self.rat != RAT_UNKNOWN else ""
            if self.rssi_dbm != RSSI_UNKNOWN:
                radio = f"{radio} {self.rssi_dbm} dBm".strip()
            parts.append(radio)
        if self.rtt_ms != U16_UNKNOWN:
            parts.append(f"rtt to server {self.rtt_ms} ms")
        parts.append(f"loss up {fmt_permille(self.up_loss)} down {fmt_permille(self.down_loss)}")
        return ", ".join(parts) + voice


# ---------------------------------------------------------------------------------------------
# Snapshots: a photo from the aircraft's camera on request (docs/PROTOCOL.md, "Snapshots")

SNAP_REQ, SNAP_INFO, SNAP_DATA, SNAP_ACK, SNAP_SYNC = range(8, 13)
SNAP_REQ_BODY = struct.Struct("<IB")  # photo id (0 from a GCS agent: the relay picks it), size
# photo id, bytes, width, height, latitude and longitude (1e-7 degrees), altitude above home (mm),
# heading (centidegrees), status, time (unix seconds, filled in by the relay)
SNAP_INFO_BODY = struct.Struct("<IIHHiiiHBI")
SNAP_DATA_HEAD = struct.Struct("<IH")  # photo id, chunk number; then the chunk
SNAP_ACK_HEAD = struct.Struct("<IB")  # photo id, flags; then a bitmap of the chunks received (bit i: chunk i)
SNAP_SYNC_BODY = struct.Struct("<I")  # the newest photo id the agent has; also asks for photos from now on
SNAP_CHUNK = 1024
SNAP_SIZES = ((320, 240), (640, 480), (1024, 768))  # small, medium, large
SNAP_MAX_BYTES = 1024 * SNAP_CHUNK
SNAP_OK, SNAP_NO_AIRCRAFT, SNAP_NO_CAMERA, SNAP_FAILED, SNAP_BUSY, SNAP_NO_ANSWER = range(6)
SNAP_PROBLEMS = {
    SNAP_NO_AIRCRAFT: "the aircraft is not connected",
    SNAP_NO_CAMERA: "the aircraft has no camera (or its CAM switch is off)",
    SNAP_FAILED: "the camera could not take the photo",
    SNAP_BUSY: "the aircraft is still sending another photo",
    SNAP_NO_ANSWER: "the aircraft did not answer",
}
ACK_DONE, ACK_HAVE_INFO = 0x01, 0x02
UNKNOWN_I32 = -0x80000000
UNKNOWN_HEADING = 0xFFFF


class PhotoInfo(NamedTuple):
    photo_id: int
    size: int = 0  # bytes; 0 when there is no photo (see status)
    width: int = 0
    height: int = 0
    lat: int = UNKNOWN_I32  # 1e-7 degrees
    lon: int = UNKNOWN_I32
    alt: int = UNKNOWN_I32  # mm above home
    heading: int = UNKNOWN_HEADING  # centidegrees
    status: int = SNAP_OK
    time: int = 0  # unix seconds

    def pack(self) -> bytes:
        return SNAP_INFO_BODY.pack(*self)

    @classmethod
    def unpack(cls, body: bytes) -> "PhotoInfo":
        return cls(*SNAP_INFO_BODY.unpack_from(body))

    @property
    def chunks(self) -> int:
        return (self.size + SNAP_CHUNK - 1) // SNAP_CHUNK


class RateControl:
    """How fast a photo may go: as fast as the link carries it without delaying the telemetry. The
    rate backs off when the round trip rises above its recent minimum, meaning a queue is building
    up in the network, and creeps back up while it does not (as LEDBAT does, RFC 6817). A token
    bucket spends it."""

    FLOOR = 256.0  # bytes/s

    def __init__(self, cap: float, start: float = 4096.0) -> None:
        self.cap = cap
        self.rate = min(cap, start)
        self.tokens = 0.0
        self.last: Optional[float] = None
        self.rtts: deque = deque()  # (time, round trip), the last minute's
        self.backed_off = -1e9

    def set_cap(self, cap: float) -> None:
        self.cap = cap
        self.rate = min(self.rate, cap)

    def rtt_sample(self, rtt: float, now: float) -> None:
        self.rtts.append((now, rtt))
        while now - self.rtts[0][0] > 60.0:
            self.rtts.popleft()
        base = min(r for _, r in self.rtts)
        if rtt > base + max(0.15, 0.5 * base):
            if now - self.backed_off >= 1.0:
                self.rate = max(self.FLOOR, self.rate / 2)
                self.backed_off = now
        else:
            self.rate = min(self.cap, self.rate + self.cap / 10)

    def budget(self, now: float) -> float:
        if self.last is not None:
            self.tokens = min(self.tokens + (now - self.last) * self.rate, max(2 * SNAP_CHUNK, 0.2 * self.rate))
        self.last = now
        return self.tokens

    def spend(self, n: int) -> None:
        self.tokens -= n


class PhotoReceiver:
    """Collects one photo from its SNAP_INFO and SNAP_DATA, in any order, and says what it has."""

    def __init__(self, photo_id: int) -> None:
        self.photo_id = photo_id
        self.info: Optional[PhotoInfo] = None
        self.parts: Dict[int, bytes] = {}
        self.news = False  # something arrived since the last ACK
        self.last_rx = 0.0
        self.acked_at = -1e9

    def on_info(self, info: PhotoInfo, now: float) -> None:
        self.last_rx, self.news = now, True
        if self.info is None and (info.status != SNAP_OK or 0 < info.size <= SNAP_MAX_BYTES):
            self.info = info
            self.parts = {i: c for i, c in self.parts.items() if i < info.chunks and len(c) == self._length(i)}

    def on_data(self, index: int, chunk: bytes, now: float) -> None:
        self.last_rx, self.news = now, True
        if index in self.parts or index >= SNAP_MAX_BYTES // SNAP_CHUNK:
            return
        if self.info is not None and (index >= self.info.chunks or len(chunk) != self._length(index)):
            return
        self.parts[index] = bytes(chunk)

    def _length(self, index: int) -> int:
        return min(SNAP_CHUNK, self.info.size - index * SNAP_CHUNK)

    @property
    def received(self) -> int:
        return sum(map(len, list(self.parts.values())))  # list(): a copy in one step, for readers in other threads

    @property
    def complete(self) -> bool:
        return self.info is not None and self.info.status == SNAP_OK and len(self.parts) == self.info.chunks

    def chunk(self, index: int) -> Optional[bytes]:
        return self.parts.get(index)

    def data(self) -> bytes:
        return b"".join(self.parts[i] for i in range(self.info.chunks))

    def ack(self) -> bytes:
        flags = (ACK_DONE if self.complete else 0) | (ACK_HAVE_INFO if self.info is not None else 0)
        top = max(self.parts, default=-1)
        bitmap = bytearray(top // 8 + 1)
        for i in self.parts:
            bitmap[i // 8] |= 1 << (i % 8)
        return SNAP_ACK_HEAD.pack(self.photo_id, flags) + bytes(bitmap)


class PhotoSender:
    """Sends one photo: its SNAP_INFO until the receiver has it, then the chunks, and again whatever
    the receiver's ACKs do not show after a while. `source` has chunk(i), which may return None for a
    chunk it does not have yet (the relay passes a photo on while it is still arriving)."""

    GIVE_UP = 60.0  # seconds without any ACK

    def __init__(self, info: PhotoInfo, source, send: Callable[[int, bytes], None], now: float) -> None:
        self.info, self.source, self.send = info, source, send
        self.started = now
        self.acked = set()
        self.sent_at: Dict[int, float] = {}
        self.info_acked = False
        self.info_sent_at = -1e9
        self.last_ack = now
        self.done = self.failed = False

    def restart(self, now: float) -> None:
        """Sends it all again, as to a new receiver (after a relay restart, one that has none of it).
        Whatever the receiver still has, its first ACK tells."""
        self.acked.clear()
        self.sent_at.clear()
        self.info_acked = False
        self.info_sent_at = -1e9
        self.last_ack = now

    def on_ack(self, flags: int, bitmap: bytes, now: float) -> None:
        self.last_ack = now
        self.info_acked = self.info_acked or bool(flags & ACK_HAVE_INFO)
        for i in range(min(len(bitmap) * 8, self.info.chunks)):
            if bitmap[i // 8] & (1 << (i % 8)):
                self.acked.add(i)
        if flags & ACK_DONE:
            self.done = True

    @property
    def progress(self) -> float:
        return len(self.acked) / self.info.chunks if self.info.chunks else 1.0

    def pump(self, now: float, rate: RateControl, rto: float) -> None:
        if self.done or self.failed:
            return
        if now - self.last_ack > self.GIVE_UP:
            self.failed = True
            return
        if not self.info_acked and now - self.info_sent_at >= rto:
            self.info_sent_at = now
            self.send(SNAP_INFO, self.info.pack())
        budget = rate.budget(now)
        for i in range(self.info.chunks):
            if i in self.acked or now - self.sent_at.get(i, -1e9) < rto:
                continue
            chunk = self.source.chunk(i)
            if chunk is None:
                continue
            if budget < len(chunk):
                break
            budget -= len(chunk)
            rate.spend(len(chunk))
            self.sent_at[i] = now
            self.send(SNAP_DATA, SNAP_DATA_HEAD.pack(self.info.photo_id, i) + chunk)


class BytesSource:
    def __init__(self, data: bytes) -> None:
        self.data = data

    def chunk(self, index: int) -> Optional[bytes]:
        return self.data[index * SNAP_CHUNK:(index + 1) * SNAP_CHUNK]


class FileSource:
    """A kept photo, read a chunk at a time as it goes out: photos passed on to an agent that comes
    back after a while need no memory."""

    def __init__(self, path: str) -> None:
        self.path = path

    def chunk(self, index: int) -> Optional[bytes]:
        try:
            with open(self.path, "rb") as f:
                f.seek(index * SNAP_CHUNK)
                return f.read(SNAP_CHUNK) or None
        except OSError:
            return None


def rto_for(rtt_ms: int) -> float:
    """How long to wait for an ACK before sending a chunk again."""
    return max(1.0, 2.5 * rtt_ms / 1000) if rtt_ms != U16_UNKNOWN else 2.0


# ---------------------------------------------------------------------------------------------
# Locator: where the aircraft is, from the LTE module's own GNSS, whatever the flight controller
# does (docs/PROTOCOL.md, "Locator")

POSITION = 13
# GNSS time (unix seconds), latitude and longitude (1e-7 degrees), altitude (mm above sea level),
# speed (cm/s), course (centidegrees), HDOP (x100), satellites used, fix, flags, seconds since the
# flight controller was last heard, the module's battery (mV, %), time (unix seconds, set by the relay)
POSITION_BODY = struct.Struct("<IiiiHHHBBBHHBI")  # all a POSITION has: before 1.4.0, nothing followed
POSITION_TEMP = struct.Struct("<b")  # then the module's chip temperature (degrees C)
FIX_NONE, FIX_2D, FIX_3D = 0, 2, 3
POS_FC_SILENT = 0x01  # nothing from the flight controller for FC_SILENT_S or more
POS_NO_GNSS = 0x02  # the module cannot read its GNSS (fix fields unknown)
FC_SILENT_S = 10
BATTERY_UNKNOWN = 0xFF
# On V2 boards the fuel gauge sits on the board's supply rail, not on the cell: while USB or the 5V pin (the
# BEC) powers the board it reads the converter feeding that rail (about 4.3 V), more than a Li-ion cell holds
EXTERNAL_POWER_MV = 4250
TEMP_UNKNOWN = -128
# The ESP32-S3's own sensor reads warmer than the air around the board. V2 boards' ESP32-S3R8 is rated
# for 65 degrees C of air around it (V1's ESP32-S3R2: 85); its datasheet ties that to the chip's octal
# PSRAM, which the firmware does not use.
TEMP_WARM = 70  # getting hot
TEMP_HOT = 80  # too hot: give the board air, out of the sun

# The locator voice: the board's speaker, switched on and off by GCS agents (docs/PROTOCOL.md, "Locator voice")
VOICE = 14
VOICE_BODY = struct.Struct("<B")  # bit 0: on


class Position(NamedTuple):
    gnss_time: int = 0
    lat: int = UNKNOWN_I32  # 1e-7 degrees
    lon: int = UNKNOWN_I32
    alt: int = UNKNOWN_I32  # mm above mean sea level
    speed: int = U16_UNKNOWN  # cm/s
    course: int = U16_UNKNOWN  # centidegrees
    hdop: int = U16_UNKNOWN  # x100
    sats: int = 0
    fix: int = FIX_NONE
    flags: int = 0
    fc_silent: int = U16_UNKNOWN  # seconds; U16_UNKNOWN: not heard since the module started
    battery_mv: int = U16_UNKNOWN
    battery_pct: int = BATTERY_UNKNOWN
    time: int = 0  # unix seconds, when the relay got it
    temp: int = TEMP_UNKNOWN  # degrees C, the module's chip

    def pack(self) -> bytes:
        return POSITION_BODY.pack(*self[:-1]) + POSITION_TEMP.pack(self.temp)

    @classmethod
    def unpack(cls, body: bytes) -> "Position":
        """From POSITION_BODY.size bytes or more: an aircraft before 1.4.0 sends no temperature."""
        temp = POSITION_TEMP.unpack_from(body, POSITION_BODY.size)[0] if len(body) > POSITION_BODY.size \
            else TEMP_UNKNOWN
        return cls(*POSITION_BODY.unpack_from(body), temp)

    @property
    def has_fix(self) -> bool:
        return self.fix >= FIX_2D and self.lat != UNKNOWN_I32 and self.lon != UNKNOWN_I32

    @property
    def fc_is_silent(self) -> bool:
        return bool(self.flags & POS_FC_SILENT)

    @property
    def on_external_power(self) -> bool:
        """USB or the BEC powers the module (its percent then says nothing about the cell)."""
        return self.battery_mv != U16_UNKNOWN and self.battery_mv >= EXTERNAL_POWER_MV

    def power_text(self) -> str:
        """'external power', 'battery 85%' (the module runs on its cell), or '' if it does not say."""
        if self.on_external_power:
            return "external power"
        return f"battery {self.battery_pct}%" if self.battery_pct != BATTERY_UNKNOWN else ""

    def describe(self) -> str:
        """For the log: where, or why not."""
        if self.flags & POS_NO_GNSS:
            return "no GNSS"
        if not self.has_fix:
            return f"no GNSS fix yet ({self.sats} satellites)"
        return f"{self.lat / 1e7:.6f}, {self.lon / 1e7:.6f}, {self.sats} satellites"


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
        self.wants_photos = False  # a GCS agent that sent SNAP_SYNC or SNAP_REQ
        self.deliveries: List[PhotoSender] = []  # photos on their way to this GCS agent
        self.photo_rate = RateControl(PhotoStore.GCS_RATE, start=16 * 1024)
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
        self.speaking = False  # a vehicle's locator voice sounds (PING_FLAG_SPEAKING)
        self.voice_failed = False  # it was asked to, but its modem does not

    def describe(self) -> str:
        info = f" ({self.info})" if self.info else ""
        return f"{ROLE_NAMES[self.role]} {fmt_addr(self.addr)}{info}"


class PhotoStore:
    """The relay's side of snapshots. Asks the aircraft for a photo when a GCS agent does, collects it,
    keeps it (in `folder`, for `days`), and passes it on to every GCS agent that wants photos, while it
    is still arriving and later to agents that come back (SNAP_SYNC names the newest one they have).
    Photo ids are the unix time of the request, so they keep growing across relay restarts."""

    REQUEST_EVERY = 2.0  # seconds, until the aircraft answers
    REQUEST_FOR = 20.0
    DROP_AFTER = 120.0  # an unfinished photo that nothing more arrived for
    GCS_RATE = 64 * 1024  # bytes/s at most to each GCS agent
    IN_MEMORY = 20  # photos whose bytes stay in memory as well as in the folder
    MAX_PHOTOS = 2000  # kept at most (without a folder: IN_MEMORY)
    RECENT = 600  # seconds: the aircraft may finish sending a photo asked for before a relay restart
    MAX_INCOMING = 4  # photos arriving at once
    SYNC_MOST = 50  # photos passed on at most to an agent that comes back: the newest

    def __init__(self, relay: "RelayServer", folder: Optional[str], days: float) -> None:
        self.relay = relay
        self.folder = folder
        self.days = days
        self.stored: Dict[int, PhotoInfo] = {}
        self.memory: Dict[int, bytes] = {}  # the newest photos' bytes, and all of them without a folder
        self.incoming: Dict[int, PhotoReceiver] = {}
        self.pending: Dict[int, list] = {}  # photo id: [size, asked at, last asked, GCS session id]
        self.last_id = 0
        self._load()

    # -- keeping photos

    def _load(self) -> None:
        if not self.folder:
            return
        try:
            os.makedirs(self.folder, exist_ok=True)
            names = os.listdir(self.folder)
        except OSError as exc:
            slog.warning("cannot use the photo folder %s (%s): photos are kept in memory only", self.folder, exc)
            self.folder = None
            return
        for name in names:
            if name.endswith(".json") and os.path.exists(os.path.join(self.folder, name[:-5] + ".jpg")):
                try:
                    with open(os.path.join(self.folder, name), encoding="utf-8") as f:
                        meta = json.load(f)
                    info = PhotoInfo(*(int(meta[field]) for field in PhotoInfo._fields))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
                self.stored[info.photo_id] = info
        self.last_id = max(self.stored, default=0)
        self.prune(time.time())
        if self.stored:
            slog.info("%d photo(s) kept in %s", len(self.stored), self.folder)

    def _path(self, photo_id: int, ext: str) -> str:
        return os.path.join(self.folder, f"{photo_id}.{ext}")

    def new_id(self) -> int:
        self.last_id = max(int(time.time()), self.last_id + 1)
        return self.last_id

    def _keep(self, info: PhotoInfo, data: bytes) -> None:
        self.stored[info.photo_id] = info
        self.memory[info.photo_id] = data
        if self.folder:
            meta = dict(info._asdict(), vehicle=self.relay.vehicle.info if self.relay.vehicle else "")
            try:
                for ext, blob in (("jpg", data), ("json", json.dumps(meta, indent=1).encode())):
                    with open(self._path(info.photo_id, ext) + ".tmp", "wb") as f:
                        f.write(blob)
                    os.replace(self._path(info.photo_id, ext) + ".tmp", self._path(info.photo_id, ext))
            except OSError as exc:
                slog.warning("cannot save photo %d: %s", info.photo_id, exc)
        self.prune(time.time())

    def _forget(self, photo_id: int) -> None:
        del self.stored[photo_id]
        self.memory.pop(photo_id, None)
        if self.folder:
            for ext in ("jpg", "json"):
                try:
                    os.remove(self._path(photo_id, ext))
                except OSError:
                    pass

    def source(self, photo_id: int):
        if photo_id in self.incoming:
            return self.incoming[photo_id]
        if photo_id in self.memory:
            return BytesSource(self.memory[photo_id])
        if self.folder and photo_id in self.stored:
            return FileSource(self._path(photo_id, "jpg"))
        return None

    def prune(self, now_unix: float) -> None:
        """Forgets photos older than `days`, and the oldest beyond MAX_PHOTOS."""
        ids = sorted(self.stored)
        most = self.MAX_PHOTOS if self.folder else self.IN_MEMORY
        for photo_id in ids[:max(0, len(ids) - most)]:
            self._forget(photo_id)
        for photo_id in [p for p, info in self.stored.items() if info.time < now_unix - self.days * 86400]:
            self._forget(photo_id)
        for photo_id in sorted(self.memory)[:-self.IN_MEMORY]:
            del self.memory[photo_id]

    # -- packets

    def on_packet(self, sess: "Session", ptype: int, body: bytes, now: float) -> None:
        if sess.role == ROLE_GCS:
            if ptype == SNAP_REQ and len(body) >= SNAP_REQ_BODY.size:
                self._request(sess, SNAP_REQ_BODY.unpack_from(body)[1], now)
            elif ptype == SNAP_SYNC and len(body) >= SNAP_SYNC_BODY.size:
                self._sync(sess, SNAP_SYNC_BODY.unpack_from(body)[0], now)
            elif ptype == SNAP_ACK and len(body) >= SNAP_ACK_HEAD.size:
                photo_id, flags = SNAP_ACK_HEAD.unpack_from(body)
                for sender in sess.deliveries:
                    if sender.info.photo_id == photo_id:
                        sender.on_ack(flags, body[SNAP_ACK_HEAD.size:], now)
        elif ptype == SNAP_INFO and len(body) >= SNAP_INFO_BODY.size:
            self._aircraft_info(sess, PhotoInfo.unpack(body), now)
        elif ptype == SNAP_DATA and len(body) > SNAP_DATA_HEAD.size:
            photo_id, index = SNAP_DATA_HEAD.unpack_from(body)
            receiver = self.incoming.get(photo_id)
            if receiver is None and self._wanted(photo_id):  # its SNAP_INFO got lost, or comes later
                self.pending.pop(photo_id, None)  # the aircraft has it: no need to ask again
                receiver = self.incoming[photo_id] = PhotoReceiver(photo_id)
            if receiver is not None:
                receiver.on_data(index, body[SNAP_DATA_HEAD.size:], now)
            elif photo_id in self.stored:  # it missed our last ACK: tell it we have it all
                self.relay._send(sess, SNAP_ACK, SNAP_ACK_HEAD.pack(photo_id, ACK_DONE | ACK_HAVE_INFO))

    def _wanted(self, photo_id: int) -> bool:
        """A photo we asked for, and not yet have. After a restart, the relay no longer knows what it
        asked for: then a recent one (ids are unix times) is taken as well."""
        if photo_id in self.stored:
            return False
        if photo_id in self.pending:
            return True
        return len(self.incoming) < self.MAX_INCOMING and abs(photo_id - time.time()) < self.RECENT

    def _tell(self, sess: "Session", info: PhotoInfo) -> None:
        self.relay._send(sess, SNAP_INFO, info.pack())

    def _request(self, gcs: "Session", size: int, now: float) -> None:
        gcs.wants_photos = True
        photo_id = self.new_id()
        vehicle = self.relay.vehicle
        if vehicle is None or now - vehicle.last_rx >= self.relay.ONLINE_TIMEOUT:
            self._tell(gcs, PhotoInfo(photo_id, status=SNAP_NO_AIRCRAFT, time=int(time.time())))
            return
        size = min(size, len(SNAP_SIZES) - 1)
        self.pending[photo_id] = [size, now, now, gcs.sid]
        self.relay._send(vehicle, SNAP_REQ, SNAP_REQ_BODY.pack(photo_id, size))
        slog.info("%s asks for a photo (%dx%d): photo %d", gcs.describe(), *SNAP_SIZES[size], photo_id)

    def _sync(self, gcs: "Session", newest: int, now: float) -> None:
        gcs.wants_photos = True
        on_the_way = {sender.info.photo_id for sender in gcs.deliveries}
        missed = sorted(p for p in set(self.stored) | {p for p, r in self.incoming.items() if r.info}
                        if p > newest and p not in on_the_way)
        for photo_id in missed[-self.SYNC_MOST:]:
            self._deliver(gcs, photo_id, now)

    def _deliver(self, gcs: "Session", photo_id: int, now: float) -> None:
        info = self.incoming[photo_id].info if photo_id in self.incoming else self.stored.get(photo_id)
        source = self.source(photo_id)
        if info is not None and source is not None:
            gcs.deliveries.append(PhotoSender(info, source, lambda t, b, s=gcs: self.relay._send(s, t, b), now))

    def _aircraft_info(self, vehicle: "Session", info: PhotoInfo, now: float) -> None:
        request = self.pending.pop(info.photo_id, None)
        info = info._replace(time=int(time.time()))
        if info.status != SNAP_OK:
            if request is not None:  # the first answer (it answers every SNAP_REQ that reaches it)
                slog.info("photo %d: %s", info.photo_id, SNAP_PROBLEMS.get(info.status, f"status {info.status}"))
                gcs = self.relay.sessions.get(request[3])
                if gcs is not None:
                    self._tell(gcs, info)
            return
        receiver = self.incoming.get(info.photo_id)
        if receiver is None:
            if info.photo_id in self.stored:
                self.relay._send(vehicle, SNAP_ACK, SNAP_ACK_HEAD.pack(info.photo_id, ACK_DONE | ACK_HAVE_INFO))
                return
            if request is None and not self._wanted(info.photo_id):
                return  # not asked for by us
            receiver = self.incoming[info.photo_id] = PhotoReceiver(info.photo_id)
        if receiver.info is None:
            receiver.on_info(info, now)
            slog.info("photo %d from the aircraft: %d KB, %dx%d", info.photo_id, info.size // 1024, info.width,
                      info.height)
            for gcs in self.relay.gcs_sessions():
                if gcs.wants_photos and all(s.info.photo_id != info.photo_id for s in gcs.deliveries):
                    self._deliver(gcs, info.photo_id, now)
        else:
            receiver.on_info(info, now)

    # -- ten or twenty times a second

    def pump(self, now: float) -> None:
        vehicle = self.relay.vehicle
        for photo_id, request in list(self.pending.items()):
            size, asked, last, gcs_id = request
            if now - asked > self.REQUEST_FOR:
                del self.pending[photo_id]
                gcs = self.relay.sessions.get(gcs_id)
                if gcs is not None:
                    self._tell(gcs, PhotoInfo(photo_id, status=SNAP_NO_ANSWER, time=int(time.time())))
            elif vehicle is not None and now - last >= self.REQUEST_EVERY:
                request[2] = now
                self.relay._send(vehicle, SNAP_REQ, SNAP_REQ_BODY.pack(photo_id, size))
        for photo_id, receiver in list(self.incoming.items()):
            if vehicle is not None and receiver.news and (receiver.complete or now - receiver.acked_at >= 0.5):
                receiver.news, receiver.acked_at = False, now
                self.relay._send(vehicle, SNAP_ACK, receiver.ack())
            if receiver.complete:
                del self.incoming[photo_id]
                self._keep(receiver.info, receiver.data())
                slog.info("photo %d complete: %d KB", photo_id, receiver.info.size // 1024)
            elif now - receiver.last_rx > self.DROP_AFTER:
                del self.incoming[photo_id]
                slog.warning("photo %d: nothing more from the aircraft, dropped with %d of %s KB", photo_id,
                             receiver.received // 1024, receiver.info.size // 1024 if receiver.info else "?")
        for gcs in self.relay.gcs_sessions():
            if gcs.deliveries:
                for sender in gcs.deliveries:
                    if now - gcs.last_rx < 5.0:  # else the agent is gone (or on a new session) or cut off
                        sender.pump(now, gcs.photo_rate, rto_for(gcs.rtt_ms))
                    elif now - sender.last_ack > sender.GIVE_UP:
                        sender.failed = True
                gcs.deliveries = [s for s in gcs.deliveries if not (s.done or s.failed)]


class LocatorStore:
    """The relay's side of the locator: passes each POSITION from the aircraft on to every GCS agent,
    and keeps the last one with a fix, in `path` so that it survives a restart. An agent that
    connects gets that one at once: the last known position, even with the aircraft long gone."""

    SAVE_EVERY = 4.0  # seconds between writes to disk: every fix, as the aircraft sends one every 5 s

    def __init__(self, relay: "RelayServer", path: Optional[str]) -> None:
        self.relay = relay
        self.path = path
        self.last: Optional[Position] = None  # the newest, fix or not
        self.last_fix: Optional[Position] = None
        self.saved_at = -1e9
        self.fc_silent = False
        self.on_cell: Optional[bool] = None  # the module runs on its own cell (False: on external power)
        if path:
            try:
                with open(path, encoding="utf-8") as f:
                    meta = json.load(f)
                meta.setdefault("temp", TEMP_UNKNOWN)  # files from before 1.4.0 have none
                self.last_fix = Position(*(int(meta[field]) for field in Position._fields))
                slog.info("last known position of the aircraft: %s", self.last_fix.describe())
            except FileNotFoundError:
                pass
            except (OSError, ValueError, KeyError, TypeError) as exc:
                slog.warning("cannot read %s: %s", path, exc)

    def from_vehicle(self, body: bytes, now: float) -> None:
        if len(body) < POSITION_BODY.size:
            return
        pos = Position.unpack(body)._replace(time=int(time.time()))
        if self.last is None or pos.has_fix != self.last.has_fix:
            slog.info("aircraft's GNSS: %s", pos.describe())
        if pos.fc_is_silent != self.fc_silent:
            self.fc_silent = pos.fc_is_silent
            if self.fc_silent:
                slog.warning("the aircraft's flight controller is silent (%s s); its LTE module still reports",
                             pos.fc_silent)
            elif pos.fc_silent != U16_UNKNOWN:  # (unknown: not heard since the module started)
                slog.info("the aircraft's flight controller talks again")
        if pos.battery_mv != U16_UNKNOWN:
            on_cell = not pos.on_external_power
            if self.on_cell is not None and on_cell != self.on_cell:
                if on_cell:
                    slog.warning("the aircraft's LTE module runs on its own cell now (%s)", pos.power_text())
                else:
                    slog.info("the aircraft's LTE module has external power again")
            self.on_cell = on_cell
        self.last = pos
        if pos.has_fix:
            self.last_fix = pos
            if now - self.saved_at >= self.SAVE_EVERY:
                self._save(pos)
                self.saved_at = now
        for gcs in self.relay.gcs_sessions():
            self.relay._send(gcs, POSITION, pos.pack())

    def welcome(self, gcs: "Session") -> None:
        """A GCS agent that just connected hears the last known position."""
        if self.last_fix is not None:
            self.relay._send(gcs, POSITION, self.last_fix.pack())

    def _save(self, pos: Position) -> None:
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            with open(self.path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(dict(pos._asdict(), latitude=pos.lat / 1e7, longitude=pos.lon / 1e7), f, indent=1)
            os.replace(self.path + ".tmp", self.path)
        except OSError as exc:
            slog.warning("cannot save the aircraft's position: %s", exc)


class VoiceSwitch:
    """The locator voice as GCS agents switch it (VOICE). While it is on, every PONG to the aircraft asks it
    to sound the speaker on its board, until an agent switches it off. Kept in `path`, so that a
    relay restart does not silence an aircraft that someone is looking for."""

    def __init__(self, path: Optional[str]) -> None:
        self.path = path
        self.on = False
        self.since = 0  # unix seconds, when it was last switched
        if path:
            try:
                with open(path, encoding="utf-8") as f:
                    meta = json.load(f)
                self.on, self.since = bool(meta["on"]), int(meta.get("since", 0))
                if self.on:
                    slog.info("the locator voice is on (switched on %s ago)", fmt_age(time.time() - self.since))
            except FileNotFoundError:
                pass
            except (OSError, ValueError, KeyError, TypeError) as exc:
                slog.warning("cannot read %s: %s", path, exc)

    def switch(self, on: bool, by: "Session") -> None:
        if on == self.on:
            return
        self.on, self.since = on, int(time.time())
        slog.info("%s switched the locator voice %s", by.describe(), "on" if on else "off")
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            with open(self.path + ".tmp", "w", encoding="utf-8") as f:
                json.dump({"on": on, "since": self.since}, f)
            os.replace(self.path + ".tmp", self.path)
        except OSError as exc:
            slog.warning("cannot save the locator voice switch: %s", exc)


class RelayServer(asyncio.DatagramProtocol):
    """Forwards MAVLink between the vehicle session and all GCS sessions (and plain TCP clients)."""

    MAX_PENDING = 32
    PENDING_TIMEOUT = 15.0
    ONLINE_TIMEOUT = 3.0  # vehicle counts as online if heard within this time
    GCS_PRESENT_TIMEOUT = 5.0
    SUMMARY_INTERVAL = 300.0

    def __init__(self, keys: Dict[int, bytes], session_timeout: float = 120.0, max_gcs: int = 8,
                 photo_folder: Optional[str] = None, photo_days: float = 7.0,
                 state_dir: Optional[str] = None) -> None:
        self.keys = keys
        self.session_timeout = session_timeout
        self.max_gcs = max_gcs
        self.photos = PhotoStore(self, photo_folder, photo_days)
        self.locator = LocatorStore(self, os.path.join(state_dir, "locator.json") if state_dir else None)
        self.voice = VoiceSwitch(os.path.join(state_dir, "voice.json") if state_dir else None)
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
        elif pkt.type == POSITION:
            if sess.role == ROLE_VEHICLE:
                self.locator.from_vehicle(pkt.body, now)
        elif pkt.type == VOICE:
            if sess.role == ROLE_GCS and len(pkt.body) >= VOICE_BODY.size:
                self.voice.switch(bool(pkt.body[0] & 0x01), sess)
        elif SNAP_REQ <= pkt.type <= SNAP_SYNC:
            self.photos.on_packet(sess, pkt.type, pkt.body, now)

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
            self.locator.welcome(sess)

    def _on_ping(self, sess: Session, body: bytes) -> None:
        if len(body) < 4:
            return
        if len(body) >= PING_BODY.size:
            _, sess.rtt_ms, sess.peer_loss, sess.rssi_dbm, sess.rat, ping_flags = PING_BODY.unpack_from(body)
            if sess.role == ROLE_GCS:
                watching = bool(ping_flags & PING_FLAG_WATCHING)
                if watching != sess.watching:
                    sess.watching = watching
                    slog.info("%s %s", sess.describe(), "is only watching" if watching else "wants telemetry")
            else:
                self._vehicle_voice(sess, bool(ping_flags & PING_FLAG_SPEAKING),
                                    bool(ping_flags & PING_FLAG_VOICE_FAILED))
        flags = PONG_GCS_PRESENT if self.gcs_present(time.monotonic()) else 0
        if sess.role == ROLE_VEHICLE and self.voice.on:
            flags |= PONG_VOICE
        self._send(sess, PONG, body[:4] + bytes([flags]))

    @staticmethod
    def _vehicle_voice(sess: Session, speaking: bool, failed: bool) -> None:
        if speaking != sess.speaking:
            slog.info("the aircraft's speaker %s (locator voice)", "sounds" if speaking else "has stopped")
        if failed and not sess.voice_failed:
            slog.warning("the aircraft cannot play the locator voice: its modem refuses it")
        sess.speaking, sess.voice_failed = speaking, failed

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

        for sess in self.gcs_sessions():  # photos to a GCS agent go only as fast as its link allows
            if sess.deliveries and sess.rtt_ms != U16_UNKNOWN:
                sess.photo_rate.rtt_sample(sess.rtt_ms / 1000, now)

        if now - self._last_summary >= self.SUMMARY_INTERVAL:
            self._last_summary = now
            slog.info("%s", self.summary(now))
            self.photos.prune(time.time())

    def status_body(self, now: float) -> bytes:
        voice = STATUS_VOICE_ON if self.voice.on else 0
        v = self.vehicle
        if v is None:
            return bytes([voice]) + NO_VEHICLE_STATUS[1:]
        idle = now - v.last_rx
        up = v.loss.permille()
        return STATUS_BODY.pack(
            (STATUS_VEHICLE_ONLINE if idle < self.ONLINE_TIMEOUT else 0) | voice
            | (STATUS_SPEAKING if v.speaking else 0) | (STATUS_VOICE_FAILED if v.voice_failed else 0),
            v.rat,
            v.rtt_ms,
            U16_UNKNOWN if up is None else up,
            v.peer_loss,
            v.rssi_dbm,
            min(IDLE_CAPPED, int(idle * 1000)),
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
        voice = f"; locator voice on for {fmt_age(time.time() - self.voice.since)}" if self.voice.on else ""
        return f"status: {vs}; {len(gcs)} GCS agent(s) ({watching} only watching), {tcp} TCP client(s)" + voice + (
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
        # vehicle: the server's last PONG asked for the locator voice; kept without a session, so that an
        # aircraft keeps sounding where it has no coverage
        self.voice_on = False
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
        self.on_packet: Optional[Callable[[int, bytes], None]] = None  # snapshot packets (SNAP_*)
        self.on_session: Optional[Callable[[], None]] = None  # each time a new session starts

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

    def send_packet(self, ptype: int, body: bytes) -> bool:
        """Sends any packet type on the session (snapshots). False while there is none."""
        if not self.session:
            return False
        self._send(ptype, body)
        return True

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
                if self.on_session is not None:
                    self.on_session()
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
                self.voice_on = bool(flags & PONG_VOICE)
        elif pkt.type == STATUS and self.on_status is not None:
            self.on_status(LinkStatus.unpack(pkt.body))
        elif pkt.type >= SNAP_REQ and self.on_packet is not None:
            self.on_packet(pkt.type, pkt.body)


class PhotoInbox:
    """The GCS agent's side of snapshots: asks for photos, collects them from the relay and saves them
    in `folder`, each with a .json beside it (where and when it was taken). On every new session it
    tells the relay the newest photo it has (SNAP_SYNC), so photos that arrived while it was away
    come too."""

    ANSWER_WITHIN = 45.0  # seconds for a photo asked for to start arriving
    GIVE_UP = 120.0  # seconds without anything more of a photo

    def __init__(self, client: TunnelClient, folder: str,
                 on_photo: Optional[Callable[[str, PhotoInfo], None]] = None) -> None:
        self.client = client
        self.folder = folder
        self.on_photo = on_photo
        self.receivers: Dict[int, PhotoReceiver] = {}
        self.finished: set = set()  # ids saved while this agent runs
        self.asked_at: Optional[float] = None  # our request, until a photo starts arriving
        self.problem = ""  # why the last request brought no photo
        self.problem_at = 0.0
        self.last_path: Optional[str] = None
        self.last_info: Optional[PhotoInfo] = None

    def newest(self) -> int:
        try:
            names = os.listdir(self.folder)
        except OSError:
            return 0
        ids = [int(n.rsplit("_", 1)[-1][:-4]) for n in names
               if n.startswith("MavLTE_") and n.endswith(".jpg") and n.rsplit("_", 1)[-1][:-4].isdigit()]
        return max(ids + list(self.finished), default=0)

    def sync(self) -> None:
        self.client.send_packet(SNAP_SYNC, SNAP_SYNC_BODY.pack(self.newest()))

    def request(self, size: int) -> bool:
        """Asks for a photo (size: index into SNAP_SIZES). False while not connected to the relay."""
        if not self.client.send_packet(SNAP_REQ, SNAP_REQ_BODY.pack(0, size)):
            return False
        self.asked_at, self.problem = time.monotonic(), ""
        return True

    def _fail(self, problem: str) -> None:
        self.problem, self.problem_at = problem, time.monotonic()

    @property
    def arriving(self) -> Optional[Tuple[PhotoInfo, int]]:
        """The photo coming in now, and how many of its bytes are here (safe to read from another
        thread, like the MavLTE window's)."""
        for receiver in list(self.receivers.values()):
            info = receiver.info
            if info is not None and info.status == SNAP_OK and len(receiver.parts) < info.chunks:
                return info, receiver.received
        return None

    def on_packet(self, ptype: int, body: bytes) -> None:
        now = time.monotonic()
        if ptype == SNAP_INFO and len(body) >= SNAP_INFO_BODY.size:
            info = PhotoInfo.unpack(body)
            if info.status != SNAP_OK:
                self._fail(SNAP_PROBLEMS.get(info.status, f"no photo (status {info.status})"))
                self.asked_at = None
                return
            receiver = self._receiver(info.photo_id)
            if receiver is not None:
                receiver.on_info(info, now)
                self.asked_at = None
        elif ptype == SNAP_DATA and len(body) > SNAP_DATA_HEAD.size:
            photo_id, index = SNAP_DATA_HEAD.unpack_from(body)
            receiver = self._receiver(photo_id)
            if receiver is not None:
                receiver.on_data(index, body[SNAP_DATA_HEAD.size:], now)

    def _receiver(self, photo_id: int) -> Optional[PhotoReceiver]:
        if photo_id in self.finished:  # the relay missed our last ACK
            self.client.send_packet(SNAP_ACK, SNAP_ACK_HEAD.pack(photo_id, ACK_DONE | ACK_HAVE_INFO))
            return None
        if photo_id not in self.receivers:
            self.receivers[photo_id] = PhotoReceiver(photo_id)
        return self.receivers[photo_id]

    def pump(self, now: float) -> None:
        for photo_id, receiver in list(self.receivers.items()):
            if receiver.news and (receiver.complete or now - receiver.acked_at >= 0.5):
                receiver.news, receiver.acked_at = False, now
                self.client.send_packet(SNAP_ACK, receiver.ack())
            if receiver.complete:
                del self.receivers[photo_id]
                self.finished.add(photo_id)
                self._save(receiver.info, receiver.data())
            elif now - receiver.last_rx > self.GIVE_UP:
                del self.receivers[photo_id]
                if receiver.info is not None:
                    self._fail("the photo stopped arriving")
        if self.asked_at is not None and now - self.asked_at > self.ANSWER_WITHIN:
            self.asked_at = None
            self._fail("no photo came")

    def _save(self, info: PhotoInfo, data: bytes) -> None:
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(info.time or info.photo_id))
        path = os.path.join(self.folder, f"MavLTE_{stamp}_{info.photo_id}.jpg")
        meta = dict(info._asdict())
        if info.lat != UNKNOWN_I32:
            meta.update(latitude=info.lat / 1e7, longitude=info.lon / 1e7)
        if info.alt != UNKNOWN_I32:
            meta.update(altitude_m=round(info.alt / 1000, 1))
        try:
            os.makedirs(self.folder, exist_ok=True)
            with open(path, "wb") as f:
                f.write(data)
            with open(path[:-4] + ".json", "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=1)
        except OSError as exc:
            glog.warning("cannot save photo %d: %s", info.photo_id, exc)
            self._fail(f"cannot save the photo: {exc.strerror or exc}")
            return
        glog.info("photo %d saved: %s (%d KB)", info.photo_id, path, len(data) // 1024)
        self.last_path, self.last_info = path, info
        if self.on_photo is not None:
            self.on_photo(path, info)


class PhotoOutbox:
    """The aircraft's side of snapshots: takes the photo the relay asks for and sends it, no faster
    than the link carries without delaying the telemetry. capture(width, height) returns the JPEG, or
    None if that failed; without it, the aircraft answers that it has no camera. where() gives latitude,
    longitude (1e-7 degrees), altitude above home (mm) and heading (centidegrees) at that moment; cap()
    the highest photo rate for the network the aircraft is on (bytes/s)."""

    def __init__(self, client: TunnelClient, capture: Optional[Callable[[int, int], Optional[bytes]]] = None,
                 where: Optional[Callable[[], Tuple[int, int, int, int]]] = None,
                 cap: Optional[Callable[[], float]] = None) -> None:
        self.client = client
        self.capture = capture
        self.where = where
        self.cap = cap or (lambda: 8192.0)
        self.sender: Optional[PhotoSender] = None
        self.rate = RateControl(self.cap())
        self.rtt_at = 0.0
        self.answers: Dict[int, int] = {}  # the last requests: photo id -> SNAP_OK (sending) or the problem
        self.last: Optional[Tuple[PhotoInfo, bool, float]] = None  # the last photo: sent (or given up), seconds

    @property
    def busy(self) -> bool:
        return self.sender is not None and not (self.sender.done or self.sender.failed)

    def on_session(self) -> None:
        """A new session: the relay may be a new one too, that knows nothing of the photo on its way."""
        if self.busy:
            self.sender.restart(time.monotonic())

    def on_packet(self, ptype: int, body: bytes) -> None:
        now = time.monotonic()
        if ptype == SNAP_REQ and len(body) >= SNAP_REQ_BODY.size:
            photo_id, size = SNAP_REQ_BODY.unpack_from(body)
            if photo_id in self.answers:  # asked again: our answer has not reached the relay yet
                if self.answers[photo_id] != SNAP_OK:  # (a photo's SNAP_INFO goes again by itself)
                    self._answer(photo_id, self.answers[photo_id])
            elif self.busy:
                self._answer(photo_id, SNAP_BUSY)
            else:
                self.answers[photo_id] = self._take(photo_id, min(size, len(SNAP_SIZES) - 1), now)
                while len(self.answers) > 16:
                    del self.answers[next(iter(self.answers))]
        elif ptype == SNAP_ACK and self.sender is not None and len(body) >= SNAP_ACK_HEAD.size:
            photo_id, flags = SNAP_ACK_HEAD.unpack_from(body)
            if photo_id == self.sender.info.photo_id:
                self.sender.on_ack(flags, body[SNAP_ACK_HEAD.size:], now)

    def _answer(self, photo_id: int, status: int) -> None:
        self.client.send_packet(SNAP_INFO, PhotoInfo(photo_id, status=status).pack())

    def _take(self, photo_id: int, size: int, now: float) -> int:
        if self.capture is None:
            self._answer(photo_id, SNAP_NO_CAMERA)
            return SNAP_NO_CAMERA
        width, height = SNAP_SIZES[size]
        try:
            jpeg = self.capture(width, height)
        except Exception as exc:  # a camera fault must not take the link down with it
            vlog.warning("camera: %s", exc)
            jpeg = None
        if not jpeg or len(jpeg) > SNAP_MAX_BYTES:
            self._answer(photo_id, SNAP_FAILED)
            return SNAP_FAILED
        lat, lon, alt, heading = self.where() if self.where else (UNKNOWN_I32, UNKNOWN_I32, UNKNOWN_I32,
                                                                  UNKNOWN_HEADING)
        info = PhotoInfo(photo_id, len(jpeg), width, height, lat, lon, alt, heading)
        self.sender = PhotoSender(info, BytesSource(jpeg), self.client.send_packet, now)
        self.rate = RateControl(self.cap())
        vlog.info("photo %d taken: %d KB, %dx%d", photo_id, len(jpeg) // 1024, width, height)
        return SNAP_OK

    def pump(self, now: float) -> None:
        sender = self.sender
        if sender is None or not self.busy:
            if sender is not None:
                vlog.info("photo %d %s", sender.info.photo_id, "sent" if sender.done else "given up")
                self.last = (sender.info, sender.done, now - sender.started)
                self.sender = None
            return
        if now - self.rtt_at >= 1.0 and self.client.rtt_ms != U16_UNKNOWN:
            self.rtt_at = now
            self.rate.set_cap(self.cap())
            self.rate.rtt_sample(self.client.rtt_ms / 1000, now)
        sender.pump(now, self.rate, rto_for(self.client.rtt_ms))


# ---------------------------------------------------------------------------------------------
# Local endpoints of the GCS agent


def udp_plan(host: str, port: int, own: bool = False) -> Tuple[Optional[Tuple[str, int]], Tuple[str, int]]:
    """How the agent's UDP port works for an address: where it sends (None: it listens, and sends
    to the GCS software that talks to it) and the local address it binds.

    0.0.0.0 (or ::) listens on that port, as the TCP port does: GCS software on any computer that
    sends to this one's address and port (Mission Planner: UDPCl) gets the telemetry. An address of
    this computer's own (own) listens on that address. Any other address sends there, to GCS
    software that listens (Mission Planner and QGroundControl: on port 14550), from a port of the
    agent's own: a loopback address to GCS software on this computer, the default."""
    host = host or "127.0.0.1"
    if is_loopback(host):
        return (host, port), ("::1" if ":" in host else "127.0.0.1", 0)
    if host in ("0.0.0.0", "::") or own:
        return None, (host, port)
    return (host, port), ("::" if ":" in host else "0.0.0.0", 0)


def is_own_address(host: str) -> bool:
    """Whether an IP address is one of this computer's: one it can bind."""
    try:
        family = socket.AF_INET6 if ipaddress.ip_address(host).version == 6 else socket.AF_INET
        with socket.socket(family, socket.SOCK_DGRAM) as s:
            s.bind((host, 0))
        return True
    except (ValueError, OSError):
        return False


def lan_address() -> Optional[str]:
    """This computer's address on its network, the one other computers reach it at, or None."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))  # sends nothing: picks the interface toward other networks
            host = s.getsockname()[0]
    except OSError:
        return None
    return None if host.startswith("0.") else host


def udp_socket(local: Tuple[str, int]) -> socket.socket:
    """A UDP socket bound to `local`. On Windows a port of our choice is ours alone: otherwise another
    program could still bind it on one address, 127.0.0.1 say, and take the packets sent there."""
    sock = socket.socket(socket.AF_INET6 if ":" in local[0] else socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if local[1] and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        sock.bind(local)
        sock.setblocking(False)
    except OSError:
        sock.close()
        raise
    return sock


class LocalUdp(asyncio.DatagramProtocol):
    """GCS software on UDP. With a target, sends there (Mission Planner and QGroundControl listen on
    127.0.0.1:14550). Without one it listens, like a server: it sends to every address that sent
    something lately, so that several GCS programs, on any computers, can connect (see udp_plan)."""

    PEER_TIMEOUT = 10.0  # seconds: GCS software sends a heartbeat every second
    MAX_PEERS = 8

    def __init__(self, target: Optional[Tuple[str, int]], on_input: Callable[[bytes], None]) -> None:
        self.target = target
        self.on_input = on_input
        self.batcher = Batcher()
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.last_rx = 0.0  # when the GCS last sent something (Mission Planner: a heartbeat every second)
        self.peers: Dict[tuple, float] = {}  # address: when it last sent something

    @property
    def local(self) -> Tuple[str, int]:
        """The local address and port: the ones to connect to, when listening."""
        return self.transport.get_extra_info("sockname")[:2] if self.transport is not None else ("", 0)

    @property
    def port(self) -> int:
        return self.local[1]

    def gcs_count(self, within: float = 3.0) -> int:
        """GCS software heard from lately."""
        now = time.monotonic()
        return sum(now - seen < within for seen in list(self.peers.values()))

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
        if addr not in self.peers and len(self.peers) >= self.MAX_PEERS:
            del self.peers[min(self.peers, key=self.peers.get)]
        self.peers[addr] = now
        for chunk in self.batcher.feed(data, now):
            self.on_input(chunk)
        chunk = self.batcher.poll(now, 0)
        if chunk:
            self.on_input(chunk)

    def send(self, payload: bytes) -> None:
        if self.transport is None:
            return
        if self.target:
            to = [self.target]
        else:  # listening: to each GCS program heard from lately
            now = time.monotonic()
            for addr, seen in list(self.peers.items()):
                if now - seen > self.PEER_TIMEOUT:
                    del self.peers[addr]
            to = list(self.peers)
        for addr in to:
            try:
                self.transport.sendto(payload, addr)
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
    relay = RelayServer(keys, session_timeout=opts.session_timeout, photo_folder=getattr(opts, "snapshot_dir", None),
                        photo_days=getattr(opts, "snapshot_days", 7.0), state_dir=getattr(opts, "state_dir", None))
    loop = asyncio.get_running_loop()
    host, port = opts.listen
    await loop.create_datagram_endpoint(lambda: relay, local_addr=(host or "0.0.0.0", port))
    slog.info("MavLTE relay %s listening on udp %s", __version__, fmt_addr((host or "0.0.0.0", port)))
    if opts.tcp_listen:
        relay.tcp = TcpGcsPort(relay, opts.tcp_allow)
        await relay.tcp.start(*opts.tcp_listen)
        allowed = ", ".join(str(n) for n in opts.tcp_allow)
        slog.info("plain TCP port for GCS software on %s, allowed: %s", fmt_addr(opts.tcp_listen), allowed)
    last_tick = time.monotonic()
    while True:
        await asyncio.sleep(0.05)  # photos move in small steps; the rest once a second
        now = time.monotonic()
        relay.photos.pump(now)
        if now - last_tick >= 1.0:
            last_tick = now
            relay.tick(now)


class StatusPrinter:
    """Logs the vehicle link status: at once when it changes, otherwise every `interval` seconds.
    where() adds the aircraft's position, if the agent knows it."""

    def __init__(self, interval: float, where: Optional[Callable[[], str]] = None) -> None:
        self.interval = interval
        self.where = where
        self.state: Optional[Tuple[bool, bool]] = None
        self.last = 0.0

    def __call__(self, status: LinkStatus) -> None:
        now = time.monotonic()
        state = (status.online, status.idle_ms == U16_UNKNOWN, status.voice_text())
        if state != self.state or now - self.last >= self.interval:
            where = self.where() if self.where else ""
            glog.info("%s%s", status.describe(), f"; {where}" if where else "")
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

    def __init__(self, server: Tuple[str, int], key: bytes, on_status=None, info: Optional[str] = None,
                 photo_dir: Optional[str] = None,
                 on_photo: Optional[Callable[[str, PhotoInfo], None]] = None) -> None:
        self.client = TunnelClient(ROLE_GCS, key, server[0], server[1], on_data=self._to_gcs,
                                   on_status=self._got_status, info=info or f"mavlte-agent/{__version__}")
        self.client.ping_flags = PING_FLAG_WATCHING  # until a port is on
        self.on_status = on_status
        self.status: Optional[LinkStatus] = None
        self.status_time = 0.0
        self.heard_at: Optional[float] = None  # when the relay last heard the aircraft (monotonic), from STATUS
        self.udp: Optional[LocalUdp] = None
        self.tcp: Optional[LocalTcp] = None
        self.to_gcs_bytes = 0  # MAVLink from the aircraft, handed to GCS software
        self.from_gcs_bytes = 0  # MAVLink from GCS software, sent to the aircraft
        # snapshots, for an agent that keeps photos (the MavLTE app)
        self.photos = PhotoInbox(self.client, photo_dir, on_photo) if photo_dir else None
        if self.photos is not None:
            self.client.on_session = self.photos.sync
        # the locator: the aircraft's own position, from its LTE module's GNSS
        self.position: Optional[Position] = None  # the newest: live, or the last known one from the relay
        self.last_fix: Optional[Position] = None
        self.module_hot = False  # its chip reached TEMP_HOT, and has not cooled below TEMP_WARM since
        self.module_on_cell: Optional[bool] = None  # it runs on its own cell (False: on external power)
        self.client.on_packet = self._on_packet
        # the locator voice: the switch asked for, sent again until the relay's STATUS shows it
        self.voice_request: Optional[bool] = None
        self.voice_until = 0.0

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

    def vehicle_silence(self) -> Optional[float]:
        """Seconds since the relay last heard the aircraft, counted on past the 65.5 s where STATUS's idle_ms
        tops out; None without an aircraft at the relay, or when it was already that long silent when this
        agent first heard of it (only "over a minute" is known then)."""
        return None if self.heard_at is None else time.monotonic() - self.heard_at

    def _got_status(self, status: LinkStatus) -> None:
        self.status, self.status_time = status, time.monotonic()
        if not status.connected:
            self.heard_at = None
        elif status.idle_ms < IDLE_CAPPED:
            self.heard_at = self.status_time - status.idle_ms / 1000
        elif self.heard_at is not None and self.status_time - self.heard_at < IDLE_CAPPED / 1000:
            self.heard_at = None  # it cannot have been heard that recently: what we knew is out of date
        if self.voice_request is not None:
            if status.voice_on == self.voice_request:
                self.voice_request = None
            elif self.status_time < self.voice_until:
                self._send_voice()
            else:
                glog.warning("the relay did not switch the locator voice %s (is it older than 1.5.0?)",
                             "on" if self.voice_request else "off")
                self.voice_request = None
        if self.on_status is not None:
            self.on_status(status)

    VOICE_TRIES_FOR = 10.0  # seconds

    def set_voice(self, on: bool) -> bool:
        """Switches the aircraft's locator voice on or off. The relay keeps the switch and passes it on to
        the aircraft, now or whenever it connects. False without a session with the relay."""
        if not self.client.is_connected:
            return False
        self.voice_request, self.voice_until = on, time.monotonic() + self.VOICE_TRIES_FOR
        glog.info("switching the aircraft's locator voice %s", "on" if on else "off")
        self._send_voice()
        return True

    def _send_voice(self) -> None:
        self.client.send_packet(VOICE, VOICE_BODY.pack(1 if self.voice_request else 0))

    def _on_packet(self, ptype: int, body: bytes) -> None:
        if ptype == POSITION:
            if len(body) >= POSITION_BODY.size:
                self._got_position(Position.unpack(body))
        elif self.photos is not None:
            self.photos.on_packet(ptype, body)

    POSITION_LIVE = 15.0  # seconds: an older report (the relay's last known one) is history

    def _got_position(self, pos: Position) -> None:
        old = self.position
        self.position = pos
        if pos.has_fix and (self.last_fix is None or pos.time >= self.last_fix.time):
            self.last_fix = pos
        age = time.time() - pos.time
        if pos.time and age >= self.POSITION_LIVE:
            glog.info("last known position of the aircraft, %s ago: %s", fmt_age(age), pos.describe())
        elif pos.fc_is_silent and not (old and old.fc_is_silent):
            glog.warning("the aircraft's flight controller is silent (%s s); its LTE module still reports: %s",
                         pos.fc_silent, pos.describe())
        elif old is None or old.has_fix != pos.has_fix or old.fc_is_silent != pos.fc_is_silent:
            glog.info("aircraft's position (its own GNSS): %s", pos.describe())
        if pos.temp != TEMP_UNKNOWN and (not pos.time or age < self.POSITION_LIVE):
            if pos.temp >= TEMP_HOT and not self.module_hot:
                self.module_hot = True
                glog.warning("the aircraft's LTE module is hot: its chip is at %d °C. Give it air, out of the sun",
                             pos.temp)
            elif pos.temp < TEMP_WARM and self.module_hot:
                self.module_hot = False
                glog.info("the aircraft's LTE module has cooled down to %d °C", pos.temp)
        if pos.battery_mv != U16_UNKNOWN and (not pos.time or age < self.POSITION_LIVE):
            on_cell = not pos.on_external_power  # the flight battery (BEC) gone: a crash, or unplugged
            if self.module_on_cell is not None and on_cell != self.module_on_cell:
                if on_cell:
                    glog.warning("the aircraft's LTE module runs on its own cell now (%s)", pos.power_text())
                else:
                    glog.info("the aircraft's LTE module has external power again")
            self.module_on_cell = on_cell

    def position_text(self) -> str:
        """The aircraft's last known position for the log, or ''."""
        pos = self.last_fix
        if pos is None:
            return ""
        age = f", {fmt_age(time.time() - pos.time)} ago" if pos.time else ""
        return f"GNSS {pos.lat / 1e7:.6f}, {pos.lon / 1e7:.6f} ({pos.sats} satellites{age})"

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
        """Sends to GCS software at `target`, or listens there if it is 0.0.0.0 or an address of this
        computer (see udp_plan). Raises OSError if the name does not resolve, or the port is taken."""
        if self._close_udp():
            await asyncio.sleep(0)  # its socket closes on the loop's next turn, and frees the port
        loop = asyncio.get_running_loop()
        host, port = target[0] or "127.0.0.1", target[1]
        if host not in ("0.0.0.0", "::"):  # a name resolves once, to the address replies come from
            try:
                infos = await loop.getaddrinfo(host, port, family=socket.AF_INET6 if ":" in host else socket.AF_INET,
                                               type=socket.SOCK_DGRAM)
            except UnicodeError as exc:  # a name the IDNA codec refuses
                raise socket.gaierror(f"{host}: {exc}") from None
            host = infos[0][4][0]
        send_to, local = udp_plan(host, port, own=is_own_address(host))
        udp = LocalUdp(send_to, self._from_gcs)
        try:
            await loop.create_datagram_endpoint(lambda: udp, sock=udp_socket(local))
            self.udp = udp
        finally:
            self._update_watching()
        if send_to is None:
            here = lan_address() if host in ("0.0.0.0", "::") else host
            glog.info("listening on udp %s (Mission Planner on any computer: connect UDPCl to %s, port %d)",
                      fmt_addr(local), here or "this computer's address", port)
        else:
            glog.info("sending MAVLink to udp %s (Mission Planner%s: connect UDP, port %d)", fmt_addr(send_to),
                      "" if is_loopback(host) else " there", port)

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
        """Raises OSError if the port is taken, socket.gaierror if the name does not resolve."""
        self._close_tcp()
        host = bind[0] or "127.0.0.1"
        tcp = LocalTcp(self._from_gcs)
        try:
            await tcp.start(host, bind[1])
            self.tcp = tcp
        except UnicodeError as exc:  # a name the IDNA codec refuses (a doubled dot, say), as in start_udp
            raise socket.gaierror(f"{host}: {exc}") from None
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
        photos = asyncio.ensure_future(self._photo_loop()) if self.photos is not None else None
        try:
            await self.client.run()
        finally:
            if photos is not None:
                photos.cancel()
            self.close_ports()

    async def _photo_loop(self) -> None:
        while True:
            await asyncio.sleep(0.1)
            self.photos.pump(time.monotonic())


async def run_gcs(opts) -> None:
    printer = StatusPrinter(opts.status_interval)
    agent = GcsAgent(opts.server, opts.key, on_status=printer)
    printer.where = agent.position_text
    if opts.udp:
        try:
            await agent.start_udp(opts.udp)
        except OSError as exc:
            glog.warning("cannot use udp %s: %s", fmt_addr(opts.udp), exc)
    if opts.tcp:
        try:
            await agent.start_tcp(opts.tcp)
        except OSError as exc:
            glog.warning("cannot use tcp %s: %s", fmt_addr(opts.tcp), exc)
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
    photos = PhotoOutbox(client)  # no camera here: it says so when asked for a photo
    client.on_packet, client.on_session = photos.on_packet, photos.on_session

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

    async def voice() -> None:  # no speaker here: it reports that it sounds, as the ESP32 does
        on = False
        while True:
            await asyncio.sleep(0.2)
            if client.voice_on != on:
                on = client.voice_on
                vlog.info("locator voice %s", "on (this vehicle has no speaker: it only says it sounds)" if on else "off")
                client.ping_flags = PING_FLAG_SPEAKING if on else 0
                client.ping_now()

    asyncio.ensure_future(flusher())
    asyncio.ensure_future(voice())
    vlog.info("connecting to relay %s:%d", *opts.server)
    await client.run()


# ---------------------------------------------------------------------------------------------
# Command line


def load_config(path: Optional[str], section: str) -> Dict[str, str]:
    if not path:
        return {}
    # interpolation=None: values are read as written, so a '%' (in a vehicle name, say) is just a '%'.
    # strict=False: a section or key written twice counts once, the later value winning (before 1.5.3,
    # update_config wrote a second [gcs] after a header line with a comment on it)
    parser = configparser.ConfigParser(inline_comment_prefixes=("#", ";"), interpolation=None, strict=False)
    try:
        if not parser.read(path, encoding="utf-8"):
            raise SystemExit(f"cannot read config file {path}")
    except (configparser.Error, UnicodeDecodeError) as exc:
        raise SystemExit(f"cannot read config file {path}: {exc}") from None
    return dict(parser[section]) if parser.has_section(section) else {}


def ini_section(line: str) -> Optional[str]:
    """The section a line of an INI file opens, as load_config reads it ("[gcs]  # laptop" -> "gcs"), or None."""
    text = re.split(r"(?:^|\s)[#;]", line, maxsplit=1)[0].strip()
    m = configparser.ConfigParser.SECTCRE.match(text)
    return m.group("header") if m else None


def update_config(path: str, section: str, values: Dict[str, str], remove=()) -> None:
    """Sets keys in one section of an INI file (and drops those in `remove`), keeping everything else,
    comments included."""
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        lines = []
    start = next((i for i, line in enumerate(lines) if ini_section(line) == section), None)
    if start is None:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(f"[{section}]")
        start = len(lines) - 1
    end = next((i for i in range(start + 1, len(lines)) if ini_section(lines[i]) is not None), len(lines))
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
    s.add_argument("--snapshot-dir", help="folder for the aircraft's photos, or 'off' (default: snapshots, in "
                                          "the service's state directory or next to the config file)")
    s.add_argument("--snapshot-days", type=float, help="days to keep photos (default 7)")
    s.add_argument("--log-level", help="debug, info, warning (default info)")

    g = sub.add_parser("gcs", help="run next to Mission Planner / QGroundControl")
    g.add_argument("--config", help="INI file with a [gcs] section")
    g.add_argument("--server", help="relay address, host:port")
    g.add_argument("--key", help="hex GCS key (same as gcs_key on the server)")
    g.add_argument("--udp", help="send MAVLink to GCS software at this UDP address (default 127.0.0.1:14550), "
                                 "or listen for it on any computer with 0.0.0.0:port; 'off' to disable")
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
        # photos, the aircraft's last known position and the locator voice switch: in systemd's StateDirectory
        # (/var/lib/mavrelay) when run as the service
        home = os.environ.get("STATE_DIRECTORY") or os.path.dirname(os.path.abspath(getattr(args, "config", None)
                                                                                      or "mavrelay.ini"))
        out.state_dir = home
        folder = str(o.get("snapshot_dir", os.path.join(home, "snapshots")))
        out.snapshot_dir = None if folder.lower() in ("off", "no", "none", "") else folder
        out.snapshot_days = float(o.get("snapshot_days", 7))
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

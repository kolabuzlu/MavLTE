#!/usr/bin/env python3
"""MavLTE Plane Simulator: the aircraft end of MavLTE on this PC, until the board arrives.

The plane's two power switches, in a window beside the MavLTE app (both start off):

- Battery: the whole plane. On starts ArduPlane SITL as the flight controller; off stops it at
  once, like pulling the battery plug.
- LTE module: the modem on the ESP32 board. On, it starts up like the real one (about 16 s, or a
  couple of seconds with Quick start), then carries the flight controller's MAVLink to your relay
  as the ESP32 firmware does. Off cuts it without a goodbye, so the relay only notices the
  silence, as it would in the air.

The module's LED shows what the RGB LED on the board shows. Network picks what the plane flies
through: no coverage, 2G (EDGE) or LTE, with figures for an aircraft (worse than for a phone on the
ground) and the troubles that make a real link patchy: latency spikes, fades, cell changes and
dropouts, at random. The A7670E falls back to 2G where there is no LTE. Signal, weak to excellent,
sets the level the module reports (the bars in the MavLTE app), slows the link, and makes the
troubles more frequent and longer. From a fair signal down, 2G is slower than the telemetry, so it
queues up and packets are lost, as they would be in the air. The network chosen in MavLTE (Auto, 2G
or LTE) applies as on the board: 2G is there wherever LTE is, and LTE only finds nothing where there
is only 2G.

The board's camera is there too, with its own switch (the CAM DIP switch on the board): MavLTE's
Snapshot button gets a picture of sky and fields, seen as the plane flies, rolls and pitches, of the
size a real one would be, sent as the firmware sends it, without crowding out the telemetry.

    python plane_sim.py            (or double-click PlaneSim.pyw)

The relay server and vehicle key come from the [vehicle] section of mavrelay.ini. Mission Planner
can also reach the plane directly on its "USB port", TCP 127.0.0.1:5780.
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import io
import logging
import math
import os
import queue
import random
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk
from typing import Callable, Dict, Optional, Tuple

import mavlte as ui
import mavrelay as mr
import sitl_demo

try:
    from PIL import Image, ImageDraw, ImageFilter
except ImportError:  # no camera then: the aircraft answers that it has none
    Image = None

APP = "MavLTE Plane Simulator"
TITLE = f"{APP} V{mr.__version__}"
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mavrelay.ini")
ICON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "planesim.png")
FC_PORT = 5762  # SITL SERIAL1: the TELEM port the ESP32 is wired to
LOGGERS = ("mavrelay", "4g-link")

# The A7670E is an LTE Cat-1 modem that falls back to 2G (GSM with EDGE) where there is no LTE; it
# has no 3G, so where a Turkish operator offers only 3G it is on 2G. Figures for an aircraft, which
# fares worse than a phone on the ground: from above the rooftops it sees many cells at once, so
# interference is high and handovers frequent (3GPP TR 36.777). With an excellent signal: name,
# access technology reported to the relay (3GPP AcT: 3 = GSM with EDGE, 7 = LTE), uplink and
# downlink (kbit/s), delay added each way on top of this PC's own path to the relay, and its jitter
# (ms), packet loss (%).
NETWORKS = (
    ("No connection", None, 0, 0, 0, 0, 100.0),
    ("2G (EDGE)", 3, 40, 80, 300, 150, 2.0),
    ("LTE (4G)", 7, 2000, 5000, 30, 20, 0.5),
)
NO_CONNECTION, NET_2G, NET_LTE = range(3)
# What makes a mobile link falter, each kind at random, with a good signal: kind, mean seconds between
# two, how long it lasts (s, from-to), delay it adds (ms, from-to), share of packets it loses. It is
# these, more than the averages, that make telemetry over a mobile network patchy.
EVENTS = {
    NET_2G: (
        ("latency spike", 20, (1, 4), (500, 2000), 0.0),  # radio retransmissions, uplink set up again
        ("fade", 45, (0.5, 2), (0, 0), 0.5),  # interference
        ("cell change", 60, (1.5, 4), (0, 0), 1.0),  # 2G data has no handover: no data meanwhile
        ("dropout", 300, (5, 15), (0, 0), 1.0),  # the network lost for a while
    ),
    NET_LTE: (
        ("handover", 30, (0.05, 0.15), (50, 150), 0.0),  # data held back meanwhile, not lost
        ("latency spike", 30, (1, 3), (200, 1000), 0.0),
        ("fade", 60, (0.3, 1.5), (0, 0), 0.4),
        ("dropout", 300, (2, 8), (0, 0), 1.0),  # radio link failure, then connecting again
    ),
}
# Signal, weak to excellent: the level the module reports (dBm), and what it does to the link: share
# of the speed left (radio link adaptation), delay (ms) and loss (%) added, and how many times more
# often and how much longer the link falters than with a good signal.
SIGNALS = (
    ("Weak", -103, 0.25, 80, 3.0, 4.0, 1.6),
    ("Fair", -93, 0.45, 30, 1.0, 2.0, 1.3),
    ("Good", -83, 0.7, 10, 0.2, 1.0, 1.0),
    ("Excellent", -70, 1.0, 0, 0.0, 0.5, 0.8),
)
GOOD_SIGNAL = 2
RAT_SWITCH_S = 3.0  # no data while the modem moves between 2G and LTE
REGISTER_AGAIN_S = 4.0  # no data while it registers again after having no coverage
MAX_BACKLOG_S = 2.0  # data queued in the network beyond this much sending time is dropped (full buffer)
# The camera: JPEG quality that gives the sizes of the board's OV5640 (Small 5-10 KB, Medium 10-30 KB,
# Large 25-80 KB), and the most of the uplink a photo may take (bytes/s), as the firmware allows it:
# the rest stays for the telemetry, and less when the round trip shows the link filling up.
CAMERA_QUALITY = 80
PHOTO_CAP = {NET_2G: 2048, NET_LTE: 32768}
# The LTE module's own GNSS (the A7670E has one): time to its first fix after the module powers on (a
# cold start; the real one takes half a minute or so under an open sky), and its typical error.
GNSS_TTFF = 25.0
GNSS_TTFF_QUICK = 3.0
GNSS_ERROR_M = 2.0
LOCATOR_INTERVAL = 5.0  # seconds between the module's position reports, as the firmware sends them
# what a V2 board's fuel gauge reads while the flight battery (the BEC) powers it: its supply rail, not the cell
RAIL_MV = 4298
VOICE_SOUND = "the two-tone alarm"  # what the board's locator voice plays (the firmware's default)
# The 18650 cell in the board's holder keeps the module on without the flight battery: a 3000 mAh cell at
# about 150 mA (ESP32, the modem idling between reports, GNSS) lasts some 20 hours.
CELL_HOURS = 20.0
# The module's chip temperature, as its ESP32-S3 measures it: the air in the fuselage plus the board's
# own heat while it runs (the modem's transmitter most), reached with a lag. Parked in the summer sun,
# a closed fuselage gets hot enough for MavLTE's amber (from 70 degrees C) and red (from 80).
AIR_C = 25.0
SUN_AIR_C = 65.0
SELF_HEAT_C = 20.0
CHIP_LAG_S = 40.0  # a real board takes minutes; quicker here, to see it happen


def speed_text(bytes_per_s: float) -> str:
    kbit = bytes_per_s * 8 / 1000
    return f"{kbit / 1000:g} Mbit/s" if kbit >= 1000 else f"{kbit:.0f} kbit/s"


def link_figures(network: int, signal: int) -> Optional[Tuple[float, float, float, float, float]]:
    """Uplink and downlink (bytes/s), delay and jitter each way (s) and loss (0-1) of a network at a
    signal level; None where there is no coverage."""
    _, rat, up, down, delay, jitter, loss = NETWORKS[network]
    if rat is None:
        return None
    _, _, share, extra_delay, extra_loss, _, _ = SIGNALS[signal]
    return (up * 125 * share, down * 125 * share, (delay + extra_delay) / 1000, jitter / 1000,
            min(1.0, (loss + extra_loss) / 100))


class Events:
    """The link's random troubles (EVENTS): each kind comes at random times, on average once per its
    interval, and lasts a random while. Worked out lazily, as packets pass."""

    def __init__(self, rng: random.Random) -> None:
        self.rng = rng
        self.kinds: tuple = ()
        self.rate = self.length = 1.0
        self.next_at: Dict[str, float] = {}
        self.active: Dict[str, Tuple[float, float, float]] = {}  # kind: until, delay added (s), loss
        self.on_start: Callable[[str, float, float, float], None] = lambda kind, start, length, loss: None

    def configure(self, network: int, signal: int, now: float) -> None:
        self.kinds = EVENTS.get(network, ())
        self.rate, self.length = SIGNALS[signal][5:7]
        self.active = {kind: a for kind, a in self.active.items() if any(k[0] == kind for k in self.kinds)}
        # random times without memory: drawing them again from now changes nothing but the rate
        self.next_at = {kind: now + self.rng.expovariate(self.rate / mean) for kind, mean, *_ in self.kinds}

    def now(self, now: float) -> Tuple[float, float]:
        """The delay added (s) and the share of packets lost at this moment."""
        delay = loss = 0.0
        for kind, mean, (d0, d1), (x0, x1), share in self.kinds:
            start = self.next_at[kind]
            if now >= start and kind not in self.active:
                length = self.rng.uniform(d0, d1) * self.length
                self.active[kind] = (start + length, self.rng.uniform(x0, x1) / 1000, share)
                self.next_at[kind] = start + length + self.rng.expovariate(self.rate / mean)
                self.on_start(kind, start, length, share)
            if kind in self.active:
                until, extra, lost = self.active[kind]
                if now >= until:
                    del self.active[kind]
                else:
                    delay, loss = delay + extra, max(loss, lost)
        return delay, loss
# seconds the modem spends starting, registering, and bringing mobile data up: as in the boot log
# of a real board (README, "A healthy start"), and shortened for Quick start
STARTUP_REAL = (9.3, 4.3, 1.9)
STARTUP_QUICK = (1.0, 0.6, 0.3)

PLANE_MODES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA", 6: "FBWB",
               7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF", 14: "AVOID_ADSB",
               15: "GUIDED", 17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL",
               22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL", 25: "LOITER2QLAND", 26: "AUTOLAND"}

YELLOW = "#e6c84a"
PURPLE = "#c26be0"
FC_SILENT = 10.0  # s without the flight controller's HEARTBEAT: the board's LED turns purple

log = logging.getLogger("mavrelay.plane")


# ---------------------------------------------------------------------------------------------
# The aircraft


class FcState:
    """What the flight controller says about itself, read from its MAVLink stream."""

    HEARTBEAT = struct.Struct("<IBBBBB")  # custom_mode, type, autopilot, base_mode, system_status, version

    def __init__(self) -> None:
        self.buf = bytearray()
        self.heartbeat = 0.0  # when the last one came (time.monotonic)
        self.mode = ""
        self.armed = False
        self.volts: Optional[float] = None
        self.alt: Optional[float] = None  # metres above home
        # for the camera and the LTE module's GNSS: where the plane is and how it lies in the air
        self.lat = self.lon = mr.UNKNOWN_I32  # 1e-7 degrees
        self.alt_mm = mr.UNKNOWN_I32  # above home
        self.alt_msl_mm = mr.UNKNOWN_I32  # above sea level
        self.heading = mr.UNKNOWN_HEADING  # centidegrees
        self.vx = self.vy = 0  # cm/s, north and east
        self.roll = self.pitch = 0.0  # degrees, right wing down and nose up positive
        self.last_rx = 0.0  # when the flight controller last sent anything (time.monotonic), 0: never

    def feed(self, data: bytes, now: float) -> None:
        if data:
            self.last_rx = now
        buf = self.buf
        buf += data
        i = 0
        while True:
            while i < len(buf) and buf[i] not in (mr.MAV_STX_V2, mr.MAV_STX_V1):
                i += 1
            if i + 3 > len(buf):
                break
            plen = buf[i + 1]
            if buf[i] == mr.MAV_STX_V2:
                if buf[i + 2] & ~mr.MAV_IFLAG_SIGNED:  # not a frame start after all
                    i += 1
                    continue
                size = 12 + plen + (mr.MAV_SIGNATURE_LEN if buf[i + 2] & mr.MAV_IFLAG_SIGNED else 0)
                if i + size > len(buf):
                    break
                sysid, compid = buf[i + 5], buf[i + 6]
                msgid = int.from_bytes(buf[i + 7:i + 10], "little")
                payload = bytes(buf[i + 10:i + 10 + plen])
            else:
                size = 8 + plen
                if i + size > len(buf):
                    break
                sysid, compid, msgid = buf[i + 3], buf[i + 4], buf[i + 5]
                payload = bytes(buf[i + 6:i + 6 + plen])
            if sysid == 1 and compid == 1:
                self._message(msgid, payload.ljust(40, b"\0"), now)  # MAVLink 2 drops trailing zeros
            i += size
        del buf[:i]

    def _message(self, msgid: int, payload: bytes, now: float) -> None:
        if msgid == 0:  # HEARTBEAT
            custom_mode, _, _, base_mode, _, _ = self.HEARTBEAT.unpack_from(payload)
            self.mode = PLANE_MODES.get(custom_mode, f"mode {custom_mode}")
            self.armed = bool(base_mode & 0x80)
            self.heartbeat = now
        elif msgid == 1:  # SYS_STATUS: voltage_battery in mV
            mv = struct.unpack_from("<H", payload, 14)[0]
            self.volts = None if mv == 0xFFFF else mv / 1000
        elif msgid == 30:  # ATTITUDE: roll and pitch in radians
            roll, pitch = struct.unpack_from("<ff", payload, 4)
            if math.isfinite(roll) and math.isfinite(pitch):
                self.roll, self.pitch = math.degrees(roll), math.degrees(pitch)
        elif msgid == 33:  # GLOBAL_POSITION_INT: lat, lon (1e-7 degrees), alt, relative_alt (mm), vx, vy
            # (cm/s), hdg (cdeg)
            self.lat, self.lon, self.alt_msl_mm, self.alt_mm = struct.unpack_from("<iiii", payload, 4)
            self.vx, self.vy = struct.unpack_from("<hh", payload, 20)
            self.heading = struct.unpack_from("<H", payload, 26)[0]
            self.alt = self.alt_mm / 1000


def camera_picture(width: int, height: int, fc: FcState, quality: int = CAMERA_QUALITY) -> bytes:
    """What the simulator's camera sees, looking ahead: sky, and fields that change as the plane flies
    (the same place gives the same fields), the horizon tilted as it rolls and moved as it pitches,
    sensor noise, and a caption. A JPEG of about the size the board's camera would make."""
    known = fc.lat != mr.UNKNOWN_I32
    rng = random.Random(f"{fc.lat // 10000},{fc.lon // 10000}" if known else 0)  # 100 m squares
    big = int(math.hypot(width, height)) + 4  # drawn larger, turned by the roll, then cut to size
    img = Image.new("RGB", (big, big))
    d = ImageDraw.Draw(img)
    pitch = max(-40.0, min(40.0, fc.pitch))
    horizon = max(0, min(big, big // 2 + int(pitch / 40 * height)))  # nose down: the horizon rises
    for y in range(horizon):  # the sky, deeper blue upwards
        t = y / max(1, horizon)
        d.line([(0, y), (big, y)], fill=(int(70 + 90 * t), int(120 + 80 * t), int(200 + 45 * t)))
    d.rectangle([0, horizon, big, big], fill=(92, 110, 70))
    colours = ((98, 128, 60), (120, 140, 70), (150, 140, 90), (84, 110, 52), (170, 160, 110), (110, 96, 64),
               (132, 150, 80))
    y, row = horizon, 0
    while y < big:  # rows of fields, larger towards the camera
        h = 2 + int(row * row * 0.9)
        x = -rng.randrange(0, 60)
        while x < big:
            w = rng.randrange(30, 90) * (1 + row // 3)
            d.rectangle([x, y, x + w, y + h], fill=rng.choice(colours))
            if rng.random() < 0.15:  # a farm track
                d.line([(x, y), (x + w, y + h)], fill=(190, 180, 150), width=max(1, row // 4))
            x += w + rng.randrange(0, 3)
        y += h + 1
        row += 1
    img = img.rotate(fc.roll, resample=Image.BILINEAR)
    left, top = (big - width) // 2, (big - height) // 2
    img = img.crop((left, top, left + width, top + height))
    noise = Image.effect_noise((width, height), 22).convert("RGB")  # real photos are neither flat nor sharp
    img = Image.blend(img, noise, 0.12).filter(ImageFilter.GaussianBlur(0.6))
    caption = [time.strftime("%H:%M:%S"), "MavLTE Plane Simulator"]
    if fc.alt is not None:
        caption.insert(1, f"{fc.alt:.0f} m")
    if known:
        caption.insert(-1, f"{fc.lat / 1e7:.5f}, {fc.lon / 1e7:.5f}")
    d = ImageDraw.Draw(img)
    d.rectangle([0, height - 16, width, height], fill=(0, 0, 0))
    d.text((4, height - 14), "  ".join(caption), fill=(255, 255, 255))
    out = io.BytesIO()
    img.save(out, "JPEG", quality=quality)
    return out.getvalue()


class Network(sitl_demo.LinkEmulator):
    """The mobile network between the module and the relay: in each direction a speed limit with a
    queue in front of it, then delay, jitter and loss, and the link's random troubles (Events); packets
    arrive in order as on a real mobile network. close() is the module losing power: nothing more
    leaves it, not even packets on their way."""

    def __init__(self, relay_addr, network: int, signal: int, rng: Optional[random.Random] = None) -> None:
        super().__init__(relay_addr, 0, 0, 0)
        self.closed = False
        self.rng = rng or random.Random()
        self.events = Events(self.rng)
        self.events.on_start = self._event_started
        self.figures = None
        self.gap_until = 0.0  # no data at all before this time (the modem changing networks)
        self.busy_until = {True: 0.0, False: 0.0}  # per direction (True: uplink): its queue is sent by then
        self.last_arrival = {True: 0.0, False: 0.0}
        self.cuts: list = []  # when coverage went, or the modem changed networks
        self.packets = 0  # both directions, offered and dropped
        self.lost = 0
        self.set_network(network, signal)

    def set_network(self, network: int, signal: int, gap: float = 0.0, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        self.figures = link_figures(network, signal)
        self.events.configure(network, signal, now)
        if self.figures is None or gap:
            self._cut(now)
            self.gap_until = max(self.gap_until, now + gap)

    def _cut(self, now: float) -> None:
        """What still waits in the queue is lost with the coverage; what was sent already arrives."""
        self.cuts = [t for t in self.cuts if now - t < 30.0] + [now]
        self.busy_until = {True: now, False: now}

    def _event_started(self, kind: str, start: float, length: float, loss: float) -> None:
        if loss >= 1.0:  # no data at all for a while
            self._cut(start)
            log.info("LTE module: %s, no data for %.1f s", kind, length)

    def condition(self, now: float) -> str:
        """What troubles the link right now, for the window."""
        return ", ".join(kind for kind, (until, _, _) in self.events.active.items() if now < until)

    def schedule(self, uplink: bool, size: int, now: float) -> Optional[Tuple[float, float]]:
        """When a packet of this many bytes, handed over now, is sent and when it arrives; None if the
        network drops it."""
        f = self.figures
        if f is None or now < self.gap_until:
            return None
        extra, burst = self.events.now(now)
        if self.rng.random() < max(f[4], burst):
            return None
        start = max(now, self.busy_until[uplink])
        if start - now > MAX_BACKLOG_S:  # the queue is full
            return None
        sent = self.busy_until[uplink] = start + (size + sitl_demo.IP_UDP_HEADERS) / (f[0] if uplink else f[1])
        arrival = sent + max(0.0, f[2] + extra + self.rng.uniform(-f[3], f[3]))
        arrival = max(arrival, self.last_arrival[uplink])  # no overtaking
        self.last_arrival[uplink] = arrival
        return sent, arrival

    def _forward(self, transport, data: bytes, addr) -> None:
        now = time.monotonic()
        self.packets += 1
        timing = self.schedule(transport is self.back, len(data), now)
        if timing is None:
            self.lost += 1
            return
        sent, arrival = timing
        asyncio.get_running_loop().call_later(arrival - now, self._send, transport, data, addr, now, sent)

    def _send(self, transport, data: bytes, addr, queued: float, sent: float) -> None:
        if any(queued <= cut < sent for cut in self.cuts):  # still queued when the coverage went
            self.lost += 1
        elif not self.closed:
            transport.sendto(data, addr)

    def close(self) -> None:
        self.closed = True
        for transport in (self.front, self.back):
            if transport is not None:
                transport.close()


class Gnss:
    """The LTE module's GNSS receiver: no fix for a while after the module powers on (a cold start),
    then where the plane is, with a real receiver's small errors. `truth` is where the plane really is:
    SITL's position while the flight controller lives, and after it dies where the plane came down."""

    def __init__(self, started: float, ttff: float, rng: Optional[random.Random] = None) -> None:
        self.started, self.ttff = started, ttff
        self.rng = rng or random.Random()

    def read(self, now: float, truth: Optional[Tuple[int, int, int, int, int]], moving: bool) -> mr.Position:
        since = now - self.started
        if truth is None or since < self.ttff:
            return mr.Position(gnss_time=int(time.time()) if since > 1 else 0, sats=min(3, int(since / 8)))
        lat, lon, alt, vx, vy = truth
        err = self.rng.gauss
        m_to_lat = 1e7 / 111_320  # 1e-7 degrees per metre
        m_to_lon = m_to_lat / max(0.01, math.cos(math.radians(lat / 1e7)))
        speed, course = (math.hypot(vx, vy), math.degrees(math.atan2(vy, vx)) % 360) if moving else (0.0, 0.0)
        return mr.Position(
            gnss_time=int(time.time()),
            lat=int(lat + err(0, GNSS_ERROR_M) * m_to_lat),
            lon=int(lon + err(0, GNSS_ERROR_M) * m_to_lon),
            alt=mr.UNKNOWN_I32 if alt == mr.UNKNOWN_I32 else int(alt + err(0, 2 * GNSS_ERROR_M) * 1000),
            speed=min(mr.U16_UNKNOWN - 1, int(speed)),
            course=int(course * 100),
            hdop=self.rng.randint(70, 110),
            sats=self.rng.randint(9, 12),
            fix=mr.FIX_3D,
        )


class Modem:
    """One power-on of the LTE module."""

    def __init__(self, gnss_ttff: float = GNSS_TTFF) -> None:
        self.started = time.monotonic()
        self.stage = "starting"  # starting, searching, registered, data (mobile data up)
        self.net: Optional[Network] = None
        self.client: Optional[mr.TunnelClient] = None
        self.batcher: Optional[mr.Batcher] = None
        self.photos: Optional[mr.PhotoOutbox] = None
        self.gnss = Gnss(self.started, gnss_ttff)
        self.position: Optional[mr.Position] = None  # the last one reported
        self.voice = False  # its locator voice sounds: the relay asks for it in its PONGs

    def send(self, chunk: bytes) -> None:
        if self.client is not None and self.client.gcs_present:  # like the firmware: held back without a GCS
            self.client.send_data(chunk)


class Plane:
    """The simulated aircraft, on an asyncio loop in a background thread. The window reads its
    state and flips its switches through call()."""

    def __init__(self, server: Tuple[str, int], key: bytes,
                 start_fc: Optional[Callable[[], subprocess.Popen]] = None, fc_port: int = FC_PORT) -> None:
        self.server, self.key, self.fc_port = server, key, fc_port
        self.server_text = mr.fmt_addr(server)
        self.start_fc = start_fc  # starts the flight controller (SITL); None: it runs by itself
        # the switches
        self.battery = False
        self.lte = False  # switched on, the module runs whenever the battery (or its cell) is on
        self.camera = Image is not None  # the board's CAM DIP switch
        self.cell = False  # an 18650 cell in the board's holder: the module runs on without the flight battery
        self.quick = False
        self.network = NET_LTE  # what the plane flies through
        self.choice = mr.NET_AUTO  # the network chosen at the relay (MavLTE's), as the module last heard it
        self.signal = GOOD_SIGNAL
        self.cell_pct = 100.0
        self.cell_at = time.monotonic()
        self.sun = False  # parked in the sun: the fuselage heats up
        self.chip_c = AIR_C  # the module's chip
        self.chip_at = time.monotonic()
        # where the plane really is (lat, lon, alt MSL mm, vx, vy), as SITL last said: the GNSS's truth
        self.truth: Optional[Tuple[int, int, int, int, int]] = None
        # flight controller
        self.fc = FcState()
        self.fc_error = ""
        self.fc_writer: Optional[asyncio.StreamWriter] = None
        self.fc_task: Optional[asyncio.Task] = None
        self.sitl: Optional[subprocess.Popen] = None
        self.dying: list = []  # SITL processes told to stop
        # LTE module
        self.modem: Optional[Modem] = None
        self.modem_task: Optional[asyncio.Task] = None
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, name="plane", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        self.loop.run_forever()
        self.loop.close()

    def call(self, fn: Callable, *args) -> None:
        """Runs fn(*args) in the plane's thread."""
        self.loop.call_soon_threadsafe(fn, *args)

    # -- the switches (in the plane's thread)

    def set_battery(self, on: bool) -> None:
        if on == self.battery:
            return
        self._cell_update()
        self.battery = on
        if on:
            self.fc, self.fc_error = FcState(), ""
            self.fc_task = self.loop.create_task(self._power_up())
        else:
            if self.cell and self.modem is not None:
                log.info("battery off: the flight controller is dead; the LTE module runs on its cell")
            else:
                log.info("battery off")
                self._modem_off()
            if self.fc_task is not None:
                self.fc_task.cancel()
                self.fc_task = None
            self._stop_sitl()

    def set_lte(self, on: bool) -> None:
        self.lte = on
        if on and (self.battery or self.cell):
            self._modem_on()
        elif not on:
            self._modem_off()

    def set_cell(self, on: bool) -> None:
        """Puts the 18650 cell in the board's holder, or takes it out."""
        self._cell_update()
        self.cell = on
        log.info("LTE module: cell %s", "in" if on else "out")
        if on and self.lte and not self.battery:
            self._modem_on()
        elif not on and not self.battery:
            self._modem_off()

    def _cell_update(self) -> None:
        """The cell charges from the flight battery (about 1% a minute) and runs the module without it."""
        now = time.monotonic()
        elapsed, self.cell_at = now - self.cell_at, now
        if not self.cell:
            return
        if self.battery:
            self.cell_pct = min(100.0, self.cell_pct + elapsed / 60)
        elif self.modem is not None:
            self.cell_pct = max(0.0, self.cell_pct - elapsed / (CELL_HOURS * 36))
            if self.cell_pct <= 0:
                log.warning("LTE module: its cell is empty")
                self._modem_off()

    def set_sun(self, on: bool) -> None:
        self._chip_update()
        self.sun = on
        log.info("LTE module: %s", f"in the sun: the fuselage heats up to {SUN_AIR_C:.0f} °C" if on
                 else "in the shade again")

    def _chip_update(self) -> None:
        """The module's chip follows the air in the fuselage, plus its own heat while it runs."""
        now = time.monotonic()
        elapsed, self.chip_at = now - self.chip_at, now
        target = (SUN_AIR_C if self.sun else AIR_C) + (SELF_HEAT_C if self.modem is not None else 0.0)
        self.chip_c += (target - self.chip_c) * (1 - math.exp(-elapsed / CHIP_LAG_S))

    def serving(self) -> int:
        """The network the module is on: what the plane flies through, as the network chosen at the relay allows.
        2G is there wherever LTE is; LTE only finds nothing where there is only 2G."""
        if self.network == NO_CONNECTION or self.choice == mr.NET_AUTO:
            return self.network
        if self.choice == mr.NET_2G:
            return NET_2G
        return NET_LTE if self.network == NET_LTE else NO_CONNECTION

    def _moved(self, old: int) -> None:
        """The module's network was old: now it is self.serving()."""
        new = self.serving()
        m = self.modem
        if m is not None and m.net is not None and new != old:
            # a real modem needs a moment to move between networks, or to register again
            gap = 0.0 if new == NO_CONNECTION else REGISTER_AGAIN_S if old == NO_CONNECTION else RAT_SWITCH_S
            m.net.set_network(new, self.signal, gap)
            log.info("LTE module: %s", f"now on {NETWORKS[new][0]}" if new != NO_CONNECTION else "no coverage"
                     if self.network == NO_CONNECTION else "no LTE here, and LTE only is chosen")

    def set_network(self, network: int) -> None:
        old, self.network = self.serving(), network
        self._moved(old)

    def set_signal(self, signal: int) -> None:
        self.signal = signal
        m = self.modem
        if m is not None and m.net is not None:
            m.net.set_network(self.serving(), signal)

    def set_camera(self, on: bool) -> None:
        self.camera = on and Image is not None
        if self.modem is not None and self.modem.photos is not None:
            self.modem.photos.capture = self._capture if self.camera else None
        log.info("camera: %s", "on" if self.camera else "off (the aircraft answers that it has none)")

    # -- flight controller

    async def _power_up(self) -> None:
        log.info("battery on")
        if self.lte:
            self._modem_on()
        await self._reap(5.0)  # a SITL that was just switched off must have let go of its ports
        if self.start_fc is not None:
            try:
                self.sitl = self.start_fc()
            except (OSError, SystemExit) as exc:
                self.fc_error = str(exc)
                log.error("%s", exc)
                return
            kill_with_us(self.sitl)
        await self._fc_link()

    async def _fc_link(self) -> None:
        """The UART between the flight controller and the ESP32: SITL's SERIAL1."""
        while True:
            try:
                reader, writer = await asyncio.open_connection("127.0.0.1", self.fc_port)
            except OSError:
                if self.sitl is not None and self.sitl.poll() is not None:
                    self.fc_error = "ArduPlane SITL stopped. Is another simulator running (sitl_demo.py, Mission Planner)?"
                    log.error("%s", self.fc_error)
                    return
                await asyncio.sleep(0.5)
                continue
            self.fc_writer = writer
            log.info("flight controller: connected (SITL SERIAL1, TCP %d)", self.fc_port)
            try:
                while True:
                    data = await reader.read(4096)
                    if not data:
                        break
                    self._from_fc(data)
            except OSError:
                pass
            finally:
                self.fc_writer = None
                writer.close()
            log.warning("flight controller: connection lost")
            await asyncio.sleep(0.5)

    def _from_fc(self, data: bytes) -> None:
        now = time.monotonic()
        fc = self.fc
        fc.feed(data, now)
        if fc.lat != mr.UNKNOWN_I32:
            self.truth = (fc.lat, fc.lon, fc.alt_msl_mm, fc.vx, fc.vy)
        m = self.modem
        if m is not None and m.batcher is not None:
            for chunk in m.batcher.feed(data, now):
                m.send(chunk)

    def _to_fc(self, payload: bytes) -> None:
        if self.fc_writer is not None:
            self.fc_writer.write(payload)

    def _stop_sitl(self) -> None:
        if self.sitl is not None:
            if self.sitl.poll() is None:
                self.sitl.terminate()
            self.dying.append(self.sitl)
            self.sitl = None

    async def _reap(self, timeout: float) -> None:
        end = time.monotonic() + timeout
        while any(p.poll() is None for p in self.dying) and time.monotonic() < end:
            await asyncio.sleep(0.1)
        for p in self.dying:
            if p.poll() is None:
                p.kill()
        self.dying.clear()

    # -- LTE module

    def _modem_on(self) -> None:
        if self.modem is None:
            self._chip_update()
            self.modem = Modem(GNSS_TTFF_QUICK if self.quick else GNSS_TTFF)
            self.modem_task = self.loop.create_task(self._modem_run(self.modem))

    def _modem_off(self) -> None:
        if self.modem is not None:
            self._chip_update()
            log.info("LTE module: power off")
            self.modem_task.cancel()  # a power cut: no goodbye to the relay
            self.modem = self.modem_task = None

    async def _modem_run(self, m: Modem) -> None:
        start, register, data = STARTUP_QUICK if self.quick else STARTUP_REAL
        log.info("LTE module: power on")
        try:
            await asyncio.sleep(start)
            m.stage = "searching"
            while self.serving() == NO_CONNECTION:  # no coverage: no network to register with
                await asyncio.sleep(0.2)
            await asyncio.sleep(register)
            m.stage = "registered"
            log.info("LTE module: registered on %s, signal %d dBm", NETWORKS[self.serving()][0],
                     SIGNALS[self.signal][1])
            await asyncio.sleep(data)
            relay = await self._lookup()
            m.net = Network(relay, self.serving(), self.signal)
            await m.net.start()
            m.batcher = mr.Batcher()
            m.client = mr.TunnelClient(mr.ROLE_VEHICLE, self.key, *m.net.address, on_data=self._to_fc,
                                       info=f"mavlte-planesim/{mr.__version__}", radio=self._radio)
            m.client.network = m.client.net_report = self.choice  # as the board: the last one, until the relay says
            m.photos = mr.PhotoOutbox(m.client, capture=self._capture if self.camera else None, where=self._where,
                                      cap=lambda: PHOTO_CAP.get(self.serving(), PHOTO_CAP[NET_2G]))
            m.client.on_packet, m.client.on_session = m.photos.on_packet, m.photos.on_session
            m.stage = "data"
            log.info("LTE module: mobile data up")
            await asyncio.gather(m.client.run(), self._flush(m))
        finally:
            if m.net is not None:
                m.net.close()

    async def _lookup(self) -> Tuple[str, int]:
        loop = asyncio.get_running_loop()
        while True:
            try:
                infos = await loop.getaddrinfo(self.server[0], self.server[1], type=socket.SOCK_DGRAM)
                return infos[0][4][:2]
            except (OSError, UnicodeError) as exc:  # UnicodeError: a name with an empty or overlong label
                log.warning("LTE module: cannot look up %s (%s); trying again", self.server[0], exc)
                await asyncio.sleep(5.0)

    async def _flush(self, m: Modem) -> None:
        """Sends what the batcher holds once it is 50 ms old, as the firmware does, and logs the
        relay session coming and going."""
        session = 0
        reported = 0.0
        while True:
            await asyncio.sleep(0.005)
            now = time.monotonic()
            chunk = m.batcher.poll(now, 0.05)
            if chunk:
                m.send(chunk)
            m.photos.pump(now)
            if m.client.voice_on != m.voice:  # the locator voice, kept while the relay is out of reach
                m.voice = m.client.voice_on
                log.info("LTE module: locator voice %s", f"on: {VOICE_SOUND} through its speaker, again and again"
                         if m.voice else "off")
                m.client.ping_flags = mr.PING_FLAG_SPEAKING if m.voice else 0
                m.client.ping_now()
            if m.client.network != self.choice:  # the network chosen at the relay, kept while it is out of reach
                old, self.choice = self.serving(), m.client.network
                log.info("LTE module: network %s, as chosen in MavLTE", mr.NET_NAMES[self.choice])
                m.client.net_report = self.choice
                m.client.ping_now()
                self._moved(old)
            if now - reported >= LOCATOR_INTERVAL and m.client.session:  # the locator, as the firmware
                reported = now
                self._cell_update()
                m.position = self._position(m, now)
                m.client.send_packet(mr.POSITION, m.position.pack())
            if m.client.session != session:
                session = m.client.session
                if session:
                    log.info("LTE module: connected to the relay %s (session %08x)", self.server_text, session)
                else:
                    log.warning("LTE module: lost the relay session")

    def _position(self, m: Modem, now: float) -> mr.Position:
        """The module's position report: its GNSS, how long the flight controller has been silent, its power
        as the board's fuel gauge reads it (the supply rail while the flight battery powers it, else its
        cell), and its chip's temperature."""
        heard = self.fc.last_rx
        silent = mr.U16_UNKNOWN if not heard else min(mr.U16_UNKNOWN - 1, int(now - heard))
        flags = mr.POS_FC_SILENT if heard and silent >= mr.FC_SILENT_S else 0
        pos = m.gnss.read(now, self.truth, moving=not flags)
        if self.battery:
            pos = pos._replace(battery_pct=100, battery_mv=RAIL_MV)
        elif self.cell:
            pct = int(round(self.cell_pct))
            pos = pos._replace(battery_pct=pct, battery_mv=mr.cell_mv(self.cell_pct))
        self._chip_update()
        return pos._replace(flags=pos.flags | flags, fc_silent=silent, temp=int(round(self.chip_c)))

    def _radio(self) -> Tuple[int, int]:
        rat = NETWORKS[self.serving()][1]
        if rat is None:
            return mr.RSSI_UNKNOWN, mr.RAT_UNKNOWN
        return SIGNALS[self.signal][1], rat

    def _capture(self, width: int, height: int) -> bytes:
        jpeg = camera_picture(width, height, self.fc)
        log.info("camera: photo %d×%d, %d KB, on its way", width, height, round(len(jpeg) / 1024))
        return jpeg

    def _where(self) -> Tuple[int, int, int, int]:
        fc = self.fc
        return fc.lat, fc.lon, fc.alt_mm, fc.heading

    # -- from the window's thread

    def close(self) -> None:
        """Switches everything off and stops the plane's thread."""
        done = threading.Event()

        def off() -> None:
            self.set_battery(False)
            self._modem_off()  # a module on its cell runs on without the battery
            done.set()

        self.call(off)
        done.wait(5)
        self.loop.call_soon_threadsafe(self.loop.call_later, 0.2, self.loop.stop)  # cancelled tasks finish
        self.thread.join(5)
        for p in self.dying:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()


_job = None


def kill_with_us(proc: subprocess.Popen) -> None:
    """On Windows, makes SITL die with this program even if it is killed, so that no SITL is left
    behind holding the simulator's ports (a Job Object that kills its processes when closed)."""
    global _job
    if sys.platform != "win32":
        return
    import ctypes
    from ctypes import wintypes

    class IoCounters(ctypes.Structure):
        _fields_ = [(name, ctypes.c_uint64) for name in ("reads", "writes", "others", "read_bytes", "write_bytes",
                                                         "other_bytes")]

    class BasicLimits(ctypes.Structure):
        _fields_ = [("user_time", ctypes.c_int64), ("job_time", ctypes.c_int64), ("flags", wintypes.DWORD),
                    ("min_ws", ctypes.c_size_t), ("max_ws", ctypes.c_size_t), ("processes", wintypes.DWORD),
                    ("affinity", ctypes.c_size_t), ("priority", wintypes.DWORD), ("scheduling", wintypes.DWORD)]

    class ExtendedLimits(ctypes.Structure):
        _fields_ = [("basic", BasicLimits), ("io", IoCounters), ("process_memory", ctypes.c_size_t),
                    ("job_memory", ctypes.c_size_t), ("peak_process_memory", ctypes.c_size_t),
                    ("peak_job_memory", ctypes.c_size_t)]

    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.CreateJobObjectW.restype = wintypes.HANDLE
    k32.CreateJobObjectW.argtypes = (wintypes.LPVOID, wintypes.LPCWSTR)
    k32.SetInformationJobObject.argtypes = (wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD)
    k32.AssignProcessToJobObject.argtypes = (wintypes.HANDLE, wintypes.HANDLE)
    if _job is None:
        job = k32.CreateJobObjectW(None, None)
        limits = ExtendedLimits()
        limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not job or not k32.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
            log.debug("no job object: %d", ctypes.get_last_error())
            return
        _job = job
    if not k32.AssignProcessToJobObject(_job, int(proc._handle)):
        log.debug("cannot add SITL to the job object: %d", ctypes.get_last_error())


# ---------------------------------------------------------------------------------------------
# The window


class SimWindow:
    POLL_MS = 100  # the module's LED blinks like the board's, in 100 ms steps, when it blinks

    def __init__(self, root: tk.Tk, plane: Plane) -> None:
        self.root, self.plane = root, plane
        self.tick = 0
        self.rate_mark: tuple = (time.monotonic(), None, 0, 0, 0, 0)
        self.rates = (0.0, 0.0)
        self.lost_share = 0.0
        self.loss_window: "collections.deque[Tuple[int, int]]" = collections.deque(maxlen=5)
        self.log_lines: "queue.Queue[str]" = queue.Queue()
        self.log_handler = ui.LogHandler(self.log_lines)
        for name in LOGGERS:
            logging.getLogger(name).addHandler(self.log_handler)
            logging.getLogger(name).setLevel(logging.INFO)
        # the module logs its own view of the relay; the tunnel's would name the emulated network
        logging.getLogger("mavrelay.vehicle").setLevel(logging.WARNING)
        self.scale = max(1.0, root.winfo_fpixels("1i") / 96.0)
        self.pad = round(8 * self.scale)
        family = "Segoe UI" if "Segoe UI" in tkfont.families(root) else "TkDefaultFont"
        self.font = (family, 10)
        self.font_small = (family, 9)
        self.font_bold = (family, 10, "bold")
        self.font_title = (family, 15, "bold")
        self.icon: Optional[tk.PhotoImage] = None
        for path in (ICON, ui.ICON):
            if os.path.exists(path):
                self.icon = tk.PhotoImage(file=path)
                root.iconphoto(True, self.icon)
                break
        ui.setup_style(root)
        self._build()
        ui.dark_title_bar(root)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.report_callback_exception = lambda _t, exc, _tb: log.error("unexpected error: %r", exc)
        self.poll()

    # -- layout

    def _build(self) -> None:
        root, s, pad = self.root, self.scale, self.pad
        root.title(TITLE)
        root.configure(bg=ui.BG)
        header = tk.Frame(root, bg=ui.SURFACE)
        header.pack(fill="x")
        who = tk.Frame(header, bg=ui.SURFACE)
        who.pack(fill="x", padx=round(16 * s), pady=round(10 * s))
        if self.icon is not None:
            self.badge = self.icon.subsample(4 if s < 1.5 else 2)  # as in MavLTE's header
            tk.Label(who, image=self.badge, bg=ui.SURFACE).pack(side="left")
        names = tk.Frame(who, bg=ui.SURFACE)
        names.pack(side="left", padx=round(12 * s))
        tk.Label(names, text=TITLE, bg=ui.SURFACE, fg=ui.TEXT, font=self.font_title).pack(anchor="w")
        tk.Label(names, text=f"ArduPlane SITL · relay {self.plane.server_text}", bg=ui.SURFACE, fg=ui.MUTED,
                 font=self.font).pack(anchor="w")
        tk.Frame(root, bg=ui.ACCENT, height=max(2, round(2 * s))).pack(fill="x")

        body = tk.Frame(root, bg=ui.BG)
        body.pack(fill="both", expand=True, padx=round(12 * s), pady=round(8 * s))

        self._section(body, "Aircraft", first=True)
        card, top = self._card(body)
        self.fc_led = ui.Led(top, s, ui.SURFACE)
        self.fc_led.pack(side="left")
        self.battery_switch = ui.Switch(top, s, ui.SURFACE, self.toggle_battery)
        self.battery_switch.pack(side="left", padx=(round(6 * s), round(8 * s)))
        tk.Label(top, text="Battery", bg=ui.SURFACE, fg=ui.TEXT, font=self.font_bold).pack(side="left")
        self.volts = tk.Label(top, text="", bg=ui.SURFACE, fg=ui.MUTED, font=self.font)
        self.volts.pack(side="right")
        self.fc_values = self._grid(card, ("Flight controller", "Altitude", "USB port"))

        self._section(body, "LTE module")
        card, top = self._card(body)
        self.lte_led = ui.Led(top, s, ui.SURFACE)
        self.lte_led.pack(side="left")
        self.lte_switch = ui.Switch(top, s, ui.SURFACE, self.toggle_lte)
        self.lte_switch.set(self.plane.lte)
        self.lte_switch.pack(side="left", padx=(round(6 * s), round(8 * s)))
        tk.Label(top, text="LTE module", bg=ui.SURFACE, fg=ui.TEXT, font=self.font_bold).pack(side="left")
        self.lte_bars = ui.Bars(top, s, ui.SURFACE)
        self.lte_bars.pack(side="right")
        self.chip_text = tk.Label(top, text="", bg=ui.SURFACE, fg=ui.MUTED, font=self.font)  # as it reports it
        self.chip_text.pack(side="right", padx=(0, round(8 * s)))
        self.lte_values = self._grid(card, ("State", "Network", "Signal", "Link", "Round trip", "GCS", "Data",
                                            "Camera", "GPS", "Voice", "Backup cell"),
                                     {"Network": self._network, "Signal": self._signal, "Camera": self._camera,
                                      "Backup cell": self._backup_cell})
        self.quick = tk.BooleanVar(value=self.plane.quick)
        self.sun = tk.BooleanVar(value=self.plane.sun)
        line = tk.Frame(card, bg=ui.SURFACE)
        line.pack(fill="x", padx=round(4 * s), pady=(0, pad))
        for text, var, command in (("Quick start (the real modem takes about 16 s)", self.quick, self.toggle_quick),
                                   (f"In the sun ({SUN_AIR_C:.0f} °C)", self.sun, self.toggle_sun)):
            tk.Checkbutton(line, text=text, variable=var, command=command, bg=ui.SURFACE, fg=ui.TEXT,
                           selectcolor=ui.FIELD, activebackground=ui.SURFACE, activeforeground=ui.TEXT,
                           font=self.font_small, bd=0, highlightthickness=0).pack(side="left",
                                                                                  padx=(0, round(10 * s)))

        self._section(body, "Log")
        frame = tk.Frame(body, bg=ui.BG)
        frame.pack(fill="both", expand=True, pady=(round(4 * s), 0))
        self.log_text = tk.Text(frame, height=7, width=54, font=("Consolas", 9), bg=ui.LOG_BG, fg="#b8bcc2",
                                relief="flat", highlightbackground=ui.BORDER, highlightthickness=1, wrap="word",
                                state="disabled")
        scroll = ttk.Scrollbar(frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)

        # on the right of the screen, so that the MavLTE app fits beside it
        root.update_idletasks()
        x = root.winfo_screenwidth() - root.winfo_reqwidth() - round(24 * s)
        root.geometry(f"+{max(0, x)}+{round(24 * s)}")

    def _section(self, parent, text: str, first: bool = False) -> None:
        tk.Label(parent, text=text, bg=ui.BG, fg=ui.MUTED, font=self.font_bold).pack(
            anchor="w", pady=(0 if first else round(8 * self.scale), 0))

    def _card(self, parent) -> Tuple[tk.Frame, tk.Frame]:
        card = tk.Frame(parent, bg=ui.SURFACE, highlightbackground=ui.BORDER, highlightthickness=1)
        card.pack(fill="x", pady=(round(4 * self.scale), 0))
        top = tk.Frame(card, bg=ui.SURFACE)
        top.pack(fill="x", padx=self.pad, pady=(self.pad, round(4 * self.scale)))
        return card, top

    def _grid(self, card, labels, widgets: Optional[Dict[str, Callable]] = None) -> Dict[str, tk.Label]:
        grid = tk.Frame(card, bg=ui.SURFACE)
        grid.pack(fill="x", padx=self.pad, pady=(0, self.pad))
        values = {}
        for row, label in enumerate(labels):
            tk.Label(grid, text=label, bg=ui.SURFACE, fg=ui.MUTED, font=self.font).grid(row=row, column=0,
                                                                                        sticky="nw", pady=1)
            if widgets and label in widgets:
                value = widgets[label](grid)
            else:
                value = tk.Label(grid, text="-", bg=ui.SURFACE, fg=ui.TEXT, font=self.font, anchor="w",
                                 justify="left", wraplength=round(270 * self.scale))
                values[label] = value
            value.grid(row=row, column=1, sticky="w", padx=(12, 0), pady=1)
        return values

    def _network(self, parent) -> tk.Frame:
        """No connection · 2G · LTE: a row of buttons, the chosen one lit."""
        s = self.scale
        frame = tk.Frame(parent, bg=ui.SURFACE)
        self.network_buttons = []
        for i, network in enumerate(NETWORKS):
            button = tk.Label(frame, text=network[0].split(" (")[0], font=self.font_small, cursor="hand2",
                              padx=round(8 * s), pady=round(2 * s))
            button.pack(side="left", padx=(0, round(3 * s)))
            button.bind("<Button-1>", lambda _e, i=i: self.set_network(i))
            self.network_buttons.append(button)
        self._light_network(self.plane.network)
        return frame

    def _light_network(self, network: int) -> None:
        for i, button in enumerate(self.network_buttons):
            button.configure(bg=ui.ACCENT if i == network else ui.FIELD, fg=ui.ON_ACCENT if i == network else ui.TEXT)

    def _signal(self, parent) -> tk.Frame:
        s = self.scale
        frame = tk.Frame(parent, bg=ui.SURFACE)
        self.signal = tk.Scale(frame, from_=0, to=len(SIGNALS) - 1, orient="horizontal", resolution=1,
                               showvalue=False, length=round(110 * s), width=round(12 * s),
                               sliderlength=round(18 * s), bd=0, highlightthickness=0, bg=ui.MUTED,
                               activebackground=ui.TEXT, troughcolor=ui.FIELD, sliderrelief="flat",
                               command=self.set_signal)
        self.signal.set(self.plane.signal)
        self.signal.pack(side="left")
        self.signal_text = tk.Label(frame, text="", bg=ui.SURFACE, fg=ui.TEXT, font=self.font)
        self.signal_text.pack(side="left", padx=(round(8 * s), 0))
        self.set_signal(str(self.plane.signal))
        return frame

    def _camera(self, parent) -> tk.Frame:
        """The board's CAM DIP switch, and what the camera is doing."""
        s = self.scale
        frame = tk.Frame(parent, bg=ui.SURFACE)
        self.camera_switch = ui.Switch(frame, s * 0.8, ui.SURFACE, self.toggle_camera)
        self.camera_switch.set(self.plane.camera)
        self.camera_switch.pack(side="left", anchor="n", pady=(round(2 * s), 0))
        self.camera_text = tk.Label(frame, text="", bg=ui.SURFACE, fg=ui.TEXT, font=self.font, anchor="w",
                                    justify="left", wraplength=round(230 * s))
        self.camera_text.pack(side="left", padx=(round(8 * s), 0))
        return frame

    # -- actions

    def toggle_battery(self, on: bool) -> None:
        self.battery_switch.set(on)
        self.plane.call(self.plane.set_battery, on)

    def toggle_lte(self, on: bool) -> None:
        self.lte_switch.set(on)
        self.plane.call(self.plane.set_lte, on)

    def toggle_quick(self) -> None:
        self.plane.quick = self.quick.get()

    def toggle_sun(self) -> None:
        self.plane.call(self.plane.set_sun, self.sun.get())

    def _backup_cell(self, parent) -> tk.Frame:
        """An 18650 cell in the board's holder: with it, the module runs on when the flight battery goes
        (a crash, say), and keeps reporting where the plane is."""
        s = self.scale
        frame = tk.Frame(parent, bg=ui.SURFACE)
        self.cell_switch = ui.Switch(frame, s * 0.8, ui.SURFACE, self.toggle_cell)
        self.cell_switch.set(self.plane.cell)
        self.cell_switch.pack(side="left", anchor="n", pady=(round(2 * s), 0))
        self.cell_text = tk.Label(frame, text="", bg=ui.SURFACE, fg=ui.TEXT, font=self.font, anchor="w",
                                  justify="left", wraplength=round(230 * s))
        self.cell_text.pack(side="left", padx=(round(8 * s), 0))
        return frame

    def toggle_cell(self, on: bool) -> None:
        self.cell_switch.set(on)
        self.plane.call(self.plane.set_cell, on)

    def toggle_camera(self, on: bool) -> None:
        on = on and Image is not None
        self.camera_switch.set(on)
        self.plane.call(self.plane.set_camera, on)

    def set_network(self, network: int) -> None:
        self.plane.call(self.plane.set_network, network)
        self._light_network(network)

    def set_signal(self, value: str) -> None:
        level = int(float(value))
        self.plane.call(self.plane.set_signal, level)
        name, dbm = SIGNALS[level][:2]
        self.signal_text.configure(text=f"{name} ({dbm} dBm)")

    def close(self) -> None:
        self.root.after_cancel(self.poll_job)
        for name in LOGGERS:
            logging.getLogger(name).removeHandler(self.log_handler)
        try:
            self.plane.close()
        finally:
            self.root.destroy()

    # -- live state, ten times a second

    def poll(self) -> None:
        self.tick += 1
        now = time.monotonic()
        self._drain_log()
        self._show_fc(now)
        self._show_lte(now, blink=self.tick % 10 < 5)
        self._show_camera()
        self._show_locator()
        self._show_voice()
        self.poll_job = self.root.after(self.POLL_MS, self.poll)

    def _show_voice(self) -> None:
        """What the board's speaker would be playing: MavLTE's Voice switch turns it on and off."""
        m = self.plane.modem
        if m is None:
            text, color = "-", ui.TEXT
        elif m.voice:
            text, color = "Sounding: a two-tone alarm, again and again", ui.GREEN
        else:
            text, color = "Off (MavLTE: Voice switch)", ui.MUTED
        label = self.lte_values["Voice"]
        if label.cget("text") != text or label.cget("fg") != color:
            label.configure(text=text, fg=color)

    def _show_locator(self) -> None:
        """What the module's GNSS reports, and its cell."""
        p = self.plane
        m = p.modem
        pos = m.position if m is not None else None
        if m is None:
            gps = "-"
        elif pos is None:
            gps = "Starting…" if m.stage != "data" else "-"
        elif pos.has_fix:
            gps = f"{pos.lat / 1e7:.5f}, {pos.lon / 1e7:.5f} · {pos.sats} satellites"
        else:
            gps = f"Searching ({pos.sats} satellites)" if p.truth is not None or pos.sats else \
                "Searching (start the flight controller to give the plane a place)"
        self._set(self.lte_values["GPS"], gps)
        temp = pos.temp if pos is not None and m is not None else mr.TEMP_UNKNOWN
        text = f"chip {temp} °C" if temp != mr.TEMP_UNKNOWN else ""
        color = ui.RED if temp >= mr.TEMP_HOT else ui.AMBER if temp >= mr.TEMP_WARM else ui.MUTED
        if self.chip_text.cget("text") != text or self.chip_text.cget("fg") != color:
            self.chip_text.configure(text=text, fg=color)
        if not p.cell:
            cell = "None: off with the flight battery"
        elif p.battery:
            cell = f"{p.cell_pct:.0f}%, charging"
        elif m is not None:
            cell = f"{p.cell_pct:.0f}%, the module runs on it"
        else:
            cell = f"{p.cell_pct:.0f}%"
        self._set(self.cell_text, cell)

    def _show_camera(self) -> None:
        p = self.plane
        photos = p.modem.photos if p.modem is not None else None
        if Image is None:
            text = "Needs Pillow: pip install pillow"
        elif not p.camera:
            text = "Off: the aircraft answers that it has no camera"
        elif not p.battery and p.modem is None:  # (on the board, the camera also runs on the module's cell)
            text = "-"
        elif photos is not None and photos.busy:
            sender = photos.sender
            info = sender.info
            text = f"Sending {info.width}×{info.height}: {sender.progress * info.size / 1024:.0f} of " \
                   f"{info.size / 1024:.0f} KB"
        elif photos is not None and photos.last is not None:
            info, sent, took = photos.last
            text = (f"Sent {info.width}×{info.height}, {info.size / 1024:.0f} KB in {took:.1f} s" if sent
                    else f"Gave up on {info.width}×{info.height}: nothing more from the relay")
        else:
            text = "Ready for Snapshot in MavLTE"
        self._set(self.camera_text, text)

    def _show_fc(self, now: float) -> None:
        p, v = self.plane, self.fc_values
        fc = p.fc
        fresh = p.battery and now - fc.heartbeat < 2.5
        if not p.battery:
            color, text = ui.LED_OFF, "Off"
        elif p.fc_error:
            color, text = ui.RED, p.fc_error
        elif fresh:
            color, text = ui.GREEN, f"{fc.mode}, {'ARMED' if fc.armed else 'disarmed'}"
        else:
            color = ui.AMBER
            text = "Waiting for its heartbeat…" if p.fc_writer is not None else "Starting ArduPlane SITL…"
        self.fc_led.set(color)
        self._set(v["Flight controller"], text)
        self._set(v["Altitude"], f"{round(fc.alt)} m above home" if fresh and fc.alt is not None else "-")
        self._set(v["USB port"], f"TCP 127.0.0.1:{sitl_demo.USB_PORT}" if p.battery else "-")
        self._set(self.volts, f"{fc.volts:.1f} V" if fresh and fc.volts is not None else "")

    def _show_lte(self, now: float, blink: bool) -> None:
        p, v = self.plane, self.lte_values
        m = p.modem
        client = m.client if m is not None else None
        session = client is not None and client.session != 0
        # the LED, as on the board (README: "The RGB LED"), from worst to best: red without mobile data
        # (blinking while it starts and searches), yellow while the relay does not answer, purple when
        # connected but the flight controller is silent, blue when ready to fly (a GCS connected or not)
        serving = p.serving()
        no_coverage = serving == NO_CONNECTION
        if m is None:
            color = ui.LED_OFF
        elif m.stage != "data" or no_coverage:
            color = ui.RED if blink else ui.LED_OFF
        elif not session:
            color = YELLOW
        elif not (p.battery and now - p.fc.heartbeat < FC_SILENT):
            color = PURPLE
        else:
            color = ui.BLUE
        self.lte_led.set(color)

        if m is None and not p.battery and not p.cell:
            state = "No power: the battery is off"
        elif m is None:
            state = "Off"
        else:
            since = f" ({now - m.started:.0f} s)"
            if m.stage == "starting":
                state = "Starting the modem…" + since
            elif no_coverage and p.network != NO_CONNECTION:
                state = "No LTE here, and LTE only is chosen in MavLTE"
            elif m.stage == "searching":
                state = ("No coverage: searching for a network…" if no_coverage
                         else "Searching for the network…") + since
            elif m.stage == "registered":
                state = f"Registered on {NETWORKS[serving][0]}, starting mobile data…" + since
            elif no_coverage:
                state = "No coverage: nothing gets through"
            elif m.net is not None and now < m.net.gap_until:
                state = f"Moving to {NETWORKS[serving][0]}: no data for a moment"
            elif session:
                state = f"Connected to the relay over {NETWORKS[serving][0].split(' (')[0]}"
                if p.choice != mr.NET_AUTO:
                    state += f" ({mr.NET_NAMES[p.choice]}, chosen in MavLTE)"
                trouble = m.net.condition(now) if m.net is not None else ""
                if trouble:
                    state += f" · {trouble}"
            elif client.hellos >= 5:
                state = "Mobile data up, but no answer from the relay"
            else:
                state = "Mobile data up, connecting to the relay…" + since
        self._set(v["State"], state)
        registered = m is not None and m.stage in ("registered", "data") and not no_coverage
        self.lte_bars.set(p.signal + 1 if registered else None)
        f = link_figures(serving, p.signal)
        self._set(v["Link"], "No coverage: nothing gets through" if f is None else
                  f"{speed_text(f[0])} up · +{f[2] * 1000:.0f} ms each way · {f[4] * 100:g}% loss")
        self._set(v["Round trip"], f"{client.rtt_ms} ms, plane ↔ relay"
                  if session and client.rtt_ms != mr.U16_UNKNOWN else "-")
        if session:
            self._set(v["GCS"], "Connected: sending telemetry" if client.gcs_present
                      else "None: telemetry held back until one connects")
        else:
            self._set(v["GCS"], "-")
        net = m.net if m is not None else None
        self._rate(net, now)
        up, down = self.rates
        data = f"↑ {up / 1000:.1f} KB/s   ↓ {down / 1000:.1f} KB/s"
        if self.lost_share >= 0.005:
            data += f" · {self.lost_share:.0%} of packets lost"
        self._set(v["Data"], data if net is not None else "-")

    def _rate(self, net: Optional[Network], now: float) -> None:
        """Traffic offered to the network over the last second, and the share of packets it lost over
        the last five (a full queue drops in bursts)."""
        t, mark, *before = self.rate_mark
        counts = (net.up_bytes, net.down_bytes, net.packets, net.lost) if net is not None else (0, 0, 0, 0)
        if net is not mark:
            self.rate_mark = (now, net, *counts)
            self.rates, self.lost_share = (0.0, 0.0), 0.0
            self.loss_window.clear()
        elif net is not None and now - t >= 1.0:
            up, down, packets, lost = (c - b for c, b in zip(counts, before))
            self.rates = (up / (now - t), down / (now - t))
            self.loss_window.append((packets, lost))
            sent = sum(p for p, _ in self.loss_window)
            self.lost_share = sum(lost for _, lost in self.loss_window) / sent if sent else 0.0
            self.rate_mark = (now, net, *counts)

    @staticmethod
    def _set(label: tk.Label, text: str) -> None:
        if label.cget("text") != text:
            label.configure(text=text)

    def _drain_log(self) -> None:
        lines = []
        while True:
            try:
                lines.append(self.log_lines.get_nowait())
            except queue.Empty:
                break
        if lines:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", "\n".join(lines) + "\n")
            excess = int(self.log_text.index("end-1c").split(".")[0]) - 300
            if excess > 0:
                self.log_text.delete("1.0", f"{excess}.0")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="The aircraft end of MavLTE on this PC: ArduPlane SITL with a "
                                            "virtual LTE module, in a window.")
    p.add_argument("--config", default=CONFIG, help="settings file with the [vehicle] section (default mavrelay.ini)")
    p.add_argument("--sitl", help="ArduPlane SITL program (default: Mission Planner's copy)")
    p.add_argument("--home", help="SITL start position: lat,lon,alt,heading")
    args = p.parse_args(argv)
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)  # sharp text on high-DPI screens
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("MavLTE.PlaneSimulator")
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    conf = mr.load_config(args.config, "vehicle") if os.path.exists(args.config) else {}
    try:
        server = mr.parse_hostport(conf.get("server", ""))
        key = mr.parse_key(conf.get("key", ""))
    except ValueError:
        root.withdraw()
        messagebox.showerror(APP, f"{args.config} needs the relay server and the vehicle key:\n\n"
                                  "[vehicle]\nserver = your-server:14650\nkey = <vehicle key>")
        root.destroy()
        return
    sitl_args = argparse.Namespace(sitl=args.sitl, speedup=1.0, home=args.home, wipe=False)
    plane = Plane(server, key, start_fc=lambda: sitl_demo.start_sitl(sitl_args, console=False))
    SimWindow(root, plane)
    root.mainloop()


if __name__ == "__main__":
    main()

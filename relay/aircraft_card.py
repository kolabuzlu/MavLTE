"""The Aircraft card: what MavLTE says about the aircraft, worked out once for its window (mavlte.py) and its web
page (mavweb.py), so that both say the same. The functions give texts, and colours by name, which the window and
the page each paint in their own palette:

    TEXT, MUTED, DIM     plain, quieter and quietest text
    GREEN, BLUE          the aircraft available, and its telemetry flowing to this agent (as the LEDs)
    AMBER, RED           a warning, a fault
    OFF                  a dark LED; a link with nothing to show

Standard library only, like mavrelay.py: the web page runs on the relay's server.
"""

from __future__ import annotations

import json
import os
import re
import time
from collections import deque
from typing import Deque, List, NamedTuple, Optional, Tuple

import mavrelay as mr

TEXT, MUTED, DIM = "text", "muted", "dim"
GREEN, BLUE, AMBER, RED, OFF = "green", "blue", "amber", "red", "off"

LIVE_S = 15.0  # a position report older than this (the module sends one every 5 s) is not live
SIZE_NAMES = ("small", "medium", "large")  # mr.SNAP_SIZES
# typical JPEG sizes from the aircraft's OV5640 (bright, detailed scenes: the upper end)
SIZE_HINTS = ("320×240, about 5-10 KB", "640×480, about 10-30 KB", "1024×768, about 25-80 KB")


class Shown(NamedTuple):
    text: str
    color: str = TEXT


def relay(client: mr.TunnelClient, rtt: bool = True) -> Tuple[str, str]:
    """The LED and the line for the agent's session with the relay. rtt: say how far away the relay is."""
    if client.session:
        ms = f" · {client.rtt_ms} ms" if rtt and client.rtt_ms != mr.U16_UNKNOWN else ""
        return GREEN, f"Connected to the relay{ms}"
    if client.hellos >= 5:
        return RED, "No answer from the relay: check server, UDP port and key"
    return AMBER, "Connecting to the relay…"


def headline(agent: Optional[mr.GcsAgent], status: Optional[mr.LinkStatus], switches: bool = True) -> Tuple[str, str]:
    """The card's LED and its first line. switches: the agent has MavLTE's TCP and UDP switches. Green while the
    aircraft is available, blue while its telemetry flows to this agent, as the LEDs on the port cards."""
    if agent is None or not agent.client.session:
        return OFF, "Not connected to the relay"
    if status is None:
        return OFF, "Waiting for news from the relay…"
    if status.online:
        if not agent.watching:
            return BLUE, "Online"
        return GREEN, "Available: switch TCP or UDP on" if switches else "Online"
    if status.idle_ms == mr.U16_UNKNOWN:
        return OFF, "Not connected to the relay (yet)"
    silence = agent.vehicle_silence()  # counts on where STATUS tops out (65.5 s)
    heard = f"{silence:.0f} s ago" if silence is not None else "over a minute ago"
    return RED, f"Offline, last heard {heard}"


def is_online(status: Optional[mr.LinkStatus]) -> bool:
    return bool(status and status.online)


def bars(status: Optional[mr.LinkStatus]) -> Optional[int]:
    """Signal strength in 0-4 bars while the aircraft is online, else None (greyed out)."""
    if not is_online(status):
        return None
    if status.rssi_dbm != mr.RSSI_UNKNOWN:
        return sum(status.rssi_dbm >= limit for limit in (-105, -95, -85, -75))
    worst = max(v for v in (status.up_loss, status.down_loss, 0) if v != mr.U16_UNKNOWN)
    return 4 if worst < 10 else 3 if worst < 50 else 2 if worst < 150 else 1


def link(status: Optional[mr.LinkStatus]) -> str:
    """The aircraft's radio and its round trip to the relay, on one line."""
    if not is_online(status):
        return "-"
    radio = [mr.RAT_NAMES.get(status.rat, "")] if status.rat != mr.RAT_UNKNOWN else []
    if status.rssi_dbm != mr.RSSI_UNKNOWN:
        radio.append(f"{status.rssi_dbm} dBm")
    parts = [" ".join(radio)] if radio else []
    if status.rtt_ms != mr.U16_UNKNOWN:
        parts.append(f"round trip {status.rtt_ms} ms")
    return " · ".join(parts) or "-"


def loss(status: Optional[mr.LinkStatus]) -> str:
    if not is_online(status):
        return "-"
    return f"up {mr.fmt_permille(status.up_loss)}, down {mr.fmt_permille(status.down_loss)}"


def live(agent: Optional[mr.GcsAgent], online: bool, now_unix: float) -> Optional[mr.Position]:
    """The LTE module's newest report while it is live: the aircraft online, and the report recent."""
    pos = agent.position if agent is not None else None
    return pos if online and pos is not None and now_unix - pos.time < LIVE_S else None


class Where(NamedTuple):
    text: str
    color: str
    fix: Optional[mr.Position]  # the position shown, which Map and Copy take; None: nothing to show


def position(agent: Optional[mr.GcsAgent], online: bool, now_unix: float) -> Where:
    """The aircraft's own GNSS position, whatever its flight controller does: live while it reports, else the
    last known one (the relay keeps it)."""
    pos = live(agent, online, now_unix)
    fix = agent.last_fix if agent is not None else None
    if pos is not None and pos.has_fix:
        return Where(f"{pos.lat / 1e7:.5f}, {pos.lon / 1e7:.5f} · {pos.sats} satellites", TEXT, pos)
    if fix is not None:
        return Where(f"last known {fix.lat / 1e7:.5f}, {fix.lon / 1e7:.5f}, {mr.fmt_age(now_unix - fix.time)} ago",
                     MUTED if online else AMBER, fix)
    if pos is not None and pos.flags & mr.POS_NO_GNSS:
        return Where("the LTE module cannot read its GNSS", MUTED, None)
    if pos is not None:
        return Where(f"GNSS searching ({pos.sats} satellites)", MUTED, None)
    return Where("-", TEXT, None)


def coordinates(pos: mr.Position) -> str:
    """What Copy copies."""
    return f"{pos.lat / 1e7:.6f}, {pos.lon / 1e7:.6f}"


def map_link(pos: mr.Position) -> str:
    """The position on Google Maps (in the browser, or the Maps app on a phone), satellite view at hand."""
    return f"https://www.google.com/maps/search/?api=1&query={pos.lat / 1e7:.6f},{pos.lon / 1e7:.6f}"


def module(agent: Optional[mr.GcsAgent], online: bool, now_unix: float) -> Tuple[Shown, Shown]:
    """The Module line (its flight controller, its power) and the chip's temperature after it, in a colour of its
    own; '-' while the module does not report."""
    pos = live(agent, online, now_unix)
    if pos is None:
        return Shown("-"), Shown("")
    if pos.fc_silent == mr.U16_UNKNOWN:
        line = Shown("flight controller not heard yet", AMBER)
    elif pos.fc_is_silent:
        line = Shown(f"flight controller silent for {mr.fmt_age(pos.fc_silent)}", RED)
    else:
        line = Shown("flight controller talking")
    power = pos.power_text()  # external power (USB, the BEC), or the module's cell and its charge
    if power:
        line = line._replace(text=f"{line.text} · {power}")
    temp = Shown("")
    if pos.temp != mr.TEMP_UNKNOWN:
        temp = Shown(f" · {pos.temp} °C", RED if pos.temp >= mr.TEMP_HOT else AMBER if pos.temp >= mr.TEMP_WARM
                     else TEXT)
    return line, temp


class Voice:
    """The Voice row, the locator voice: while it is on, the speaker on the aircraft's board sounds, to find it in
    the last metres. The switch shows the relay's: the relay keeps it, whoever switched it, also while the
    aircraft is away."""

    ANSWER_S = 10.0  # an aircraft that has not said it sounds by then may not know the voice

    def __init__(self) -> None:
        self.waiting_at: Optional[float] = None  # on, the aircraft online, not sounding yet: since when

    def show(self, agent: Optional[mr.GcsAgent], status: Optional[mr.LinkStatus],
             now: float) -> Optional[Tuple[bool, Shown]]:
        """The switch's position and the line beside it; None: no news, keep what is shown."""
        if agent is None or not agent.client.session:
            return False, Shown("Not connected to the relay", DIM)
        pending = agent.voice_request
        if pending is not None:  # sent, and not yet in the relay's STATUS
            return pending, Shown("Switching on…" if pending else "Switching off…", TEXT)
        if status is None:
            return None
        waiting = status.voice_on and status.online and not (status.speaking or status.voice_failed)
        if not waiting:
            self.waiting_at = None
        elif self.waiting_at is None:
            self.waiting_at = now
        if not status.voice_on:
            line = Shown("Off: switch on to hear the aircraft", DIM)
        elif status.voice_failed:
            line = Shown("On, but the aircraft cannot play it", RED)  # its modem refuses
        elif status.speaking and status.online:
            line = Shown("On: the aircraft's speaker is sounding", GREEN)
        elif status.speaking:  # and it goes on without the relay
            line = Shown("On: it was sounding when last heard", AMBER)
        elif not status.online:
            line = Shown("On: sounds once the aircraft is back", AMBER)
        elif now - self.waiting_at < self.ANSWER_S:
            line = Shown("On: waiting for the aircraft…", TEXT)
        else:  # it does not say whether it sounds
            line = Shown("On, but no answer: firmware before 1.5.0?", AMBER)
        return status.voice_on, line


class Camera(NamedTuple):
    text: str
    color: str
    fraction: Optional[float]  # how much of the photo is here, while one arrives
    ready: bool  # Snapshot can be pressed
    busy: bool  # a photo is asked for or arriving: the size stays as it is


def camera(agent: Optional[mr.GcsAgent], online: bool, size: int, now: float) -> Camera:
    """Snapshot, at the bottom of the card: a photo from the aircraft's camera, by way of the relay. size: the
    one chosen (an index into SIZE_NAMES)."""
    if agent is None or not agent.client.session:
        return Camera("Not connected to the relay", DIM, None, False, False)
    photos = agent.photos
    arriving = photos.arriving
    if arriving is not None:
        info, got = arriving
        return Camera(f"Arriving: {got / 1024:.0f} of {info.size / 1024:.0f} KB ({info.width}×{info.height})",
                      TEXT, got / info.size, False, True)
    if photos.asked_at is not None:
        return Camera("Asking the aircraft…", TEXT, None, False, True)
    if photos.problem and now - photos.problem_at < 60.0:
        return Camera(f"No photo: {photos.problem}", AMBER, None, online, False)
    if not online:
        return Camera("The aircraft is offline", DIM, None, False, False)
    return Camera(SIZE_HINTS[size], DIM, None, True, False)


# ---------------------------------------------------------------------------------------------
# The moving map: where the aircraft has been, and where it points


class Track:
    """The aircraft's path for the moving map: its LTE module's GNSS fixes (one every 5 s) as the agent gets them,
    the first often the relay's last known position. Fed and read on one thread."""

    MOST = 2000  # fixes kept: almost three hours

    def __init__(self) -> None:
        self.fixes: Deque[mr.Position] = deque(maxlen=self.MOST)

    def add(self, pos: Optional[mr.Position]) -> bool:
        """Keeps a fix newer than the last one kept; True if it did."""
        if pos is None or not pos.has_fix or (self.fixes and pos.time <= self.fixes[-1].time):
            return False
        self.fixes.append(pos)
        return True

    @property
    def newest(self) -> Optional[mr.Position]:
        return self.fixes[-1] if self.fixes else None


def heading(pos: mr.Position) -> Optional[float]:
    """Where the aircraft moves, in degrees from north, from its GNSS: None below 2 m/s (no telling then)."""
    if pos.course == mr.U16_UNKNOWN or pos.speed == mr.U16_UNKNOWN or pos.speed < 200:
        return None
    return pos.course / 100


# ---------------------------------------------------------------------------------------------
# Photos, as PhotoInbox saves them: MavLTE_<when>_<photo id>.jpg, each with a .json beside it


def photo_files(folder: str) -> List[str]:
    """The photos in the folder, oldest first (by photo id: the relay's clock when it was asked for)."""
    try:
        names = os.listdir(folder)
    except OSError:
        return []
    found = [(int(m.group(1)), os.path.join(folder, name)) for name in names
             for m in [re.fullmatch(r"MavLTE_.*_(\d+)\.jpg", name)] if m]
    return [path for _, path in sorted(found)]


def photo_meta(path: str) -> dict:
    try:
        with open(path[:-4] + ".json", encoding="utf-8") as f:
            meta = json.load(f)
        return meta if isinstance(meta, dict) else {}
    except (OSError, ValueError):
        return {}


def photo_time(meta: dict) -> int:
    """When the photo was taken (unix seconds), or 0."""
    when = meta.get("time") or meta.get("photo_id")
    return when if isinstance(when, int) and when > 0 else 0


def photo_caption(meta: dict, short: bool = False, clock: bool = True) -> str:
    """When and where the photo was taken, from the .json beside it. clock: False leaves the time out, for a
    page that writes it in its viewer's own time zone (photo_time)."""
    parts = []
    when = photo_time(meta)
    if clock and when:
        parts.append(time.strftime("%H:%M:%S" if short else "%d %b %Y, %H:%M:%S", time.localtime(when)))
    if not short and isinstance(meta.get("latitude"), (int, float)):
        parts.append(f"{meta['latitude']:.6f}, {meta.get('longitude', 0):.6f}")
    if isinstance(meta.get("altitude_m"), (int, float)):
        parts.append(f"{meta['altitude_m']:.0f} m" + ("" if short else " above home"))
    heading = meta.get("heading")
    if isinstance(heading, int) and heading != mr.UNKNOWN_HEADING:
        parts.append(f"heading {heading / 100:.0f}°")
    if not short and meta.get("width"):
        parts.append(f"{meta['width']}×{meta.get('height')}, {meta.get('size', 0) / 1024:.0f} KB")
    return " · ".join(parts)

#!/usr/bin/env python3
"""MavLTE: the GCS agent as a window.

Gives Mission Planner (or QGroundControl) a TCP and a UDP port, each with its own switch. Two LEDs
on each: Available (green, left) while the aircraft's LTE module is online at the relay, whatever
the switches; Connected (blue, next to the switch) while it is online and that port is on. Both
switches start off. With both off the app only watches: the aircraft holds its telemetry back.
Snapshot asks the aircraft for a photo from its camera, whatever the switches; photos are kept in
Pictures\\MavLTE, each with a .json beside it saying when and where it was taken. Position shows
where the LTE module's own GNSS puts the aircraft, live or last known; Map shows it on a moving map
(Esri World Imagery) with its track. The Voice
switch (the locator voice) sounds the speaker on the aircraft's board (a two-tone alarm) until it
is switched off, to find it in the last metres; the relay keeps the switch, also while the
aircraft is offline. ☰ → Flight logs lists the flight log's files on the SD card in the aircraft's
LTE module and copies them into Documents\\MavLTE\\Logs, over 4G or over the board's USB cable.

    python mavlte.py            (or double-click MavLTE.pyw, or run MavLTE.exe: build_release.py)

Settings live in the [gcs] section of mavrelay.ini next to this file, the same file that
start-gcs.bat and `mavrelay.py gcs --config mavrelay.ini` use; MavLTE.exe keeps its own (see
settings_file). Tkinter, and Pillow to show the photos (without it they open in the system's viewer).
"""

from __future__ import annotations

import asyncio
import collections
import ctypes
import gc
import io
import logging
import math
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import uuid
import webbrowser
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Callable, Dict, List, Optional, Tuple

import aircraft_card
import boardusb
import maptiles
import mavrelay as mr
from aircraft_card import SIZE_NAMES, photo_caption, photo_files, photo_meta

try:
    from PIL import Image, ImageTk
except ImportError:  # photos then open in the system's viewer
    Image = ImageTk = None

APP = "MavLTE"
# MavLTE.exe unpacks its files (the icon) into a temporary folder: sys._MEIPASS
HERE = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
ICON = os.path.join(HERE, "mavlte.png")


def settings_file() -> str:
    """mavrelay.ini next to this file. MavLTE.exe uses one next to the exe if there is one (to carry
    the app around on a USB stick), otherwise its own in %LOCALAPPDATA%\\MavLTE: a newer exe finds
    it there again, and the key does not travel with a copy of the exe."""
    if not getattr(sys, "frozen", False):
        return os.path.join(HERE, "mavrelay.ini")
    beside = os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "mavrelay.ini")
    if os.path.exists(beside):
        return beside
    base = os.environ.get("LOCALAPPDATA") or os.path.join(os.path.expanduser("~"), ".config")
    return os.path.join(base, APP, "mavrelay.ini")


CONFIG = settings_file()

# the MavGCS dark palette
BG = "#1e1e1e"
SURFACE = "#252526"
FIELD = "#2e2e30"
BORDER = "#454549"
LOG_BG = "#16171a"
TEXT = "#e6e6e6"
MUTED = "#b0b0b4"
DIM = "#7d8ea0"
ACCENT = "#5ccf5c"  # MavGreen
ON_ACCENT = "#04210f"
GREEN = "#5ccf5c"
BLUE = "#4d9bff"
AMBER = "#d8a23a"
RED = "#ff5555"
LED_OFF = "#3a3b3e"
TRACK_OFF = "#4a4b4f"
# the Aircraft card's colours (aircraft_card.py) in this palette
PALETTE = {aircraft_card.TEXT: TEXT, aircraft_card.MUTED: MUTED, aircraft_card.DIM: DIM, aircraft_card.GREEN: GREEN,
           aircraft_card.BLUE: BLUE, aircraft_card.AMBER: AMBER, aircraft_card.RED: RED, aircraft_card.OFF: LED_OFF}

log = logging.getLogger("mavrelay.app")


# ---------------------------------------------------------------------------------------------
# Settings


@dataclass
class Settings:
    name: str = "My UAV"
    server: str = ""
    key: str = ""
    tcp: str = "127.0.0.1:5760"
    udp: str = "127.0.0.1:14550"
    photo_dir: str = ""  # empty: Pictures\MavLTE
    photo_size: str = "medium"
    photo_exposure: int = 0  # steps brighter (+) or darker (-), see aircraft_card.EXPOSURE_MOST
    problem: str = ""  # why the settings file could not be read: the app then starts without it

    @classmethod
    def load(cls, path: str) -> "Settings":
        s = cls()
        try:
            conf = mr.load_config(path, "gcs") if os.path.exists(path) else {}
        except SystemExit as exc:  # unreadable (a broken line, another encoding): start, and say why
            conf = {}
            s.problem = str(exc)
        s.name = conf.get("name", "").strip() or s.name
        s.server = conf.get("server", "").strip()
        s.key = conf.get("key", "").strip()
        for kind in ("tcp", "udp"):
            value = conf.get(kind, "").strip()
            if value and value.lower() not in ("off", "no", "none"):
                setattr(s, kind, value)
        s.photo_dir = conf.get("photo_dir", "").strip()
        size = conf.get("photo_size", "").strip().lower()
        s.photo_size = size if size in SIZE_NAMES else s.photo_size
        try:
            exposure = int(conf.get("photo_exposure", "0").strip() or 0)
        except ValueError:
            exposure = 0
        s.photo_exposure = max(-aircraft_card.EXPOSURE_MOST, min(aircraft_card.EXPOSURE_MOST, exposure))
        return s

    def photo_folder(self) -> str:
        return os.path.expanduser(self.photo_dir) if self.photo_dir else os.path.join(pictures_folder(), APP)

    def save(self, path: str, fields: Tuple[str, ...] = ("name", "server", "key", "tcp", "udp")) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)  # first save of MavLTE.exe
        mr.update_config(path, "gcs", {field: getattr(self, field) for field in fields},
                         remove=("tcp_on", "udp_on"))  # switch states of earlier versions


def name_problem(name: str) -> Optional[str]:
    """Why a vehicle name cannot be kept in mavrelay.ini as it is, or None."""
    if re.search(r"(^|\s)[#;]", name):
        return ("The vehicle name cannot have a # or ; at the start or after a space: mavrelay.ini "
                "would read the rest as a comment. Leave out the space, as in UAV#2.")
    if any(ord(c) < 32 or ord(c) == 127 for c in name):  # a line break pasted in: the file would not read again
        return "The vehicle name cannot have a line break or a tab in it."
    return None


def parse_address(host: str, port: str) -> Tuple[str, int]:
    host = host.strip() or "127.0.0.1"
    try:
        number = int(port)
    except ValueError:
        number = 0
    if not 1 <= number <= 65535:
        raise ValueError("The port must be a number from 1 to 65535.")
    return host, number


# ---------------------------------------------------------------------------------------------
# Photos


def known_folder(folder_id: str, name: str) -> str:
    """One of the user's folders, where Windows keeps it (in OneDrive, say)."""
    if sys.platform == "win32":
        class GUID(ctypes.Structure):
            _fields_ = [("a", ctypes.c_uint32), ("b", ctypes.c_uint16), ("c", ctypes.c_uint16),
                        ("d", ctypes.c_ubyte * 8)]

        u = uuid.UUID(folder_id)
        guid = GUID(u.time_low, u.time_mid, u.time_hi_version, (ctypes.c_ubyte * 8)(*u.bytes[8:]))
        path = ctypes.c_wchar_p()
        try:
            if ctypes.windll.shell32.SHGetKnownFolderPath(ctypes.byref(guid), 0, None, ctypes.byref(path)) == 0:
                try:
                    return path.value
                finally:
                    ctypes.windll.ole32.CoTaskMemFree(path)
        except (AttributeError, OSError):
            pass
    return os.path.join(os.path.expanduser("~"), name)


def pictures_folder() -> str:
    return known_folder("33E28130-4E1E-4676-835A-98395C3BC3BB", "Pictures")  # FOLDERID_Pictures


def logs_folder() -> str:
    """Where the flight logs go: Documents\\MavLTE\\Logs."""
    return os.path.join(known_folder("FDD39AD0-238F-46AF-ADB4-6C85480369C7", "Documents"), APP, "Logs")


def reveal(path: str) -> None:
    """Shows the file in Explorer (or its folder in the system's file manager)."""
    try:
        if sys.platform == "win32":
            subprocess.Popen(["explorer", "/select,", os.path.normpath(path)])
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", os.path.dirname(path)])
    except OSError as exc:
        log.warning("cannot open the folder: %s", exc)


def open_file(path: str) -> None:
    try:
        if sys.platform == "win32":
            os.startfile(path)
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", path])
    except OSError as exc:
        log.warning("cannot open %s: %s", path, exc)


# ---------------------------------------------------------------------------------------------
# The agent, on its own asyncio loop in a background thread


class AgentRunner:
    def __init__(self) -> None:
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.loop.set_exception_handler(self._loop_error)  # into MavLTE's log (the exe has no console)
        self.thread = threading.Thread(target=self._run, name="agent", daemon=True)
        self.thread.start()
        self.agent: Optional[mr.GcsAgent] = None
        self.task: Optional[asyncio.Task] = None

    def _run(self) -> None:
        self.loop.run_forever()
        self.loop.close()

    def _call(self, coro, timeout: float = 30.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    @property
    def running(self) -> bool:
        return self.agent is not None

    def start(self, server: Tuple[str, int], key: bytes, photo_dir: Optional[str] = None,
              on_photo: Optional[Callable[[str, mr.PhotoInfo], None]] = None) -> None:
        """on_photo(path, info) is called on the agent's thread for each photo saved."""
        async def go() -> None:
            self.agent = mr.GcsAgent(server, key, info=f"mavlte-app/{mr.__version__}", photo_dir=photo_dir,
                                     on_photo=on_photo)
            self.task = asyncio.ensure_future(self.agent.run())
            self.task.add_done_callback(self._ended)

        self._call(go())

    def snapshot(self, size: int, exposure: int = 0) -> bool:
        """Asks the aircraft for a photo. False while there is no session with the relay."""
        async def go() -> bool:
            agent = self.agent
            return agent is not None and agent.photos is not None and agent.photos.request(size, exposure)

        return self._call(go())

    def set_voice(self, on: bool) -> bool:
        """Switches the aircraft's locator voice. False while there is no session with the relay."""
        async def go() -> bool:
            return self.agent is not None and self.agent.set_voice(on)

        return self._call(go())

    def set_network(self, mode: int) -> bool:
        """Chooses the aircraft's mobile network (mr.NET_*). False while there is no session with the relay."""
        async def go() -> bool:
            return self.agent is not None and self.agent.set_network(mode)

        return self._call(go())

    def files(self, use: Callable[[mr.FileFetcher], bool]) -> bool:
        """use(the agent's FileFetcher), on the agent's loop: its list(), get() or stop(). False without an agent, or
        what use() returns."""
        async def go() -> bool:
            return self.agent is not None and use(self.agent.files)

        return self._call(go())

    @staticmethod
    def _loop_error(loop, context: dict) -> None:
        log.error("agent: %s", context.get("message", "unexpected error"), exc_info=context.get("exception"))

    @staticmethod
    def _ended(task: asyncio.Task) -> None:
        if not task.cancelled() and task.exception() is not None:
            log.error("agent stopped: %r", task.exception())

    def stop(self) -> None:
        async def go() -> None:
            if self.task is not None:
                self.task.cancel()
                try:
                    await self.task
                except BaseException:
                    pass
            if self.agent is not None:
                self.agent.close_ports()  # also any opened after the agent's task had ended
            self.agent = self.task = None

        self._call(go())

    def set_output(self, kind: str, address: Optional[Tuple[str, int]]) -> Optional[str]:
        """Switches the TCP or UDP port on (address) or off (None). Returns an error text or None."""
        async def go() -> Optional[str]:
            agent = self.agent
            if agent is None:
                return "not running"
            if address is None:
                agent.stop_tcp() if kind == "tcp" else agent.stop_udp()
                return None
            try:
                await (agent.start_tcp(address) if kind == "tcp" else agent.start_udp(address))
            except OSError as exc:
                if isinstance(exc, socket.gaierror):
                    return f"cannot find {address[0]} ({exc.strerror or exc})"
                return f"{kind.upper()} port {address[1]} is in use by another program ({exc.strerror or exc})"
            return None

        return self._call(go())

    def close(self) -> None:
        if self.running:
            self.stop()
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)


# ---------------------------------------------------------------------------------------------
# Widgets


class Switch(tk.Canvas):
    def __init__(self, master, scale: float, bg: str, command: Callable[[bool], None]) -> None:
        self.w, self.h = round(38 * scale), round(20 * scale)
        super().__init__(master, width=self.w, height=self.h, bg=bg, highlightthickness=0, cursor="hand2")
        self.on = False
        self.bind("<Button-1>", lambda _e: command(not self.on))
        self._draw()

    def set(self, on: bool) -> None:
        if on != self.on:
            self.on = on
            self._draw()

    def _draw(self) -> None:
        self.delete("all")
        w, h = self.w - 1, self.h - 1
        fill = ACCENT if self.on else TRACK_OFF
        self.create_oval(0, 0, h, h, fill=fill, outline=fill)
        self.create_oval(w - h, 0, w, h, fill=fill, outline=fill)
        self.create_rectangle(h / 2, 0, w - h / 2, h, fill=fill, outline=fill)
        p = max(2, round(h / 9))
        x = w - h + p if self.on else p
        knob = "white" if self.on else MUTED
        self.create_oval(x, p, x + h - 2 * p, h - p, fill=knob, outline=knob)


class Led(tk.Canvas):
    def __init__(self, master, scale: float, bg: str) -> None:
        d = round(12 * scale)
        super().__init__(master, width=d, height=d, bg=bg, highlightthickness=0)
        self.color = LED_OFF
        self.dot = self.create_oval(1, 1, d - 2, d - 2, fill=LED_OFF, outline="")

    def set(self, color: str) -> None:
        if color != self.color:
            self.color = color
            self.itemconfigure(self.dot, fill=color)


class Bars(tk.Canvas):
    """Signal strength in 0-4 bars; None greys them out."""

    def __init__(self, master, scale: float, bg: str) -> None:
        w, h = round(20 * scale), round(14 * scale)
        super().__init__(master, width=w, height=h, bg=bg, highlightthickness=0)
        step = w / 4
        self.level: Optional[int] = -1
        self.bars = [self.create_rectangle(i * step + 1, h - (i + 1) * h / 4, (i + 1) * step - 1, h, fill=LED_OFF,
                                           outline="") for i in range(4)]
        self.set(None)

    def set(self, level: Optional[int]) -> None:
        if level != self.level:
            self.level = level
            for i, bar in enumerate(self.bars):
                self.itemconfigure(bar, fill=ACCENT if level is not None and i < level else LED_OFF)


def dark_entry(master, app: "App", var: tk.StringVar, width: int, show: str = "") -> tk.Entry:
    return tk.Entry(master, textvariable=var, width=width, show=show, font=app.font, bg=FIELD, fg=TEXT,
                    insertbackground=TEXT, relief="flat", bd=3, highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=ACCENT, disabledbackground=SURFACE, disabledforeground=DIM,
                    selectbackground=ACCENT, selectforeground=ON_ACCENT)


def dark_title_bar(window: tk.Misc) -> None:
    """Dark Windows title bar, like the rest of the window (Windows 10 20H1 and newer)."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        window.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(window.winfo_id())
        on = ctypes.c_int(1)
        for attribute in (20, 19):  # DWMWA_USE_IMMERSIVE_DARK_MODE, and its number before Windows 10 20H1
            if ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, attribute, ctypes.byref(on), ctypes.sizeof(on)) == 0:
                break
    except (AttributeError, OSError):
        pass


def work_area() -> Optional[Tuple[int, int, int, int]]:
    """The screen above the taskbar, (left, top, right, bottom); None but on Windows."""
    if sys.platform != "win32":
        return None
    try:
        from ctypes import wintypes
        area = wintypes.RECT()
        if not ctypes.windll.user32.SystemParametersInfoW(0x30, 0, ctypes.byref(area), 0):  # SPI_GETWORKAREA
            return None
    except (AttributeError, OSError):
        return None
    return area.left, area.top, area.right, area.bottom


def frame_height() -> Optional[int]:
    """What Windows adds to a window's height: its title bar with the edge above it (SM_CYCAPTION, SM_CYSIZEFRAME,
    SM_CXPADDEDBORDER), and the line below (39 pixels at 125 %)."""
    try:
        return sum(ctypes.windll.user32.GetSystemMetrics(i) for i in (4, 33, 92)) + 1
    except (AttributeError, OSError):
        return None


def place_on_screen(window: tk.Tk, scale: float) -> None:
    """Keeps a window above the taskbar (a 1080p screen at 125% has 1020 pixels there): never taller than that room,
    its content giving way at the bottom margins as when maximized (MavLTE asks 1002 pixels there, plus 39 for
    Windows), and opened at the top of the screen when where Windows would put it hides its bottom."""
    area = work_area()
    if area is None:
        return
    left, top, right, bottom = area
    window.update_idletasks()
    height = window.winfo_reqheight()
    frame = frame_height()
    if frame is not None:  # (maximizing still fills the screen: it is not held to this)
        most = max(window.minsize()[1], bottom - top - frame)
        window.maxsize(window.winfo_screenwidth(), most)
        height = min(height, most)
    if height + round(100 * scale) > bottom - top:  # its title bar, and Windows' offset
        window.geometry(f"+{left + round(24 * scale)}+{top}")


class ChannelCard(tk.Frame):
    """One port for GCS software, like the cards of commercial telemetry apps."""

    def __init__(self, app: "App", kind: str, address: str) -> None:
        super().__init__(app.body, bg=SURFACE, highlightbackground=BORDER, highlightthickness=1)
        self.kind = kind
        s, pad = app.scale, round(8 * app.scale)
        top = tk.Frame(self, bg=SURFACE)
        top.pack(fill="x", padx=pad, pady=(pad, round(2 * s)))
        self.available = Led(top, s, SURFACE)  # the aircraft's LTE module is online, switch or not
        self.available.pack(side="left")
        self.led = Led(top, s, SURFACE)  # connected: online, and this port is on
        self.led.pack(side="left", padx=(round(4 * s), 0))
        self.switch = Switch(top, s, SURFACE, lambda on: app.toggle(self, on))
        self.switch.pack(side="left", padx=(round(6 * s), round(8 * s)))
        self.title = tk.Label(top, text=app.settings.name, bg=SURFACE, fg=TEXT, font=app.font_bold)
        self.title.pack(side="left")
        self.bars = Bars(top, s, SURFACE)
        self.bars.pack(side="right")

        host, _, port = address.rpartition(":")
        self.host = tk.StringVar(value=host or "127.0.0.1")
        self.port = tk.StringVar(value=port)
        grid = tk.Frame(self, bg=SURFACE)
        grid.pack(fill="x", padx=pad)
        self.entries = []
        for row, (label, var) in enumerate((("Protocol", None), ("IP", self.host), ("Port", self.port))):
            tk.Label(grid, text=label, bg=SURFACE, fg=MUTED, font=app.font).grid(row=row, column=0, sticky="w",
                                                                                 pady=1)
            if var is None:
                tk.Label(grid, text=kind.upper(), bg=SURFACE, fg=TEXT, font=app.font).grid(
                    row=row, column=1, sticky="w", padx=(12, 0))
            else:
                entry = dark_entry(grid, app, var, 16 if label == "IP" else 7)
                entry.grid(row=row, column=1, sticky="w", padx=(12, 0), pady=1)
                self.entries.append(entry)
        self.status = tk.Label(self, text="Off", bg=SURFACE, fg=DIM, font=app.font_small, anchor="w",
                               justify="left", wraplength=round(300 * s))
        self.status.pack(fill="x", padx=pad, pady=(round(2 * s), pad))
        # as wide as the card, so that an address fits on one line and the window stays short; never
        # wider: the label would widen the card, which would widen the label again
        self.bind("<Configure>", lambda e: self.status.configure(wraplength=max(100, e.width - 2 * pad - 4)))

    def address(self) -> Tuple[str, int]:
        return parse_address(self.host.get(), self.port.get())

    def text(self) -> str:
        return f"{self.host.get().strip() or '127.0.0.1'}:{self.port.get().strip()}"

    def set_on(self, on: bool) -> None:
        self.switch.set(on)
        for entry in self.entries:
            entry.configure(state="disabled" if on else "normal")

    def show(self, text: str, color: str = DIM) -> None:
        if self.status.cget("text") != text or self.status.cget("fg") != color:
            self.status.configure(text=text, fg=color)


class Thumbnail(tk.Canvas):
    """The last photo, small; a drawn camera until there is one."""

    def __init__(self, master, scale: float, command: Callable[[], None]) -> None:
        self.w, self.h = round(72 * scale), round(54 * scale)
        super().__init__(master, width=self.w, height=self.h, bg=LOG_BG, highlightthickness=1,
                         highlightbackground=BORDER, cursor="hand2")
        self.image = None
        self.path: Optional[str] = None
        self.bind("<Button-1>", lambda _e: command())
        self.show(None)

    def show(self, path: Optional[str]) -> None:
        self.path = path
        self.delete("all")
        self.image = None
        if path and Image is not None:
            try:
                with Image.open(path) as img:
                    img.thumbnail((self.w, self.h))
                    self.image = ImageTk.PhotoImage(img)
            except Exception as exc:  # a broken file must not break the window
                log.warning("cannot show %s: %s", os.path.basename(path), exc)
        if self.image is not None:
            self.create_image(self.w // 2 + 1, self.h // 2 + 1, image=self.image)
            return
        cx, cy, u = self.w / 2, self.h / 2, self.h / 8
        self.create_rectangle(cx - 2.3 * u, cy - 1.2 * u, cx + 2.3 * u, cy + 1.8 * u, outline=DIM, width=2)
        self.create_rectangle(cx - 1.0 * u, cy - 1.8 * u, cx + 0.4 * u, cy - 1.2 * u, outline=DIM, fill=DIM)
        self.create_oval(cx - 0.9 * u, cy - 0.6 * u, cx + 0.9 * u, cy + 1.2 * u, outline=DIM, width=2)


class Segments(tk.Frame):
    """One of a few choices, side by side (selected -1: none). compact: no taller than a line of text."""

    def __init__(self, master, app: "App", labels: Tuple[str, ...], selected: int,
                 command: Callable[[int], None], compact: bool = False) -> None:
        super().__init__(master, bg=BORDER, padx=1, pady=1)
        self.command = command
        self.labels = []
        for i, text in enumerate(labels):
            label = tk.Label(self, text=text, font=app.font_small, padx=round((6 if compact else 8) * app.scale),
                             pady=0 if compact else round(3 * app.scale), cursor="hand2")
            label.pack(side="left", padx=(1 if i else 0, 0))
            label.bind("<Button-1>", lambda _e, i=i: self.select(i, True))
            self.labels.append(label)
        self.selected = -1
        self.enabled = True
        self.pending = False  # the selected one asked for, not in effect yet: in amber
        self.select(selected)

    def set_pending(self, pending: bool) -> None:
        if pending != self.pending:
            self.pending = pending
            self._paint()

    def select(self, index: int, clicked: bool = False) -> None:
        if clicked and not self.enabled:
            return
        self.selected = index
        self._paint()
        if clicked:
            self.command(index)

    def set_enabled(self, enabled: bool) -> None:
        if enabled != self.enabled:
            self.enabled = enabled
            self._paint()

    def _paint(self) -> None:
        for i, label in enumerate(self.labels):
            on = i == self.selected
            if self.enabled:
                label.configure(bg=(AMBER if self.pending else ACCENT) if on else FIELD, fg=ON_ACCENT if on else TEXT)
            else:
                label.configure(bg=BORDER if on else FIELD, fg=MUTED if on else DIM)


class Stepper(tk.Frame):
    """A value from -most to +most, one step down or up with the buttons at its sides; as tall as Segments."""

    def __init__(self, master, app: "App", value: int, most: int, text: Callable[[int], str],
                 command: Callable[[int], None]) -> None:
        super().__init__(master, bg=BORDER, padx=1, pady=1)
        self.most, self.text, self.command = most, text, command
        pad, pady = round(7 * app.scale), round(3 * app.scale)
        self.down = tk.Label(self, text="−", font=app.font_small, padx=pad, pady=pady, cursor="hand2")
        self.down.pack(side="left")
        widest = max((text(v) for v in range(-most, most + 1)), key=len)
        self.label = tk.Label(self, text=text(value), font=app.font_small, width=len(widest), padx=round(2 * app.scale),
                              pady=pady)
        self.label.pack(side="left", padx=1)
        self.up = tk.Label(self, text="+", font=app.font_small, padx=pad, pady=pady, cursor="hand2")
        self.up.pack(side="left")
        self.down.bind("<Button-1>", lambda _e: self.step(-1))
        self.up.bind("<Button-1>", lambda _e: self.step(1))
        self.value, self.enabled = value, True
        self._paint()

    def step(self, by: int) -> None:
        value = max(-self.most, min(self.most, self.value + by))
        if self.enabled and value != self.value:
            self.value = value
            self._paint()
            self.command(value)

    def set_enabled(self, enabled: bool) -> None:
        if enabled != self.enabled:
            self.enabled = enabled
            self._paint()

    def _paint(self) -> None:
        self.label.configure(text=self.text(self.value), bg=FIELD, fg=TEXT if self.enabled else DIM)
        for button, end in ((self.down, -self.most), (self.up, self.most)):
            usable = self.enabled and self.value != end
            button.configure(bg=FIELD, fg=TEXT if usable else DIM, cursor="hand2" if usable else "")


class CameraRow(tk.Frame):
    """Snapshot, at the bottom of the Aircraft card: a photo from the aircraft's camera, by way of the
    relay."""

    SIZES = ("Small", "Medium", "Large")

    def __init__(self, app: "App", master: tk.Misc) -> None:
        super().__init__(master, bg=SURFACE)
        s, pad = app.scale, round(8 * app.scale)
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")
        inner = tk.Frame(self, bg=SURFACE)
        inner.pack(fill="x", padx=pad, pady=round(6 * s))
        self.thumbnail = Thumbnail(inner, s, app.show_photo)  # the last photo: click for the viewer
        self.thumbnail.pack(side="right", anchor="n", padx=(pad, 0))
        left = tk.Frame(inner, bg=SURFACE)
        left.pack(side="left", fill="both", expand=True)
        top = tk.Frame(left, bg=SURFACE)
        top.pack(fill="x")
        self.button = ttk.Button(top, text="Snapshot", style="Accent.TButton", command=app.snapshot)
        self.button.pack(side="left")
        self.size = Segments(top, app, self.SIZES, SIZE_NAMES.index(app.settings.photo_size), app.choose_size)
        self.size.pack(side="left", padx=(pad, 0))
        # the exposure (1.8.9): brighter for the ground beside the white aircraft that fills the middle, or darker
        self.exposure = Stepper(top, app, app.settings.photo_exposure, aircraft_card.EXPOSURE_MOST,
                                aircraft_card.exposure_text, app.choose_exposure)
        self.exposure.pack(side="left", padx=(pad, 0))
        self.status = tk.Label(left, text="", bg=SURFACE, fg=DIM, font=app.font_small, anchor="w", justify="left")
        self.status.pack(fill="x", pady=(round(3 * s), 0))
        self.bar_h = max(3, round(3 * s))
        # arriving: how much (width 1: a canvas asks for 10 cm unless told, and would widen the window)
        self.bar = tk.Canvas(left, width=1, height=self.bar_h, bg=SURFACE, highlightthickness=0)
        self.bar.pack(fill="x", pady=(round(2 * s), 0))
        self.fraction: Optional[float] = -1.0
        self.last = tk.Label(left, text="", bg=SURFACE, fg=MUTED, font=app.font_small, anchor="w", justify="left")
        self.last.pack(fill="x", pady=(round(2 * s), 0))
        self.enabled = True

    def show(self, text: str, color: str = DIM, fraction: Optional[float] = None) -> None:
        if self.status.cget("text") != text or self.status.cget("fg") != color:
            self.status.configure(text=text, fg=color)
        if fraction != self.fraction:
            self.fraction = fraction
            self.bar.delete("all")
            if fraction is not None:
                w = self.bar.winfo_width()
                self.bar.create_rectangle(0, 0, w, self.bar_h, fill=FIELD, outline="")
                self.bar.create_rectangle(0, 0, round(w * min(1.0, fraction)), self.bar_h, fill=ACCENT, outline="")

    def set_enabled(self, enabled: bool) -> None:
        if enabled != self.enabled:
            self.enabled = enabled
            self.button.state(["!disabled"] if enabled else ["disabled"])


class VoiceRow(tk.Frame):
    """The locator voice, a line of the Aircraft card: while it is on, the speaker on the aircraft's board
    sounds again and again, to find it in the last metres. The relay keeps the switch."""

    def __init__(self, app: "App", master: tk.Misc) -> None:
        super().__init__(master, bg=SURFACE)
        s = app.scale
        self.switch = Switch(self, s * 0.7, SURFACE, app.toggle_voice)  # no taller than a line of text
        self.switch.pack(side="left")
        self.status = tk.Label(self, text="", bg=SURFACE, fg=DIM, font=app.font, anchor="w", justify="left")
        self.status.pack(side="left", padx=(round(6 * s), 0))

    def show(self, text: str, color: str = DIM) -> None:
        if self.status.cget("text") != text or self.status.cget("fg") != color:
            self.status.configure(text=text, fg=color)


class LogWindow(tk.Toplevel):
    """MavLTE's log (☰ → Show log): what the app did and heard, newest at the bottom, in a window of its own, as the
    main window has no height to spare on a 1080p laptop (maximized, it had none at all for the log)."""

    def __init__(self, app: "App") -> None:
        super().__init__(app.root)
        self.app = app
        self.configure(bg=BG)
        if app.icon is not None:
            self.iconphoto(False, app.icon)
        self.title(f"{app.settings.name} · {APP} log")
        frame = tk.Frame(self, bg=BG, padx=round(8 * app.scale), pady=round(8 * app.scale))
        frame.pack(fill="both", expand=True)
        self.text = tk.Text(frame, height=24, width=96, font=("Consolas", 9), bg=LOG_BG, fg="#b8bcc2", relief="flat",
                            highlightbackground=BORDER, highlightthickness=1, wrap="word", insertbackground=TEXT,
                            state="disabled")
        scroll = ttk.Scrollbar(frame, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.text.pack(side="left", fill="both", expand=True)
        self.bind("<Escape>", lambda _e: self.destroy())
        dark_title_bar(self)
        self.add(list(app.log_history))

    def add(self, lines: List[str]) -> None:
        if not lines:
            return
        at_end = self.text.yview()[1] >= 0.999  # scrolled back to read: it stays there
        self.text.configure(state="normal")
        self.text.insert("end", "\n".join(lines) + "\n")
        excess = int(self.text.index("end-1c").split(".")[0]) - App.LOG_LINES
        if excess > 0:
            self.text.delete("1.0", f"{excess}.0")
        if at_end:
            self.text.see("end")
        self.text.configure(state="disabled")


class PhotoViewer(tk.Toplevel):
    """A photo from the aircraft, fitted to the window, with when and where it was taken. The arrow
    keys (or the buttons) go through the others in the folder."""

    def __init__(self, app: "App") -> None:
        super().__init__(app.root)
        self.app = app
        self.configure(bg=BG)
        if app.icon is not None:
            self.iconphoto(False, app.icon)
        s, pad = app.scale, round(10 * app.scale)
        self.bar = bar = tk.Frame(self, bg=SURFACE)
        bar.pack(side="bottom", fill="x")
        self.caption = tk.Label(bar, text="", bg=SURFACE, fg=TEXT, font=app.font, anchor="w", justify="left")
        self.caption.pack(fill="x", padx=pad, pady=(round(8 * s), round(6 * s)))
        bar.bind("<Configure>", lambda e: self.caption.configure(wraplength=max(100, e.width - 2 * pad)))
        buttons = tk.Frame(bar, bg=SURFACE)
        buttons.pack(fill="x", padx=pad, pady=(0, round(8 * s)))
        self.older = ttk.Button(buttons, text="◀ Older", command=lambda: self.step(-1))
        self.older.pack(side="left")
        self.newer = ttk.Button(buttons, text="Newer ▶", command=lambda: self.step(1))
        self.newer.pack(side="left", padx=(round(6 * s), 0))
        ttk.Button(buttons, text="Show in folder", command=lambda: self.path and reveal(self.path)).pack(side="right")
        self.picture = tk.Label(self, bg=LOG_BG, bd=0)
        self.picture.pack(fill="both", expand=True)
        self.bind("<Left>", lambda _e: self.step(-1))
        self.bind("<Right>", lambda _e: self.step(1))
        self.bind("<Escape>", lambda _e: self.destroy())
        self.picture.bind("<Configure>", lambda _e: self._fit_later())
        self.path: Optional[str] = None
        self.paths: List[str] = []
        self.full = None
        self.shown = None
        self.fit_job: Optional[str] = None
        self.sized = False
        dark_title_bar(self)

    def show(self, path: str) -> None:
        self.paths = photo_files(os.path.dirname(path)) or [path]
        self.path = path if path in self.paths else self.paths[-1]
        index = self.paths.index(self.path)
        self.older.state(["!disabled"] if index > 0 else ["disabled"])
        self.newer.state(["!disabled"] if index < len(self.paths) - 1 else ["disabled"])
        meta = photo_meta(self.path)
        self.caption.configure(text=photo_caption(meta) or os.path.basename(self.path))
        self.title(f"{APP} photo {index + 1} of {len(self.paths)}")
        try:
            with Image.open(self.path) as img:
                img.load()
                self.full = img.copy()
        except Exception as exc:
            self.full = None
            self.picture.configure(image="", text=f"Cannot show this photo: {exc}", fg=MUTED, font=self.app.font)
            return
        if not self.sized:  # the photo at its own size, if the screen has room
            self.sized = True
            self.update_idletasks()
            w = max(round(520 * self.app.scale), min(self.full.width, round(self.winfo_screenwidth() * 0.8)))
            self.caption.configure(wraplength=w - round(20 * self.app.scale))
            self.update_idletasks()
            h = min(self.full.height, round(self.winfo_screenheight() * 0.75))
            self.geometry(f"{w}x{h + self.bar.winfo_reqheight()}")
        self._fit()

    def is_newest(self) -> bool:
        return not self.paths or self.path == self.paths[-1]

    def step(self, delta: int) -> None:
        if self.path in self.paths:
            index = self.paths.index(self.path) + delta
            if 0 <= index < len(self.paths):
                self.show(self.paths[index])

    def _fit_later(self) -> None:
        if self.fit_job is not None:
            self.after_cancel(self.fit_job)
        self.fit_job = self.after(80, self._fit)

    def _fit(self) -> None:
        self.fit_job = None
        if self.full is None:
            return
        w, h = max(1, self.picture.winfo_width()), max(1, self.picture.winfo_height())
        scale = min(w / self.full.width, h / self.full.height)
        if scale >= 1 or w < 10:  # never larger than it is: the pixels are all there are
            img = self.full
        else:
            img = self.full.resize((max(1, round(self.full.width * scale)), max(1, round(self.full.height * scale))),
                                   Image.LANCZOS)
        self.shown = ImageTk.PhotoImage(img)
        self.picture.configure(image=self.shown, text="")


class MapWindow(tk.Toplevel):
    """The moving map: the aircraft on Esri World Imagery, with its track since MavLTE started (the relay's last
    known position first), from the LTE module's own GNSS, one fix every 5 s. It follows the aircraft until
    dragged; the wheel, the + and - keys or the buttons zoom, and Follow brings it back to the aircraft."""

    ZOOM = 16  # to begin with: about a kilometre across
    TRAIL = "#ffd23f"  # the track, yellow: it shows on fields and forest alike
    CASING = "#101010"

    def __init__(self, app: "App") -> None:
        super().__init__(app.root)
        self.app = app
        self.configure(bg=BG)
        if app.icon is not None:
            self.iconphoto(False, app.icon)
        self.title(f"{app.settings.name} · {APP} map")
        s, pad = app.scale, round(8 * app.scale)
        bar = tk.Frame(self, bg=SURFACE)
        bar.pack(side="bottom", fill="x")
        ttk.Button(bar, text="Google Maps", command=app.open_google_maps).pack(side="right", padx=(0, pad), pady=pad)
        self.follow_button = ttk.Button(bar, text="Follow", command=self.follow)
        self.follow_button.pack(side="right", padx=(0, round(6 * s)))
        for text, step in (("+", 1), ("−", -1)):
            ttk.Button(bar, text=text, width=3, command=lambda step=step: self.zoom_by(step)).pack(
                side="right", padx=(0, round(6 * s)))
        self.where = tk.Label(bar, text="", bg=SURFACE, fg=TEXT, font=app.font, anchor="w")
        self.where.pack(side="left", fill="x", expand=True, padx=(pad, 0))
        self.canvas = c = tk.Canvas(self, width=round(600 * s), height=round(420 * s), bg=LOG_BG,
                                    highlightthickness=0, cursor="fleur")
        c.pack(fill="both", expand=True)
        self.zoom = self.ZOOM
        self.following = True
        self.center: Optional[Tuple[float, float]] = None  # latitude, longitude of the middle, once dragged
        self.drag: Optional[Tuple[int, int]] = None
        self.photos: Dict[maptiles.Key, ImageTk.PhotoImage] = {}  # the tiles on the canvas
        self.drawn: Optional[tuple] = None  # what the map was drawn for: it draws again when that changes
        c.bind("<ButtonPress-1>", lambda e: setattr(self, "drag", (e.x, e.y)))
        c.bind("<B1-Motion>", self._move)
        c.bind("<ButtonRelease-1>", lambda _e: setattr(self, "drag", None))
        c.bind("<MouseWheel>", lambda e: self.zoom_by(1 if e.delta > 0 else -1, (e.x, e.y)))
        c.bind("<Button-4>", lambda e: self.zoom_by(1, (e.x, e.y)))  # the wheel on X11
        c.bind("<Button-5>", lambda e: self.zoom_by(-1, (e.x, e.y)))
        c.bind("<Configure>", lambda _e: self.draw())
        for key, step in (("<plus>", 1), ("<KP_Add>", 1), ("<minus>", -1), ("<KP_Subtract>", -1)):
            self.bind(key, lambda _e, step=step: self.zoom_by(step))
        self.bind("<Escape>", lambda _e: self.destroy())
        self._paint_follow()
        dark_title_bar(self)
        self.job = self.after(100, self._tick)

    def destroy(self) -> None:
        if getattr(self, "job", None):  # (not there if the window failed while it was made)
            self.after_cancel(self.job)
        super().destroy()

    def _tick(self) -> None:
        if self.app.tiles.arrived() or self._drawn_for() != self.drawn:
            self.draw()
        self.job = self.after(250, self._tick)

    def _drawn_for(self) -> tuple:
        track = self.app.track
        return (self.zoom, self.center, track.newest, len(track.fixes), self.app.fix_live, self.app.shown_where,
                self.canvas.winfo_width(), self.canvas.winfo_height())

    def _view(self) -> Optional[Tuple[float, float]]:
        """The latitude and longitude in the middle of the map: the aircraft's, while it follows it."""
        newest = self.app.track.newest
        if self.center is None and newest is not None:
            return newest.lat / 1e7, newest.lon / 1e7
        return self.center

    def draw(self) -> None:
        c, s = self.canvas, self.app.scale
        w, h = c.winfo_width(), c.winfo_height()
        if w <= 1 or h <= 1:  # not laid out (yet): the size it asked for
            w, h = int(c.cget("width")), int(c.cget("height"))
        self.drawn = self._drawn_for()
        text, color = self.app.shown_where  # as the card's Position
        self.where.configure(text=text, fg=color)
        c.delete("all")
        view = self._view()
        if view is None:
            c.create_text(w / 2, h / 2, text="No position from the aircraft yet", fill=MUTED, font=self.app.font)
            return
        cx, cy = maptiles.to_pixel(*view, self.zoom)
        photos: Dict[maptiles.Key, ImageTk.PhotoImage] = {}
        for col, row in maptiles.tiles_around(cx, cy, w, h, self.zoom):
            key = maptiles.tile_key(self.zoom, col, row)
            photo = photos.get(key) or self.photos.get(key) or self._picture(key)
            if photo is not None:
                photos[key] = photo
                c.create_image(round(col * maptiles.TILE - cx + w / 2), round(row * maptiles.TILE - cy + h / 2),
                               image=photo, anchor="nw")
        self.photos = photos
        points: List[float] = []
        for pos in self.app.track.fixes:
            x, y = maptiles.to_pixel(pos.lat / 1e7, pos.lon / 1e7, self.zoom)
            points += (x - cx + w / 2, y - cy + h / 2)
        if len(points) >= 4:
            for fill, width in ((self.CASING, 5), (self.TRAIL, 3)):
                c.create_line(points, fill=fill, width=round(width * s), capstyle="round", joinstyle="round")
        newest = self.app.track.newest
        if newest is not None:
            x, y = maptiles.to_pixel(newest.lat / 1e7, newest.lon / 1e7, self.zoom)
            self._aircraft(x - cx + w / 2, y - cy + h / 2, aircraft_card.heading(newest),
                           GREEN if self.app.fix_live else AMBER)
        credit = c.create_text(w - round(4 * s), h - round(2 * s), text="Imagery: " + maptiles.ATTRIBUTION,
                               anchor="se", fill=MUTED, font=self.app.font_small)
        x0, y0, x1, y1 = c.bbox(credit)
        c.tag_lower(c.create_rectangle(x0 - 4, y0, x1 + 4, y1 + 2, fill=LOG_BG, outline=""), credit)

    def _picture(self, key: maptiles.Key) -> Optional[ImageTk.PhotoImage]:
        data = self.app.tiles.get(key)
        if data is None:
            return None
        try:
            with Image.open(io.BytesIO(data)) as img:
                return ImageTk.PhotoImage(img)
        except Exception as exc:  # not a picture: the tile stays empty
            log.debug("map tile %s: %s", key, exc)
            return None

    def _aircraft(self, x: float, y: float, heading: Optional[float], color: str) -> None:
        """An arrow where it points, while it moves; a dot when it does not."""
        c, s = self.canvas, self.app.scale
        if heading is None:
            r = 7 * s
            c.create_oval(x - r, y - r, x + r, y + r, fill=color, outline=self.CASING, width=2)
            return
        a = math.radians(heading)
        shape = []
        for angle, length in ((0, 14), (140, 10), (180, 4), (220, 10)):  # tip, right wing, tail notch, left wing
            b = a + math.radians(angle)
            shape += (x + length * s * math.sin(b), y - length * s * math.cos(b))
        c.create_polygon(shape, fill=color, outline=self.CASING, width=2)

    def _move(self, event) -> None:
        view = self._view()
        if self.drag is None or view is None:
            return
        cx, cy = maptiles.to_pixel(*view, self.zoom)
        self.center = maptiles.to_latlon(cx - (event.x - self.drag[0]), cy - (event.y - self.drag[1]), self.zoom)
        self.drag = (event.x, event.y)
        if self.following:
            self.following = False
            self._paint_follow()
        self.draw()

    def zoom_by(self, step: int, at: Optional[Tuple[int, int]] = None) -> None:
        """One zoom level in or out; at: the pointer, whose place stays under it (unless following)."""
        zoom = max(maptiles.MIN_ZOOM, min(maptiles.MAX_ZOOM, self.zoom + step))
        view = self._view()
        if zoom == self.zoom or view is None:
            return
        if at is not None and not self.following:
            dx, dy = at[0] - self.canvas.winfo_width() / 2, at[1] - self.canvas.winfo_height() / 2
            cx, cy = maptiles.to_pixel(*view, self.zoom)
            x, y = maptiles.to_pixel(*maptiles.to_latlon(cx + dx, cy + dy, self.zoom), zoom)
            self.center = maptiles.to_latlon(x - dx, y - dy, zoom)
        self.zoom = zoom
        self.draw()

    def follow(self) -> None:
        self.following, self.center = True, None
        self._paint_follow()
        self.draw()

    def _paint_follow(self) -> None:
        self.follow_button.configure(style="Accent.TButton" if self.following else "TButton")


# ---------------------------------------------------------------------------------------------
# Flight logs (README, "Flight log")

Entry = Tuple[str, int, int, int]  # a log file on the aircraft's card: name, bytes, first and last time (unix s)
NO_BOARD = ("No MavLTE board on a USB port. Plug the board's USB-C into this computer: unplug the BEC first, "
            "as USB-C and the board's 5V pin are one supply.")


def log_copy_name(name: str, start: int) -> str:
    """The copy's name: when the log began (this computer's time), then its name on the card, as
    "2026-10-02 12-35 LOG00012.csv"; just "LOG00012.csv" if the board never knew the time."""
    stem = name[:-4] if name.upper().endswith(".CSV") else name
    stem = re.sub(r"[^A-Za-z0-9_-]", "_", stem)  # (the board's names are LOGnnnnn: nothing else can make a path)
    when = time.strftime("%Y-%m-%d %H-%M ", time.localtime(start)) if start else ""
    return f"{when}{stem}.csv"


def size_text(n: int) -> str:
    if n < 1024:
        return f"{n} B"
    if n < 1024 * 1024:
        return f"{n / 1024:.0f} KB"
    return f"{n / 1024 / 1024:.1f} MB"


def length_text(start: int, end: int) -> str:
    if not start or end < start:
        return ""
    s = end - start
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60} min"
    return f"{s // 3600} h {s % 3600 // 60:02d} min"


class AirLogs:
    """The aircraft's log files over 4G, through the relay: the agent's FileFetcher. Its callbacks come on the agent's
    thread."""

    def __init__(self, runner: AgentRunner) -> None:
        self.runner = runner
        self.done: Optional[Callable[[bool, str], None]] = None  # the download going on

    def list(self, first: int, done: Callable[[Optional[List[Entry]], int, str], None]) -> Optional[str]:
        """Asks; why it cannot, or None."""
        if not self.runner.running:
            return "Not connected: set the relay server and key first (☰ → Settings)"
        if not self.runner.files(lambda files: files.list(first, done)):
            return "Not connected to the relay (yet)"
        return None

    def get(self, name: str, offset: int, write: Callable[[int, bytes], None], progress: Callable[[int, int], None],
            done: Callable[[bool, str], None]) -> Optional[str]:
        def ended(ok: bool, problem: str) -> None:
            self.done = None
            done(ok, problem)

        self.done = ended
        if not self.runner.running or not self.runner.files(lambda files: files.get(name, offset, write, progress,
                                                                                      ended)):
            self.done = None
            return "Not connected to the relay"
        return None

    def stop(self) -> None:
        def stop(files: mr.FileFetcher) -> bool:
            files.stop()
            done, self.done = self.done, None
            if done is not None:  # (the fetcher says nothing more of a download it was told to stop)
                done(False, "stopped")
            return True

        if self.runner.running:
            self.runner.files(stop)

    def close(self) -> None:
        self.stop()


class UsbLogs:
    """The board's log files over its USB cable (boardusb.py): the port, in a thread of its own, opened when first
    needed and closed with this. Its callbacks come on that thread."""

    def __init__(self) -> None:
        self.jobs: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self.stopped = threading.Event()
        self.link: Optional[boardusb.BoardLink] = None
        threading.Thread(target=self._run, name="usb-logs", daemon=True).start()

    def list(self, first: int, done: Callable[[Optional[List[Entry]], int, str], None]) -> Optional[str]:
        self.jobs.put(("list", first, done))
        return None

    def get(self, name: str, offset: int, write: Callable[[int, bytes], None], progress: Callable[[int, int], None],
            done: Callable[[bool, str], None]) -> Optional[str]:
        self.stopped.clear()
        self.jobs.put(("get", name, offset, write, progress, done))
        return None

    def stop(self) -> None:
        self.stopped.set()

    def close(self) -> None:
        self.stopped.set()
        self.jobs.put(None)

    @property
    def where(self) -> str:
        link = self.link
        return f"{link.port}, board firmware {link.version}" if link is not None else ""

    def _connect(self) -> boardusb.BoardLink:
        if self.link is not None:
            return self.link
        if boardusb.serial is None:
            raise boardusb.UsbError("Over USB, MavLTE needs pyserial: pip install pyserial")
        problems = []
        for port in boardusb.board_ports():
            link = boardusb.BoardLink(port)
            try:
                link.open()
            except boardusb.UsbError as exc:
                problems.append(str(exc))
                continue
            log.info("flight logs over USB: %s, board firmware %s", port, link.version)
            self.link = link
            return link
        raise boardusb.UsbError("; ".join(problems) if problems else NO_BOARD)

    def _run(self) -> None:
        while True:
            job = self.jobs.get()
            if job is None:
                if self.link is not None:
                    self.link.close()
                return
            kind, *args = job
            try:
                link = self._connect()
                if kind == "list":
                    entries, total = link.list(args[0])
                    args[1](entries, total, "")
                else:
                    name, offset, write, progress, done = args
                    link.get(name, offset, write, progress, self.stopped)
                    done(True, "")
            except Exception as exc:  # also a full disk while writing, or a line that came broken: the job ends
                why = str(exc) if isinstance(exc, boardusb.UsbError) else (
                    getattr(exc, "strerror", None) or f"{type(exc).__name__}: {exc}")
                if not isinstance(exc, boardusb.UsbError):
                    log.exception("flight logs over USB")
                fatal = not isinstance(exc, boardusb.UsbError) or exc.fatal
                if fatal and self.link is not None:  # the cable pulled out, say: open it again next time
                    try:
                        self.link.close()
                    except Exception:
                        pass
                    self.link = None
                if kind == "list":
                    args[1](None, 0, why)
                else:
                    args[4](False, why)


class LogsWindow(tk.Toplevel):
    """The aircraft's flight logs (README, "Flight log"): the files on the SD card in its LTE module, newest first,
    listed and copied into Documents\\MavLTE\\Logs over 4G (through the relay) or over the board's USB cable. A copy
    that stopped short, or of a file that has grown since (the one being written), goes on from where it ends."""

    SOURCES = ("4G", "USB cable")
    COLUMNS = (("file", "File", 110), ("started", "Started", 170), ("length", "Length", 90), ("size", "Size", 80),
               ("here", "Here", 90))
    QUIET = 45.0  # s without a byte: a download is given up (its source went away)
    LIST_QUIET = 30.0  # s without the list asked for: given up (4G answers within 12 s, USB within 15 s)

    def __init__(self, app: "App") -> None:
        super().__init__(app.root)
        self.app = app
        s, pad = app.scale, round(8 * app.scale)
        self.configure(bg=BG)
        if app.icon is not None:
            self.iconphoto(False, app.icon)
        self.title(f"{app.settings.name} · {APP} flight logs")
        self.events: "queue.Queue[Callable[[], None]]" = queue.Queue()  # the sources' callbacks, run here
        self.source: Optional[object] = None
        self.entries: List[Entry] = []
        self.total = 0
        self.listing = False
        self.listing_at = 0.0  # when the list was asked for
        self.download: Optional[dict] = None  # the copy being made
        self.waiting: List[Entry] = []  # to copy after it

        top = tk.Frame(self, bg=SURFACE)
        top.pack(fill="x")
        row = tk.Frame(top, bg=SURFACE)
        row.pack(fill="x", padx=pad, pady=(pad, round(4 * s)))
        tk.Label(row, text="From", bg=SURFACE, fg=MUTED, font=app.font).pack(side="left")
        self.from_ = Segments(row, app, self.SOURCES, 0, self.use)
        self.from_.pack(side="left", padx=(pad, 0))
        ttk.Button(row, text="Refresh", command=self.refresh).pack(side="right")
        self.where = tk.Label(top, text="", bg=SURFACE, fg=DIM, font=app.font_small, anchor="w", justify="left",
                              wraplength=round(560 * s))
        self.where.pack(fill="x", padx=pad, pady=(0, pad))

        bottom = tk.Frame(self, bg=SURFACE)
        bottom.pack(side="bottom", fill="x")
        buttons = tk.Frame(bottom, bg=SURFACE)
        buttons.pack(side="right", padx=pad, pady=pad)
        self.more_button = ttk.Button(buttons, text="More", command=self.more)
        self.more_button.pack(side="left")
        ttk.Button(buttons, text="Folder", command=self.open_folder).pack(side="left", padx=(round(6 * s), 0))
        self.stop_button = ttk.Button(buttons, text="Stop", command=self.stop)
        self.stop_button.pack(side="left", padx=(round(6 * s), 0))
        self.get_button = ttk.Button(buttons, text="Download", style="Accent.TButton", command=self.download_selected)
        self.get_button.pack(side="left", padx=(round(6 * s), 0))
        left = tk.Frame(bottom, bg=SURFACE)
        left.pack(side="left", fill="both", expand=True, padx=(pad, 0), pady=pad)
        self.status = tk.Label(left, text="", bg=SURFACE, fg=DIM, font=app.font_small, anchor="w", justify="left",
                               wraplength=round(300 * s))
        self.status.pack(fill="x")
        # as wide as the space beside the buttons: a long text wraps instead of widening the window
        left.bind("<Configure>", lambda e: self.status.configure(wraplength=max(100, e.width - 4)))
        self.bar_h = max(3, round(3 * s))
        self.bar = tk.Canvas(left, width=1, height=self.bar_h, bg=SURFACE, highlightthickness=0)
        self.bar.pack(fill="x", pady=(round(3 * s), 0))
        self.geometry(f"{round(700 * s)}x{round(440 * s)}")
        self.minsize(round(560 * s), round(300 * s))

        ttk.Style(self).configure("Treeview", rowheight=round(22 * s), font=app.font)
        ttk.Style(self).configure("Treeview.Heading", font=app.font_small)
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(body, columns=[c[0] for c in self.COLUMNS], show="headings", height=12,
                                 selectmode="extended")
        for key, heading, width in self.COLUMNS:
            self.tree.heading(key, text=heading, anchor="w")
            self.tree.column(key, width=round(width * s), anchor="w", stretch=key == "started")
        scroll = ttk.Scrollbar(body, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.tree.pack(side="left", fill="both", expand=True)
        self.tree.bind("<Double-1>", lambda _e: self.open_or_download())
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._buttons())
        self.bind("<Escape>", lambda _e: self.destroy())
        dark_title_bar(self)
        self.job = self.after(100, self._tick)
        self.use(0)

    def destroy(self) -> None:
        if getattr(self, "job", None):
            self.after_cancel(self.job)
        if self.source is not None:
            self.source.close()
            self.source = None
        if self.download is not None:
            self.download["file"].close()
            self.download = None
        super().destroy()

    # -- the source

    def use(self, index: int) -> None:
        """4G or the USB cable."""
        self.stop()
        if self.download is not None:  # the old source's: it ends here (its last word, when it comes, is passed by)
            try:
                self.download["file"].close()
            except OSError:
                pass
            self.download = None
        if self.source is not None:
            self.source.close()
        self.source = AirLogs(self.app.runner) if index == 0 else UsbLogs()
        self.from_.select(index)
        self.entries, self.total, self.listing = [], 0, False
        self._fill()
        self.where.configure(text="Through the relay, from the SD card in the aircraft's LTE module. " + self._goes()
                             if index == 0 else "Over the board's USB cable. " + NO_BOARD.split(". ", 1)[1])
        self.refresh()

    @staticmethod
    def _goes() -> str:
        """Where the copies go, inside the user's folder without it: "Documents\\\\MavLTE\\\\Logs"."""
        folder = logs_folder()
        try:
            inside = os.path.relpath(folder, os.path.expanduser("~"))
        except ValueError:  # on another drive
            inside = ".."
        return f"Copies go to {folder if inside.startswith('..') else inside}."

    def refresh(self) -> None:
        if not self.listing and self.download is None:
            self.entries, self.total = [], 0
            self._list(0)

    def more(self) -> None:
        if not self.listing and len(self.entries) < self.total:
            self._list(len(self.entries))

    def _list(self, first: int) -> None:
        source = self.source
        self.listing = True
        self.listing_at = time.monotonic()
        self._show("Asking the aircraft for its files…" if isinstance(source, AirLogs)
                   else "Looking for the board on the USB ports…")
        problem = source.list(first, lambda entries, total, problem: self.events.put(
            lambda: self._listed(source, first, entries, total, problem)))
        if problem:
            self.listing = False
            self._show(problem, RED)
        self._buttons()

    def _listed(self, source, first: int, entries: Optional[List[Entry]], total: int, problem: str) -> None:
        if source is not self.source:  # from before a switch
            return
        self.listing = False
        if entries is None:
            self._show(problem, RED)
        else:
            self.entries = self.entries[:first] + entries
            self.total = total
            self._show(f"{total} file{'s' if total != 1 else ''} on the card" + (
                f", the newest {len(self.entries)} shown" if len(self.entries) < total else "") if total else
                "No log files on the card yet")
            if isinstance(source, UsbLogs):
                self.where.configure(text=f"Over the board's USB cable: {source.where}. {self._goes()}")
        self._fill()
        self._buttons()

    # -- the list

    def _copy(self, entry: Entry) -> Tuple[str, int]:
        """Where the copy of entry goes, and how much of it is there. A copy larger than the file is of another one
        with the same name (from a card formatted since, or another board): this one goes beside it, as "(2)"."""
        base = os.path.join(logs_folder(), log_copy_name(entry[0], entry[2]))
        for n in range(1, 100):
            path = base if n == 1 else f"{base[:-4]} ({n}).csv"
            try:
                have = os.path.getsize(path)
            except OSError:
                return path, 0
            if have <= entry[1]:
                return path, have
        return base, 0

    def _fill(self) -> None:
        selected = {self.tree.set(item, "file") for item in self.tree.selection()}
        self.tree.delete(*self.tree.get_children())
        for entry in self.entries:
            name, size, start, end = entry
            _, have = self._copy(entry)
            here = "" if not have else "saved" if have >= size else f"{have * 100 // max(1, size)} %"
            started = time.strftime("%a %d %b %Y  %H:%M", time.localtime(start)) if start else "time unknown"
            item = self.tree.insert("", "end", values=(name, started, length_text(start, end), size_text(size), here))
            if name in selected:
                self.tree.selection_add(item)

    def _selected(self) -> List[Entry]:
        names = {self.tree.set(item, "file") for item in self.tree.selection()}
        return [entry for entry in self.entries if entry[0] in names]

    def _buttons(self) -> None:
        idle = self.download is None
        self.get_button.state(["!disabled"] if idle and self.entries else ["disabled"])
        self.stop_button.state(["disabled"] if idle else ["!disabled"])
        self.more_button.state(["!disabled"] if idle and not self.listing and len(self.entries) < self.total
                               else ["disabled"])

    # -- copies

    def download_selected(self) -> None:
        """Copies the files selected (the newest if none is), one after another."""
        if self.download is not None:
            return
        self.waiting = self._selected() or self.entries[:1]
        self._next()

    def open_or_download(self) -> None:
        chosen = self._selected()
        if not chosen:
            return
        path, have = self._copy(chosen[0])
        if have and have >= chosen[0][1]:
            open_file(path)
        elif self.download is None:
            self.waiting = chosen[:1]
            self._next()

    def _next(self) -> None:
        if not self.waiting or self.source is None:
            self._buttons()
            return
        entry = self.waiting.pop(0)
        path, have = self._copy(entry)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            f = open(path, "ab")
        except OSError as exc:
            self.waiting.clear()
            self._show(f"Cannot write {path}: {exc.strerror or exc}", RED)
            self._buttons()
            return
        d = {"entry": entry, "path": path, "file": f, "from": have, "have": have, "size": entry[1],
             "t0": time.monotonic(), "news": time.monotonic()}

        def write(offset: int, data: bytes) -> None:  # on the source's thread, in order
            if d.get("error"):
                return
            try:
                f.write(data)
            except ValueError:  # closed: the window went
                pass
            except OSError as exc:  # a full disk, say: _tick ends the download (not from here, the source's thread)
                d["error"] = f"cannot write {path}: {exc.strerror or exc}"

        def progress(have_: int, size: int) -> None:
            d["have"], d["size"], d["news"] = have_, size, time.monotonic()

        self.download = d
        source = self.source
        problem = source.get(entry[0], have, write, progress,
                             lambda ok, problem: self.events.put(lambda: self._done(d, ok, problem)))
        if problem:
            self._done(d, False, problem)
        self._buttons()

    def stop(self) -> None:
        self.waiting.clear()
        if self.download is not None and self.source is not None:
            self.source.stop()

    def _done(self, d: dict, ok: bool, problem: str) -> None:
        if d is not self.download:
            return
        self.download = None
        try:
            d["file"].close()
        except OSError as exc:  # the last bytes could not be written either (a full disk)
            ok, problem = False, d.get("error") or f"cannot write {d['path']}: {exc.strerror or exc}"
        if d.get("error"):
            ok, problem = False, d["error"]
        name = d["entry"][0]
        if ok:
            got = d["have"] - d["from"]
            self._show(f"{name} saved ({size_text(d['have'])}" + (f", {size_text(got)} new" if d["from"] and got
                                                                   else "") + "): Folder shows it", GREEN)
            self.entries = [(e[0], max(e[1], d["size"]), e[2], e[3]) if e[0] == name else e for e in self.entries]
        elif problem == "stopped":
            self._show(f"Stopped: {size_text(d['have'])} of {name} here; Download goes on from there.", AMBER)
        else:
            self.waiting.clear()
            self._show(f"{name}: {problem}", RED)
        self._fill()
        self._next()

    def open_folder(self) -> None:
        chosen = self._selected()
        if chosen:
            path, have = self._copy(chosen[0])
            if have:
                reveal(path)
                return
        folder = logs_folder()
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as exc:
            self._show(f"Cannot make {folder}: {exc.strerror or exc}", RED)
            return
        open_file(folder)

    # -- four times a second

    def _tick(self) -> None:
        try:  # an unexpected error must not stop this window for good
            self._tick_once()
        except Exception:
            log.exception("flight logs window")
        finally:
            self.job = self.after(250, self._tick)

    def _tick_once(self) -> None:
        while True:
            try:
                self.events.get_nowait()()
            except queue.Empty:
                break
        if self.listing and time.monotonic() - self.listing_at > self.LIST_QUIET:  # its answer will not come
            self.listing = False  # (the agent restarted with new settings while it asked, say)
            self._show("No answer to the list: Refresh asks again", RED)
            self._buttons()
        d = self.download
        if d is not None:
            now = time.monotonic()
            if d.get("error"):  # the copy could not be written: the download stops
                if self.source is not None:
                    self.source.stop()
                self._done(d, False, d["error"])
            elif now - d["news"] > self.QUIET:  # its source went away (the agent restarted with new settings, say)
                if self.source is not None:
                    self.source.stop()
                self._done(d, False, "no answer: the download stopped")
            else:
                rate = (d["have"] - d["from"]) / max(0.5, now - d["t0"])
                self._show(f"{d['entry'][0]}: {size_text(d['have'])} of {size_text(d['size'])}, "
                           f"{rate / 1024:.1f} KB/s", TEXT, d["have"] / max(1, d["size"]))
        elif self.bar.find_all():
            self.bar.delete("all")

    def _show(self, text: str, color: str = DIM, fraction: Optional[float] = None) -> None:
        if self.status.cget("text") != text or self.status.cget("fg") != color:
            self.status.configure(text=text, fg=color)
        self.bar.delete("all")
        if fraction is not None:
            w = self.bar.winfo_width()
            self.bar.create_rectangle(0, 0, w, self.bar_h, fill=FIELD, outline="")
            self.bar.create_rectangle(0, 0, round(w * min(1.0, fraction)), self.bar_h, fill=ACCENT, outline="")


class SettingsDialog(tk.Toplevel):
    def __init__(self, app: "App") -> None:
        super().__init__(app.root)
        self.app = app
        self.title(f"{APP} settings")
        self.configure(bg=BG)
        self.transient(app.root)
        self.resizable(False, False)
        if app.icon is not None:
            self.iconphoto(False, app.icon)
        form = tk.Frame(self, bg=BG, padx=16, pady=14)
        form.pack(fill="both", expand=True)
        self.name = tk.StringVar(value=app.settings.name)
        self.server = tk.StringVar(value=app.settings.server)
        self.key = tk.StringVar(value=app.settings.key)
        self.show_key = tk.BooleanVar(value=False)
        fields = (("Vehicle name", self.name, ""), ("Relay server", self.server, "host:port, e.g. 203.0.113.10:14650"),
                  ("GCS key", self.key, "64 hex characters, printed by the relay's installer"))
        for row, (label, var, hint) in enumerate(fields):
            tk.Label(form, text=label, bg=BG, fg=TEXT, font=app.font).grid(row=2 * row, column=0, sticky="w",
                                                                           pady=(6, 0))
            entry = dark_entry(form, app, var, 48, show="•" if var is self.key else "")
            entry.grid(row=2 * row, column=1, sticky="we", pady=(6, 0), padx=(10, 0))
            if var is self.key:
                self.key_entry = entry
            if hint:
                tk.Label(form, text=hint, bg=BG, fg=DIM, font=app.font_small).grid(row=2 * row + 1, column=1,
                                                                                  sticky="w", padx=(10, 0))
        tk.Checkbutton(form, text="Show key", variable=self.show_key, bg=BG, fg=TEXT, selectcolor=FIELD,
                       activebackground=BG, activeforeground=TEXT, font=app.font, bd=0, highlightthickness=0,
                       command=lambda: self.key_entry.configure(show="" if self.show_key.get() else "•")
                       ).grid(row=6, column=1, sticky="w", padx=(6, 0), pady=(4, 0))
        tk.Label(form, text="Trying it with the SITL demo on this PC? Relay server 127.0.0.1:14650;\n"
                            "sitl_demo.py --no-agent puts its key into mavrelay.ini for you.",
                 bg=BG, fg=DIM, font=app.font_small, justify="left").grid(row=7, column=0, columnspan=2,
                                                                          sticky="w", pady=(10, 0))
        buttons = tk.Frame(form, bg=BG)
        buttons.grid(row=8, column=0, columnspan=2, sticky="e", pady=(14, 0))
        ttk.Button(buttons, text="Cancel", command=self.destroy).pack(side="right")
        ttk.Button(buttons, text="Save", style="Accent.TButton", command=self.save).pack(side="right", padx=(0, 8))
        self.bind("<Return>", lambda _e: self.save())
        self.bind("<Escape>", lambda _e: self.destroy())
        dark_title_bar(self)
        self.grab_set()
        self.focus_set()

    def save(self) -> None:
        server = self.server.get().strip()
        key = self.key.get().strip()
        try:
            host, port = mr.parse_hostport(server)
            host.encode("idna")  # a UnicodeError (a ValueError) for a typo like 10.0.0..1: no host name at all
            if not host or not 1 <= port <= 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror(APP, "The relay server must look like host:port, e.g. 203.0.113.10:14650.",
                                 parent=self)
            return
        try:
            mr.parse_key(key)
        except ValueError as exc:
            messagebox.showerror(APP, f"GCS key: {exc}.", parent=self)
            return
        name = self.name.get().strip() or "My UAV"
        problem = name_problem(name)
        if problem:
            messagebox.showerror(APP, problem, parent=self)
            return
        self.destroy()
        self.app.apply_settings(name, server, key)


# ---------------------------------------------------------------------------------------------
# The window


class LogHandler(logging.Handler):
    def __init__(self, sink: "queue.Queue[str]") -> None:
        super().__init__()
        self.sink = sink
        self.setFormatter(logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        self.sink.put(self.format(record))


def setup_style(root: tk.Tk) -> None:
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TButton", background=FIELD, foreground=TEXT, bordercolor=BORDER, lightcolor=FIELD,
                    darkcolor=FIELD, focuscolor=FIELD, padding=(14, 4))
    style.map("TButton", background=[("pressed", BORDER), ("active", "#3a3b3e")], foreground=[("disabled", DIM)])
    style.configure("Accent.TButton", background=ACCENT, foreground=ON_ACCENT, bordercolor=ACCENT,
                    lightcolor=ACCENT, darkcolor=ACCENT, focuscolor=ACCENT)
    style.map("Accent.TButton", background=[("disabled", FIELD), ("pressed", "#4cb84c"), ("active", "#72d972")],
              foreground=[("disabled", DIM)], bordercolor=[("disabled", BORDER)],
              lightcolor=[("disabled", FIELD)], darkcolor=[("disabled", FIELD)])
    style.configure("Vertical.TScrollbar", background=FIELD, troughcolor=LOG_BG, bordercolor=LOG_BG,
                    arrowcolor=MUTED, lightcolor=FIELD, darkcolor=FIELD, gripcount=0)
    style.map("Vertical.TScrollbar", background=[("active", "#3a3b3e")])
    style.configure("Treeview", background=LOG_BG, fieldbackground=LOG_BG, foreground=TEXT, bordercolor=BORDER,
                    lightcolor=LOG_BG, darkcolor=LOG_BG)
    style.map("Treeview", background=[("selected", "#2c4a33")], foreground=[("selected", TEXT)])
    style.configure("Treeview.Heading", background=FIELD, foreground=MUTED, bordercolor=BORDER, lightcolor=FIELD,
                    darkcolor=FIELD, relief="flat")
    style.map("Treeview.Heading", background=[("active", "#3a3b3e")])


class App:
    POLL_MS = 250
    LOG_LINES = 500  # kept for the log window

    def __init__(self, root: tk.Tk, config_path: str = CONFIG) -> None:
        self.root = root
        self.config_path = config_path
        self.settings = Settings.load(config_path)
        self.runner = AgentRunner()
        self.log_lines: "queue.Queue[str]" = queue.Queue()
        self.log_history: "collections.deque[str]" = collections.deque(maxlen=self.LOG_LINES)
        self.log_window: Optional[LogWindow] = None
        self.log_handler = LogHandler(self.log_lines)
        logging.getLogger("mavrelay").addHandler(self.log_handler)
        logging.getLogger("mavrelay").setLevel(logging.INFO)
        self.rate_mark = (time.monotonic(), 0, 0)
        self.rates = (0.0, 0.0)
        self.photos_in: "queue.Queue[Tuple[str, mr.PhotoInfo]]" = queue.Queue()  # saved by the agent's thread
        self.clicked_at = -1e9  # Snapshot: the photo that comes next opens in the viewer
        self.viewer: Optional[PhotoViewer] = None
        self.track = aircraft_card.Track()  # where the aircraft has been since MavLTE started, for the map
        self.tiles = maptiles.TileLoader()
        self.map_window: Optional[MapWindow] = None
        self.logs_window: Optional[LogsWindow] = None
        self.fix_live = False  # the newest fix is live (the aircraft reports), not the last known
        self.shown_where: Tuple[str, str] = ("", TEXT)  # Position's text and colour, for the map too
        self.lan: Optional[str] = None  # this computer's address on its network, for the cards
        self.lan_at = -1e9
        self.gc_at = time.monotonic()  # the garbage collector runs from poll(), on this thread only (see main)
        self.set_aside_at: Optional[float] = None  # when to set aside again what lives (see set_aside)

        self.scale = max(1.0, root.winfo_fpixels("1i") / 96.0)
        family = "Segoe UI" if "Segoe UI" in tkfont.families(root) else "TkDefaultFont"
        self.font = (family, 10)
        self.font_small = (family, 9)
        self.font_bold = (family, 10, "bold")
        self.font_title = (family, 15, "bold")
        self.icon: Optional[tk.PhotoImage] = None
        if os.path.exists(ICON):
            self.icon = tk.PhotoImage(file=ICON)
            root.iconphoto(True, self.icon)
        setup_style(root)
        self._build()
        dark_title_bar(root)
        place_on_screen(root, self.scale)
        self._connect()
        self.poll()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.report_callback_exception = self._report
        if self.settings.problem:
            log.warning("%s", self.settings.problem)
            root.after(200, lambda: messagebox.showwarning(
                APP, f"{self.settings.problem}\n\nMavLTE starts without those settings. Set the relay and key "
                     "again (☰ → Settings), or fix that file, or delete it, and start MavLTE again."))
        if not (self.settings.server and self.settings.key):
            root.after(300, self.open_settings)

    # -- layout

    def _build(self) -> None:
        root, s = self.root, self.scale
        root.title(f"{APP} V{mr.__version__}")
        root.configure(bg=BG)
        root.minsize(round(360 * s), round(420 * s))

        header = tk.Frame(root, bg=SURFACE)
        header.pack(fill="x")
        bar = tk.Frame(header, bg=SURFACE)
        bar.pack(fill="x", padx=round(10 * s), pady=(round(6 * s), 0))  # (the window's spacing fits 1080p at 125%)
        menu_button = tk.Label(bar, text="☰", bg=SURFACE, fg=MUTED, font=(self.font[0], 14), cursor="hand2")
        menu_button.pack(side="left")
        menu_button.bind("<Enter>", lambda _e: menu_button.configure(fg=TEXT))
        menu_button.bind("<Leave>", lambda _e: menu_button.configure(fg=MUTED))
        tk.Label(bar, text=APP, bg=SURFACE, fg=TEXT, font=self.font_bold).pack(side="left", padx=round(8 * s))
        self.menu = tk.Menu(root, tearoff=0, bg=SURFACE, fg=TEXT, activebackground=FIELD, activeforeground=TEXT,
                            bd=0)
        self.menu.add_command(label="Settings…", command=self.open_settings)
        self.menu.add_command(label="Show log", command=self.toggle_log)  # entry 1, labelled by _menu_labels
        self.menu.add_command(label="Photo folder", command=self.open_photo_folder)
        self.menu.add_command(label="Flight logs…", command=self.open_logs)
        self.menu.add_separator()
        self.menu.add_command(label="Exit", command=self.close)
        self.menu.configure(postcommand=self._menu_labels)
        menu_button.bind("<Button-1>", lambda e: self.menu.tk_popup(e.x_root, e.y_root))

        who = tk.Frame(header, bg=SURFACE)
        who.pack(fill="x", padx=round(16 * s), pady=(round(6 * s), round(3 * s)))
        if self.icon is not None:
            self.badge = self.icon.subsample(4 if s < 1.5 else 2)  # whole-pixel steps keep the pixel art crisp
            tk.Label(who, image=self.badge, bg=SURFACE).pack(side="left")
        names = tk.Frame(who, bg=SURFACE)
        names.pack(side="left", padx=round(14 * s))
        self.name_label = tk.Label(names, text=self.settings.name, bg=SURFACE, fg=TEXT, font=self.font_title)
        self.name_label.pack(anchor="w")
        self.server_label = tk.Label(names, text="", bg=SURFACE, fg=MUTED, font=self.font)
        self.server_label.pack(anchor="w")
        line = tk.Frame(header, bg=SURFACE)
        line.pack(fill="x", padx=round(16 * s), pady=(0, round(8 * s)))
        self.relay_led = Led(line, s, SURFACE)
        self.relay_led.pack(side="left")
        self.relay_text = tk.Label(line, text="", bg=SURFACE, fg=TEXT, font=self.font, anchor="w")
        self.relay_text.pack(side="left", padx=round(6 * s))
        tk.Frame(root, bg=ACCENT, height=max(2, round(2 * s))).pack(fill="x")

        self.body = tk.Frame(root, bg=BG)
        self.body.pack(fill="both", expand=True, padx=round(12 * s), pady=round(6 * s))
        tk.Label(self.body, text="Data transfer", bg=BG, fg=MUTED, font=self.font_bold).pack(anchor="w")
        self.tcp_card = ChannelCard(self, "tcp", self.settings.tcp)
        self.tcp_card.pack(fill="x", pady=(round(4 * s), round(5 * s)))
        self.udp_card = ChannelCard(self, "udp", self.settings.udp)
        self.udp_card.pack(fill="x")

        tk.Label(self.body, text="Aircraft", bg=BG, fg=MUTED, font=self.font_bold).pack(anchor="w",
                                                                                       pady=(round(6 * s), 0))
        craft = tk.Frame(self.body, bg=SURFACE, highlightbackground=BORDER, highlightthickness=1)
        craft.pack(fill="x", pady=(round(3 * s), 0))
        pad = round(8 * s)
        top = tk.Frame(craft, bg=SURFACE)
        top.pack(fill="x", padx=pad, pady=(pad, round(2 * s)))
        self.craft_led = Led(top, s, SURFACE)
        self.craft_led.pack(side="left")
        self.craft_state = tk.Label(top, text="", bg=SURFACE, fg=TEXT, font=self.font_bold)
        self.craft_state.pack(side="left", padx=round(8 * s))
        self.craft_bars = Bars(top, s, SURFACE)
        self.craft_bars.pack(side="right")
        # the aircraft's mobile network (1.8.8): automatic (LTE, and 2G while LTE fails), 2G or LTE only
        self.network = Segments(top, self, aircraft_card.NETWORK_NAMES, -1, self.choose_network, compact=True)
        self.network.pack(side="right", padx=(0, round(10 * s)))
        self.network.set_enabled(False)
        grid = tk.Frame(craft, bg=SURFACE)
        grid.pack(fill="x", padx=pad, pady=(0, pad))
        self.craft_values = {}
        for row, label in enumerate(("Link", "Packet loss", "Traffic", "Position", "Module", "Voice")):
            tk.Label(grid, text=label, bg=SURFACE, fg=MUTED, font=self.font).grid(row=row, column=0, sticky="w")
            if label == "Voice":  # the locator voice: the board's speaker, for the last metres to the aircraft
                self.voice = VoiceRow(self, grid)
                self.voice.grid(row=row, column=1, sticky="w", padx=(12, 0))
                continue
            cell = tk.Frame(grid, bg=SURFACE)
            cell.grid(row=row, column=1, sticky="w", padx=(12, 0))
            value = tk.Label(cell, text="-", bg=SURFACE, fg=TEXT, font=self.font, anchor="w")
            value.pack(side="left")
            self.craft_values[label] = value
            if label == "Position":  # the LTE module's own GNSS: where to look for the aircraft
                self.map_link = self._link(cell, "Map", self.open_map)
                self.copy_link = self._link(cell, "Copy", self.copy_position)
            elif label == "Module":  # its chip's temperature, in a colour of its own
                self.chip_temp = tk.Label(cell, text="", bg=SURFACE, fg=TEXT, font=self.font, anchor="w", padx=0)
                self.chip_temp.pack(side="left")
        self.shown_fix: Optional[mr.Position] = None  # the position Map and Copy use
        self.voice_state = aircraft_card.Voice()
        self.camera = CameraRow(self, craft)
        self.camera.pack(fill="x")
        newest = photo_files(self.settings.photo_folder())
        self.camera.thumbnail.show(newest[-1] if newest else None)
        self._show_last(newest[-1] if newest else None)
        self._update_server_label()

    def _update_server_label(self) -> None:
        self.server_label.configure(text=f"relay {self.settings.server}" if self.settings.server
                                    else "no relay server yet: ☰ → Settings")
        self.name_label.configure(text=self.settings.name)
        for card in (self.tcp_card, self.udp_card):
            card.title.configure(text=self.settings.name)

    # -- actions

    def _connect(self) -> bool:
        """Connects to the relay as soon as there are settings. With both switches off the agent
        only watches, so the Available LEDs work while the aircraft holds its telemetry back."""
        if self.runner.running:
            return True
        try:
            server = mr.parse_hostport(self.settings.server)
            key = mr.parse_key(self.settings.key)
        except ValueError:
            return False
        self.runner.start(server, key, self.settings.photo_folder(),
                          on_photo=lambda path, info: self.photos_in.put((path, info)))
        return True

    def snapshot(self) -> None:
        if not self.camera.enabled:
            return
        size, exposure = self.camera.size.selected, self.camera.exposure.value
        if self.runner.running and self.runner.snapshot(size, exposure):
            self.clicked_at = time.monotonic()
            log.info("asking the aircraft for a photo (%s, %d×%d, %s)", SIZE_NAMES[size], *mr.SNAP_SIZES[size],
                     aircraft_card.exposure_text(exposure))
        else:
            self.camera.show("Not connected to the relay", RED)

    def toggle_voice(self, on: bool) -> None:
        if self.runner.running and self.runner.set_voice(on):
            self.voice.switch.set(on)  # the relay confirms within a second or two (_show_voice)
        else:
            self.voice.show("Not connected to the relay", RED)

    def choose_network(self, index: int) -> None:
        self.network.set_pending(True)  # asked for: the aircraft takes it within seconds
        if not (self.runner.running and self.runner.set_network(index)):
            self._show_network(self.runner.agent, None)  # no session: as it was (the next poll greys it out)

    def choose_exposure(self, steps: int) -> None:
        self.settings.photo_exposure = steps
        self._remember("photo_exposure")
        self.camera.show(aircraft_card.exposure_hint(steps))

    def choose_size(self, index: int) -> None:
        self.settings.photo_size = SIZE_NAMES[index]
        self._remember("photo_size")
        self.camera.show(aircraft_card.SIZE_HINTS[index])

    def show_photo(self, path: Optional[str] = None) -> None:
        """Opens the photo (the last one: the thumbnail's) in the viewer."""
        path = path or self.camera.thumbnail.path
        if not path or not os.path.exists(path):
            newest = photo_files(self.settings.photo_folder())
            if not newest:
                return
            path = newest[-1]
        if Image is None:  # no Pillow: the system's viewer
            open_file(path)
            return
        if self.viewer is None or not self.viewer.winfo_exists():
            self.viewer = PhotoViewer(self)
        self.viewer.show(path)
        self.viewer.deiconify()
        self.viewer.lift()

    def open_photo_folder(self) -> None:
        folder = self.settings.photo_folder()
        newest = photo_files(folder)
        if newest:
            reveal(newest[-1])
            return
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as exc:
            messagebox.showerror(APP, f"Cannot make the photo folder {folder}: {exc.strerror or exc}")
            return
        open_file(folder)

    def toggle(self, card: ChannelCard, on: bool) -> None:
        if not on:
            if self.runner.running:
                self.runner.set_output(card.kind, None)
            card.set_on(False)
            card.show("Off")
            if not (self.tcp_card.switch.on or self.udp_card.switch.on):
                log.info("both ports off: only watching, the aircraft holds its telemetry back")
            return
        try:
            address = card.address()
        except ValueError as exc:
            card.show(str(exc), RED)
            return
        if not self._connect():
            card.show("Set the relay server and GCS key first (☰ → Settings).", RED)
            self.open_settings()
            return
        error = self.runner.set_output(card.kind, address)
        if error:
            card.show(error, RED)
            return
        card.set_on(True)
        self._remember()

    def _remember(self, *fields: str) -> None:
        """Saves the port addresses and the given settings: only what this window changes, so that
        what other programs write to mavrelay.ini meanwhile (sitl_demo.py's key, say) stays."""
        self.settings.tcp, self.settings.udp = self.tcp_card.text(), self.udp_card.text()
        try:
            self.settings.save(self.config_path, ("tcp", "udp") + fields)
        except (OSError, ValueError) as exc:  # ValueError: a file in another encoding (the app still closes)
            log.warning("cannot save settings: %s", exc)

    def apply_settings(self, name: str, server: str, key: str) -> None:
        changed = (server, key) != (self.settings.server, self.settings.key)
        self.settings.name, self.settings.server, self.settings.key = name, server, key
        self._update_server_label()
        self._remember("name", "server", "key")
        if changed:  # reconnect with the new server or key
            self.track = aircraft_card.Track()  # another relay may have another aircraft
            cards = [card for card in (self.tcp_card, self.udp_card) if card.switch.on]
            if self.runner.running:
                self.runner.stop()
                if gc.get_freeze_count():  # the old connection was set aside with the rest (see set_aside): let the
                    gc.unfreeze()  # collections free it, and set aside anew once it is surely gone
                    self.set_aside_at = time.monotonic() + 10.0
            for card in cards:
                card.set_on(False)
            self._connect()
            for card in cards:
                self.toggle(card, True)

    def open_settings(self) -> None:
        SettingsDialog(self)

    def open_logs(self) -> None:
        if self.logs_window is None or not self.logs_window.winfo_exists():
            self.logs_window = LogsWindow(self)
        self.logs_window.deiconify()
        self.logs_window.lift()
        self.logs_window.focus_set()

    def log_shown(self) -> bool:
        return self.log_window is not None and self.log_window.winfo_exists()

    def toggle_log(self) -> None:
        """Opens the log window beside the main one, or closes it."""
        if self.log_shown():
            self.log_window.destroy()
        else:
            self.log_window = win = LogWindow(self)
            win.update_idletasks()
            root = self.root
            x = root.winfo_rootx() + root.winfo_width() + round(8 * self.scale)
            if x + win.winfo_reqwidth() > root.winfo_screenwidth():  # no room on the right: on the left
                x = max(0, root.winfo_rootx() - win.winfo_reqwidth() - round(16 * self.scale))
            win.geometry(f"+{x}+{max(0, root.winfo_rooty() - round(30 * self.scale))}")
        self._menu_labels()

    def _menu_labels(self) -> None:
        """Show log or Hide log, as the log window is: it may have been closed with its own ✕."""
        self.menu.entryconfigure(1, label="Hide log" if self.log_shown() else "Show log")

    def close(self) -> None:
        self.root.after_cancel(self.poll_job)
        self._remember()
        logging.getLogger("mavrelay").removeHandler(self.log_handler)
        try:
            self.runner.close()
        finally:
            self.root.destroy()

    def _report(self, exc_type, exc, tb) -> None:
        log.error("unexpected error: %r", exc)
        messagebox.showerror(APP, f"Unexpected error: {exc!r}")

    # -- live state, four times a second

    def poll(self) -> None:
        try:  # an unexpected error must not freeze every LED and line in the window
            self._poll()
        except Exception:
            log.exception("unexpected error while updating the window")
        finally:
            self.poll_job = self.root.after(self.POLL_MS, self.poll)

    def _poll(self) -> None:
        if time.monotonic() - self.gc_at >= 5.0:
            self.gc_at = time.monotonic()
            gc.collect()
            if self.set_aside_at is not None and self.gc_at >= self.set_aside_at:
                self.set_aside_at = None
                gc.freeze()  # (just collected: what is left lives on)
        self._drain_log()
        agent = self.runner.agent
        now = time.monotonic()
        status = None
        if agent is not None:
            self.track.add(agent.last_fix)
        if agent is None:
            self.relay_led.set(LED_OFF)
            self._set(self.relay_text, "Not connected: set the relay server and key (☰ → Settings)")
        else:
            led, text = aircraft_card.relay(agent.client)
            self.relay_led.set(PALETTE[led])
            self._set(self.relay_text, text)
            if agent.client.session:
                status = agent.vehicle_status()
        online = aircraft_card.is_online(status)
        bars = aircraft_card.bars(status)

        for card in (self.tcp_card, self.udp_card):
            card.available.set(GREEN if online else LED_OFF)
            card.led.set(BLUE if card.switch.on and online else LED_OFF)
            card.bars.set(bars)
        if agent is not None:
            if self.tcp_card.switch.on and agent.tcp is not None:
                n = len(agent.tcp.clients)
                host, _, port = self.tcp_card.text().rpartition(":")
                where = f"TCP, {self._reached_at(host, now)}, port {port}"
                self.tcp_card.show(*self._port_status(online, f"{n} GCS connected" if n else "", where))
            udp = agent.udp
            if self.udp_card.switch.on and udp is not None:
                if udp.target is None:  # it listens: GCS software on any computer connects to it
                    n = udp.gcs_count()
                    gcs = f"{n} GCS connected" if n else ""
                    where = f"UDPCl, {self._reached_at(udp.local[0], now)}, port {udp.port}"
                else:
                    host, port = udp.target
                    gcs = "Mission Planner connected" if now - udp.last_rx < 3.0 else ""
                    where = f"UDP, port {port}" if mr.is_loopback(host) else f"UDP {host}, port {port}"
                self.udp_card.show(*self._port_status(online, gcs, where))
        self._show_aircraft(agent, status, now)
        self._show_network(agent, status)
        self._show_voice(agent, status, now)
        self._take_photos(now)
        self._show_camera(agent, online, now)

    def _take_photos(self, now: float) -> None:
        """Photos the agent saved: the newest goes on the thumbnail; after a Snapshot click, into the
        viewer too (photos that come in by themselves, missed while MavLTE was closed, do not pop up)."""
        arrived = []
        while True:
            try:
                arrived.append(self.photos_in.get_nowait())
            except queue.Empty:
                break
        if not arrived:
            return
        path = max(arrived, key=lambda item: item[1].photo_id)[0]
        newest = photo_files(os.path.dirname(path))  # one missed earlier may come in after a newer one
        path = newest[-1] if newest else path
        self.camera.thumbnail.show(path)
        self._show_last(path)
        viewer_open = self.viewer is not None and self.viewer.winfo_exists()
        if now - self.clicked_at < 120.0:
            self.clicked_at = -1e9
            self.show_photo(path)
        elif viewer_open and self.viewer.is_newest():
            self.viewer.show(path)
        if len(arrived) > 1:
            log.info("%d photos came in", len(arrived))

    def _show_last(self, path: Optional[str]) -> None:
        text = ""
        if path:
            caption = photo_caption(photo_meta(path), short=True)
            text = f"Last photo {caption}" if caption else "Last photo"
        if self.camera.last.cget("text") != text:
            self.camera.last.configure(text=text)

    def _show_camera(self, agent, online: bool, now: float) -> None:
        cam = aircraft_card.camera(agent, online, self.camera.size.selected, now)
        self.camera.show(cam.text, PALETTE[cam.color], cam.fraction)
        self.camera.set_enabled(cam.ready)
        self.camera.size.set_enabled(not cam.busy)
        self.camera.exposure.set_enabled(not cam.busy)

    def _show_network(self, agent, status: Optional[mr.LinkStatus]) -> None:
        shown = aircraft_card.network(agent, status)
        if shown is not None:
            index = -1 if shown.choice is None else shown.choice
            if index != self.network.selected:
                self.network.select(index)
            self.network.set_enabled(shown.usable)
            self.network.set_pending(shown.pending)

    def _show_voice(self, agent, status: Optional[mr.LinkStatus], now: float) -> None:
        shown = self.voice_state.show(agent, status, now)
        if shown is not None:
            on, line = shown
            self.voice.switch.set(on)
            self.voice.show(line.text, PALETTE[line.color])

    def _reached_at(self, host: str, now: float) -> str:
        """The address GCS software connects to, for a port on `host`. On all of them (0.0.0.0): this
        computer's on its network, which other computers use (GCS software on this one: 127.0.0.1 too)."""
        if host != "0.0.0.0":
            return host
        if now - self.lan_at >= 5.0:  # Wi-Fi may change it
            self.lan_at, self.lan = now, mr.lan_address()
        return self.lan or "this computer's address"

    @staticmethod
    def _port_status(online: bool, gcs: str, where: str) -> Tuple[str, str]:
        """A switched-on port's status line and its colour. gcs: what is connected to it, or ''.
        The switch stays on while the aircraft is away, so that the telemetry comes back by itself. One line,
        so that the window keeps its height (the longest, UDPCl on a network address: 505 of 552 pixels at 125%)."""
        if not online:
            return f"Waiting for the aircraft · {gcs or 'Mission Planner: ' + where}", DIM
        return (gcs, GREEN) if gcs else (f"Waiting for Mission Planner: {where}", DIM)

    def _show_aircraft(self, agent, status: Optional[mr.LinkStatus], now: float) -> None:
        v = self.craft_values
        if agent is not None and now - self.rate_mark[0] >= 1.0:
            t, down, up = self.rate_mark
            self.rates = ((agent.to_gcs_bytes - down) / (now - t), (agent.from_gcs_bytes - up) / (now - t))
            self.rate_mark = (now, agent.to_gcs_bytes, agent.from_gcs_bytes)
        led, text = aircraft_card.headline(agent, status)
        self.craft_led.set(PALETTE[led])
        self._set(self.craft_state, text)
        if agent is None or not agent.client.session:
            for label in v.values():
                self._set(label, "-")
            self._paint(v["Link"], "-", TEXT)  # (amber or red no longer)
            self.craft_bars.set(None)
            self.rate_mark = (now, agent.to_gcs_bytes, agent.from_gcs_bytes) if agent else (now, 0, 0)
            self.rates = (0.0, 0.0)
            self._show_position(agent, False)
            return
        self.craft_bars.set(aircraft_card.bars(status))
        self._paint(v["Link"], aircraft_card.link(status), PALETTE[aircraft_card.link_color(status)])
        self._set(v["Packet loss"], aircraft_card.loss(status))
        down, up = self.rates
        self._set(v["Traffic"], f"↓ {down / 1000:.1f} KB/s telemetry, ↑ {up / 1000:.1f} KB/s commands")
        self._show_position(agent, aircraft_card.is_online(status))

    def _show_position(self, agent, online: bool) -> None:
        """Map and Copy take the position shown."""
        now_unix = agent.relay_time() if agent is not None else time.time()  # the clock the reports carry
        where = aircraft_card.position(agent, online, now_unix)
        self.shown_fix = where.fix
        live = aircraft_card.live(agent, online, now_unix)
        self.fix_live = live is not None and live.has_fix
        self.shown_where = (where.text, PALETTE[where.color])
        self._paint(self.craft_values["Position"], where.text, PALETTE[where.color])
        for link in (self.map_link, self.copy_link):
            self._paint(link, link.cget("text"), BLUE if where.fix else LED_OFF)
        line, temp = aircraft_card.module(agent, online, now_unix)
        self._paint(self.craft_values["Module"], line.text, PALETTE[line.color])
        self._paint(self.chip_temp, temp.text, PALETTE[temp.color])

    def _link(self, parent: tk.Misc, text: str, command: Callable[[], None]) -> tk.Label:
        link = tk.Label(parent, text=text, bg=SURFACE, fg=LED_OFF, font=self.font_small, cursor="hand2")
        link.pack(side="left", padx=(round(8 * self.scale), 0))
        link.bind("<Button-1>", lambda _e: command())
        return link

    def open_map(self) -> None:
        """The moving map; without Pillow, which reads its tiles, Google Maps in the browser."""
        if self.shown_fix is None:
            return
        if Image is None:
            self.open_google_maps()
            return
        if self.map_window is None or not self.map_window.winfo_exists():
            self.map_window = MapWindow(self)
        self.map_window.deiconify()
        self.map_window.lift()
        self.map_window.focus_set()

    def open_google_maps(self) -> None:
        pos = self.shown_fix
        if pos is not None:
            webbrowser.open(aircraft_card.map_link(pos))

    def copy_position(self) -> None:
        pos = self.shown_fix
        if pos is None:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(aircraft_card.coordinates(pos))
        self.copy_link.configure(text="Copied")
        self.root.after(1500, lambda: self.copy_link.configure(text="Copy"))

    @staticmethod
    def _paint(label: tk.Label, text: str, color: str) -> None:
        if label.cget("text") != text or label.cget("fg") != color:
            label.configure(text=text, fg=color)

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
        self.log_history.extend(lines)
        if lines and self.log_shown():
            self.log_window.add(lines)


def set_aside() -> None:
    """Sets aside (freezes) all that lives now: the collections every 5 s (App._poll) skip it from then on. Nearly all
    of MavLTE's objects are made at start-up and live as long as it does; checking them all took 7-9 ms each time,
    while the agent's thread, forwarding telemetry and commands, waited. What is made later is still checked (some
    0.2 ms). What is set aside is still freed the moment nothing uses it; only reference cycles in it would stay, so
    App.apply_settings lets them go when it replaces the agent (its connection has some)."""
    gc.collect()
    gc.freeze()


def main() -> None:
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)  # sharp text on high-DPI screens
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("MavLTE.Agent")  # own taskbar icon
        except (AttributeError, OSError):
            pass
    # Tk's images and variables in reference cycles must be freed on Tk's own thread. Left to itself, the garbage
    # collector may run on the agent's thread and wait there on Tk, while Tk waits on the agent (AgentRunner._call):
    # both stuck for 30 s, telemetry included. So it runs only from App.poll(), every few seconds.
    gc.disable()
    root = tk.Tk()
    App(root)
    set_aside()  # before Tk runs anything queued, such as a settings dialog that will close again
    root.mainloop()


if __name__ == "__main__":
    main()

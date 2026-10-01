#!/usr/bin/env python3
"""MavLTE: the GCS agent as a window.

Gives Mission Planner (or QGroundControl) a TCP and a UDP port, each with its own switch. Two LEDs
on each: Available (green, left) while the aircraft's LTE module is online at the relay, whatever
the switches; Connected (blue, next to the switch) while it is online and that port is on. Both
switches start off. With both off the app only watches: the aircraft holds its telemetry back.
Snapshot asks the aircraft for a photo from its camera, whatever the switches; photos are kept in
Pictures\\MavLTE, each with a .json beside it saying when and where it was taken. Position shows
where the LTE module's own GNSS puts the aircraft, live or last known, with a map link. The Voice
switch (the locator voice) makes the aircraft speak through the speaker on its board until it is
switched off, to find it in the last metres; the relay keeps the switch, also while the aircraft
is offline.

    python mavlte.py            (or double-click MavLTE.pyw, or run MavLTE.exe: build_release.py)

Settings live in the [gcs] section of mavrelay.ini next to this file, the same file that
start-gcs.bat and `mavrelay.py gcs --config mavrelay.ini` use; MavLTE.exe keeps its own (see
settings_file). Tkinter, and Pillow to show the photos (without it they open in the system's viewer).
"""

from __future__ import annotations

import asyncio
import ctypes
import json
import logging
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
from typing import Callable, List, Optional, Tuple

import mavrelay as mr

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

log = logging.getLogger("mavrelay.app")


# ---------------------------------------------------------------------------------------------
# Settings


SIZE_NAMES = ("small", "medium", "large")  # mr.SNAP_SIZES


@dataclass
class Settings:
    name: str = "My UAV"
    server: str = ""
    key: str = ""
    tcp: str = "127.0.0.1:5760"
    udp: str = "127.0.0.1:14550"
    photo_dir: str = ""  # empty: Pictures\MavLTE
    photo_size: str = "medium"

    @classmethod
    def load(cls, path: str) -> "Settings":
        s = cls()
        try:
            conf = mr.load_config(path, "gcs") if os.path.exists(path) else {}
        except SystemExit:
            conf = {}
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


def pictures_folder() -> str:
    """The user's Pictures folder, where Windows keeps it (in OneDrive, say)."""
    if sys.platform == "win32":
        class GUID(ctypes.Structure):
            _fields_ = [("a", ctypes.c_uint32), ("b", ctypes.c_uint16), ("c", ctypes.c_uint16),
                        ("d", ctypes.c_ubyte * 8)]

        u = uuid.UUID("33E28130-4E1E-4676-835A-98395C3BC3BB")  # FOLDERID_Pictures
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
    return os.path.join(os.path.expanduser("~"), "Pictures")


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


def photo_caption(meta: dict, short: bool = False) -> str:
    """When and where the photo was taken, from the .json beside it."""
    parts = []
    when = meta.get("time") or meta.get("photo_id")
    if isinstance(when, int) and when > 0:
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

    def snapshot(self, size: int) -> bool:
        """Asks the aircraft for a photo. False while there is no session with the relay."""
        async def go() -> bool:
            agent = self.agent
            return agent is not None and agent.photos is not None and agent.photos.request(size)

        return self._call(go())

    def set_voice(self, on: bool) -> bool:
        """Switches the aircraft's locator voice. False while there is no session with the relay."""
        async def go() -> bool:
            return self.agent is not None and self.agent.set_voice(on)

        return self._call(go())

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


def place_on_screen(window: tk.Tk, scale: float) -> None:
    """Opens a window at the top of the screen when where Windows would put it hides its bottom behind the
    taskbar (a 1080p laptop at 125% has 1020 pixels above the taskbar)."""
    if sys.platform != "win32":
        return
    try:
        from ctypes import wintypes
        area = wintypes.RECT()
        if not ctypes.windll.user32.SystemParametersInfoW(0x30, 0, ctypes.byref(area), 0):  # SPI_GETWORKAREA
            return
    except (AttributeError, OSError):
        return
    window.update_idletasks()
    if window.winfo_reqheight() + round(100 * scale) > area.bottom - area.top:  # its title bar, and Windows' offset
        window.geometry(f"+{area.left + round(24 * scale)}+{area.top}")


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
    """One of a few choices, side by side."""

    def __init__(self, master, app: "App", labels: Tuple[str, ...], selected: int,
                 command: Callable[[int], None]) -> None:
        super().__init__(master, bg=BORDER, padx=1, pady=1)
        self.command = command
        self.labels = []
        for i, text in enumerate(labels):
            label = tk.Label(self, text=text, font=app.font_small, padx=round(8 * app.scale), pady=round(3 * app.scale),
                             cursor="hand2")
            label.pack(side="left", padx=(1 if i else 0, 0))
            label.bind("<Button-1>", lambda _e, i=i: self.select(i, True))
            self.labels.append(label)
        self.selected = -1
        self.enabled = True
        self.select(selected)

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
                label.configure(bg=ACCENT if on else FIELD, fg=ON_ACCENT if on else TEXT)
            else:
                label.configure(bg=BORDER if on else FIELD, fg=MUTED if on else DIM)


class CameraRow(tk.Frame):
    """Snapshot, at the bottom of the Aircraft card: a photo from the aircraft's camera, by way of the
    relay."""

    SIZES = ("Small", "Medium", "Large")

    def __init__(self, app: "App", master: tk.Misc) -> None:
        super().__init__(master, bg=SURFACE)
        s, pad = app.scale, round(8 * app.scale)
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")
        inner = tk.Frame(self, bg=SURFACE)
        inner.pack(fill="x", padx=pad, pady=pad)
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
        self.status = tk.Label(left, text="", bg=SURFACE, fg=DIM, font=app.font_small, anchor="w", justify="left")
        self.status.pack(fill="x", pady=(round(5 * s), 0))
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
    """The locator voice, a line of the Aircraft card: while it is on, the aircraft says its phrase through
    the speaker on its board, again and again, to be found in the last metres. The relay keeps the switch."""

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


class App:
    POLL_MS = 250

    def __init__(self, root: tk.Tk, config_path: str = CONFIG) -> None:
        self.root = root
        self.config_path = config_path
        self.settings = Settings.load(config_path)
        self.runner = AgentRunner()
        self.log_lines: "queue.Queue[str]" = queue.Queue()
        self.log_handler = LogHandler(self.log_lines)
        logging.getLogger("mavrelay").addHandler(self.log_handler)
        logging.getLogger("mavrelay").setLevel(logging.INFO)
        self.rate_mark = (time.monotonic(), 0, 0)
        self.rates = (0.0, 0.0)
        self.photos_in: "queue.Queue[Tuple[str, mr.PhotoInfo]]" = queue.Queue()  # saved by the agent's thread
        self.clicked_at = -1e9  # Snapshot: the photo that comes next opens in the viewer
        self.viewer: Optional[PhotoViewer] = None
        self.lan: Optional[str] = None  # this computer's address on its network, for the cards
        self.lan_at = -1e9

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
        bar.pack(fill="x", padx=round(10 * s), pady=(round(8 * s), 0))
        menu_button = tk.Label(bar, text="☰", bg=SURFACE, fg=MUTED, font=(self.font[0], 14), cursor="hand2")
        menu_button.pack(side="left")
        menu_button.bind("<Enter>", lambda _e: menu_button.configure(fg=TEXT))
        menu_button.bind("<Leave>", lambda _e: menu_button.configure(fg=MUTED))
        tk.Label(bar, text=APP, bg=SURFACE, fg=TEXT, font=self.font_bold).pack(side="left", padx=round(8 * s))
        self.menu = tk.Menu(root, tearoff=0, bg=SURFACE, fg=TEXT, activebackground=FIELD, activeforeground=TEXT,
                            bd=0)
        self.menu.add_command(label="Settings…", command=self.open_settings)
        self.menu.add_command(label="Show log", command=self.toggle_log)
        self.menu.add_command(label="Photo folder", command=self.open_photo_folder)
        self.menu.add_separator()
        self.menu.add_command(label="Exit", command=self.close)
        menu_button.bind("<Button-1>", lambda e: self.menu.tk_popup(e.x_root, e.y_root))

        who = tk.Frame(header, bg=SURFACE)
        who.pack(fill="x", padx=round(16 * s), pady=(round(8 * s), round(4 * s)))
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
        line.pack(fill="x", padx=round(16 * s), pady=(0, round(10 * s)))
        self.relay_led = Led(line, s, SURFACE)
        self.relay_led.pack(side="left")
        self.relay_text = tk.Label(line, text="", bg=SURFACE, fg=TEXT, font=self.font, anchor="w")
        self.relay_text.pack(side="left", padx=round(6 * s))
        tk.Frame(root, bg=ACCENT, height=max(2, round(2 * s))).pack(fill="x")

        self.body = tk.Frame(root, bg=BG)
        self.body.pack(fill="both", expand=True, padx=round(12 * s), pady=round(8 * s))
        tk.Label(self.body, text="Data transfer", bg=BG, fg=MUTED, font=self.font_bold).pack(anchor="w")
        self.tcp_card = ChannelCard(self, "tcp", self.settings.tcp)
        self.tcp_card.pack(fill="x", pady=(round(4 * s), round(6 * s)))
        self.udp_card = ChannelCard(self, "udp", self.settings.udp)
        self.udp_card.pack(fill="x")

        tk.Label(self.body, text="Aircraft", bg=BG, fg=MUTED, font=self.font_bold).pack(anchor="w",
                                                                                       pady=(round(8 * s), 0))
        craft = tk.Frame(self.body, bg=SURFACE, highlightbackground=BORDER, highlightthickness=1)
        craft.pack(fill="x", pady=(round(4 * s), 0))
        pad = round(8 * s)
        top = tk.Frame(craft, bg=SURFACE)
        top.pack(fill="x", padx=pad, pady=(pad, round(2 * s)))
        self.craft_led = Led(top, s, SURFACE)
        self.craft_led.pack(side="left")
        self.craft_state = tk.Label(top, text="", bg=SURFACE, fg=TEXT, font=self.font_bold)
        self.craft_state.pack(side="left", padx=round(8 * s))
        self.craft_bars = Bars(top, s, SURFACE)
        self.craft_bars.pack(side="right")
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
        self.voice_waiting_at: Optional[float] = None  # voice on, aircraft online, not speaking yet: since when
        self.camera = CameraRow(self, craft)
        self.camera.pack(fill="x")
        newest = photo_files(self.settings.photo_folder())
        self.camera.thumbnail.show(newest[-1] if newest else None)
        self._show_last(newest[-1] if newest else None)

        self.log_frame = tk.Frame(self.body, bg=BG)
        self.log_text = tk.Text(self.log_frame, height=7, width=48, font=("Consolas", 9), bg=LOG_BG, fg="#b8bcc2",
                                relief="flat", highlightbackground=BORDER, highlightthickness=1, wrap="word",
                                insertbackground=TEXT, state="disabled")
        scroll = ttk.Scrollbar(self.log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)
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
        size = self.camera.size.selected
        if self.runner.running and self.runner.snapshot(size):
            self.clicked_at = time.monotonic()
            log.info("asking the aircraft for a photo (%s, %d×%d)", SIZE_NAMES[size], *mr.SNAP_SIZES[size])
        else:
            self.camera.show("Not connected to the relay", RED)

    def toggle_voice(self, on: bool) -> None:
        if self.runner.running and self.runner.set_voice(on):
            self.voice.switch.set(on)  # the relay confirms within a second or two (_show_voice)
        else:
            self.voice.show("Not connected to the relay", RED)

    def choose_size(self, index: int) -> None:
        self.settings.photo_size = SIZE_NAMES[index]
        self._remember("photo_size")
        self.camera.show(self.SIZE_HINTS[index])

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
        except OSError as exc:
            log.warning("cannot save settings: %s", exc)

    def apply_settings(self, name: str, server: str, key: str) -> None:
        changed = (server, key) != (self.settings.server, self.settings.key)
        self.settings.name, self.settings.server, self.settings.key = name, server, key
        self._update_server_label()
        self._remember("name", "server", "key")
        if changed:  # reconnect with the new server or key
            cards = [card for card in (self.tcp_card, self.udp_card) if card.switch.on]
            if self.runner.running:
                self.runner.stop()
            for card in cards:
                card.set_on(False)
            self._connect()
            for card in cards:
                self.toggle(card, True)

    def open_settings(self) -> None:
        SettingsDialog(self)

    def toggle_log(self) -> None:
        if self.log_frame.winfo_ismapped():
            self.log_frame.pack_forget()
            self.menu.entryconfigure(1, label="Show log")
        else:
            self.log_frame.pack(fill="both", expand=True, pady=(round(8 * self.scale), 0))
            self.menu.entryconfigure(1, label="Hide log")

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
        self._drain_log()
        agent = self.runner.agent
        now = time.monotonic()
        status = None
        if agent is None:
            self.relay_led.set(LED_OFF)
            self._set(self.relay_text, "Not connected: set the relay server and key (☰ → Settings)")
        else:
            client = agent.client
            if client.session:
                rtt = f" · {client.rtt_ms} ms" if client.rtt_ms != mr.U16_UNKNOWN else ""
                self.relay_led.set(GREEN)
                self._set(self.relay_text, f"Connected to the relay{rtt}")
                status = agent.vehicle_status()
            elif client.hellos >= 5:
                self.relay_led.set(RED)
                self._set(self.relay_text, "No answer from the relay: check server, UDP port and key")
            else:
                self.relay_led.set(AMBER)
                self._set(self.relay_text, "Connecting to the relay…")
        online = bool(status and status.online)
        bars = self._bars(status) if online else None

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
        self._show_voice(agent, status, now)
        self._take_photos(now)
        self._show_camera(agent, online, now)
        self.poll_job = self.root.after(self.POLL_MS, self.poll)

    # size hints: typical JPEG sizes from the aircraft's OV5640 (bright, detailed scenes: the upper end)
    SIZE_HINTS = ("320×240, about 5-10 KB", "640×480, about 10-30 KB", "1024×768, about 25-80 KB")

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
        card = self.camera
        photos = agent.photos if agent is not None else None
        connected = agent is not None and bool(agent.client.session)
        arriving = photos.arriving if photos is not None else None
        busy = False
        if not connected:
            card.show("Not connected to the relay")
        elif arriving is not None:
            info, got = arriving
            card.show(f"Arriving: {got / 1024:.0f} of {info.size / 1024:.0f} KB ({info.width}×{info.height})", TEXT,
                      got / info.size)
            busy = True
        elif photos.asked_at is not None:
            card.show("Asking the aircraft…", TEXT)
            busy = True
        elif photos.problem and now - photos.problem_at < 60.0:
            card.show(f"No photo: {photos.problem}", AMBER)
        elif not online:
            card.show("The aircraft is offline")
        else:
            card.show(self.SIZE_HINTS[card.size.selected])
        card.set_enabled(connected and online and not busy)
        card.size.set_enabled(not busy)

    VOICE_ANSWER_S = 10.0  # an aircraft that has not said it speaks by then may not know the voice

    def _show_voice(self, agent, status: Optional[mr.LinkStatus], now: float) -> None:
        """The switch shows the relay's: it keeps it, whoever switched it, also while the aircraft is away."""
        row = self.voice
        if agent is None or not agent.client.session:
            row.switch.set(False)
            row.show("Not connected to the relay")
            return
        pending = agent.voice_request
        if pending is not None:  # sent, and not yet in the relay's STATUS
            row.switch.set(pending)
            row.show("Switching on…" if pending else "Switching off…", TEXT)
            return
        if status is None:
            return
        row.switch.set(status.voice_on)
        waiting = status.voice_on and status.online and not (status.speaking or status.voice_failed)
        if not waiting:
            self.voice_waiting_at = None
        elif self.voice_waiting_at is None:
            self.voice_waiting_at = now
        if not status.voice_on:
            row.show("Off: switch on to make the aircraft talk")
        elif status.voice_failed:
            row.show("On, but the aircraft cannot speak", RED)  # its modem refuses
        elif status.speaking and status.online:
            row.show("On: the aircraft is speaking", GREEN)
        elif status.speaking:  # and it goes on without the relay
            row.show("On: it was speaking when last heard", AMBER)
        elif not status.online:
            row.show("On: speaks once the aircraft is back", AMBER)
        elif now - self.voice_waiting_at < self.VOICE_ANSWER_S:
            row.show("On: waiting for the aircraft…", TEXT)
        else:  # it does not say whether it speaks
            row.show("On, but no answer: firmware before 1.5.0?", AMBER)

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
        The switch stays on while the aircraft is away, so that the telemetry comes back by itself."""
        if not online:
            return f"Waiting for the aircraft\n{gcs or 'Mission Planner: ' + where}", DIM
        return (gcs, GREEN) if gcs else (f"Waiting for Mission Planner: {where}", DIM)

    @staticmethod
    def _bars(status: mr.LinkStatus) -> int:
        if status.rssi_dbm != mr.RSSI_UNKNOWN:
            return sum(status.rssi_dbm >= limit for limit in (-105, -95, -85, -75))
        worst = max(v for v in (status.up_loss, status.down_loss, 0) if v != mr.U16_UNKNOWN)
        return 4 if worst < 10 else 3 if worst < 50 else 2 if worst < 150 else 1

    def _show_aircraft(self, agent, status: Optional[mr.LinkStatus], now: float) -> None:
        v = self.craft_values
        if agent is not None and now - self.rate_mark[0] >= 1.0:
            t, down, up = self.rate_mark
            self.rates = ((agent.to_gcs_bytes - down) / (now - t), (agent.from_gcs_bytes - up) / (now - t))
            self.rate_mark = (now, agent.to_gcs_bytes, agent.from_gcs_bytes)
        if agent is None or not agent.client.session:
            self.craft_led.set(LED_OFF)
            self._set(self.craft_state, "Not connected to the relay")
            for label in v.values():
                self._set(label, "-")
            self.craft_bars.set(None)
            self.rate_mark = (now, agent.to_gcs_bytes, agent.from_gcs_bytes) if agent else (now, 0, 0)
            self.rates = (0.0, 0.0)
            self._show_position(agent, False)
            return
        if status is None:
            self.craft_led.set(LED_OFF)
            self._set(self.craft_state, "Waiting for news from the relay…")
        elif status.online:  # green when available, blue when connected, as the LEDs on the cards
            self.craft_led.set(GREEN if agent.watching else BLUE)
            self._set(self.craft_state, "Available: switch TCP or UDP on" if agent.watching else "Online")
        elif status.idle_ms == mr.U16_UNKNOWN:
            self.craft_led.set(LED_OFF)
            self._set(self.craft_state, "Not connected to the relay (yet)")
        else:
            self.craft_led.set(RED)
            self._set(self.craft_state, f"Offline, last heard {status.idle_ms / 1000:.0f} s ago")
        online = bool(status and status.online)
        self.craft_bars.set(self._bars(status) if online else None)
        if online:  # the aircraft's radio and its round trip to the relay, on one line
            link = [mr.RAT_NAMES.get(status.rat, "")] if status.rat != mr.RAT_UNKNOWN else []
            if status.rssi_dbm != mr.RSSI_UNKNOWN:
                link.append(f"{status.rssi_dbm} dBm")
            link = [" ".join(link)] if link else []
            if status.rtt_ms != mr.U16_UNKNOWN:
                link.append(f"round trip {status.rtt_ms} ms")
            self._set(v["Link"], " · ".join(link) or "-")
            self._set(v["Packet loss"], f"up {mr.fmt_permille(status.up_loss)}, down {mr.fmt_permille(status.down_loss)}")
        else:
            for label in ("Link", "Packet loss"):
                self._set(v[label], "-")
        down, up = self.rates
        self._set(v["Traffic"], f"↓ {down / 1000:.1f} KB/s telemetry, ↑ {up / 1000:.1f} KB/s commands")
        self._show_position(agent, online)

    LIVE_S = 15.0  # a position report older than this (the module sends one every 5 s) is not live

    def _show_position(self, agent, online: bool) -> None:
        """The aircraft's own GNSS position, whatever its flight controller does: live while it reports,
        else the last known one (the relay keeps it). Map and Copy take the one shown."""
        pos = agent.position if agent is not None else None
        fix = agent.last_fix if agent is not None else None
        now_unix = time.time()
        live = online and pos is not None and now_unix - pos.time < self.LIVE_S
        shown, color = None, TEXT
        if live and pos.has_fix:
            shown = pos
            text = f"{pos.lat / 1e7:.5f}, {pos.lon / 1e7:.5f} · {pos.sats} satellites"
        elif fix is not None:
            shown = fix
            text = f"last known {fix.lat / 1e7:.5f}, {fix.lon / 1e7:.5f}, {mr.fmt_age(now_unix - fix.time)} ago"
            color = AMBER if not online else MUTED
        elif live and pos.flags & mr.POS_NO_GNSS:
            text, color = "the LTE module cannot read its GNSS", MUTED
        elif live:
            text, color = f"GNSS searching ({pos.sats} satellites)", MUTED
        else:
            text, color = "-", TEXT
        self.shown_fix = shown
        self._paint(self.craft_values["Position"], text, color)
        for link in (self.map_link, self.copy_link):
            self._paint(link, link.cget("text"), BLUE if shown else LED_OFF)

        module, color = "-", TEXT
        temp, temp_color = "", TEXT
        if live:
            if pos.fc_silent == mr.U16_UNKNOWN:
                module, color = "flight controller not heard yet", AMBER
            elif pos.fc_is_silent:
                module, color = f"flight controller silent for {mr.fmt_age(pos.fc_silent)}", RED
            else:
                module = "flight controller talking"
            if pos.battery_pct != mr.BATTERY_UNKNOWN:
                module += f" · battery {pos.battery_pct}%"
            if pos.temp != mr.TEMP_UNKNOWN:
                temp = f" · {pos.temp} °C"
                temp_color = RED if pos.temp >= mr.TEMP_HOT else AMBER if pos.temp >= mr.TEMP_WARM else TEXT
        self._paint(self.craft_values["Module"], module, color)
        self._paint(self.chip_temp, temp, temp_color)

    def _link(self, parent: tk.Misc, text: str, command: Callable[[], None]) -> tk.Label:
        link = tk.Label(parent, text=text, bg=SURFACE, fg=LED_OFF, font=self.font_small, cursor="hand2")
        link.pack(side="left", padx=(round(8 * self.scale), 0))
        link.bind("<Button-1>", lambda _e: command())
        return link

    def open_map(self) -> None:
        """The position on Google Maps (in the browser, or the Maps app on a phone), satellite view at hand."""
        pos = self.shown_fix
        if pos is not None:
            webbrowser.open(f"https://www.google.com/maps/search/?api=1&query={pos.lat / 1e7:.6f},{pos.lon / 1e7:.6f}")

    def copy_position(self) -> None:
        pos = self.shown_fix
        if pos is None:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(f"{pos.lat / 1e7:.6f}, {pos.lon / 1e7:.6f}")
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
        if lines:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", "\n".join(lines) + "\n")
            excess = int(self.log_text.index("end-1c").split(".")[0]) - 500
            if excess > 0:
                self.log_text.delete("1.0", f"{excess}.0")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")


def main() -> None:
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)  # sharp text on high-DPI screens
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("MavLTE.Agent")  # own taskbar icon
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()

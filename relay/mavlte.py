#!/usr/bin/env python3
"""MavLTE: the GCS agent as a window.

Gives Mission Planner (or QGroundControl) a TCP and a UDP port, each with its own switch. Two LEDs
on each: Available (green, left) while the aircraft's LTE module is online at the relay, whatever
the switches; Connected (blue, next to the switch) while it is online and that port is on. Both
switches start off. With both off the app only watches: the aircraft holds its telemetry back.

    python mavlte.py            (or double-click MavLTE.pyw, or run MavLTE.exe: build_release.py)

Settings live in the [gcs] section of mavrelay.ini next to this file, the same file that
start-gcs.bat and `mavrelay.py gcs --config mavrelay.ini` use; MavLTE.exe keeps its own (see
settings_file). Standard library only (Tkinter).
"""

from __future__ import annotations

import asyncio
import logging
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Callable, Optional, Tuple

import mavrelay as mr

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


@dataclass
class Settings:
    name: str = "My UAV"
    server: str = ""
    key: str = ""
    tcp: str = "127.0.0.1:5760"
    udp: str = "127.0.0.1:14550"

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
        return s

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

    def start(self, server: Tuple[str, int], key: bytes) -> None:
        async def go() -> None:
            self.agent = mr.GcsAgent(server, key, info=f"mavlte-app/{mr.__version__}")
            self.task = asyncio.ensure_future(self.agent.run())
            self.task.add_done_callback(self._ended)

        self._call(go())

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
                if kind == "tcp":
                    return f"TCP port {address[1]} is in use by another program ({exc.strerror or exc})"
                return f"cannot use UDP {address[0]}:{address[1]} ({exc.strerror or exc})"
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
    style.map("TButton", background=[("pressed", BORDER), ("active", "#3a3b3e")])
    style.configure("Accent.TButton", background=ACCENT, foreground=ON_ACCENT, bordercolor=ACCENT,
                    lightcolor=ACCENT, darkcolor=ACCENT, focuscolor=ACCENT)
    style.map("Accent.TButton", background=[("pressed", "#4cb84c"), ("active", "#72d972")])
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
        for row, label in enumerate(("Signal", "Round trip", "Packet loss", "Traffic")):
            tk.Label(grid, text=label, bg=SURFACE, fg=MUTED, font=self.font).grid(row=row, column=0, sticky="w")
            value = tk.Label(grid, text="-", bg=SURFACE, fg=TEXT, font=self.font, anchor="w")
            value.grid(row=row, column=1, sticky="w", padx=(12, 0))
            self.craft_values[label] = value

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
        self.runner.start(server, key)
        return True

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
                self.tcp_card.show(f"{n} GCS connected" if n else
                                   f"Waiting for Mission Planner: TCP, {self.tcp_card.text().replace(':', ', port ')}",
                                   GREEN if n else DIM)
            if self.udp_card.switch.on and agent.udp is not None:
                heard = now - agent.udp.last_rx < 3.0
                host, port = agent.udp.target
                where = f"UDP, port {port}" if mr.is_loopback(host) else f"UDP {host}, port {port}"
                self.udp_card.show("Mission Planner connected" if heard else f"Waiting for Mission Planner: {where}",
                                   GREEN if heard else DIM)
        self._show_aircraft(agent, status, now)
        self.poll_job = self.root.after(self.POLL_MS, self.poll)

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
        if online:
            radio = [mr.RAT_NAMES.get(status.rat, "")] if status.rat != mr.RAT_UNKNOWN else []
            if status.rssi_dbm != mr.RSSI_UNKNOWN:
                radio.append(f"{status.rssi_dbm} dBm")
            self._set(v["Signal"], " ".join(radio) or "not reported")
            self._set(v["Round trip"], f"{status.rtt_ms} ms, aircraft ↔ relay" if status.rtt_ms != mr.U16_UNKNOWN
                      else "-")
            self._set(v["Packet loss"], f"up {mr.fmt_permille(status.up_loss)}, down {mr.fmt_permille(status.down_loss)}")
        else:
            for label in ("Signal", "Round trip", "Packet loss"):
                self._set(v[label], "-")
        down, up = self.rates
        self._set(v["Traffic"], f"↓ {down / 1000:.1f} KB/s telemetry, ↑ {up / 1000:.1f} KB/s commands")

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

#!/usr/bin/env python3
"""Plane simulator: the aircraft end of MavLTE on this PC, until the board arrives.

The plane's two power switches, in a window beside the MavLTE app:

- Battery: the whole plane. On starts ArduPlane SITL as the flight controller; off stops it at
  once, like pulling the battery plug.
- LTE module: the modem on the ESP32 board. On, it starts up like the real one (about 16 s, or a
  couple of seconds with Quick start), then carries the flight controller's MAVLink to your relay
  as the ESP32 firmware does. Off cuts it without a goodbye, so the relay only notices the
  silence, as it would in the air.

The module's LED blinks like the RGB LED on the board. Coverage sets the signal the module
reports (the bars in the MavLTE app) and the delay and loss on its link.

    python plane_sim.py            (or double-click PlaneSim.pyw)

The relay server and vehicle key come from the [vehicle] section of mavrelay.ini. Mission Planner
can also reach the plane directly on its "USB port", TCP 127.0.0.1:5780.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
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

APP = "Plane simulator"
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mavrelay.ini")
FC_PORT = 5762  # SITL SERIAL1: the TELEM port the ESP32 is wired to
RAT_LTE = 7
LOGGERS = ("mavrelay", "4g-link")

# name, signal the module reports (dBm), added delay and jitter each way (ms), packet loss (%)
COVERAGE = (
    ("No signal", None, 0, 0, 100.0),
    ("Weak", -101, 120, 60, 5.0),
    ("Fair", -91, 40, 20, 1.0),
    ("Good", -81, 10, 5, 0.2),
    ("Excellent", -65, 0, 0, 0.0),
)
# seconds the modem spends starting, registering, and bringing mobile data up: as in the boot log
# of a real board (README, "A healthy start"), and shortened for Quick start
STARTUP_REAL = (9.3, 4.3, 1.9)
STARTUP_QUICK = (1.0, 0.6, 0.3)

PLANE_MODES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA", 6: "FBWB",
               7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF", 14: "AVOID_ADSB",
               15: "GUIDED", 17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL",
               22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL", 25: "LOITER2QLAND", 26: "AUTOLAND"}

YELLOW = "#e6c84a"
BLUE = "#4d9bff"

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

    def feed(self, data: bytes, now: float) -> None:
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
        elif msgid == 33:  # GLOBAL_POSITION_INT: relative_alt in mm
            self.alt = struct.unpack_from("<i", payload, 16)[0] / 1000


class Network(sitl_demo.LinkEmulator):
    """The mobile network between the module and the relay. close() is the module losing power:
    nothing more leaves it, not even packets still on their way."""

    def __init__(self, relay_addr, level: int) -> None:
        super().__init__(relay_addr, 0, 0, 0)
        self.closed = False
        self.set_level(level)

    def set_level(self, level: int) -> None:
        _, _, delay, jitter, loss = COVERAGE[level]
        self.delay, self.jitter, self.loss = delay / 1000, jitter / 1000, loss / 100

    def _forward(self, transport, data: bytes, addr) -> None:
        if self.loss and random.random() < self.loss:
            self.dropped += 1
            return
        delay = max(0.0, self.delay + random.uniform(-self.jitter, self.jitter))
        asyncio.get_running_loop().call_later(delay, self._send, transport, data, addr)

    def _send(self, transport, data: bytes, addr) -> None:
        if not self.closed:
            transport.sendto(data, addr)

    def close(self) -> None:
        self.closed = True
        for transport in (self.front, self.back):
            if transport is not None:
                transport.close()


class Modem:
    """One power-on of the LTE module."""

    def __init__(self) -> None:
        self.started = time.monotonic()
        self.stage = "starting"  # starting, searching, registered, data (mobile data up)
        self.net: Optional[Network] = None
        self.client: Optional[mr.TunnelClient] = None
        self.batcher: Optional[mr.Batcher] = None

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
        self.lte = True  # the module is fitted and switched on: it runs whenever the battery is on
        self.quick = False
        self.coverage = len(COVERAGE) - 1
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
        self.battery = on
        if on:
            self.fc, self.fc_error = FcState(), ""
            self.fc_task = self.loop.create_task(self._power_up())
        else:
            log.info("battery off")
            self._modem_off()
            if self.fc_task is not None:
                self.fc_task.cancel()
                self.fc_task = None
            self._stop_sitl()

    def set_lte(self, on: bool) -> None:
        self.lte = on
        if on and self.battery:
            self._modem_on()
        elif not on:
            self._modem_off()

    def set_coverage(self, level: int) -> None:
        self.coverage = level
        m = self.modem
        if m is not None and m.net is not None:
            m.net.set_level(level)

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
        self.fc.feed(data, now)
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
            self.modem = Modem()
            self.modem_task = self.loop.create_task(self._modem_run(self.modem))

    def _modem_off(self) -> None:
        if self.modem is not None:
            log.info("LTE module: power off")
            self.modem_task.cancel()  # a power cut: no goodbye to the relay
            self.modem = self.modem_task = None

    async def _modem_run(self, m: Modem) -> None:
        start, register, data = STARTUP_QUICK if self.quick else STARTUP_REAL
        log.info("LTE module: power on")
        try:
            await asyncio.sleep(start)
            m.stage = "searching"
            while COVERAGE[self.coverage][1] is None:  # no signal: no network to register with
                await asyncio.sleep(0.2)
            await asyncio.sleep(register)
            m.stage = "registered"
            log.info("LTE module: registered, LTE, signal %d dBm", COVERAGE[self.coverage][1] or 0)
            await asyncio.sleep(data)
            relay = await self._lookup()
            m.net = Network(relay, self.coverage)
            await m.net.start()
            m.batcher = mr.Batcher()
            m.client = mr.TunnelClient(mr.ROLE_VEHICLE, self.key, *m.net.address, on_data=self._to_fc,
                                       info=f"mavlte-planesim/{mr.__version__}", radio=self._radio)
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
            except OSError as exc:
                log.warning("LTE module: cannot look up %s (%s); trying again", self.server[0], exc)
                await asyncio.sleep(5.0)

    async def _flush(self, m: Modem) -> None:
        """Sends what the batcher holds once it is 50 ms old, as the firmware does, and logs the
        relay session coming and going."""
        session = 0
        while True:
            await asyncio.sleep(0.005)
            chunk = m.batcher.poll(time.monotonic(), 0.05)
            if chunk:
                m.send(chunk)
            if m.client.session != session:
                session = m.client.session
                if session:
                    log.info("LTE module: connected to the relay %s (session %08x)", self.server_text, session)
                else:
                    log.warning("LTE module: lost the relay session")

    def _radio(self) -> Tuple[int, int]:
        rssi = COVERAGE[self.coverage][1]
        return (mr.RSSI_UNKNOWN if rssi is None else rssi), RAT_LTE

    # -- from the window's thread

    def close(self) -> None:
        """Switches everything off and stops the plane's thread."""
        done = threading.Event()

        def off() -> None:
            self.set_battery(False)
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
    POLL_MS = 100  # the module's LED blinks like the board's, in 100 ms steps

    def __init__(self, root: tk.Tk, plane: Plane) -> None:
        self.root, self.plane = root, plane
        self.tick = 0
        self.rate_mark: tuple = (time.monotonic(), None, 0, 0)
        self.rates = (0.0, 0.0)
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
        if os.path.exists(ui.ICON):
            self.icon = tk.PhotoImage(file=ui.ICON)
            root.iconphoto(True, self.icon)
        ui.setup_style(root)
        self._build()
        ui.dark_title_bar(root)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.report_callback_exception = lambda _t, exc, _tb: log.error("unexpected error: %r", exc)
        self.poll()

    # -- layout

    def _build(self) -> None:
        root, s, pad = self.root, self.scale, self.pad
        root.title(APP)
        root.configure(bg=ui.BG)
        header = tk.Frame(root, bg=ui.SURFACE)
        header.pack(fill="x")
        who = tk.Frame(header, bg=ui.SURFACE)
        who.pack(fill="x", padx=round(16 * s), pady=round(10 * s))
        tk.Label(who, text="✈", bg=ui.SURFACE, fg=ui.ACCENT, font=(self.font[0], 24)).pack(side="left")
        names = tk.Frame(who, bg=ui.SURFACE)
        names.pack(side="left", padx=round(12 * s))
        tk.Label(names, text=APP, bg=ui.SURFACE, fg=ui.TEXT, font=self.font_title).pack(anchor="w")
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
        self.lte_values = self._grid(card, ("State", "Coverage", "Round trip", "GCS", "Data"),
                                     {"Coverage": self._coverage})
        self.quick = tk.BooleanVar(value=self.plane.quick)
        tk.Checkbutton(card, text="Quick start (the real modem takes about 16 s)", variable=self.quick,
                       command=self.toggle_quick, bg=ui.SURFACE, fg=ui.TEXT, selectcolor=ui.FIELD,
                       activebackground=ui.SURFACE, activeforeground=ui.TEXT, font=self.font_small, bd=0,
                       highlightthickness=0).pack(anchor="w", padx=round(4 * s), pady=(0, pad))

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

    def _coverage(self, parent) -> tk.Frame:
        s = self.scale
        frame = tk.Frame(parent, bg=ui.SURFACE)
        self.coverage = tk.Scale(frame, from_=0, to=len(COVERAGE) - 1, orient="horizontal", resolution=1,
                                 showvalue=False, length=round(110 * s), width=round(12 * s),
                                 sliderlength=round(18 * s), bd=0, highlightthickness=0, bg=ui.MUTED,
                                 activebackground=ui.TEXT, troughcolor=ui.FIELD, sliderrelief="flat",
                                 command=self.set_coverage)
        self.coverage.set(self.plane.coverage)
        self.coverage.pack(side="left")
        self.coverage_text = tk.Label(frame, text="", bg=ui.SURFACE, fg=ui.TEXT, font=self.font)
        self.coverage_text.pack(side="left", padx=(round(8 * s), 0))
        self.set_coverage(str(self.plane.coverage))
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

    def set_coverage(self, value: str) -> None:
        level = int(float(value))
        self.plane.call(self.plane.set_coverage, level)
        name, rssi = COVERAGE[level][:2]
        self.coverage_text.configure(text=name if rssi is None else f"{name} ({rssi} dBm)")

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
        self._show_lte(now, blink=self.tick % 10 < 5, flash=self.tick % 10 < 2)
        self.poll_job = self.root.after(self.POLL_MS, self.poll)

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

    def _show_lte(self, now: float, blink: bool, flash: bool) -> None:
        p, v = self.plane, self.lte_values
        m = p.modem
        client = m.client if m is not None else None
        session = client is not None and client.session != 0
        # the LED, as on the board (README: "The RGB LED")
        if m is None:
            color = ui.LED_OFF
        elif m.stage != "data":
            color = YELLOW if blink else ui.LED_OFF
        elif not session:
            color = BLUE
        else:
            color = ui.GREEN if client.gcs_present or flash else ui.LED_OFF
        self.lte_led.set(color)

        if not p.battery:
            state = "No power: the battery is off"
        elif m is None:
            state = "Off"
        else:
            since = f" ({now - m.started:.0f} s)"
            if m.stage == "starting":
                state = "Starting the modem…" + since
            elif m.stage == "searching":
                state = ("No signal: searching for the network…" if p.coverage == 0
                         else "Searching for the network…") + since
            elif m.stage == "registered":
                state = "Registered (LTE), starting mobile data…" + since
            elif p.coverage == 0:
                state = "No signal: nothing gets through"
            elif session:
                state = "Connected to the relay"
            elif client.hellos >= 5:
                state = "Mobile data up, but no answer from the relay"
            else:
                state = "Mobile data up, connecting to the relay…" + since
        self._set(v["State"], state)
        self.lte_bars.set(p.coverage if m is not None and m.stage in ("registered", "data") else None)
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
        self._set(v["Data"], f"↑ {up / 1000:.1f} KB/s   ↓ {down / 1000:.1f} KB/s" if net is not None else "-")

    def _rate(self, net: Optional[Network], now: float) -> None:
        t, mark, up, down = self.rate_mark
        if net is not mark:
            self.rate_mark = (now, net, net.up_bytes if net else 0, net.down_bytes if net else 0)
            self.rates = (0.0, 0.0)
        elif net is not None and now - t >= 1.0:
            self.rates = ((net.up_bytes - up) / (now - t), (net.down_bytes - down) / (now - t))
            self.rate_mark = (now, net, net.up_bytes, net.down_bytes)

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

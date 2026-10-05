"""The board's USB link (firmware/main/usbproto.h): MavLTE lists and downloads the aircraft's flight log over the USB
cable from the board's USB-C socket, the port it is flashed through, while the board runs. Needs pyserial.

    link = BoardLink("COM9")
    link.open()                                   # HELLO, then 2 Mbaud
    entries, total = link.list()                  # [(name, size, start, end)], newest first
    link.get("LOG00012.CSV", 0, write, progress)  # write(offset, data) in order, progress(have, size)
    link.close()
"""

from __future__ import annotations

import base64
import binascii
import re
import threading
import time
import zlib
from typing import Callable, List, Optional, Tuple

import mavrelay as mr

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # the Logs window then offers 4G only
    serial = list_ports = None

VID = 0x1A86  # WCH, the maker of the board's USB serial chip
PID = 0x55D3  # its CH343, behind the V2 board's USB-C socket
SLOW = 115200  # the board's rate after a reset, and after 20 s without a command
# about 65 KB/s of a file: on a V2 board's CH343, faster rates lose bytes now and then (2 Mbaud: a line in about
# 200 KB, 1.5 Mbaud more often), 921600 none in 2.7 MB
FAST = 921600
BROKEN_MOST = 5  # lines in a row that arrive broken: the cable or port does not carry this
HELLO_WAIT = 1.0  # s for the answer to HELLO
SPEED_CHECK = 2.0  # the board's: after SPEED, the first command must come within this at the new rate
IDLE_BACK = 15.0  # s without a command: the board may be back at SLOW soon (it goes after 20 s, USB_IDLE_MS)
LOG_NAME = re.compile(r"LOG\d{5}\.CSV", re.IGNORECASE)  # the only names the board's card holds
REPLY = re.compile(rb"@(MAVLTE|FILE|END|SIZE|D|DONE|ERR|SPEED)\b ?([^\r\n]*)")
Entry = Tuple[str, int, int, int]  # name, bytes, start, end (unix seconds, 0: unknown)


class UsbError(Exception):
    """fatal: the port or the board stopped answering (the link has to be opened again), rather than the board
    answering with a problem."""

    def __init__(self, text: str, fatal: bool = False) -> None:
        super().__init__(text)
        self.fatal = fatal


def board_ports() -> List[str]:
    """The serial ports the board may be on: WCH's, its CH343 first. Other makers' ports are left alone (an RC
    link or a flight controller on USB, say)."""
    if list_ports is None:
        return []
    ports = sorted((p for p in list_ports.comports() if p.vid == VID), key=lambda p: (p.pid != PID, p.device))
    return [p.device for p in ports]


class BoardLink:
    """One board on a serial port. Its methods block; one thread at a time. A command line the board does not
    answer as expected raises UsbError."""

    def __init__(self, port: str, opener: Optional[Callable[[], object]] = None) -> None:
        self.port = port
        self.opener = opener  # for tests: a fake port
        self.ser = None
        self.buf = b""
        self.version = ""
        self.card = False
        self.last_cmd = 0.0  # when the last command went (monotonic)

    def open(self, fast: bool = True) -> None:
        if self.opener is not None:
            self.ser = self.opener()
        else:
            if serial is None:
                raise UsbError("needs pyserial: pip install pyserial")
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = self.port, SLOW, 0.05
            s.dtr = s.rts = False  # the board's reset and boot lines stay as they are: it goes on running
            try:
                s.open()
            except serial.SerialException as exc:
                raise UsbError(f"cannot open {self.port}: {exc}", fatal=True) from exc
            self.ser = s
        try:
            try:
                self.hello()
            except UsbError:  # perhaps still at the fast rate, from a session that ended a moment ago
                self.ser.baudrate = FAST
                self.buf = b""
                self.hello(tries=2)
            if fast and self.ser.baudrate != FAST:
                self.speed(FAST)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        """Closes the port, with the board back at its usual rate for whatever opens the port next (a serial
        monitor, say)."""
        if self.ser is None:
            return
        try:
            if self.ser.baudrate != SLOW:
                self._send(f"MAVLTE SPEED {SLOW}")
                self._reply({"SPEED"}, 1.0)
        except (UsbError, TimeoutError):
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _send(self, text: str) -> None:
        try:
            self.ser.write(text.encode("ascii") + b"\n")
        except Exception as exc:  # the cable pulled out
            raise UsbError(f"{self.port}: {exc}", fatal=True) from exc
        self.last_cmd = time.monotonic()

    def _awake(self) -> None:
        """After a pause (the list looked at for a while, say) the board may be back at SLOW: found at either rate,
        then at FAST again."""
        if self.ser.baudrate == SLOW or time.monotonic() - self.last_cmd < IDLE_BACK:
            return
        self.buf = b""
        try:
            self.hello(tries=1)  # still at FAST: that keeps it there
        except UsbError:
            self.ser.baudrate = SLOW
            self.buf = b""
            self.hello()
            self.speed(FAST)

    def _reply(self, until: set, timeout: float) -> Tuple[str, bytes]:
        """The next reply whose keyword is in `until`: a line with a known keyword anywhere in it (a piece of the
        board's own log may sit in front); the rest is passed by."""
        end = time.monotonic() + timeout
        while True:
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                m = REPLY.search(line)
                if m and m.group(1).decode() in until:
                    return m.group(1).decode(), m.group(2)
            if time.monotonic() > end:
                raise TimeoutError
            try:
                self.buf += self.ser.read(max(1, self.ser.in_waiting))
            except Exception as exc:
                raise UsbError(f"{self.port}: {exc}", fatal=True) from exc
            if len(self.buf) > 1 << 16 and b"\n" not in self.buf:
                self.buf = b""  # noise, at a rate the board is not at

    def hello(self, tries: int = 4) -> None:
        for _ in range(tries):
            self._send("MAVLTE HELLO")
            try:
                _, rest = self._reply({"MAVLTE"}, HELLO_WAIT)
            except TimeoutError:
                self.buf = b""
                continue
            words = rest.decode("ascii", "replace").split()
            self.version = words[0] if words else "?"
            self.card = len(words) > 1 and words[1] == "ok"
            return
        raise UsbError(f"no MavLTE board answers on {self.port} (firmware before 1.8.0?)", fatal=True)

    def speed(self, baud: int) -> None:
        """Both ends to `baud`; back to SLOW if nothing gets through at it."""
        self._send(f"MAVLTE SPEED {baud}")
        try:
            _, rest = self._reply({"SPEED"}, 2.0)
        except TimeoutError:
            return
        if rest.strip() != str(baud).encode():  # it stays where it is
            return
        self.ser.baudrate = baud
        self.buf = b""
        try:
            self.hello(tries=2)
        except UsbError:  # the board goes back by itself
            time.sleep(SPEED_CHECK + 0.5)
            self.ser.baudrate = SLOW
            self.buf = b""
            self.hello()

    def list(self, first: int = 0) -> Tuple[List[Entry], int]:
        """Up to 40 files, newest first, from index `first` on, and how many the card has."""
        self._awake()
        self._send(f"MAVLTE LIST {first}")
        entries: List[Entry] = []
        while True:
            try:
                kind, rest = self._reply({"FILE", "END", "ERR"}, 15.0)
            except TimeoutError:
                raise UsbError("the board did not answer", fatal=True) from None
            if kind == "ERR":
                raise UsbError(self._problem(rest))
            if kind == "END":
                words = rest.split()
                return entries, int(words[0]) if words and words[0].isdigit() else len(entries)
            words = rest.decode("ascii", "replace").split()
            if len(words) >= 4 and LOG_NAME.fullmatch(words[0]) and all(w.isdigit() for w in words[1:4]):
                entries.append((words[0], int(words[1]), int(words[2]), int(words[3])))
            # (anything else came broken, or is not a log file: passed by)

    def get(self, name: str, offset: int, write: Callable[[int, bytes], None], progress: Callable[[int, int], None],
            stopped: Optional[threading.Event] = None) -> int:
        """Downloads `name` from `offset` on; returns its size. A line that arrives broken (or not at all) asks again
        from where the good ones end. stopped: set to stop it (UsbError "stopped")."""
        if not LOG_NAME.fullmatch(name):
            raise UsbError(f"not a log file's name: {name!r}")
        size, broken = -1, 0
        self._awake()
        while True:
            self._send(f"MAVLTE GET {name} {offset}")
            try:
                kind, rest = self._reply({"SIZE", "ERR"}, 5.0)
            except TimeoutError:
                raise UsbError("the board did not answer", fatal=True) from None
            if kind == "ERR":
                raise UsbError(self._problem(rest))
            words = rest.split()
            if len(words) < 2 or not words[1].isdigit():  # came broken: ask again
                broken += 1
                if broken > BROKEN_MOST:
                    raise UsbError("too many broken lines: try another cable or USB port", fatal=True)
                self.stop()
                continue
            size = int(words[1])
            progress(offset, size)
            while True:
                if stopped is not None and stopped.is_set():
                    self.stop()
                    raise UsbError("stopped")
                try:
                    kind, rest = self._reply({"D", "DONE", "ERR"}, 5.0)
                except TimeoutError:
                    kind = ""
                if kind == "ERR":
                    raise UsbError(self._problem(rest))
                if kind == "DONE" and offset >= size:
                    return size
                data = self._data(rest, offset) if kind == "D" else None
                if data is None:  # broken, missing, or the end before all of it came: from here again
                    broken += 1
                    if broken > BROKEN_MOST:
                        raise UsbError("too many broken lines: try another cable or USB port", fatal=True)
                    if kind != "DONE":
                        self.stop()
                    break
                broken = 0
                write(offset, data)
                offset += len(data)
                progress(offset, size)

    @staticmethod
    def _data(rest: bytes, offset: int) -> Optional[bytes]:
        try:
            at, b64, crc = rest.split()
            data = base64.b64decode(b64, validate=True)
        except (ValueError, binascii.Error):
            return None
        if int(at) != offset or zlib.crc32(data) != int(crc, 16):
            return None
        return data

    def stop(self) -> None:
        """Ends a GET: the board says so, and what was on its way is passed by."""
        self._send("MAVLTE STOP")
        try:
            self._reply({"ERR", "DONE"}, 2.0)
        except TimeoutError:
            pass
        self.buf = b""

    @staticmethod
    def _problem(rest: bytes) -> str:
        words = rest.decode("ascii", "replace").split(None, 1)
        try:
            return mr.FILE_PROBLEMS.get(int(words[0]), words[1] if len(words) > 1 else "error")
        except (ValueError, IndexError):
            return rest.decode("ascii", "replace")

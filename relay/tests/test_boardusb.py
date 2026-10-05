"""Tests for boardusb.py, MavLTE's side of the board's USB link (firmware/main/usbproto.h), against a fake board.
Run from the relay directory:  python -m unittest -v"""

import base64
import os
import sys
import threading
import unittest
import zlib
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import boardusb  # noqa: E402
import mavrelay as mr  # noqa: E402


def log_file(lines, first=1790000000):
    rows = [b"time_utc,uptime_s,gnss_fix"] + [b"2026-10-02T09:%02d:%02dZ,%d,3" % (i // 60 % 60, i % 60, i)
                                              for i in range(lines)]
    return b"\n".join(rows) + b"\n"


class FakeBoard:
    """The board's side of the link, as firmware/main/usblink.c has it, behind a serial port's read() and write()
    (pyserial's): BoardLink's opener returns it."""

    def __init__(self, files=None, card=True, fast_ok=True, version="1.8.0"):
        self.files = dict(files or {})  # name -> bytes
        self.times = {}  # name -> (start, end)
        self.card = card
        self.fast_ok = fast_ok  # nothing gets through at a fast rate on a bad cable
        self.version = version
        self.baudrate = boardusb.SLOW  # the PC's end, which BoardLink sets
        self.rate = boardusb.SLOW  # the board's
        self.pending = None  # the rate it switches to once its answer to SPEED is out
        self.broken = False
        self.out = bytearray()
        self.inbox = b""
        self.get = None  # [name, next, size]
        self.corrupt = set()  # offsets whose line goes out broken, once
        self.always_broken = False
        self.drop = set()  # offsets whose line goes missing, once
        self.noise = b""  # a piece of the board's log in front of the next reply
        self.commands = []
        self.closed = False
        self.bad_size = 0  # @SIZE answers that go out garbled

    def _synced(self):
        if self.broken and self.baudrate == boardusb.SLOW:  # the board went back by itself
            self.rate, self.broken = boardusb.SLOW, False
        return self.baudrate == self.rate and not self.broken

    def write(self, data):
        if not self._synced():
            return len(data)  # noise to the board
        self.inbox += data
        while b"\n" in self.inbox:
            line, self.inbox = self.inbox.split(b"\n", 1)
            self._command(line.decode().strip())
        return len(data)

    @property
    def in_waiting(self):
        if self.get is not None and len(self.out) < 4096:
            self._more()
        return len(self.out)

    def read(self, n=1):
        self.in_waiting  # noqa: B018 (the download goes on as the PC reads)
        data = bytes(self.out[:n])
        del self.out[:n]
        synced = self._synced()
        if self.pending is not None and not self.out:
            self.rate, self.pending = self.pending, None
            self.broken = not self.fast_ok and self.rate != boardusb.SLOW
        return data if synced else bytes(b ^ 0x5A for b in data)

    def close(self):
        self.closed = True

    def _reply(self, line):
        self.out += self.noise + line.encode() + b"\n"
        self.noise = b""

    def _command(self, line):
        words = line.split()
        if len(words) < 2 or words[0] != "MAVLTE":
            return
        self.commands.append(line)
        cmd = words[1]
        if cmd == "HELLO":
            self._reply(f"@MAVLTE {self.version} {'ok' if self.card else 'none'}")
        elif cmd == "LIST":
            first = int(words[2]) if len(words) > 2 else 0
            if not self.card:
                self._reply("@ERR 1 no SD card")
                return
            names = sorted(self.files, reverse=True)
            for name in names[first:first + 40]:
                start, end = self.times.get(name, (0, 0))
                self._reply(f"@FILE {name} {len(self.files[name])} {start} {end}")
            self._reply(f"@END {len(names)}")
        elif cmd == "GET":
            self.get = None
            name, offset = words[2].upper(), int(words[3]) if len(words) > 3 else 0
            if not self.card:
                self._reply("@ERR 1 no SD card")
            elif name not in self.files:
                self._reply("@ERR 2 no such file")
            else:
                size = len(self.files[name])
                if self.bad_size:
                    self.bad_size -= 1
                    self._reply(f"@SIZE {name} 12x4")
                    return
                self.get = [name, min(offset, size), size]
                self._reply(f"@SIZE {name} {size}")
        elif cmd == "SPEED":
            baud = int(words[2])
            self._reply(f"@SPEED {baud}")
            self.pending = baud
        elif cmd == "STOP" and self.get is not None:
            self.get = None
            self._reply("@ERR 5 stopped")

    def _more(self):
        name, at, size = self.get
        if at >= size:
            self.get = None
            self._reply(f"@DONE {name} {size}")
            return
        chunk = self.files[name][at:at + 768]
        b64 = base64.b64encode(chunk).decode()
        self.get[1] += len(chunk)
        if at in self.drop:  # lost on the way
            self.drop.discard(at)
            return
        if at in self.corrupt or self.always_broken:  # a byte gone wrong on the way
            self.corrupt.discard(at)
            b64 = b64[:7] + ("B" if b64[7] != "B" else "C") + b64[8:]
        self._reply(f"@D {at} {b64} {zlib.crc32(chunk):08x}")


class BoardLinkTest(unittest.TestCase):
    def link(self, board):
        link = boardusb.BoardLink("COM99", opener=lambda: board)
        link.open()
        return link

    def download(self, link, name, offset=0, stopped=None):
        got, seen = bytearray(), []

        def write(at, data):
            self.assertEqual(at, offset + len(got))
            got.extend(data)

        size = link.get(name, offset, write, lambda have, size: seen.append((have, size)), stopped)
        return bytes(got), size, seen

    def test_after_a_pause_the_board_is_found_again(self):
        # the board goes back to SLOW after 20 s without a command; a list or download after a longer look at the
        # list found it again at either rate (1.8.5 review: before, the first one failed)
        board = FakeBoard(files={"LOG00001.CSV": b"x" * 2000})
        link = self.link(board)
        self.assertEqual(board.rate, boardusb.FAST)
        board.rate = boardusb.SLOW  # what 20 s without a command did
        link.last_cmd -= boardusb.IDLE_BACK + 1
        self.assertEqual(link.list()[1], 1)
        self.assertEqual((board.rate, link.ser.baudrate), (boardusb.FAST, boardusb.FAST))
        board.rate = boardusb.SLOW
        link.last_cmd -= boardusb.IDLE_BACK + 1
        self.assertEqual(self.download(link, "LOG00001.CSV")[0], b"x" * 2000)

    def test_broken_answers_and_strange_names_are_passed_by(self):
        board = FakeBoard(files={"LOG00002.CSV": b"y" * 100, "NOTES.TXT": b"z", "LOG2.CSV": b"w"})
        link = self.link(board)
        entries, total = link.list()
        self.assertEqual([e[0] for e in entries], ["LOG00002.CSV"])  # only log files' names
        board.bad_size = 2  # garbled answers to GET: asked again
        self.assertEqual(self.download(link, "LOG00002.CSV")[0], b"y" * 100)
        with self.assertRaises(boardusb.UsbError) as caught:
            link.get("..\\..\\boot.ini", 0, lambda at, data: None, lambda have, size: None)
        self.assertIn("not a log file's name", str(caught.exception))

    def test_hello_then_fast(self):
        board = FakeBoard(files={"LOG00001.CSV": b"x"})
        link = self.link(board)
        self.assertEqual((link.version, link.card), ("1.8.0", True))
        self.assertEqual((board.baudrate, board.rate), (boardusb.FAST, boardusb.FAST))
        self.assertEqual(board.commands[:3], ["MAVLTE HELLO", f"MAVLTE SPEED {boardusb.FAST}", "MAVLTE HELLO"])

    def test_still_fast_from_before_and_back_to_slow_on_close(self):
        board = FakeBoard(files={"LOG00001.CSV": b"x"})
        board.rate = boardusb.FAST  # a session that ended a moment ago: the board has not gone back yet
        with mock.patch.object(boardusb, "HELLO_WAIT", 0.1):
            link = self.link(board)
        self.assertEqual((link.version, board.baudrate), ("1.8.0", boardusb.FAST))
        link.close()  # the board back at its usual rate, for a serial monitor say
        self.assertEqual((board.rate, board.closed), (boardusb.SLOW, True))

    def test_back_to_slow_where_fast_does_not_get_through(self):
        board = FakeBoard(files={"LOG00001.CSV": log_file(50)}, fast_ok=False)
        with mock.patch.object(boardusb, "SPEED_CHECK", 0.05):
            link = self.link(board)
        self.assertEqual(board.baudrate, boardusb.SLOW)
        data, size, _ = self.download(link, "LOG00001.CSV")
        self.assertEqual(data, board.files["LOG00001.CSV"])

    def test_list_newest_first_a_page_at_a_time(self):
        board = FakeBoard(files={f"LOG{n:05d}.CSV": bytes(n) for n in range(1, 46)})
        board.times["LOG00045.CSV"] = (1790000000, 1790003600)
        link = self.link(board)
        entries, total = link.list()
        self.assertEqual((len(entries), total), (40, 45))
        self.assertEqual(entries[0], ("LOG00045.CSV", 45, 1790000000, 1790003600))
        entries, total = link.list(40)
        self.assertEqual([e[0] for e in entries], [f"LOG{n:05d}.CSV" for n in range(5, 0, -1)])

    def test_download_resume_and_broken_lines(self):
        data = log_file(400)
        board = FakeBoard(files={"LOG00007.CSV": data})
        link = self.link(board)
        got, size, seen = self.download(link, "log00007.csv")  # any case
        self.assertEqual((got, size), (data, len(data)))
        self.assertEqual(seen[-1], (len(data), len(data)))
        got, _, _ = self.download(link, "LOG00007.CSV", 5000)  # from where a copy ends
        self.assertEqual(got, data[5000:])
        board.corrupt = {768 * 3, 768 * 9}  # two lines go wrong: asked again from there
        got, _, _ = self.download(link, "LOG00007.CSV")
        self.assertEqual(got, data)
        self.assertEqual(sum(c.startswith("MAVLTE GET") for c in board.commands), 5)
        got, size, _ = self.download(link, "LOG00007.CSV", len(data))  # nothing more
        self.assertEqual((got, size), (b"", len(data)))
        board.drop = {len(data) // 768 * 768}  # the last line lost: @DONE comes before all of it
        got, _, _ = self.download(link, "LOG00007.CSV")
        self.assertEqual(got, data)
        board.always_broken = True  # a cable that does not carry it
        with self.assertRaises(boardusb.UsbError) as caught:
            self.download(link, "LOG00007.CSV")
        self.assertIn("too many broken lines", str(caught.exception))

    def test_problems(self):
        link = self.link(FakeBoard(files={"LOG00001.CSV": b"x"}))
        with self.assertRaises(boardusb.UsbError) as caught:
            self.download(link, "LOG00002.CSV")
        self.assertEqual((str(caught.exception), caught.exception.fatal), (mr.FILE_PROBLEMS[mr.FILE_NOT_FOUND],
                                                                         False))
        link = self.link(FakeBoard(card=False))
        self.assertFalse(link.card)
        with self.assertRaises(boardusb.UsbError) as caught:
            link.list()
        self.assertEqual(str(caught.exception), mr.FILE_PROBLEMS[mr.FILE_NO_CARD])

    def test_stop(self):
        board = FakeBoard(files={"LOG00001.CSV": log_file(2000)})
        link = self.link(board)
        stopped = threading.Event()
        got = bytearray()

        def write(at, data):
            got.extend(data)
            if len(got) > 3000:
                stopped.set()

        with self.assertRaises(boardusb.UsbError) as caught:
            link.get("LOG00001.CSV", 0, write, lambda have, size: None, stopped)
        self.assertEqual(str(caught.exception), "stopped")
        self.assertIn("MAVLTE STOP", board.commands)
        self.assertIsNone(board.get)
        link.hello()  # and it goes on
        self.assertLess(len(got), 6000)

    def test_log_lines_around_the_replies(self):
        board = FakeBoard(files={"LOG00001.CSV": log_file(10)})
        board.out += b"I (5123) modem: registered with Turkcell, LTE\r\n"
        board.noise = b"I (5124) bri"  # a log line cut short by the reply
        link = self.link(board)
        self.assertEqual(link.version, "1.8.0")

    def test_no_board(self):
        class Silent(FakeBoard):
            def _command(self, line):
                pass

        with self.assertRaises(boardusb.UsbError) as caught:
            self.link(Silent())
        self.assertTrue(caught.exception.fatal)
        self.assertIn("no MavLTE board answers on COM99", str(caught.exception))

    def test_board_ports(self):
        ports = [SimpleNamespace(device="COM3", vid=0x10C4, pid=0xEA60),  # an RC link's CP2102: left alone
                 SimpleNamespace(device="COM7", vid=0x1A86, pid=0x7523),  # some other WCH chip
                 SimpleNamespace(device="COM9", vid=0x1A86, pid=0x55D3),  # the board's CH343
                 SimpleNamespace(device="COM4", vid=None, pid=None)]
        with mock.patch.object(boardusb, "list_ports", SimpleNamespace(comports=lambda: ports)):
            self.assertEqual(boardusb.board_ports(), ["COM9", "COM7"])


if __name__ == "__main__":
    unittest.main()

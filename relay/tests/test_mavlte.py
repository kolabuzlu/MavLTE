"""The MavLTE app window, driven through its own switches against a real relay and a fake aircraft.

The window stays hidden. Skipped where Tk cannot open a display.
"""

import asyncio
import gc
import io
import json
import os
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest
import weakref
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import boardusb  # noqa: E402
import maptiles  # noqa: E402
import mavrelay as mr  # noqa: E402
from test_boardusb import FakeBoard  # noqa: E402
from test_mavrelay import KEY_G, KEY_V, v2_frame  # noqa: E402

try:
    import tkinter as tk

    import mavlte

    _root = tk.Tk()
    _root.destroy()
    del _root
    HAVE_TK = True
except Exception:  # no tkinter, or no display
    HAVE_TK = False


def jpeg():
    try:
        from PIL import Image
    except ImportError:  # the app shows no thumbnail then, but keeps the bytes all the same
        return b"\xff\xd8" + bytes(range(256)) * 20 + b"\xff\xd9"
    out = io.BytesIO()
    Image.new("RGB", (64, 48), (70, 120, 60)).save(out, "JPEG")
    return out.getvalue()


JPEG = jpeg()


def flight_log(lines, first=1790933700, untimed=0):
    """A flight log as the board writes it: the header, `untimed` lines before it knew the time, then a line a second."""
    rows = [b"time_utc,uptime_s,gnss_fix,gnss_sats"] + [b",%d,0,0" % i for i in range(untimed)]
    for i in range(lines):
        stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(first + i)).encode()
        rows.append(stamp + b",%d,3,11,41.1234567,28.9876543" % (untimed + i))
    return b"\n".join(rows) + b"\n"


def free_port(kind=socket.SOCK_DGRAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@unittest.skipUnless(HAVE_TK, "needs Tk and a display")
class GuiTest(unittest.TestCase):
    def setUp(self):
        # relay and aircraft on their own asyncio loop
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.vehicle_inbox = []
        self.vehicle_quiet = False

        async def start():
            transport, _ = await self.loop.create_datagram_endpoint(lambda: self.relay,
                                                                    local_addr=("127.0.0.1", 0))
            port = transport.get_extra_info("sockname")[1]
            self.vehicle = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", port,
                                           on_data=self.vehicle_inbox.append)
            # its camera takes JPEG; the tests may take it away (capture None: "no camera")
            self.outbox = outbox = mr.PhotoOutbox(self.vehicle, capture=lambda width, height: JPEG,
                                                  where=lambda: (411234567, 289876543, 120_000, 4500),
                                                  cap=lambda: 64 * 1024)
            self.vehicle.on_packet, self.vehicle.on_session = outbox.on_packet, outbox.on_session
            self.vehicle_task = asyncio.ensure_future(self.vehicle.run())

            async def ticks():
                seq = 0
                while True:
                    await asyncio.sleep(0.1)
                    now = time.monotonic()
                    self.relay.tick(now)
                    self.relay.photos.pump(now)
                    outbox.pump(now)
                    seq += 1
                    if not self.vehicle_quiet:
                        self.vehicle.send_data(v2_frame(0, bytes(9), seq))  # 10 heartbeats a second

            asyncio.ensure_future(ticks())
            return port

        relay_port = asyncio.run_coroutine_threadsafe(start(), self.loop).result(5)
        self.udp_port, self.tcp_port = free_port(), free_port(socket.SOCK_STREAM)
        fd, self.config = tempfile.mkstemp(suffix=".ini")
        os.close(fd)
        self.photos = tempfile.mkdtemp()  # never the user's Pictures folder
        with open(self.config, "w") as f:
            f.write("# my settings\n[gcs]\nname = Test UAV\nserver = 127.0.0.1:%d\nkey = %s\n"
                    "udp = 127.0.0.1:%d\ntcp = 127.0.0.1:%d\nudp_on = yes\ntcp_on = yes\n"  # older version
                    "photo_dir = %s\n" % (relay_port, KEY_G.hex(), self.udp_port, self.tcp_port, self.photos))
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = mavlte.App(self.root, config_path=self.config)

    def close_window(self):
        """Tk objects must be destroyed in the main thread: never leave them to the garbage collector,
        which may run in one of the network threads."""
        try:
            self.app.close()
        except tk.TclError:
            pass
        self.app = self.root = None
        gc.collect()

    def tearDown(self):
        self.close_window()

        async def cancel_all():
            self.relay.transport.close()
            tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        asyncio.run_coroutine_threadsafe(cancel_all(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()
        os.remove(self.config)
        shutil.rmtree(self.photos, ignore_errors=True)

    def pump(self, cond, timeout=8.0, what="condition"):
        end = time.time() + timeout
        while not cond():
            if time.time() > end:
                self.fail(f"{what} not met in time")
            self.root.update()
            time.sleep(0.02)

    def text(self, label):
        return label.cget("text")

    def test_switches_leds_and_traffic(self):
        app = self.app
        # both switches off: connected, but only watching
        self.pump(lambda: self.text(app.relay_text).startswith("Connected to the relay"), what="watching")
        self.assertTrue(app.runner.agent.watching)
        self.pump(lambda: app.udp_card.available.color == mavlte.GREEN, what="Available LED")
        self.assertEqual(app.tcp_card.available.color, mavlte.GREEN)
        self.assertEqual(app.udp_card.led.color, mavlte.LED_OFF)
        self.pump(lambda: self.text(app.craft_state) == "Available: switch TCP or UDP on", what="aircraft available")
        self.assertEqual(app.craft_led.color, mavlte.GREEN)
        self.pump(lambda: not self.vehicle.gcs_present, what="aircraft told there is no GCS")
        self.assertEqual(app.runner.agent.to_gcs_bytes, 0)  # the relay sends a watcher no telemetry

        mp_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # plays Mission Planner on UDP
        mp_udp.bind(("127.0.0.1", self.udp_port))
        mp_udp.settimeout(3)
        self.addCleanup(mp_udp.close)

        app.toggle(app.udp_card, True)
        self.assertTrue(app.udp_card.switch.on and not app.runner.agent.watching)
        self.assertEqual(str(app.udp_card.entries[1].cget("state")), "disabled")  # no editing while on
        self.pump(lambda: app.udp_card.led.color == mavlte.BLUE, what="UDP Connected LED blue")
        self.assertEqual(self.text(app.craft_state), "Online")
        self.assertEqual(app.craft_led.color, mavlte.BLUE)  # as the Connected LED
        self.pump(lambda: self.vehicle.gcs_present, what="aircraft told a GCS is there")
        self.assertEqual(app.tcp_card.led.color, mavlte.LED_OFF)  # switched off: stays dark

        data, agent_addr = mp_udp.recvfrom(4096)
        self.assertEqual(data[:1], b"\xfd")
        mp_udp.sendto(v2_frame(76, bytes(33), 1), agent_addr)  # a command from Mission Planner
        self.pump(lambda: v2_frame(76, bytes(33), 1) in self.vehicle_inbox, what="command at the aircraft")
        self.pump(lambda: self.text(app.udp_card.status) == "Mission Planner connected", what="UDP status")
        self.pump(lambda: "0.0 KB/s telemetry" not in self.text(app.craft_values["Traffic"]), what="traffic")

        app.toggle(app.tcp_card, True)
        mp_tcp = socket.create_connection(("127.0.0.1", self.tcp_port), timeout=3)
        self.addCleanup(mp_tcp.close)
        self.pump(lambda: self.text(app.tcp_card.status) == "1 GCS connected", what="TCP client counted")
        self.assertEqual(mp_tcp.recv(100)[:1], b"\xfd")
        self.pump(lambda: app.tcp_card.led.color == mavlte.BLUE, what="TCP Connected LED blue")

        app.toggle(app.udp_card, False)
        self.assertFalse(app.runner.agent.watching)  # TCP still on
        self.pump(lambda: app.udp_card.led.color == mavlte.LED_OFF, what="UDP LED off")
        self.assertEqual(self.text(app.udp_card.status), "Off")
        self.assertEqual(app.udp_card.available.color, mavlte.GREEN)

        app.toggle(app.tcp_card, False)  # both off: only watching again
        self.assertTrue(app.runner.agent.watching)
        self.pump(lambda: self.text(app.craft_state) == "Available: switch TCP or UDP on", what="watching again")
        self.assertEqual(app.craft_led.color, mavlte.GREEN)
        self.pump(lambda: not self.vehicle.gcs_present, what="aircraft holds its telemetry back again")
        self.assertEqual(app.tcp_card.led.color, mavlte.LED_OFF)
        self.assertEqual(app.tcp_card.available.color, mavlte.GREEN)

        saved = mavlte.Settings.load(self.config)
        self.assertEqual((saved.name, saved.key), ("Test UAV", KEY_G.hex()))
        self.assertEqual(saved.udp, "127.0.0.1:%d" % self.udp_port)
        with open(self.config) as f:
            text = f.read()
        self.assertTrue(text.startswith("# my settings"))  # the rest of the file is kept
        self.assertNotIn("udp_on", text)  # switch states of earlier versions are cleaned up

    def test_busy_port_is_reported(self):
        app = self.app
        busy = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        busy.bind(("127.0.0.1", 0))
        busy.listen()
        self.addCleanup(busy.close)
        app.tcp_card.port.set(str(busy.getsockname()[1]))
        app.toggle(app.tcp_card, True)
        self.assertFalse(app.tcp_card.switch.on)
        self.assertIn("in use", self.text(app.tcp_card.status))
        self.assertTrue(app.runner.agent.watching)  # still only watching
        app.tcp_card.port.set("99999")
        app.toggle(app.tcp_card, True)
        self.assertIn("1 to 65535", self.text(app.tcp_card.status))

    def test_udp_port_problems_are_reported(self):
        app = self.app
        busy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        busy.bind(("127.0.0.1", 0))
        self.addCleanup(busy.close)
        app.udp_card.host.set("0.0.0.0")
        app.udp_card.port.set(str(busy.getsockname()[1]))
        app.toggle(app.udp_card, True)
        self.assertFalse(app.udp_card.switch.on)
        self.assertIn(f"UDP port {busy.getsockname()[1]} is in use", self.text(app.udp_card.status))
        app.udp_card.host.set("10.0.0..1")
        app.toggle(app.udp_card, True)
        self.assertFalse(app.udp_card.switch.on)
        self.assertTrue(self.text(app.udp_card.status).startswith("cannot find 10.0.0..1"))
        self.assertTrue(app.runner.agent.watching)

    def test_tcp_typo_is_reported(self):  # the 1.5.2 review: it gave an "Unexpected error" box
        app = self.app
        app.tcp_card.host.set("10.0.0..1")
        app.toggle(app.tcp_card, True)
        self.assertFalse(app.tcp_card.switch.on)
        self.assertTrue(self.text(app.tcp_card.status).startswith("cannot find 10.0.0..1"))
        self.assertTrue(app.runner.agent.watching)

    def test_udp_for_gcs_software_on_other_computers(self):
        app = self.app
        port = free_port()
        app.udp_card.host.set("0.0.0.0")  # as for TCP: it listens (Mission Planner on any computer: UDPCl)
        app.udp_card.port.set(str(port))
        self.pump(lambda: app.udp_card.available.color == mavlte.GREEN, what="aircraft available")
        app.toggle(app.udp_card, True)
        self.assertTrue(app.udp_card.switch.on)
        here = mr.lan_address() or "this computer's address"  # what to type on the other computer
        self.pump(lambda: self.text(app.udp_card.status) == f"Waiting for Mission Planner: UDPCl, {here}, port {port}",
                  what="UDP waits for GCS software")
        planner = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        planner.bind(("127.0.0.1", 0))
        planner.settimeout(3)
        self.addCleanup(planner.close)
        planner.sendto(v2_frame(0, bytes(9), 1), ("127.0.0.1", port))  # its heartbeat: here I am
        self.pump(lambda: self.text(app.udp_card.status) == "1 GCS connected", what="GCS counted")
        data, addr = planner.recvfrom(4096)
        self.assertEqual((data[:1], addr[1]), (b"\xfd", port))
        self.assertEqual(mavlte.Settings.load(self.config).udp, f"0.0.0.0:{port}")

    def test_starts_with_both_switches_off(self):
        # even though the settings file (from an earlier version) says they were on
        self.pump(lambda: self.text(self.app.relay_text).startswith("Connected to the relay"), what="initial state")
        self.assertFalse(self.app.tcp_card.switch.on or self.app.udp_card.switch.on)
        self.assertTrue(self.app.runner.agent.watching)
        self.assertEqual(self.app.root.title(), f"MavLTE V{mr.__version__}")

    def test_available_led_follows_the_aircraft(self):
        app = self.app
        self.pump(lambda: app.udp_card.available.color == mavlte.GREEN, what="available")
        self.relay.ONLINE_TIMEOUT = 0.5  # quicker than the real 3 s
        self.vehicle_quiet = True
        self.loop.call_soon_threadsafe(self.vehicle_task.cancel)  # the LTE module loses power
        self.pump(lambda: app.udp_card.available.color == mavlte.LED_OFF, what="dark once the aircraft is quiet")
        self.assertEqual(app.tcp_card.available.color, mavlte.LED_OFF)
        self.pump(lambda: self.text(app.craft_state).startswith("Offline"), what="aircraft offline")
        app.toggle(app.udp_card, True)  # allowed without the aircraft: its telemetry comes when it does
        self.assertTrue(app.udp_card.switch.on)
        self.pump(lambda: self.text(app.udp_card.status).startswith("Waiting for the aircraft · Mission Planner: UDP"),
                  what="waiting for the aircraft (one line: the window keeps its height)")
        self.assertEqual(app.udp_card.led.color, mavlte.LED_OFF)

    def test_close_keeps_what_others_wrote_meanwhile(self):
        # sitl_demo.py --no-agent writes its relay and key while the app is open
        mr.update_config(self.config, "gcs", {"server": "127.0.0.1:14650", "key": KEY_V.hex()})
        self.app.close()  # as the user closing the window
        saved = mavlte.Settings.load(self.config)
        self.assertEqual((saved.server, saved.key), ("127.0.0.1:14650", KEY_V.hex()))
        self.assertEqual(saved.name, "Test UAV")

    def test_snapshot(self):
        app = self.app
        shown = []
        app.show_photo = lambda path=None: shown.append(path)  # the viewer, without opening a window
        self.pump(lambda: app.camera.enabled, what="Snapshot button ready")
        self.assertEqual(self.text(app.camera.status), "640×480, about 10-30 KB")  # medium unless chosen
        app.camera.size.select(0, clicked=True)
        self.assertEqual(mavlte.Settings.load(self.config).photo_size, "small")  # remembered
        app.snapshot()
        self.pump(lambda: shown, what="the photo in the viewer")
        [path] = shown
        self.assertEqual(os.path.dirname(path), self.photos)
        with open(path, "rb") as f:
            self.assertEqual(f.read(), JPEG)
        with open(path[:-4] + ".json") as f:
            self.assertEqual(json.load(f)["width"], 320)
        self.assertEqual(app.camera.thumbnail.path, path)
        self.assertTrue(self.text(app.camera.last).startswith("Last photo "))
        self.assertIn("120 m", self.text(app.camera.last))
        self.pump(lambda: app.camera.enabled, what="ready for the next")

    def test_snapshot_without_a_camera(self):
        app = self.app
        self.outbox.capture = None
        self.pump(lambda: app.camera.enabled, what="Snapshot button ready")
        app.snapshot()
        self.pump(lambda: self.text(app.camera.status).startswith("No photo"), what="the aircraft's answer")
        self.assertEqual(self.text(app.camera.status), "No photo: " + mr.SNAP_PROBLEMS[mr.SNAP_NO_CAMERA])
        self.assertEqual(app.camera.status.cget("fg"), mavlte.AMBER)

    def test_position_of_the_aircraft(self):
        app = self.app
        value = lambda label: self.text(app.craft_values[label])  # noqa: E731

        def report(**kw):
            fields = dict(gnss_time=int(time.time()), lat=411234567, lon=289876543, alt=150_000, sats=11,
                          fix=mr.FIX_3D, fc_silent=0)
            fields.update(kw)
            self.loop.call_soon_threadsafe(self.vehicle.send_packet, mr.POSITION, mr.Position(**fields).pack())

        self.pump(lambda: app.camera.enabled, what="aircraft online")
        report(fix=mr.FIX_NONE, lat=mr.UNKNOWN_I32, lon=mr.UNKNOWN_I32, sats=3)
        self.pump(lambda: value("Position") == "GNSS searching (3 satellites)", what="searching")
        self.assertEqual(value("Module"), "flight controller talking")
        self.assertEqual(app.map_link.cget("fg"), mavlte.LED_OFF)  # nothing to show on a map yet
        report(flags=mr.POS_FC_SILENT, fc_silent=130, battery_pct=78, battery_mv=3950)  # it came down
        self.pump(lambda: value("Position") == "41.12346, 28.98765 · 11 satellites", what="live position")
        self.assertEqual(value("Module"), "flight controller silent for 2 min · battery 78%, 3.95 V")
        self.assertEqual(app.craft_values["Module"].cget("fg"), mavlte.RED)
        self.assertEqual(app.map_link.cget("fg"), mavlte.BLUE)
        copied, opened = [], []
        with mock.patch.object(app.root, "clipboard_clear"), \
                mock.patch.object(app.root, "clipboard_append", copied.append), \
                mock.patch.object(mavlte.webbrowser, "open", opened.append):
            app.copy_position()
            app.open_google_maps()  # from the moving map (Map)
        self.assertEqual(copied, ["41.123457, 28.987654"])
        self.assertEqual(opened, ["https://www.google.com/maps/search/?api=1&query=41.123457,28.987654"])

        # the aircraft goes quiet too: its last known position stays, in amber
        self.relay.ONLINE_TIMEOUT = 0.5
        self.vehicle_quiet = True
        self.loop.call_soon_threadsafe(self.vehicle_task.cancel)
        self.pump(lambda: value("Position").startswith("last known 41.12346, 28.98765, "), what="last known")
        self.assertEqual(app.craft_values["Position"].cget("fg"), mavlte.AMBER)
        self.assertEqual(value("Module"), "-")

    @unittest.skipUnless(mavlte.Image is not None if HAVE_TK else False, "the moving map needs Pillow")
    def test_moving_map(self):
        app = self.app
        from PIL import Image
        out = io.BytesIO()
        Image.new("RGB", (256, 256), (60, 90, 50)).save(out, "JPEG")
        asked = []
        app.tiles.fetch = lambda url: asked.append(url) or out.getvalue()  # Esri, without the internet

        def report(**kw):
            fields = dict(gnss_time=int(time.time()), lat=411234567, lon=289876543, alt=150_000, sats=11,
                          fix=mr.FIX_3D, fc_silent=0)
            fields.update(kw)
            self.loop.call_soon_threadsafe(self.vehicle.send_packet, mr.POSITION, mr.Position(**fields).pack())

        self.pump(lambda: app.camera.enabled, what="aircraft online")
        report(speed=1500, course=9000)  # flying east
        self.pump(lambda: app.track.newest is not None and app.shown_fix is not None, what="a fix on the track")
        app.open_map()
        win = app.map_window
        win.withdraw()  # drawn all the same, at the size it asks for
        canvas = win.canvas
        kinds = lambda: [canvas.type(item) for item in canvas.find_all()]  # noqa: E731
        self.pump(lambda: "image" in kinds(), what="tiles on the map")
        x, y = maptiles.to_pixel(41.1234567, 28.9876543, 16)
        self.assertIn(f"/World_Imagery/MapServer/tile/16/{int(y // 256)}/{int(x // 256)}", " ".join(asked))
        [arrow] = [item for item in canvas.find_all() if canvas.type(item) == "polygon"]
        x0, y0, x1, y1 = canvas.bbox(arrow)
        middle = (int(canvas.cget("width")) / 2, int(canvas.cget("height")) / 2)  # it follows the aircraft
        self.assertLess(abs((x0 + x1) / 2 - middle[0]) + abs((y0 + y1) / 2 - middle[1]), 12)
        self.assertGreater(x1 - x0, y1 - y0)  # pointing east: wider than tall
        self.assertEqual(canvas.itemcget(arrow, "fill"), mavlte.GREEN)  # live
        self.assertEqual(win.where.cget("text"), self.text(app.craft_values["Position"]))

        second = time.time() + 1.1
        self.pump(lambda: time.time() > second, what="a second: the relay stamps fixes in whole seconds")
        report(lat=411244567)  # 111 m north, standing still
        self.pump(lambda: len(app.track.fixes) == 2, what="the second fix")
        self.pump(lambda: kinds().count("line") == 2 and "oval" in kinds(), what="the track, and a dot")

        win.zoom_by(1)
        self.assertEqual(win.zoom, 17)
        win.drag = (100, 100)
        win._move(SimpleNamespace(x=160, y=140))  # dragged: it stops following
        self.assertFalse(win.following)
        self.assertEqual(str(win.follow_button.cget("style")), "TButton")
        win.follow()
        self.assertEqual((win.following, win.center, str(win.follow_button.cget("style"))), (True, None,
                                                                                            "Accent.TButton"))
        win.destroy()

        opened = []
        with mock.patch.object(mavlte, "Image", None), mock.patch.object(mavlte.webbrowser, "open", opened.append):
            app.open_map()  # without Pillow, which reads the tiles: Google Maps, as before
        self.assertEqual(opened, ["https://www.google.com/maps/search/?api=1&query=41.124457,28.987654"])

    def test_chip_temperature(self):
        app = self.app
        chip = app.chip_temp  # the module's chip, after the rest of the Module row, in a colour of its own

        def report(temp):
            pos = mr.Position(gnss_time=int(time.time()), sats=3, fc_silent=0, battery_pct=78, temp=temp)
            self.loop.call_soon_threadsafe(self.vehicle.send_packet, mr.POSITION, pos.pack())

        self.pump(lambda: app.camera.enabled, what="aircraft online")
        report(45)
        self.pump(lambda: self.text(chip) == " · 45 °C", what="the chip's temperature")
        self.assertEqual(self.text(app.craft_values["Module"]), "flight controller talking · battery 78%")
        self.assertEqual(chip.cget("fg"), mavlte.TEXT)
        for temp, color in ((72, mavlte.AMBER), (85, mavlte.RED)):
            report(temp)
            self.pump(lambda: self.text(chip) == f" · {temp} °C", what=f"{temp} °C")
            self.assertEqual(chip.cget("fg"), color)
        report(mr.TEMP_UNKNOWN)  # an aircraft from before 1.4.0
        self.pump(lambda: self.text(chip) == "", what="no temperature")

    def test_module_power(self):
        app = self.app
        module = app.craft_values["Module"]

        def report(mv, pct):
            pos = mr.Position(gnss_time=int(time.time()), sats=3, fc_silent=0, battery_mv=mv, battery_pct=pct, temp=30)
            self.loop.call_soon_threadsafe(self.vehicle.send_packet, mr.POSITION, pos.pack())

        self.pump(lambda: app.camera.enabled, what="aircraft online")
        report(4298, 100)  # USB or the BEC: the gauge reads the board's supply rail
        self.pump(lambda: self.text(module) == "flight controller talking · external power", what="external power")
        report(4012, 88)  # on its own cell
        self.pump(lambda: self.text(module) == "flight controller talking · battery 88%, 4.01 V", what="on the cell")

    def test_network(self):
        """1.8.8: the network selector beside the bars: the relay's choice; greyed out without the relay. A click
        reaches the aircraft by way of the relay, and the link line tells what the aircraft makes of it."""
        app = self.app
        selector = app.network
        self.vehicle.net_report = mr.NET_AUTO  # as the firmware since 1.8.8

        async def modem():  # the aircraft, as the firmware: set to the network the relay chooses, and says so
            while True:
                await asyncio.sleep(0.05)
                if self.vehicle.network != self.vehicle.net_report & 0x03:
                    await asyncio.sleep(0.5)  # it takes a while to switch
                    self.vehicle.net_report = self.vehicle.network
                    self.vehicle.ping_now()

        self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(modem()))
        self.pump(lambda: app.camera.enabled and selector.enabled, what="aircraft online")
        self.assertEqual(selector.selected, mr.NET_AUTO)
        self.assertEqual([label.cget("text") for label in selector.labels], ["Auto", "2G", "LTE"])

        self.assertFalse(selector.pending)
        selector.select(mr.NET_2G, clicked=True)
        self.assertEqual((selector.selected, selector.pending), (mr.NET_2G, True))  # amber until the aircraft has it
        self.assertEqual(selector.labels[mr.NET_2G].cget("bg"), mavlte.AMBER)
        self.pump(lambda: self.relay.network.mode == mr.NET_2G, what="2G at the relay")
        self.pump(lambda: self.vehicle.net_report == mr.NET_2G and not selector.pending, what="switched")
        self.assertEqual((selector.selected, selector.labels[mr.NET_2G].cget("bg")), (mr.NET_2G, mavlte.ACCENT))

        self.relay.network.mode = mr.NET_AUTO  # chosen elsewhere (the web page)
        self.pump(lambda: selector.selected == mr.NET_AUTO and self.vehicle.net_report == mr.NET_AUTO,
                  what="automatic, as chosen elsewhere")

        def fall_back():  # LTE fails up there: its 2G fallback
            self.vehicle.net_report = mr.NET_AUTO | mr.NET_FALLBACK
            self.vehicle.ping_now()

        self.loop.call_soon_threadsafe(fall_back)
        self.pump(lambda: self.text(app.craft_values["Link"]).endswith(" · LTE failed"), what="fallback")
        self.assertEqual(app.craft_values["Link"].cget("fg"), mavlte.AMBER)
        self.assertEqual((selector.selected, selector.pending), (mr.NET_AUTO, False))

        app._show_network(None, None)  # no relay: greyed out, and a click does nothing
        self.assertEqual((selector.enabled, selector.selected, selector.pending), (False, -1, False))
        selector.select(mr.NET_LTE, clicked=True)
        self.assertEqual(selector.selected, -1)
        self.assertEqual(self.relay.network.mode, mr.NET_AUTO)

    def test_locator_voice(self):
        app = self.app
        row = app.voice
        modem = {"answer": "speaks"}

        async def voice():  # the aircraft, as the firmware: says whether it speaks while the relay asks it to
            while True:
                await asyncio.sleep(0.05)
                flags = {"speaks": mr.PING_FLAG_SPEAKING, "refuses": mr.PING_FLAG_VOICE_FAILED,
                         "before 1.5.0": 0}[modem["answer"]] if self.vehicle.voice_on else 0
                if flags != self.vehicle.ping_flags:
                    self.vehicle.ping_flags = flags
                    self.vehicle.ping_now()

        self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(voice()))
        self.pump(lambda: app.camera.enabled, what="aircraft online")
        self.pump(lambda: self.text(row.status).startswith("Off:"), what="voice off")
        self.assertFalse(row.switch.on)

        app.toggle_voice(True)
        self.assertTrue(row.switch.on)
        self.pump(lambda: self.text(row.status) == "On: the aircraft's speaker is sounding", what="sounding")
        self.assertEqual(row.status.cget("fg"), mavlte.GREEN)
        self.assertTrue(self.relay.voice.on and self.vehicle.voice_on)

        modem["answer"] = "refuses"
        self.pump(lambda: row.status.cget("fg") == mavlte.RED, what="cannot speak")
        self.assertEqual(self.text(row.status), "On, but the aircraft cannot play it")

        app.toggle_voice(False)
        self.pump(lambda: self.text(row.status).startswith("Off:"), what="off again")
        self.assertFalse(row.switch.on or self.relay.voice.on)
        self.pump(lambda: not self.vehicle.voice_on, what="the aircraft told")

        modem["answer"] = "before 1.5.0"  # firmware that knows nothing of the voice
        with mock.patch.object(mavlte.aircraft_card.Voice, "ANSWER_S", 0.5):
            app.toggle_voice(True)
            self.pump(lambda: "before 1.5.0" in self.text(row.status), what="hint at old firmware")

        modem["answer"] = "speaks"
        self.pump(lambda: self.text(row.status) == "On: the aircraft's speaker is sounding", what="sounding again")
        self.vehicle_quiet = True  # the aircraft drops off the air
        self.loop.call_soon_threadsafe(self.vehicle_task.cancel)
        self.pump(lambda: self.text(row.status).startswith("On: it was sounding when last heard"), timeout=10,
                  what="sounding while offline")
        self.assertTrue(row.switch.on)  # the relay keeps the switch

    def logs_window(self, copies):
        self.app.open_logs()
        win = self.app.logs_window
        win.withdraw()
        return win

    def test_flight_logs_over_4g(self):
        """☰ → Flight logs: the files on the aircraft's card, copied through the relay; a copy cut short goes on."""
        app = self.app
        card, copies = tempfile.mkdtemp(), tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, card, True)
        self.addCleanup(shutil.rmtree, copies, True)
        newest = flight_log(400, untimed=20)
        with open(os.path.join(card, "LOG00002.CSV"), "wb") as f:
            f.write(newest)
        with open(os.path.join(card, "LOG00001.CSV"), "wb") as f:
            f.write(flight_log(0, untimed=3))  # no GNSS and no relay that time: no time at all
        files = mr.FileOutbox(self.vehicle, card, cap=lambda: 64 * 1024)
        photos = self.outbox

        def on_packet(ptype, body):
            photos.on_packet(ptype, body)
            files.on_packet(ptype, body)

        async def pump_files():
            self.vehicle.on_packet = on_packet
            while True:
                await asyncio.sleep(0.02)
                files.pump(time.monotonic())

        self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(pump_files()))
        self.pump(lambda: app.camera.enabled, what="aircraft online")
        with mock.patch.object(mavlte, "logs_folder", lambda: copies):
            win = self.logs_window(copies)
            tree = win.tree
            self.pump(lambda: len(tree.get_children()) == 2, what="the list")
            first, second = tree.get_children()
            self.assertEqual([tree.set(first, c) for c in ("file", "length", "here")], ["LOG00002.CSV", "6 min", ""])
            start = 1790933700 - 20  # reckoned back to its first line, from before the board knew the time
            self.assertEqual(tree.set(first, "started"), time.strftime("%a %d %b %Y  %H:%M", time.localtime(start)))
            self.assertEqual(tree.set(second, "started"), "time unknown")
            self.assertEqual(win.status.cget("text"), "2 files on the card")

            tree.selection_set(first)
            win.download_selected()
            path = os.path.join(copies, mavlte.log_copy_name("LOG00002.CSV", start))
            self.assertTrue(os.path.basename(path).endswith(" LOG00002.csv"))
            self.pump(lambda: win.download is None and os.path.exists(path), timeout=15, what="the copy")
            with open(path, "rb") as f:
                self.assertEqual(f.read(), newest)
            self.assertEqual(tree.set(tree.get_children()[0], "here"), "saved")
            self.assertIn("LOG00002.CSV saved", win.status.cget("text"))

            with open(path, "r+b") as f:  # a copy cut short: Download goes on from where it ends
                f.truncate(10000)
            win._fill()
            self.assertEqual(tree.set(tree.get_children()[0], "here"), f"{10000 * 100 // len(newest)} %")
            tree.selection_set(tree.get_children()[0])
            win.download_selected()
            self.pump(lambda: win.download is None, timeout=15, what="the rest")
            with open(path, "rb") as f:
                self.assertEqual(f.read(), newest)
            self.assertIn(", ", win.status.cget("text"))  # "... KB new"
            win.destroy()

    def test_flight_logs_over_usb(self):
        app = self.app
        copies = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, copies, True)
        data = flight_log(300)
        board = FakeBoard(files={"LOG00003.CSV": data})
        board.times["LOG00003.CSV"] = (1790933700, 1790933999)
        found = []
        real_link = boardusb.BoardLink
        with mock.patch.object(mavlte, "logs_folder", lambda: copies), \
                mock.patch.object(boardusb, "board_ports", lambda: list(found)), \
                mock.patch.object(boardusb, "serial", object()), \
                mock.patch.object(boardusb, "BoardLink", lambda port: real_link(port, opener=lambda: board)):
            win = self.logs_window(copies)
            win.use(1)  # the USB cable: no board on any port yet
            self.pump(lambda: win.status.cget("text") == mavlte.NO_BOARD, what="no board")
            found.append("COM99")  # plugged in
            win.refresh()
            self.pump(lambda: len(win.tree.get_children()) == 1, what="the list over USB")
            self.assertIn("COM99, board firmware 1.8.0", win.where.cget("text"))
            self.assertEqual(win.tree.set(win.tree.get_children()[0], "length"), "4 min")
            win.download_selected()  # nothing selected: the newest
            path = os.path.join(copies, mavlte.log_copy_name("LOG00003.CSV", 1790933700))
            self.pump(lambda: win.download is None and os.path.exists(path), what="the copy over USB")
            with open(path, "rb") as f:
                self.assertEqual(f.read(), data)
            win.use(0)  # back to 4G: the port is closed
            self.pump(lambda: board.closed, what="the port closed")
            win.destroy()

    def test_log_window(self):
        """☰ → Show log opens the log in a window of its own; Hide log, or its own ✕, closes it, and the menu says
        which (with the log inside the main window, a maximized MavLTE had no room for it and the menu kept
        saying Hide log)."""
        app = self.app
        label = lambda: app.menu.entrycget(1, "label")  # noqa: E731
        self.root.state("zoomed")  # maximized, as the user had it
        self.pump(lambda: app.log_history, what="something logged")
        self.assertEqual(label(), "Show log")
        app.toggle_log()
        win = app.log_window
        win.withdraw()
        self.assertEqual(label(), "Hide log")
        text = lambda: win.text.get("1.0", "end")  # noqa: E731
        self.assertIn(app.log_history[0], text())  # what came before it opened
        mavlte.log.info("a line while the window is open")
        self.pump(lambda: "a line while the window is open" in text(), what="the new line in the window")
        app.toggle_log()
        self.assertFalse(app.log_shown())
        self.assertEqual(label(), "Show log")
        app.toggle_log()
        app.log_window.destroy()  # its own ✕
        app._menu_labels()  # as the menu does when it opens
        self.assertEqual(label(), "Show log")
        mavlte.log.info("a line while it is closed")
        self.pump(lambda: any("a line while it is closed" in line for line in app.log_history),
                  what="kept while closed")
        app.toggle_log()
        app.log_window.withdraw()
        self.assertIn("a line while it is closed", app.log_window.text.get("1.0", "end"))

    def test_wrong_key_says_no_answer(self):
        app = self.app
        app.apply_settings(app.settings.name, app.settings.server, KEY_V.hex())  # not the GCS key: no answer
        self.pump(lambda: self.text(app.relay_text).startswith("No answer from the relay"), timeout=10,
                  what="no-answer message")

    def test_start_up_set_aside_and_another_relay_frees_the_old_connection(self):
        """1.8.5: the collections every 5 s skip what start-up made (set aside: some 0.2 ms instead of 7-9 ms, which
        the agent's thread waited too); another relay or key frees the old connection, set aside with the rest, and
        sets aside anew once it is gone."""
        app = self.app
        self.addCleanup(gc.unfreeze)  # (the test process goes on as before)
        self.pump(lambda: self.text(app.relay_text).startswith("Connected to the relay"), what="connected")
        old = weakref.ref(app.runner.agent)
        mavlte.set_aside()
        self.assertGreater(gc.get_freeze_count(), 0)
        app.apply_settings(app.settings.name, app.settings.server, KEY_V.hex())  # another key: a new connection
        self.assertEqual(gc.get_freeze_count(), 0)  # the old one is collected again
        self.assertIsNotNone(app.set_aside_at)
        self.pump(lambda: self.text(app.relay_text).startswith("No answer from the relay"), timeout=10,
                  what="the new connection under way")
        app.gc_at = app.set_aside_at = time.monotonic() - 10  # both due
        app._poll()
        self.assertIsNone(old())  # gone, reference cycles and all
        self.assertGreater(gc.get_freeze_count(), 0)  # set aside anew
        self.assertIsNone(app.set_aside_at)


@unittest.skipUnless(HAVE_TK, "needs Tk and a display")
class PlaceOnScreenTest(unittest.TestCase):
    """1.8.6: MavLTE is never taller than the screen above the taskbar (a 1080p screen at 125%: 1020 pixels, of which
    Windows' title bar and frame take 39): its content gives way at the bottom margins, as when maximized."""

    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.root.minsize(450, 525)  # MavLTE's, at 125%
        tk.Frame(self.root, width=600, height=1002).pack()  # what MavLTE asks there

    def place(self, area, frame=39):
        with mock.patch.object(mavlte, "work_area", return_value=area), \
                mock.patch.object(mavlte, "frame_height", return_value=frame):
            mavlte.place_on_screen(self.root, 1.25)

    def test_held_to_the_room_above_the_taskbar_and_opened_at_the_top(self):
        self.place((0, 0, 1920, 1020))
        self.assertEqual(self.root.maxsize()[1], 981)
        self.assertTrue(self.root.geometry().endswith("+30+0"), self.root.geometry())

    def test_room_to_spare_changes_nothing(self):
        self.place((0, 0, 2560, 1400))
        self.assertEqual(self.root.maxsize()[1], 1361)  # more than it asks
        self.assertFalse(self.root.geometry().endswith("+30+0"))  # where Windows puts it

    def test_a_tiny_screen_keeps_the_smallest_size(self):
        self.place((0, 0, 800, 480))
        self.assertEqual(self.root.maxsize()[1], 525)

    def test_not_windows_or_no_metrics(self):
        with mock.patch.object(self.root, "maxsize", wraps=self.root.maxsize) as maxsize:
            self.place(None)  # not Windows
            self.place((0, 0, 1920, 1020), frame=None)  # no title bar metrics: not held, still opened at the top
        self.assertEqual([c for c in maxsize.call_args_list if c.args], [])  # (Tk's own maximum stays)
        self.assertTrue(self.root.geometry().endswith("+30+0"))


@unittest.skipUnless("mavlte" in sys.modules, "needs tkinter")
class SettingsFileTest(unittest.TestCase):
    """Where MavLTE.exe keeps its settings (sys.frozen and sys.executable as PyInstaller sets them)."""

    def test_exe_settings(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        exe = os.path.join(tmp.name, "MavLTE", "MavLTE.exe")
        os.makedirs(os.path.dirname(exe))
        local = os.path.join(tmp.name, "AppData", "Local")
        with mock.patch.object(sys, "frozen", True, create=True), mock.patch.object(sys, "executable", exe), \
                mock.patch.dict(os.environ, {"LOCALAPPDATA": local}):
            path = mavlte.settings_file()
            self.assertEqual(path, os.path.join(local, "MavLTE", "mavrelay.ini"))
            mavlte.Settings(server="relay.example.com:14650", key=KEY_G.hex()).save(path)  # makes the folder
            self.assertEqual(mavlte.Settings.load(path).server, "relay.example.com:14650")

            beside = os.path.join(os.path.dirname(exe), "mavrelay.ini")
            open(beside, "w").close()
            self.assertEqual(mavlte.settings_file(), beside)  # a file next to the exe wins
        self.assertEqual(mavlte.settings_file(), os.path.join(mavlte.HERE, "mavrelay.ini"))  # from source

    def test_vehicle_names(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "mavrelay.ini")
        mavlte.Settings(name="33% Cub", server="relay.example.com:14650", key=KEY_G.hex()).save(path)
        self.assertEqual(mavlte.Settings.load(path).name, "33% Cub")  # a % once stopped the app from starting
        for fine in ("33% Cub", "UAV#2", "Talon; blue"):
            self.assertIsNone(mavlte.name_problem(fine), fine)
        for cut_short in ("UAV #2", "#1 UAV", "Plane ;blue"):  # the INI file would read a comment there
            self.assertIsNotNone(mavlte.name_problem(cut_short), cut_short)
        for broken in ("Talon\nblue", "Talon\tblue"):  # pasted with a line break: the file would not read again
            self.assertIsNotNone(mavlte.name_problem(broken), repr(broken))


class LogCopiesTest(unittest.TestCase):
    def test_copy_names(self):
        self.assertEqual(mavlte.log_copy_name("LOG00012.CSV", 0), "LOG00012.csv")
        self.assertEqual(mavlte.log_copy_name("..\\..\\x.CSV", 0), "______x.csv")  # never a path
        self.assertTrue(mavlte.log_copy_name("LOG00012.CSV", 1790000000).endswith(" LOG00012.csv"))

    def test_a_larger_copy_of_the_same_name_is_another_file(self):
        # a card formatted since, or another board: its LOG00012.CSV must not go on at the end of the old copy
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        with mock.patch.object(mavlte, "logs_folder", lambda: tmp.name):
            with open(os.path.join(tmp.name, "LOG00012.csv"), "wb") as f:
                f.write(b"x" * 5000)
            path, have = mavlte.LogsWindow._copy(None, ("LOG00012.CSV", 3000, 0, 0))
            self.assertEqual((os.path.basename(path), have), ("LOG00012 (2).csv", 0))
            path, have = mavlte.LogsWindow._copy(None, ("LOG00012.CSV", 8000, 0, 0))  # the same one, grown
            self.assertEqual((os.path.basename(path), have), ("LOG00012.csv", 5000))

    def test_a_commented_header_survives_saving(self):
        """The 1.5.2 review: "[gcs]  # ..." got a second [gcs] on saving, and MavLTE did not start again."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "mavrelay.ini")
        with open(path, "w") as f:
            f.write("[gcs]  # laptop agent\nserver = relay.example.com:14650\nkey = %s\n" % KEY_G.hex())
        settings = mavlte.Settings.load(path)
        settings.tcp = "127.0.0.1:5761"
        settings.save(path, ("tcp", "udp"))  # as closing the app does
        again = mavlte.Settings.load(path)
        self.assertEqual((again.server, again.tcp, again.problem), ("relay.example.com:14650", "127.0.0.1:5761", ""))
        with open(path) as f:
            self.assertEqual(f.read().count("[gcs]"), 1)
        with open(path, "a") as f:  # a file 1.5.2 had already spoilt: read all the same
            f.write("\n[gcs]\nudp = 127.0.0.1:14551\n")
        self.assertEqual(mavlte.Settings.load(path).udp, "127.0.0.1:14551")

    def test_an_unreadable_file_does_not_stop_the_app(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "mavrelay.ini")
        with open(path, "wb") as f:
            f.write("[gcs]\nname = Kuş\nserver = relay.example.com:14650\n".encode("cp1254"))  # Turkish ANSI
        settings = mavlte.Settings.load(path)
        self.assertEqual(settings.server, "")
        self.assertIn("cannot read config file", settings.problem)
        root = tk.Tk()
        root.withdraw()
        try:
            with mock.patch.object(mavlte.messagebox, "showwarning") as warn, \
                    mock.patch.object(mavlte.App, "open_settings"):
                app = mavlte.App(root, config_path=path)  # it starts, says why, and closes
                end = time.time() + 2
                while not warn.called and time.time() < end:
                    root.update()
                    time.sleep(0.02)
                app.close()
            self.assertIn("cannot read config file", warn.call_args[0][1])
        finally:
            try:
                root.destroy()
            except tk.TclError:
                pass
            gc.collect()


@unittest.skipUnless("mavlte" in sys.modules, "needs tkinter")
class PhotoFilesTest(unittest.TestCase):
    def test_photos_in_the_folder_and_their_captions(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        for name in ("MavLTE_2026-09-30_10-00-00_1790000000.jpg", "MavLTE_2026-09-29_23-59-59_990.jpg",
                     "MavLTE_x.jpg", "other.jpg", "MavLTE_2026-09-30_10-00-00_1790000000.json"):
            open(os.path.join(tmp.name, name), "w").close()
        found = [os.path.basename(p) for p in mavlte.photo_files(tmp.name)]
        self.assertEqual(found, ["MavLTE_2026-09-29_23-59-59_990.jpg", "MavLTE_2026-09-30_10-00-00_1790000000.jpg"])
        self.assertEqual(mavlte.photo_files(os.path.join(tmp.name, "none")), [])

        info = mr.PhotoInfo(1790000000, 20480, 640, 480, 411234567, 289876543, 120_000, 4500, mr.SNAP_OK, 1790000003)
        meta = dict(info._asdict(), latitude=41.1234567, longitude=28.9876543, altitude_m=120.0)
        caption = mavlte.photo_caption(meta)
        self.assertIn("41.123457, 28.987654", caption)
        self.assertIn("120 m above home", caption)
        self.assertIn("heading 45°", caption)
        self.assertIn("640×480, 20 KB", caption)
        self.assertIn(time.strftime("%H:%M:%S", time.localtime(1790000003)), caption)
        short = mavlte.photo_caption(dict(mr.PhotoInfo(1790000000, 100, 320, 240, time=1790000003)._asdict()),
                                     short=True)
        self.assertEqual(short, time.strftime("%H:%M:%S", time.localtime(1790000003)))  # nothing else known
        self.assertEqual(mavlte.photo_meta(os.path.join(tmp.name, "other.jpg")), {})

    def test_photo_folder_setting(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = os.path.join(tmp.name, "mavrelay.ini")
        settings = mavlte.Settings.load(path)
        self.assertEqual((settings.photo_size, settings.photo_dir), ("medium", ""))
        self.assertEqual(os.path.basename(settings.photo_folder()), "MavLTE")
        with open(path, "w") as f:
            f.write("[gcs]\nphoto_dir = %s\nphoto_size = LARGE\n" % tmp.name)
        settings = mavlte.Settings.load(path)
        self.assertEqual((settings.photo_folder(), settings.photo_size), (tmp.name, "large"))


if __name__ == "__main__":
    unittest.main()

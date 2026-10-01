"""The plane simulator: its aircraft (battery, LTE module, network) against a real relay, with a fake
flight controller in place of SITL, and its window. Skipped where tkinter is missing."""

import asyncio
import gc
import io
import json
import math
import os
import shutil
import tempfile
import random
import struct
import subprocess
import sys
import threading
import time
import unittest
from unittest import mock

RELAY_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, RELAY_DIR)
sys.path.insert(0, os.path.dirname(__file__))

import mavrelay as mr  # noqa: E402
from test_mavrelay import KEY_G, KEY_V, v2_frame  # noqa: E402

try:
    import plane_sim
    import mavlte as ui
except ImportError:  # no tkinter
    plane_sim = None

try:
    import tkinter as tk

    _root = tk.Tk()
    _root.destroy()
    del _root
    HAVE_DISPLAY = plane_sim is not None
except Exception:  # no tkinter, or no display
    HAVE_DISPLAY = False


def fc_telemetry(seq: int) -> bytes:
    """What SITL's SERIAL1 sends: HEARTBEAT (FBWA, armed), SYS_STATUS (12.6 V), ATTITUDE (rolled 10 degrees
    right, nose 3 degrees down), GLOBAL_POSITION_INT (over Istanbul, 120 m above home, heading 45)."""
    heartbeat = struct.pack("<IBBBBB", 5, 1, 3, 0x81, 4, 3)
    sys_status = bytes(14) + struct.pack("<H", 12600) + bytes(15)
    attitude = struct.pack("<Iffffff", 0, math.radians(10), math.radians(-3), 0.8, 0, 0, 0)
    position = struct.pack("<IiiiihhhH", 0, 411234567, 289876543, 170_000, 120_000, 0, 0, 0, 4500)
    return (v2_frame(0, heartbeat, seq) + v2_frame(1, sys_status, seq + 1) + v2_frame(30, attitude, seq + 2)
            + v2_frame(33, position, seq + 3))


class Ground:
    """A relay, a GCS session on it and a fake flight controller, on their own asyncio loop."""

    def __init__(self) -> None:
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.gcs_inbox = []
        self.fc_inbox = bytearray()
        self.fc_clients = []
        self.relay_port, self.fc_port = self.run(self._start())

    def run(self, coro, timeout=5.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    async def _start(self):
        transport, _ = await self.loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", 0))
        relay_port = transport.get_extra_info("sockname")[1]
        self.gcs = mr.TunnelClient(mr.ROLE_GCS, KEY_G, "127.0.0.1", relay_port, on_data=self.gcs_inbox.append)
        asyncio.ensure_future(self.gcs.run())
        asyncio.ensure_future(self._ticks())
        self.fc_server = await asyncio.start_server(self._fc_client, "127.0.0.1", 0)
        return relay_port, self.fc_server.sockets[0].getsockname()[1]

    async def _ticks(self):
        while True:
            await asyncio.sleep(0.1)
            now = time.monotonic()
            self.relay.tick(now)
            self.relay.photos.pump(now)

    async def _fc_client(self, reader, writer):  # plays SITL's SERIAL1
        self.fc_clients.append(writer)

        async def talk():
            seq = 0
            while True:
                writer.write(fc_telemetry(seq))
                seq += 4
                await asyncio.sleep(0.05)

        task = asyncio.ensure_future(talk())
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                self.fc_inbox += data
        except OSError:
            pass
        finally:
            task.cancel()
            self.fc_clients.remove(writer)
            writer.close()

    def telemetry_at_gcs(self) -> bool:
        return b"\xfd\x09\x00\x00" in b"".join(list(self.gcs_inbox))  # a HEARTBEAT frame

    def close(self) -> None:
        async def stop():
            self.fc_server.close()
            self.relay.transport.close()
            tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        self.run(stop())
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()


def wait_for(test, cond, what, timeout=8.0, pump=None):
    end = time.time() + timeout
    while not cond():
        if time.time() > end:
            test.fail(f"{what} not met in time")
        if pump is not None:
            pump()
        time.sleep(0.02)


@unittest.skipUnless(plane_sim, "needs tkinter")
class PlaneTest(unittest.TestCase):
    def setUp(self):
        for name, value in (("STARTUP_QUICK", (0.05, 0.05, 0.05)), ("REGISTER_AGAIN_S", 0.2), ("RAT_SWITCH_S", 0.2),
                            ("EVENTS", {}),  # the link's random troubles have tests of their own
                            ("GNSS_TTFF_QUICK", 0.5), ("LOCATOR_INTERVAL", 0.2)):
            patcher = mock.patch.object(plane_sim, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.ground = Ground()
        self.addCleanup(self.ground.close)
        self.plane = plane_sim.Plane(("127.0.0.1", self.ground.relay_port), KEY_V, fc_port=self.ground.fc_port)
        self.plane.quick = True
        self.addCleanup(self.plane.close)

    def session(self) -> int:
        m = self.plane.modem
        return m.client.session if m is not None and m.client is not None else 0

    def test_battery_lte_module_and_network(self):
        p, g = self.plane, self.ground
        p.call(p.set_lte, True)
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.fc.mode == "FBWA", "heartbeat from the flight controller")
        self.assertTrue(p.fc.armed)
        self.assertAlmostEqual(p.fc.volts, 12.6)
        self.assertAlmostEqual(p.fc.alt, 120.0)

        # the module is switched on, so it starts with the battery and carries the telemetry
        wait_for(self, self.session, "relay session")
        first = self.session()
        wait_for(self, g.telemetry_at_gcs, "telemetry at the GCS")
        good_dbm = plane_sim.SIGNALS[plane_sim.GOOD_SIGNAL][1]
        wait_for(self, lambda: g.relay.vehicle.rssi_dbm == good_dbm, "signal reported to the relay")
        command = v2_frame(76, bytes(33), 9)
        g.loop.call_soon_threadsafe(g.gcs.send_data, command)
        wait_for(self, lambda: command in bytes(g.fc_inbox), "command at the flight controller")

        # module off: the relay just stops hearing it; no goodbye
        p.call(p.set_lte, False)
        wait_for(self, lambda: p.modem is None, "module off")
        time.sleep(0.3)
        heard = g.relay.vehicle.last_rx
        time.sleep(0.5)
        self.assertEqual(g.relay.vehicle.last_rx, heard)

        # on again: a new session, as the modem gets a new address
        p.call(p.set_lte, True)
        wait_for(self, lambda: self.session() not in (0, first), "new relay session")

        # the network and signal the module reports reach the relay (and MavLTE)
        self.assertEqual(g.relay.vehicle.rat, 7)  # LTE
        p.call(p.set_signal, 0)
        wait_for(self, lambda: g.relay.vehicle.rssi_dbm == plane_sim.SIGNALS[0][1], "weak signal at the relay")
        p.call(p.set_network, plane_sim.NET_2G)
        wait_for(self, lambda: g.relay.vehicle.rat == 3, "EDGE at the relay")
        p.call(p.set_signal, 3)
        # no coverage: nothing gets through (once what was sent before has arrived: up to 0.53 s on weak 2G)
        p.call(p.set_network, plane_sim.NO_CONNECTION)
        time.sleep(1.0)
        heard = g.relay.vehicle.last_rx
        time.sleep(0.6)
        self.assertEqual(g.relay.vehicle.last_rx, heard)
        p.call(p.set_network, plane_sim.NET_LTE)
        wait_for(self, lambda: g.relay.vehicle.last_rx > heard, "heard again with coverage")

        # battery off: flight controller and module lose power together
        p.call(p.set_battery, False)
        wait_for(self, lambda: not g.fc_clients, "flight controller link closed")
        self.assertIsNone(p.modem)

    def test_no_signal_no_registration(self):
        p = self.plane
        p.call(p.set_network, plane_sim.NO_CONNECTION)
        p.call(p.set_lte, True)
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.modem is not None and p.modem.stage == "searching", "searching")
        time.sleep(0.4)
        self.assertEqual(p.modem.stage, "searching")  # nothing to register with
        p.call(p.set_network, plane_sim.NET_2G)
        wait_for(self, self.session, "relay session once there is coverage")

    def test_mistyped_relay_host_keeps_the_module_trying(self):
        plane = plane_sim.Plane(("10.0.0..1", self.ground.relay_port), KEY_V, fc_port=self.ground.fc_port)
        plane.quick = True
        self.addCleanup(plane.close)
        with self.assertLogs("mavrelay.plane", "WARNING") as logs:
            plane.call(plane.set_lte, True)
            plane.call(plane.set_battery, True)
            wait_for(self, lambda: any("cannot look up" in line for line in logs.output), "lookup retried")
        self.assertFalse(plane.modem_task.done())  # an empty label raises UnicodeError, not OSError

    @unittest.skipUnless(plane_sim is not None and plane_sim.Image is not None, "needs Pillow")
    def test_snapshot(self):
        p, g = self.plane, self.ground
        folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, folder, True)
        saved = []
        inbox = mr.PhotoInbox(g.gcs, folder, on_photo=lambda path, info: saved.append((path, info)))

        async def attach():  # the ground's GCS becomes a photo-taking MavLTE
            g.gcs.on_packet = inbox.on_packet

            async def pump():
                while True:
                    await asyncio.sleep(0.05)
                    inbox.pump(time.monotonic())

            asyncio.ensure_future(pump())

        g.run(attach())
        p.call(p.set_lte, True)
        p.call(p.set_battery, True)
        wait_for(self, self.session, "relay session")
        wait_for(self, lambda: p.fc.lat != mr.UNKNOWN_I32, "position from the flight controller")
        self.assertEqual((p.fc.lat, p.fc.lon, p.fc.heading), (411234567, 289876543, 4500))
        self.assertAlmostEqual(p.fc.roll, 10.0, places=3)
        self.assertAlmostEqual(p.fc.pitch, -3.0, places=3)
        wait_for(self, lambda: g.relay.vehicle is not None, "the aircraft online at the relay")
        g.loop.call_soon_threadsafe(inbox.request, 1)
        wait_for(self, lambda: saved or inbox.problem, "the photo at the GCS", timeout=15)
        self.assertEqual(inbox.problem, "")
        path, info = saved[0]
        self.assertEqual((info.width, info.height, info.lat, info.heading), (640, 480, 411234567, 4500))
        with plane_sim.Image.open(path) as img:
            self.assertEqual(img.size, (640, 480))
        with open(path[:-4] + ".json") as f:
            self.assertEqual(json.load(f)["altitude_m"], 120.0)
        wait_for(self, lambda: p.modem.photos.last is not None, "the plane knows it was sent")
        self.assertTrue(p.modem.photos.last[1])

        p.call(p.set_camera, False)  # the CAM DIP switch off
        g.loop.call_soon_threadsafe(inbox.request, 0)
        wait_for(self, lambda: inbox.problem, "the answer")
        self.assertEqual(inbox.problem, mr.SNAP_PROBLEMS[mr.SNAP_NO_CAMERA])

    def test_locator_after_a_crash(self):
        """The flight battery goes (a crash): the LTE module runs on in its cell and keeps reporting where
        the plane is, and that the flight controller is silent."""
        p, g = self.plane, self.ground
        positions = []

        async def attach():
            g.gcs.on_packet = lambda ptype, body: positions.append(mr.Position.unpack(body)) if ptype == mr.POSITION \
                else None

        g.run(attach())
        silent = mock.patch.object(mr, "FC_SILENT_S", 1)
        silent.start()
        self.addCleanup(silent.stop)
        p.call(p.set_cell, True)
        p.call(p.set_lte, True)
        p.call(p.set_battery, True)
        wait_for(self, lambda: any(pos.has_fix for pos in positions), "a GNSS fix at the ground station")
        pos = [pos for pos in positions if pos.has_fix][-1]
        self.assertLess(abs(pos.lat - 411234567) + abs(pos.lon - 289876543), 2000)  # within a few metres
        self.assertFalse(pos.fc_is_silent)
        self.assertEqual(pos.battery_pct, 100)
        self.assertTrue(plane_sim.AIR_C <= pos.temp <= plane_sim.AIR_C + plane_sim.SELF_HEAT_C)  # its chip

        p.call(p.set_battery, False)
        wait_for(self, lambda: not g.fc_clients, "flight controller dead")
        wait_for(self, lambda: positions[-1].fc_is_silent, "the module reports the silence")
        last = positions[-1]
        self.assertTrue(last.has_fix)
        self.assertLess(abs(last.lat - 411234567) + abs(last.lon - 289876543), 2000)  # where it came down
        self.assertEqual(last.speed, 0)
        self.assertIsNotNone(p.modem)  # on its cell

        p.call(p.set_cell, False)  # the cell out as well: now it is gone
        wait_for(self, lambda: p.modem is None, "module off")

    def test_locator_voice(self):
        """MavLTE's Locator voice switch, through the relay: the module speaks, also where it has no coverage,
        until the switch is off again."""
        p, g = self.plane, self.ground

        async def switch(on):
            g.gcs.send_packet(mr.VOICE, mr.VOICE_BODY.pack(1 if on else 0))

        p.call(p.set_cell, True)  # down, on its cell
        p.call(p.set_lte, True)
        wait_for(self, lambda: self.session() != 0, "module connected")
        self.assertFalse(p.modem.voice)
        with self.assertLogs("mavrelay.plane", "INFO") as logs:
            g.run(switch(True))
            wait_for(self, lambda: p.modem.voice, "the module speaks")
        self.assertIn("locator voice on", " ".join(logs.output))
        wait_for(self, lambda: g.relay.vehicle.speaking, "the relay hears that it speaks")

        p.call(p.set_network, plane_sim.NO_CONNECTION)  # in a valley: no coverage
        time.sleep(1.5)
        self.assertTrue(p.modem.voice)  # it goes on speaking
        p.call(p.set_network, plane_sim.NET_LTE)
        g.run(switch(False))
        wait_for(self, lambda: not p.modem.voice, "the module stops speaking", timeout=15)
        wait_for(self, lambda: not g.relay.vehicle.speaking, "the relay hears that it stopped")

    def test_chip_temperature(self):
        """The module's chip: the air in the fuselage, plus its own heat while it runs, followed with a lag;
        in the sun hot enough for MavLTE's amber, then red."""
        p = self.plane
        p._chip_update()
        self.assertAlmostEqual(p.chip_c, plane_sim.AIR_C, delta=0.1)  # a cold board, in the shade
        p.modem = mock.Mock()  # running, as far as its heat goes
        self.addCleanup(setattr, p, "modem", None)
        p.chip_at -= 10 * plane_sim.CHIP_LAG_S
        p._chip_update()
        self.assertAlmostEqual(p.chip_c, plane_sim.AIR_C + plane_sim.SELF_HEAT_C, delta=0.1)
        self.assertLess(p.chip_c, mr.TEMP_WARM)
        p.set_sun(True)
        p.chip_at -= plane_sim.CHIP_LAG_S  # a lag later: most of the way
        p._chip_update()
        self.assertTrue(mr.TEMP_WARM <= p.chip_c < mr.TEMP_HOT, p.chip_c)
        p.chip_at -= 10 * plane_sim.CHIP_LAG_S
        p._chip_update()
        self.assertGreaterEqual(p.chip_c, mr.TEMP_HOT)

    def test_module_starts_switched_off(self):
        p = self.plane
        self.assertFalse(p.lte)
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.fc.mode == "FBWA", "flight controller up")
        time.sleep(0.3)
        self.assertIsNone(p.modem)


@unittest.skipUnless(plane_sim is not None and plane_sim.Image is not None, "needs tkinter and Pillow")
class CameraTest(unittest.TestCase):
    def test_pictures_are_photo_sized(self):
        fc = plane_sim.FcState()
        fc.feed(fc_telemetry(0), time.monotonic())
        # the sizes a JPEG from the board's OV5640 has (bright, detailed scenes at the upper end)
        for (width, height), (low, high) in zip(mr.SNAP_SIZES, ((4, 12), (10, 35), (25, 90))):
            jpeg = plane_sim.camera_picture(width, height, fc)
            self.assertTrue(low * 1024 <= len(jpeg) <= high * 1024, f"{width}x{height}: {len(jpeg)} bytes")
            with plane_sim.Image.open(io.BytesIO(jpeg)) as img:
                self.assertEqual((img.format, img.size), ("JPEG", (width, height)))

    def test_without_a_position(self):
        jpeg = plane_sim.camera_picture(320, 240, plane_sim.FcState())  # the flight controller not heard yet
        self.assertEqual(jpeg[:2], b"\xff\xd8")


@unittest.skipUnless(plane_sim, "needs tkinter")
class NetworkTest(unittest.TestCase):
    """The mobile network model on its own, with exact figures."""

    def setUp(self):
        patcher = mock.patch.object(plane_sim, "EVENTS", {})
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_speed_queue_and_order(self):
        net = plane_sim.Network(("127.0.0.1", 9), plane_sim.NET_LTE, 3)
        net.figures = (1000.0, 1000.0, 0.1, 0.05, 0.0)  # 1000 bytes/s each way, 100 ms ± 50, no loss
        size = 500 - 28  # 500 bytes on the air with the IP and UDP headers
        timings = [net.schedule(True, size, 0.0) for _ in range(6)]
        self.assertIsNone(timings[5])  # 2.5 s of queue: more than the network holds
        for k, (sent, arrival) in enumerate(timings[:5]):  # each waits for those before it, then travels
            self.assertAlmostEqual(sent, 0.5 * (k + 1))
            self.assertAlmostEqual(arrival, sent + 0.1, delta=0.05 + 1e-9)
        arrivals = [arrival for _, arrival in timings[:5]]
        self.assertEqual(arrivals, sorted(arrivals))  # in order despite the jitter
        self.assertAlmostEqual(net.schedule(False, size, 0.0)[1], 0.6, delta=0.05 + 1e-9)  # downlink: own queue

    def test_queue_is_lost_with_the_coverage(self):
        net = plane_sim.Network(("127.0.0.1", 9), plane_sim.NET_LTE, 3)
        net.figures = (1000.0, 1000.0, 0.1, 0.0, 0.0)
        first, _ = net.schedule(True, 472, 0.0)  # on the air until 0.5 s
        second, _ = net.schedule(True, 472, 0.0)  # waits its turn, sent at 1.0 s
        net.set_network(plane_sim.NO_CONNECTION, 3, now=0.7)  # the coverage goes in between

        class Transport:
            out = []

            def sendto(self, data, addr):
                self.out.append(data)

        transport = Transport()
        net._send(transport, b"first", ("relay", 1), 0.0, first)
        net._send(transport, b"second", ("relay", 1), 0.0, second)
        self.assertEqual(transport.out, [b"first"])  # sent before: arrives; still queued: lost
        self.assertEqual(net.lost, 1)

    def test_no_coverage_and_network_change(self):
        net = plane_sim.Network(("127.0.0.1", 9), plane_sim.NET_2G, 3)
        self.assertIsNotNone(net.schedule(True, 100, time.monotonic()))
        net.set_network(plane_sim.NET_LTE, 3, gap=0.5)  # moving from 2G to LTE
        self.assertIsNone(net.schedule(True, 100, time.monotonic()))
        self.assertIsNotNone(net.schedule(True, 100, time.monotonic() + 0.6))
        net.set_network(plane_sim.NO_CONNECTION, 3)
        self.assertIsNone(net.schedule(True, 100, time.monotonic() + 1.0))

    def test_figures(self):
        up, down, delay, jitter, loss = plane_sim.link_figures(plane_sim.NET_2G, 0)  # weak 2G
        self.assertAlmostEqual(up * 8 / 1000, 10)  # kbit/s: far slower than the telemetry (about 22)
        self.assertAlmostEqual(delay, 0.38)
        fair_2g = plane_sim.link_figures(plane_sim.NET_2G, 1)[0] * 8 / 1000
        self.assertLess(fair_2g, 22)  # from a fair signal down, 2G cannot carry the telemetry
        self.assertIsNone(plane_sim.link_figures(plane_sim.NO_CONNECTION, 3))
        self.assertEqual(plane_sim.speed_text(plane_sim.link_figures(plane_sim.NET_LTE, 3)[0]), "2 Mbit/s")


@unittest.skipUnless(plane_sim, "needs tkinter")
class EventsTest(unittest.TestCase):
    """The link's random troubles, with a fixed random seed."""

    def run_events(self, table, signal, seconds, seed=1):
        with mock.patch.object(plane_sim, "EVENTS", {plane_sim.NET_LTE: table}):
            events = plane_sim.Events(random.Random(seed))
            events.configure(plane_sim.NET_LTE, signal, 0.0)
        started = []
        events.on_start = lambda kind, start, length, loss: started.append((start, length))
        losses = [events.now(t / 20) for t in range(int(seconds * 20))]  # 20 packets a second
        return started, losses

    def test_troubles_come_and_go(self):
        started, losses = self.run_events((("dropout", 10, (2, 2), (0, 0), 1.0),), plane_sim.GOOD_SIGNAL, 600)
        self.assertTrue(40 <= len(started) <= 60, len(started))  # about one per 10 s + 2 s
        self.assertTrue(all(length == 2 for _, length in started))
        lost = sum(1 for _, loss in losses if loss == 1.0) / 20
        self.assertAlmostEqual(lost, 2 * len(started), delta=2 * 2)  # seconds without data

    def test_weaker_signal_more_trouble(self):
        table = (("latency spike", 20, (1, 3), (200, 800), 0.0),)
        weak, _ = self.run_events(table, 0, 2000)
        excellent, _ = self.run_events(table, 3, 2000)
        self.assertGreater(len(weak), 4 * len(excellent))  # 4 times as often vs half as often

    def test_dropout_loses_the_queue(self):
        with mock.patch.object(plane_sim, "EVENTS", {plane_sim.NET_LTE: (("dropout", 1e-6, (3, 3), (0, 0), 1.0),)}):
            net = plane_sim.Network(("127.0.0.1", 9), plane_sim.NET_LTE, plane_sim.GOOD_SIGNAL, random.Random(2))
        now = time.monotonic()
        with self.assertLogs("mavrelay.plane", "INFO") as logs:
            self.assertIsNone(net.schedule(True, 100, now + 0.01))  # the dropout has begun: nothing gets through
        self.assertIn("dropout, no data for 3.0 s", logs.output[0])
        self.assertTrue(net.cuts)  # and what waited in the queue is lost
        self.assertEqual(net.condition(now + 0.02), "dropout")


@unittest.skipUnless(HAVE_DISPLAY, "needs Tk and a display")
class WindowTest(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(plane_sim, "EVENTS", {})  # the link's random troubles: see EventsTest
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_window_follows_the_plane(self):
        ground = Ground()
        self.addCleanup(ground.close)
        plane = plane_sim.Plane(("127.0.0.1", ground.relay_port), KEY_V, fc_port=ground.fc_port)
        root = tk.Tk()
        root.withdraw()
        win = plane_sim.SimWindow(root, plane)
        try:
            text = lambda label: label.cget("text")  # noqa: E731
            self.assertEqual(root.title(), f"MavLTE Plane Simulator V{mr.__version__}")
            root.update()
            self.assertFalse(win.lte_switch.on)  # both switches start off
            self.assertEqual(text(win.lte_values["State"]), "No power: the battery is off")
            self.assertEqual(win.lte_led.color, ui.LED_OFF)
            win.quick.set(True)
            win.toggle_quick()
            with mock.patch.object(plane_sim, "STARTUP_QUICK", (0.05, 0.05, 0.05)):
                win.toggle_lte(True)
                win.toggle_battery(True)
                wait_for(self, lambda: text(win.fc_values["Flight controller"]) == "FBWA, ARMED",
                         "flight controller shown", pump=root.update)
                self.assertEqual(win.fc_led.color, ui.GREEN)
                self.assertEqual(text(win.volts), "12.6 V")
                wait_for(self, lambda: text(win.lte_values["State"]) == "Connected to the relay over LTE",
                         "module connected",
                         pump=root.update)
                wait_for(self, lambda: win.lte_led.color == ui.BLUE, "steady blue: a GCS is there",
                         pump=root.update)
                expected = "Ready for Snapshot in MavLTE" if plane_sim.Image is not None \
                    else "Needs Pillow: pip install pillow"
                self.assertEqual(text(win.camera_text), expected)
                win.toggle_camera(False)
                wait_for(self, lambda: text(win.camera_text) == "Off: the aircraft answers that it has no camera",
                         "camera off", pump=root.update)
                self.assertFalse(plane.camera)

                voice = win.lte_values["Voice"]  # what the board's speaker would be saying
                self.assertEqual(text(voice), "Off (MavLTE: Voice switch)")

                async def switch_voice():
                    ground.gcs.send_packet(mr.VOICE, mr.VOICE_BODY.pack(1))

                ground.run(switch_voice())
                wait_for(self, lambda: text(voice) == "Sounding: a two-tone alarm, again and again", "voice shown",
                         pump=root.update)
                self.assertEqual(voice.cget("fg"), ui.GREEN)
                win.toggle_lte(False)
                wait_for(self, lambda: text(win.lte_values["State"]) == "Off", "module off", pump=root.update)
                self.assertEqual(win.lte_led.color, ui.LED_OFF)
        finally:
            win.close()  # also closes the plane; Tk objects go in this thread
            win = root = None
            gc.collect()

    def test_led_red_yellow_green_blue(self):
        ground = Ground()
        self.addCleanup(ground.close)
        plane = plane_sim.Plane(("127.0.0.1", ground.relay_port), KEY_V, fc_port=ground.fc_port)
        plane.quick = True
        root = tk.Tk()
        root.withdraw()
        win = plane_sim.SimWindow(root, plane)
        seen = set()

        def pump():
            root.update()
            seen.add(win.lte_led.color)

        try:
            with mock.patch.object(plane_sim, "STARTUP_QUICK", (0.6, 0.3, 0.3)), \
                    mock.patch.object(mr, "LINK_TIMEOUT", 2.5):
                win.toggle_lte(True)
                win.toggle_battery(True)
                wait_for(self, lambda: ui.RED in seen, "red while there is no mobile data", pump=pump)
                wait_for(self, lambda: win.lte_led.color == ui.BLUE, "blue: connected, a GCS is there", pump=pump)

                def gcs_watches(on):  # like MavLTE with both switches off
                    ground.gcs.ping_flags = mr.PING_FLAG_WATCHING if on else 0
                    ground.gcs.ping_now()

                ground.loop.call_soon_threadsafe(gcs_watches, True)
                wait_for(self, lambda: win.lte_led.color == ui.GREEN, "green: connected, no GCS", pump=pump)
                ground.loop.call_soon_threadsafe(gcs_watches, False)
                wait_for(self, lambda: win.lte_led.color == ui.BLUE, "blue again", pump=pump)
                ground.loop.call_soon_threadsafe(setattr, ground.relay, "keys", {})  # the relay stops answering
                wait_for(self, lambda: win.lte_led.color == plane_sim.YELLOW, "yellow: no answer from the relay",
                         pump=pump)
        finally:
            win.close()
            win = root = None
            gc.collect()


def process_gone(pid: int, timeout: float) -> bool:
    import ctypes
    from ctypes import wintypes

    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.OpenProcess.restype = wintypes.HANDLE
    k32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
    k32.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    k32.CloseHandle.argtypes = (wintypes.HANDLE,)
    handle = k32.OpenProcess(0x00100000, False, pid)  # SYNCHRONIZE
    if not handle:
        return True
    try:
        return k32.WaitForSingleObject(handle, int(timeout * 1000)) == 0
    finally:
        k32.CloseHandle(handle)


@unittest.skipUnless(sys.platform == "win32" and plane_sim, "Windows job objects")
class KillWithUsTest(unittest.TestCase):
    def test_sitl_dies_with_the_simulator(self):
        # stands in for the simulator: starts a long-running "SITL", ties it to itself, and is then killed
        code = ("import subprocess, sys, time; sys.path.insert(0, %r); import plane_sim; "
                "sitl = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']); "
                "plane_sim.kill_with_us(sitl); print(sitl.pid, flush=True); time.sleep(60)") % RELAY_DIR
        sim = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
        pid = int(sim.stdout.readline())
        try:
            sim.kill()
            sim.wait(5)
            self.assertTrue(process_gone(pid, 5.0), "SITL outlived the simulator")
        finally:
            sim.stdout.close()
            if not process_gone(pid, 0):
                os.kill(pid, 9)


if __name__ == "__main__":
    unittest.main()

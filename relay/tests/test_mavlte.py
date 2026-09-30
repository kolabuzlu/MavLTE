"""The MavLTE app window, driven through its own switches against a real relay and a fake aircraft.

The window stays hidden. Skipped where Tk cannot open a display.
"""

import asyncio
import gc
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import mavrelay as mr  # noqa: E402
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
            self.vehicle_task = asyncio.ensure_future(self.vehicle.run())

            async def ticks():
                seq = 0
                while True:
                    await asyncio.sleep(0.1)
                    self.relay.tick(time.monotonic())
                    seq += 1
                    if not self.vehicle_quiet:
                        self.vehicle.send_data(v2_frame(0, bytes(9), seq))  # 10 heartbeats a second

            asyncio.ensure_future(ticks())
            return port

        relay_port = asyncio.run_coroutine_threadsafe(start(), self.loop).result(5)
        self.udp_port, self.tcp_port = free_port(), free_port(socket.SOCK_STREAM)
        fd, self.config = tempfile.mkstemp(suffix=".ini")
        os.close(fd)
        with open(self.config, "w") as f:
            f.write("# my settings\n[gcs]\nname = Test UAV\nserver = 127.0.0.1:%d\nkey = %s\n"
                    "udp = 127.0.0.1:%d\ntcp = 127.0.0.1:%d\nudp_on = yes\ntcp_on = yes\n"  # older version
                    % (relay_port, KEY_G.hex(), self.udp_port, self.tcp_port))
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
        self.pump(lambda: self.text(app.udp_card.status).startswith("Waiting for the aircraft\nMission Planner: UDP"),
                  what="waiting for the aircraft")
        self.assertEqual(app.udp_card.led.color, mavlte.LED_OFF)

    def test_close_keeps_what_others_wrote_meanwhile(self):
        # sitl_demo.py --no-agent writes its relay and key while the app is open
        mr.update_config(self.config, "gcs", {"server": "127.0.0.1:14650", "key": KEY_V.hex()})
        self.app.close()  # as the user closing the window
        saved = mavlte.Settings.load(self.config)
        self.assertEqual((saved.server, saved.key), ("127.0.0.1:14650", KEY_V.hex()))
        self.assertEqual(saved.name, "Test UAV")

    def test_wrong_key_says_no_answer(self):
        app = self.app
        app.apply_settings(app.settings.name, app.settings.server, KEY_V.hex())  # not the GCS key: no answer
        self.pump(lambda: self.text(app.relay_text).startswith("No answer from the relay"), timeout=10,
                  what="no-answer message")


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


if __name__ == "__main__":
    unittest.main()

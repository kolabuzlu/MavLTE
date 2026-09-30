"""The plane simulator: its aircraft (battery, LTE module, coverage) against a real relay, with a fake
flight controller in place of SITL, and its window. Skipped where tkinter is missing."""

import asyncio
import gc
import os
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
    """What SITL's SERIAL1 sends: HEARTBEAT (FBWA, armed), SYS_STATUS (12.6 V), GLOBAL_POSITION_INT (120 m)."""
    heartbeat = struct.pack("<IBBBBB", 5, 1, 3, 0x81, 4, 3)
    sys_status = bytes(14) + struct.pack("<H", 12600) + bytes(15)
    position = bytes(16) + struct.pack("<i", 120_000) + bytes(8)
    return v2_frame(0, heartbeat, seq) + v2_frame(1, sys_status, seq + 1) + v2_frame(33, position, seq + 2)


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
            self.relay.tick(time.monotonic())

    async def _fc_client(self, reader, writer):  # plays SITL's SERIAL1
        self.fc_clients.append(writer)

        async def talk():
            seq = 0
            while True:
                writer.write(fc_telemetry(seq))
                seq += 3
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
        patcher = mock.patch.object(plane_sim, "STARTUP_QUICK", (0.05, 0.05, 0.05))
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

    def test_battery_lte_module_and_coverage(self):
        p, g = self.plane, self.ground
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.fc.mode == "FBWA", "heartbeat from the flight controller")
        self.assertTrue(p.fc.armed)
        self.assertAlmostEqual(p.fc.volts, 12.6)
        self.assertAlmostEqual(p.fc.alt, 120.0)

        # the module is switched on, so it starts with the battery and carries the telemetry
        wait_for(self, self.session, "relay session")
        first = self.session()
        wait_for(self, g.telemetry_at_gcs, "telemetry at the GCS")
        wait_for(self, lambda: g.relay.vehicle.rssi_dbm == -65, "signal reported to the relay")
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

        # coverage: the signal the module reports, and no signal gets nothing through
        p.call(p.set_coverage, 1)
        wait_for(self, lambda: g.relay.vehicle.rssi_dbm == -101, "weak signal at the relay")
        p.call(p.set_coverage, 0)
        time.sleep(0.4)
        heard = g.relay.vehicle.last_rx
        time.sleep(0.6)
        self.assertEqual(g.relay.vehicle.last_rx, heard)
        p.call(p.set_coverage, 4)
        wait_for(self, lambda: g.relay.vehicle.last_rx > heard, "heard again with a signal")

        # battery off: flight controller and module lose power together
        p.call(p.set_battery, False)
        wait_for(self, lambda: not g.fc_clients, "flight controller link closed")
        self.assertIsNone(p.modem)

    def test_no_signal_no_registration(self):
        p = self.plane
        p.call(p.set_coverage, 0)
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.modem is not None and p.modem.stage == "searching", "searching")
        time.sleep(0.4)
        self.assertEqual(p.modem.stage, "searching")  # nothing to register with
        p.call(p.set_coverage, 3)
        wait_for(self, self.session, "relay session once there is a signal")

    def test_module_switched_off_stays_off_with_the_battery(self):
        p = self.plane
        p.call(p.set_lte, False)
        p.call(p.set_battery, True)
        wait_for(self, lambda: p.fc.mode == "FBWA", "flight controller up")
        time.sleep(0.3)
        self.assertIsNone(p.modem)


@unittest.skipUnless(HAVE_DISPLAY, "needs Tk and a display")
class WindowTest(unittest.TestCase):
    def test_window_follows_the_plane(self):
        ground = Ground()
        self.addCleanup(ground.close)
        plane = plane_sim.Plane(("127.0.0.1", ground.relay_port), KEY_V, fc_port=ground.fc_port)
        root = tk.Tk()
        root.withdraw()
        win = plane_sim.SimWindow(root, plane)
        try:
            text = lambda label: label.cget("text")  # noqa: E731
            self.assertEqual(root.title(), "Plane simulator")
            root.update()
            self.assertEqual(text(win.lte_values["State"]), "No power: the battery is off")
            self.assertEqual(win.lte_led.color, ui.LED_OFF)
            win.quick.set(True)
            win.toggle_quick()
            with mock.patch.object(plane_sim, "STARTUP_QUICK", (0.05, 0.05, 0.05)):
                win.toggle_battery(True)
                wait_for(self, lambda: text(win.fc_values["Flight controller"]) == "FBWA, ARMED",
                         "flight controller shown", pump=root.update)
                self.assertEqual(win.fc_led.color, ui.GREEN)
                self.assertEqual(text(win.volts), "12.6 V")
                wait_for(self, lambda: text(win.lte_values["State"]) == "Connected to the relay", "module connected",
                         pump=root.update)
                wait_for(self, lambda: win.lte_led.color == ui.GREEN, "steady green: a GCS is there",
                         pump=root.update)
                win.toggle_lte(False)
                wait_for(self, lambda: text(win.lte_values["State"]) == "Off", "module off", pump=root.update)
                self.assertEqual(win.lte_led.color, ui.LED_OFF)
        finally:
            win.close()  # also closes the plane; Tk objects go in this thread
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

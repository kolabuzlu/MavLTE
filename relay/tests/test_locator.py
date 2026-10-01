"""Tests for the locator: the aircraft's own GNSS position through the relay. Run from the relay
directory:  python -m unittest -v"""

import asyncio
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mavrelay as mr  # noqa: E402

KEY_V = bytes(range(32))
KEY_G = bytes(range(100, 132))


def fix(**kw):
    values = dict(gnss_time=int(time.time()), lat=411234567, lon=289876543, alt=150_000, speed=1200, course=4500,
                  hdop=90, sats=11, fix=mr.FIX_3D, fc_silent=0, battery_mv=3950, battery_pct=78, temp=47)
    values.update(kw)
    return mr.Position(**values)


class PositionTest(unittest.TestCase):
    def test_roundtrip(self):
        pos = fix(flags=mr.POS_FC_SILENT, fc_silent=42, time=1790000000)
        self.assertEqual((mr.POSITION_BODY.size, len(pos.pack())), (34, 35))  # 34 before 1.4.0
        self.assertEqual(mr.Position.unpack(pos.pack()), pos)
        self.assertTrue(pos.has_fix and pos.fc_is_silent)
        self.assertEqual(pos.describe(), "41.123457, 28.987654, 11 satellites")
        self.assertEqual(mr.Position.unpack(fix(temp=-5).pack()).temp, -5)

    def test_same_bytes_as_the_firmware(self):  # firmware/test/host/test_core.c, test_locator_packet
        pos = mr.Position(gnss_time=1790841600, lat=411234567, lon=-289876543, alt=150500, speed=632, course=4560,
                          hdop=90, sats=14, fix=mr.FIX_3D, flags=mr.POS_FC_SILENT, fc_silent=42, battery_mv=3950,
                          battery_pct=78, temp=47)
        self.assertEqual(pos.pack().hex(), "0013be6a07f18218c1d5b8eee44b02007802d0115a000e03012a006e0f4e000000002f")
        self.assertEqual(mr.Position(flags=mr.POS_NO_GNSS).pack().hex(),
                         "00000000000000800000008000000080ffffffffffff000002ffffffffff0000000080")

    def test_reports_before_1_4_0_have_no_temperature(self):
        old = fix().pack()[:34]
        self.assertEqual(mr.Position.unpack(old), fix(temp=mr.TEMP_UNKNOWN))
        self.assertEqual(mr.Position.unpack(fix().pack() + b"later"), fix())  # what comes later is skipped

    def test_no_fix(self):
        self.assertFalse(mr.Position(sats=3).has_fix)
        self.assertEqual(mr.Position(sats=3).describe(), "no GNSS fix yet (3 satellites)")
        self.assertFalse(fix(fix=mr.FIX_NONE).has_fix)
        self.assertEqual(mr.Position(flags=mr.POS_NO_GNSS).describe(), "no GNSS")
        self.assertEqual(mr.Position().fc_silent, mr.U16_UNKNOWN)  # not heard since the module started

    def test_power(self):
        """On V2 boards the gauge reads the supply rail: above what a Li-ion cell holds, that is external power."""
        rail = mr.Position(battery_mv=4298, battery_pct=100)  # the first board on USB, no cell in it
        self.assertEqual((rail.on_external_power, rail.power_text()), (True, "external power"))
        cell = mr.Position(battery_mv=3950, battery_pct=78)
        self.assertEqual((cell.on_external_power, cell.power_text()), (False, "battery 78%"))
        self.assertTrue(mr.Position(battery_mv=mr.EXTERNAL_POWER_MV).on_external_power)
        self.assertFalse(mr.Position(battery_mv=mr.EXTERNAL_POWER_MV - 1, battery_pct=100).on_external_power)
        self.assertEqual(mr.Position(battery_pct=55).power_text(), "battery 55%")  # millivolts unknown
        self.assertEqual(mr.Position().power_text(), "")  # no gauge

    def test_ages(self):
        self.assertEqual([mr.fmt_age(s) for s in (-3, 5, 60, 150, 3600, 3700, 86399, 86400, 3 * 86400)],
                         ["0 s", "5 s", "1 min", "2 min", "1 h", "1 h 1 min", "23 h 59 min", "1 day", "3 days"])


class RelayLocatorTest(unittest.TestCase):
    """The relay's LocatorStore, driven packet by packet."""

    PLANE = ("203.0.113.5", 5000)
    LAPTOP = ("198.51.100.7", 4000)
    TABLET = ("198.51.100.8", 4001)

    def setUp(self):
        self.state = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.state, True)
        self.sent = []
        self.seq = {}
        self.relay = self.new_relay()

    def new_relay(self):
        relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G}, state_dir=self.state)
        sent = self.sent

        class Transport:
            def sendto(self, data, addr):
                sent.append((mr.decode(data), addr))

        relay.connection_made(Transport())
        return relay

    def connect(self, role, addr, watching=False):
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, mr.HELLO, role, 0, 0, os.urandom(8) + b"test"), addr)
        sid = [p.session for p, a in self.sent if a == addr and p.type == mr.WELCOME][-1]
        self.seq[sid] = 0
        flags = mr.PING_FLAG_WATCHING if watching else 0
        self.packet(role, sid, addr, mr.PING, mr.PING_BODY.pack(0, 50, 0, -80, 7, flags))
        return sid

    def packet(self, role, sid, addr, ptype, body):
        self.seq[sid] += 1
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, ptype, role, sid, self.seq[sid], body), addr)

    def positions_to(self, addr):
        return [mr.Position.unpack(p.body) for p, a in self.sent if a == addr and p.type == mr.POSITION]

    def test_passed_on_to_every_agent(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.connect(mr.ROLE_GCS, self.TABLET, watching=True)  # watching (no telemetry) still hears it
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, mr.Position(sats=2).pack())
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, fix().pack())
        for addr in (self.LAPTOP, self.TABLET):
            got = self.positions_to(addr)
            self.assertEqual([p.has_fix for p in got], [False, True])
            self.assertAlmostEqual(got[1].time, time.time(), delta=5)  # the relay's clock on it
            self.assertEqual(got[1]._replace(time=0), fix())
        self.assertEqual(self.relay.locator.last_fix._replace(time=0), fix())

    def test_only_from_the_aircraft(self):
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.connect(mr.ROLE_GCS, self.TABLET)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.POSITION, fix(lat=1).pack())
        self.assertEqual(self.positions_to(self.TABLET), [])
        self.assertIsNone(self.relay.locator.last_fix)

    def test_last_known_position_survives_and_greets(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, fix(flags=mr.POS_FC_SILENT, fc_silent=30).pack())
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, mr.Position(sats=1).pack())  # the fix lost
        with open(os.path.join(self.state, "locator.json")) as f:
            saved = json.load(f)
        self.assertEqual((saved["lat"], saved["latitude"], saved["fc_silent"]), (411234567, 41.1234567, 30))
        # a relay started later knows it, and tells each agent that connects, the aircraft long gone
        self.relay = self.new_relay()
        self.connect(mr.ROLE_GCS, self.LAPTOP)
        [greeting] = self.positions_to(self.LAPTOP)
        self.assertEqual((greeting.lat, greeting.lon, greeting.sats), (411234567, 289876543, 11))
        self.assertAlmostEqual(greeting.time, time.time(), delta=5)

    def test_short_or_broken_reports(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, fix().pack()[:20])
        self.assertEqual(self.positions_to(self.LAPTOP), [])
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION, fix().pack()[:34])  # an aircraft before 1.4.0
        [old] = self.positions_to(self.LAPTOP)
        self.assertEqual(old._replace(time=0), fix(temp=mr.TEMP_UNKNOWN))
        os.makedirs(self.state, exist_ok=True)
        with open(os.path.join(self.state, "locator.json"), "w") as f:
            f.write("{not json")
        self.assertIsNone(self.new_relay().locator.last_fix)  # a broken file is not fatal

    def test_power_changes(self):
        """The flight battery goes (a crash, or unplugged): the module runs on its cell, and says so."""
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        report = lambda mv, pct: self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.POSITION,  # noqa: E731
                                             fix(battery_mv=mv, battery_pct=pct).pack())
        with self.assertLogs("mavrelay.relay", "INFO") as logs:
            report(4298, 100)  # the first report: nothing changes
            report(4298, 100)
            report(4105, 97)
            report(4100, 97)
            report(4298, 100)
        power = [line for line in logs.output if "LTE module" in line]
        self.assertEqual(power, ["WARNING:mavrelay.relay:the aircraft's LTE module runs on its own cell now (battery 97%)",
                                 "INFO:mavrelay.relay:the aircraft's LTE module has external power again"])

    def test_saved_before_1_4_0(self):
        saved = dict(fix(time=1790000000)._asdict(), latitude=41.1234567, longitude=28.9876543)
        del saved["temp"]
        with open(os.path.join(self.state, "locator.json"), "w") as f:
            json.dump(saved, f)
        self.assertEqual(self.new_relay().locator.last_fix, fix(time=1790000000, temp=mr.TEMP_UNKNOWN))


class LiveLocatorTest(unittest.IsolatedAsyncioTestCase):
    """Aircraft, relay and GCS agent on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop

    async def asyncSetUp(self):
        self.state = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.state, True)
        self.tasks = []
        loop = asyncio.get_running_loop()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G}, state_dir=self.state)
        self.transport, _ = await loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", 0))
        self.port = self.transport.get_extra_info("sockname")[1]

    async def asyncTearDown(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.transport.close()

    async def until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail("condition not met in time")
            await asyncio.sleep(0.02)

    async def test_agent_follows_the_aircraft(self):
        vehicle = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", self.port, on_data=lambda d: None)
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks += [asyncio.ensure_future(vehicle.run()), asyncio.ensure_future(agent.run())]
        await self.until(lambda: vehicle.is_connected and agent.client.is_connected and self.relay.vehicle)
        self.assertEqual(agent.position_text(), "")
        with self.assertLogs("mavrelay.gcs", "INFO") as logs:
            vehicle.send_packet(mr.POSITION, mr.Position(sats=2).pack())
            await self.until(lambda: agent.position is not None)
            vehicle.send_packet(mr.POSITION, fix(flags=mr.POS_FC_SILENT, fc_silent=12).pack())
            await self.until(lambda: agent.last_fix is not None)
        self.assertTrue(any("no GNSS fix yet" in line for line in logs.output))
        self.assertTrue(any("flight controller is silent (12 s)" in line for line in logs.output))
        self.assertTrue(agent.position.fc_is_silent)
        self.assertEqual(agent.position_text(), "GNSS 41.123457, 28.987654 (11 satellites, 0 s ago)")

        with self.assertLogs("mavrelay.gcs", "INFO") as logs:  # the module's chip gets hot, then cools down
            for temp in (75, 85, 78, 65):
                vehicle.send_packet(mr.POSITION, fix(temp=temp).pack())
                await self.until(lambda: agent.position.temp == temp)
        hot = [line for line in logs.output if "°C" in line]
        self.assertEqual(len(hot), 2)
        self.assertIn("WARNING:mavrelay.gcs:the aircraft's LTE module is hot: its chip is at 85 °C", hot[0])
        self.assertIn("cooled down to 65 °C", hot[1])

        with self.assertLogs("mavrelay.gcs", "INFO") as logs:  # (so far on its cell:) the BEC, then a crash
            for mv, pct in ((4298, 100), (4298, 100), (4120, 98)):
                vehicle.send_packet(mr.POSITION, fix(battery_mv=mv, battery_pct=pct, gnss_time=int(time.time()) + mv
                                                     + pct).pack())
                await self.until(lambda: agent.position.battery_mv == mv)
        power = [line for line in logs.output if "LTE module" in line]
        self.assertEqual(power, ["INFO:mavrelay.gcs:the aircraft's LTE module has external power again",
                                 "WARNING:mavrelay.gcs:the aircraft's LTE module runs on its own cell now (battery 98%)"])

        # an agent that connects later hears the last known one at once, even with the aircraft gone
        vehicle_task = self.tasks[0]
        vehicle_task.cancel()
        late = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks.append(asyncio.ensure_future(late.run()))
        await self.until(lambda: late.last_fix is not None)
        self.assertEqual((late.last_fix.lat, late.last_fix.lon), (411234567, 289876543))


if __name__ == "__main__":
    unittest.main()

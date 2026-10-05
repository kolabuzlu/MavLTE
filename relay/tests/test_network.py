"""Tests for the aircraft's mobile network (1.8.8): chosen by a GCS agent (MavLTE, its web page), kept by the relay,
passed on to the aircraft in its PONGs, and what the aircraft says back in its PINGs (the network it is set to, and its
fallback to 2G while LTE fails). Run from the relay directory:  python -m unittest -v"""

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


def status(flags=mr.STATUS_VEHICLE_ONLINE, net=None, idle=200):
    body = mr.STATUS_BODY.pack(flags, 7, 85, 3, 0, -71, idle) + mr.QUALITY.pack(-5)
    return mr.LinkStatus.unpack(body if net is None else body + mr.NET_REPORT.pack(net))


class NetworkStatusTest(unittest.TestCase):
    def test_status_byte(self):
        st = status(net=mr.NET_2G | mr.STATUS_NET_REPORTED | mr.NET_2G << mr.STATUS_NET_SHIFT)
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_2G, mr.NET_2G, False))
        self.assertEqual(st.network_text(), "network 2G only")
        self.assertTrue(st.describe().endswith("; network 2G only"))
        st = status(net=mr.NET_AUTO | mr.STATUS_NET_REPORTED | mr.NET_AUTO << mr.STATUS_NET_SHIFT
                    | mr.STATUS_NET_FALLBACK)
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_AUTO, mr.NET_AUTO, True))
        self.assertTrue(st.describe().endswith("; on 2G because LTE failed"))
        st = status(net=mr.NET_LTE | mr.STATUS_NET_REPORTED | mr.NET_2G << mr.STATUS_NET_SHIFT)
        self.assertEqual(st.network_text(), "network LTE only, the aircraft is still set to 2G only")
        # the vehicle's part only counts when it reports one (firmware before 1.8.8 does not)
        st = status(net=mr.NET_LTE | mr.NET_2G << mr.STATUS_NET_SHIFT | mr.STATUS_NET_FALLBACK)
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_LTE, None, False))
        # the two bits' unused fourth value counts as automatic
        st = status(net=3 | mr.STATUS_NET_REPORTED | 3 << mr.STATUS_NET_SHIFT)
        self.assertEqual((st.network, st.vehicle_network), (mr.NET_AUTO, mr.NET_AUTO))

    def test_old_relays(self):
        st = status()  # before 1.8.8: no byte
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_AUTO, None, False))
        self.assertEqual(st.network_text(), "")
        self.assertNotIn("network", st.describe())  # the log line stays as it was while all is automatic
        self.assertEqual(mr.LinkStatus(True, 7, 85, 3, 0, -71, 200).network, mr.NET_AUTO)  # built by older code


class RelayNetworkTest(unittest.TestCase):
    """The relay's side, driven packet by packet."""

    PLANE = ("203.0.113.5", 5000)
    LAPTOP = ("198.51.100.7", 4000)

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

    def connect(self, role, addr):
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, mr.HELLO, role, 0, 0, os.urandom(8) + b"test"), addr)
        sid = [p.session for p, a in self.sent if a == addr and p.type == mr.WELCOME][-1]
        self.seq[sid] = 0
        self.ping(role, sid, addr)
        return sid

    def packet(self, role, sid, addr, ptype, body):
        self.seq[sid] += 1
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, ptype, role, sid, self.seq[sid], body), addr)

    def ping(self, role, sid, addr, report=None):
        """A PING, a vehicle's with its quality; and with its network since 1.8.8 (report, the byte)."""
        body = mr.PING_BODY.pack(0, 50, 0, -80, 7, 0)
        if role == mr.ROLE_VEHICLE:
            body += mr.QUALITY.pack(-5) + (b"" if report is None else mr.NET_REPORT.pack(report))
        self.packet(role, sid, addr, mr.PING, body)

    def choose(self, sid, mode):
        self.packet(mr.ROLE_GCS, sid, self.LAPTOP, mr.NETWORK, mr.NETWORK_BODY.pack(mode))

    def last_pong_network(self, addr):
        flags = [p.body[4] for p, a in self.sent if a == addr and p.type == mr.PONG][-1]
        return flags >> mr.PONG_NET_SHIFT & 0x03

    def last_status(self, addr):
        self.relay.tick(time.monotonic())
        return mr.LinkStatus.unpack([p.body for p, a in self.sent if a == addr and p.type == mr.STATUS][-1])

    def test_chosen_by_an_agent_passed_on_and_reported_back(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.assertEqual(self.last_pong_network(self.PLANE), mr.NET_AUTO)
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.network, st.vehicle_network), (mr.NET_AUTO, None))  # firmware before 1.8.8: no report

        with self.assertLogs("mavrelay.relay", "INFO") as logs:
            self.choose(laptop, mr.NET_2G)
        self.assertIn("chose the aircraft's network: 2G only", logs.output[0])
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, report=mr.NET_AUTO)
        self.assertEqual(self.last_pong_network(self.PLANE), mr.NET_2G)
        self.ping(mr.ROLE_GCS, laptop, self.LAPTOP)
        self.assertEqual(self.last_pong_network(self.LAPTOP), mr.NET_AUTO)  # only the aircraft is told
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_2G, mr.NET_AUTO, False))
        with self.assertLogs("mavrelay.relay", "INFO") as logs:  # the aircraft has switched
            self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, report=mr.NET_2G)
        self.assertIn("the aircraft's network is set to 2G only", logs.output[0])
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.network, st.vehicle_network), (mr.NET_2G, mr.NET_2G))
        self.assertIn("network 2G only for", self.relay.summary(time.monotonic()))

        # automatic again, and LTE fails up there: its fallback to 2G, reported and logged
        self.choose(laptop, mr.NET_AUTO)
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, report=mr.NET_AUTO)
        self.assertEqual(self.last_pong_network(self.PLANE), mr.NET_AUTO)
        with self.assertLogs("mavrelay.relay", "WARNING") as logs:
            self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, report=mr.NET_AUTO | mr.NET_FALLBACK)
        self.assertIn("the aircraft is on 2G: LTE failed", logs.output[0])
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.network, st.vehicle_network, st.fallback), (mr.NET_AUTO, mr.NET_AUTO, True))
        self.assertIn("the aircraft is on its 2G fallback", self.relay.summary(time.monotonic()))
        with self.assertLogs("mavrelay.relay", "INFO") as logs:
            self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, report=mr.NET_AUTO)
        self.assertIn("2G fallback has ended", logs.output[0])
        self.assertFalse(self.last_status(self.LAPTOP).fallback)
        # a PING as before 1.8.8 again (another board): no report
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE)
        self.assertIsNone(self.last_status(self.LAPTOP).vehicle_network)
        # each change of network is a new data call, so a new session: its first report counts as a change too
        other = ("203.0.113.9", 6000)
        plane = self.connect(mr.ROLE_VEHICLE, other)
        with self.assertLogs("mavrelay.relay", "INFO") as logs:
            self.ping(mr.ROLE_VEHICLE, plane, other, report=mr.NET_LTE)
        self.assertTrue(any("the aircraft's network is set to LTE only" in line for line in logs.output))

    def test_only_agents_choose_and_only_networks_it_knows(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.NETWORK, mr.NETWORK_BODY.pack(mr.NET_2G))
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.NETWORK, b"")  # too short
        self.choose(laptop, 3)  # unused
        self.choose(laptop, 0xFF)
        self.assertEqual(self.relay.network.mode, mr.NET_AUTO)
        self.choose(laptop, mr.NET_LTE)
        self.assertEqual(self.relay.network.mode, mr.NET_LTE)

    def test_kept_without_the_aircraft_and_across_a_restart(self):
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.choose(laptop, mr.NET_LTE)
        st = self.last_status(self.LAPTOP)  # no aircraft yet: the choice holds for when it comes
        self.assertEqual((st.connected, st.network, st.vehicle_network), (False, mr.NET_LTE, None))
        with open(os.path.join(self.state, "network.json")) as f:
            self.assertEqual(json.load(f)["mode"], "lte")

        with self.assertLogs("mavrelay.relay", "INFO") as logs:  # restarted, it still tells the aircraft
            self.relay = self.new_relay()
        self.assertIn("the aircraft's network: LTE only", logs.output[0])
        self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.assertEqual(self.last_pong_network(self.PLANE), mr.NET_LTE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.choose(laptop, mr.NET_AUTO)
        self.assertEqual(self.new_relay().network.mode, mr.NET_AUTO)

    def test_broken_file(self):
        for text in ("{not json", '{"mode": "3g"}', '{"since": 5}'):
            with open(os.path.join(self.state, "network.json"), "w") as f:
                f.write(text)
            with self.assertLogs("mavrelay.relay", "WARNING"):
                self.assertEqual(self.new_relay().network.mode, mr.NET_AUTO)


class LiveNetworkTest(unittest.IsolatedAsyncioTestCase):
    """Aircraft, relay and GCS agent on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop

    async def asyncSetUp(self):
        self.tasks = []
        loop = asyncio.get_running_loop()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.transport, _ = await loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", 0))
        self.port = self.transport.get_extra_info("sockname")[1]

        async def ticks():
            while True:
                await asyncio.sleep(0.1)
                self.relay.tick(time.monotonic())

        self.vehicle = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", self.port, on_data=lambda d: None)
        self.vehicle.net_report = mr.NET_AUTO

        async def modem():  # as the firmware: set to the network the relay chooses, and says so
            while True:
                await asyncio.sleep(0.05)
                if self.vehicle.network != self.vehicle.net_report & 0x03:
                    self.vehicle.net_report = self.vehicle.network
                    self.vehicle.ping_now()

        self.agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks += [asyncio.ensure_future(t) for t in (ticks(), modem(), self.agent.run(), self.vehicle.run())]

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

    def status(self):
        return self.agent.vehicle_status()

    async def test_choice_reaches_the_aircraft_and_back(self):
        await self.until(lambda: self.status() and self.status().online and self.status().vehicle_network == 0)
        with self.assertLogs("mavrelay.gcs", "INFO"):
            self.assertTrue(self.agent.set_network(mr.NET_2G))
        await self.until(lambda: self.vehicle.network == mr.NET_2G)
        await self.until(lambda: self.status().vehicle_network == mr.NET_2G)
        self.assertIsNone(self.agent.network_request)  # the relay's STATUS showed it: no more sending

        # the aircraft loses the relay: it keeps the network last chosen
        self.vehicle._drop_session()
        self.assertEqual(self.vehicle.network, mr.NET_2G)
        await self.until(lambda: self.vehicle.is_connected)

        self.assertTrue(self.agent.set_network(mr.NET_AUTO))
        await self.until(lambda: self.status().network == self.status().vehicle_network == mr.NET_AUTO)
        with self.assertRaises(ValueError):
            self.agent.set_network(3)

    async def test_sent_again_until_the_relay_has_it(self):
        await self.until(lambda: self.agent.client.is_connected)
        choose, dropped = self.relay.network.choose, []

        def lossy(mode, by):  # the first one gets lost on the way
            if not dropped:
                dropped.append(mode)
                return
            choose(mode, by)

        self.relay.network.choose = lossy
        self.agent.set_network(mr.NET_LTE)
        await self.until(lambda: self.relay.network.mode == mr.NET_LTE)
        self.assertEqual(dropped, [mr.NET_LTE])
        await self.until(lambda: self.agent.network_request is None)

    async def test_a_relay_that_does_not_know_it(self):
        await self.until(lambda: self.agent.client.is_connected)
        self.relay.network.choose = lambda mode, by: None  # a relay before 1.8.8 ignores the packet
        self.agent.NETWORK_TRIES_FOR = 1.0
        with self.assertLogs("mavrelay.gcs", "WARNING") as logs:
            self.agent.set_network(mr.NET_2G)
            await self.until(lambda: self.agent.network_request is None)
        self.assertIn("older than 1.8.8", logs.output[0])

    async def test_without_a_session(self):
        agent = mr.GcsAgent(("127.0.0.1", 9), KEY_G)  # never started
        self.assertFalse(agent.set_network(mr.NET_2G))
        self.assertIsNone(agent.network_request)


if __name__ == "__main__":
    unittest.main()

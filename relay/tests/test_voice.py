"""Tests for the locator voice: MavLTE's switch, kept by the relay, passed on to the aircraft in its PONGs,
and what the aircraft says back in its PINGs. Run from the relay directory:  python -m unittest -v"""

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


def status(flags, idle=200):
    return mr.LinkStatus.unpack(mr.STATUS_BODY.pack(flags, 7, 85, 3, 0, -71, idle))


class VoiceStatusTest(unittest.TestCase):
    def test_flags(self):
        st = status(mr.STATUS_VEHICLE_ONLINE | mr.STATUS_VOICE_ON | mr.STATUS_SPEAKING)
        self.assertEqual((st.online, st.voice_on, st.speaking, st.voice_failed), (True, True, True, False))
        self.assertEqual(st.voice_text(), "on, the aircraft speaks")
        self.assertTrue(st.describe().endswith("; locator voice on, the aircraft speaks"))
        st = status(mr.STATUS_VEHICLE_ONLINE | mr.STATUS_VOICE_ON | mr.STATUS_VOICE_FAILED)
        self.assertEqual(st.voice_text(), "on, but the aircraft cannot speak")
        self.assertEqual(status(mr.STATUS_VOICE_ON | mr.STATUS_SPEAKING, idle=40000).voice_text(),
                         "on, the aircraft was speaking when last heard")
        self.assertEqual(status(mr.STATUS_VEHICLE_ONLINE | mr.STATUS_VOICE_ON).voice_text(), "on, waiting for the aircraft")

    def test_off_and_old_relays(self):
        st = status(mr.STATUS_VEHICLE_ONLINE)
        self.assertEqual((st.voice_on, st.voice_text()), (False, "off"))
        self.assertNotIn("voice", st.describe())  # the log line stays as it was while the voice is off
        old = mr.LinkStatus.unpack(mr.STATUS_BODY.pack(1, 7, 85, 3, 0, -71, 200)[:10])  # cut short: unknown fields
        self.assertFalse(old.voice_on)
        self.assertEqual(mr.LinkStatus(True, 7, 85, 3, 0, -71, 200).voice_text(), "off")  # built by older code

    def test_no_vehicle_yet(self):
        st = mr.LinkStatus.unpack(bytes([mr.STATUS_VOICE_ON]) + mr.NO_VEHICLE_STATUS[1:])
        self.assertFalse(st.connected)
        self.assertEqual(st.describe(), "vehicle: not connected to the server; locator voice on, waiting for the aircraft")


class RelayVoiceTest(unittest.TestCase):
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

    def ping(self, role, sid, addr, flags=0):
        self.packet(role, sid, addr, mr.PING, mr.PING_BODY.pack(0, 50, 0, -80, 7, flags))

    def last_pong_flags(self, addr):
        return [p.body[4] for p, a in self.sent if a == addr and p.type == mr.PONG][-1]

    def last_status(self, addr):
        self.relay.tick(time.monotonic())
        return mr.LinkStatus.unpack([p.body for p, a in self.sent if a == addr and p.type == mr.STATUS][-1])

    def test_switched_by_an_agent_and_passed_on_in_pongs(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.assertFalse(self.last_pong_flags(self.PLANE) & mr.PONG_VOICE)
        self.assertFalse(self.last_status(self.LAPTOP).voice_on)

        with self.assertLogs("mavrelay.relay", "INFO") as logs:
            self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, mr.VOICE_BODY.pack(1))
        self.assertIn("switched the locator voice on", logs.output[0])
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE)
        self.assertTrue(self.last_pong_flags(self.PLANE) & mr.PONG_VOICE)
        self.ping(mr.ROLE_GCS, laptop, self.LAPTOP)
        self.assertFalse(self.last_pong_flags(self.LAPTOP) & mr.PONG_VOICE)  # only the aircraft is asked
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.voice_on, st.speaking), (True, False))

        with self.assertLogs("mavrelay.relay", "INFO") as logs:  # the aircraft says it speaks
            self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, mr.PING_FLAG_SPEAKING)
        self.assertIn("the aircraft speaks", logs.output[0])
        self.assertTrue(self.last_status(self.LAPTOP).speaking)
        with self.assertLogs("mavrelay.relay", "WARNING"):  # ... or that it cannot
            self.ping(mr.ROLE_VEHICLE, plane, self.PLANE, mr.PING_FLAG_VOICE_FAILED)
        st = self.last_status(self.LAPTOP)
        self.assertEqual((st.speaking, st.voice_failed), (False, True))

        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, mr.VOICE_BODY.pack(0))
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE)
        self.assertFalse(self.last_pong_flags(self.PLANE) & mr.PONG_VOICE)
        self.assertFalse(self.last_status(self.LAPTOP).voice_on)

    def test_only_agents_switch_it(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.VOICE, mr.VOICE_BODY.pack(1))
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, b"")  # too short
        self.assertFalse(self.relay.voice.on)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, bytes([0xFE]))  # bit 0 only: the rest is for later
        self.assertFalse(self.relay.voice.on)

    def test_kept_without_the_aircraft_and_across_a_restart(self):
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, mr.VOICE_BODY.pack(1))
        st = self.last_status(self.LAPTOP)  # no aircraft yet: the switch holds for when it comes
        self.assertEqual((st.connected, st.voice_on), (False, True))
        with open(os.path.join(self.state, "voice.json")) as f:
            self.assertTrue(json.load(f)["on"])
        self.assertIn("locator voice on for", self.relay.summary(time.monotonic()))

        with self.assertLogs("mavrelay.relay", "INFO") as logs:  # restarted, it still asks the aircraft
            self.relay = self.new_relay()
        self.assertIn("the locator voice is on", logs.output[0])
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.assertTrue(self.last_pong_flags(self.PLANE) & mr.PONG_VOICE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.VOICE, mr.VOICE_BODY.pack(0))
        self.ping(mr.ROLE_VEHICLE, plane, self.PLANE)
        self.assertFalse(self.last_pong_flags(self.PLANE) & mr.PONG_VOICE)
        self.assertFalse(self.new_relay().voice.on)

    def test_broken_file(self):
        with open(os.path.join(self.state, "voice.json"), "w") as f:
            f.write("{not json")
        with self.assertLogs("mavrelay.relay", "WARNING"):
            self.assertFalse(self.new_relay().voice.on)


class LiveVoiceTest(unittest.IsolatedAsyncioTestCase):
    """Aircraft, relay and GCS agent on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop

    async def asyncSetUp(self):
        self.tasks = []
        loop = asyncio.get_running_loop()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.transport, _ = await loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", 0))
        self.port = self.transport.get_extra_info("sockname")[1]
        self.speaks = True  # the aircraft's modem takes the phrases

        async def ticks():
            while True:
                await asyncio.sleep(0.1)
                self.relay.tick(time.monotonic())

        self.vehicle = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", self.port, on_data=lambda d: None)

        async def voice():  # as the firmware: speaks while the relay asks it to
            while True:
                await asyncio.sleep(0.05)
                flags = (mr.PING_FLAG_SPEAKING if self.speaks else mr.PING_FLAG_VOICE_FAILED) \
                    if self.vehicle.voice_on else 0
                if flags != self.vehicle.ping_flags:
                    self.vehicle.ping_flags = flags
                    self.vehicle.ping_now()

        self.agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks += [asyncio.ensure_future(t) for t in (ticks(), voice(), self.agent.run())]
        self.vehicle_task = asyncio.ensure_future(self.vehicle.run())
        self.tasks.append(self.vehicle_task)

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

    async def test_switch_reaches_the_aircraft_and_back(self):
        await self.until(lambda: self.vehicle.is_connected and self.status() and self.status().online)
        with self.assertLogs("mavrelay.gcs", "INFO"):
            self.assertTrue(self.agent.set_voice(True))
        await self.until(lambda: self.vehicle.voice_on)
        await self.until(lambda: self.status().speaking)
        self.assertIsNone(self.agent.voice_request)  # the relay's STATUS showed it: no more sending

        # the aircraft loses the relay: it keeps speaking where it has no coverage
        self.vehicle._drop_session()
        self.assertTrue(self.vehicle.voice_on)
        await self.until(lambda: self.vehicle.is_connected)

        self.speaks = False  # a modem that refuses the phrases
        await self.until(lambda: self.status().voice_failed and not self.status().speaking)

        self.assertTrue(self.agent.set_voice(False))
        await self.until(lambda: not self.vehicle.voice_on)
        await self.until(lambda: not self.status().voice_on and not self.status().voice_failed)

    async def test_sent_again_until_the_relay_has_it(self):
        await self.until(lambda: self.agent.client.is_connected)
        switch, dropped = self.relay.voice.switch, []

        def lossy(on, by):  # the first one gets lost on the way
            if not dropped:
                dropped.append(on)
                return
            switch(on, by)

        self.relay.voice.switch = lossy
        self.agent.set_voice(True)
        await self.until(lambda: self.relay.voice.on)
        self.assertEqual(dropped, [True])
        await self.until(lambda: self.agent.voice_request is None)

    async def test_a_relay_that_does_not_know_it(self):
        await self.until(lambda: self.agent.client.is_connected)
        self.relay.voice.switch = lambda on, by: None  # a relay before 1.5.0 ignores the packet
        self.agent.VOICE_TRIES_FOR = 1.0
        with self.assertLogs("mavrelay.gcs", "WARNING") as logs:
            self.agent.set_voice(True)
            await self.until(lambda: self.agent.voice_request is None)
        self.assertIn("older than 1.5.0", logs.output[0])

    async def test_without_a_session(self):
        agent = mr.GcsAgent(("127.0.0.1", 9), KEY_G)  # never started
        self.assertFalse(agent.set_voice(True))
        self.assertIsNone(agent.voice_request)


if __name__ == "__main__":
    unittest.main()

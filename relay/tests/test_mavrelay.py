"""Tests for mavrelay. Run from the relay directory:  python -m unittest -v"""

import asyncio
import os
import random
import re
import socket
import sys
import tempfile
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mavrelay as mr  # noqa: E402

KEY_V = bytes(range(32))
KEY_G = bytes(range(100, 132))

# Shared with firmware/test/host/test_core.c: the C code must produce exactly these bytes.
# The body is a MAVLink 2 HEARTBEAT from ArduPlane (sysid 1, compid 1, seq 42) with a valid CRC.
GOLDEN_KEY = bytes.fromhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f")
GOLDEN_BODY = bytes.fromhex("fd0900002a0101000000000000000103410303ca52")
GOLDEN_PACKET = "a5010301d4c3b2a107000000" + GOLDEN_BODY.hex() + "96bcbb3483d85430869d9997548c9380"


def v2_frame(msgid=0, payload=b"\x00" * 9, seq=0, signed=False):
    header = bytes([0xFD, len(payload), 0x01 if signed else 0x00, 0, seq & 0xFF, 1, 1]) + msgid.to_bytes(3, "little")
    return header + payload + b"\xab\xcd" + (b"\x5a" * 13 if signed else b"")


def v1_frame(msgid=0, payload=b"\x00" * 9, seq=0):
    return bytes([0xFE, len(payload), seq & 0xFF, 1, 1, msgid]) + payload + b"\xab\xcd"


def random_frames(rng, count):
    frames = []
    for i in range(count):
        # payloads full of start bytes, to prove the framer does not look inside frames
        payload = bytes(rng.choice([0xFD, 0xFE, rng.randrange(256)]) for _ in range(rng.randrange(0, 256)))
        kind = rng.randrange(3)
        if kind == 0:
            frames.append(v1_frame(rng.randrange(256), payload, i))
        else:
            frames.append(v2_frame(rng.randrange(1 << 24), payload, i, signed=kind == 2))
    return frames


def boundaries(data):
    """Offsets after which a fresh framer is between frames."""
    framer = mr.MavFramer()
    return [i + 1 for i, b in enumerate(data) if framer.push(b)]


class ProtocolTest(unittest.TestCase):
    def test_roundtrip(self):
        pkt = mr.encode(KEY_V, mr.DATA, mr.ROLE_VEHICLE, 0x12345678, 42, b"hello")
        self.assertEqual(len(pkt), mr.HEADER.size + 5 + mr.TAG_LEN)
        self.assertTrue(mr.verify(KEY_V, pkt))
        self.assertEqual(mr.decode(pkt), mr.Packet(mr.DATA, mr.ROLE_VEHICLE, 0x12345678, 42, b"hello"))

    def test_golden_vector(self):
        pkt = mr.encode(GOLDEN_KEY, mr.DATA, mr.ROLE_VEHICLE, 0xA1B2C3D4, 7, GOLDEN_BODY)
        self.assertEqual(pkt.hex(), GOLDEN_PACKET)
        self.assertEqual(boundaries(GOLDEN_BODY), [len(GOLDEN_BODY)])

    def test_tampering_and_wrong_key_rejected(self):
        pkt = bytearray(mr.encode(KEY_V, mr.DATA, mr.ROLE_VEHICLE, 1, 1, b"abc"))
        self.assertFalse(mr.verify(KEY_G, bytes(pkt)))
        for i in range(len(pkt)):
            bad = bytearray(pkt)
            bad[i] ^= 0x01
            self.assertFalse(mr.verify(KEY_V, bytes(bad)), f"flip in byte {i} not detected")

    def test_decode_rejects_foreign_datagrams(self):
        self.assertIsNone(mr.decode(b""))
        self.assertIsNone(mr.decode(v2_frame() * 2))
        pkt = bytearray(mr.encode(KEY_V, mr.PING, mr.ROLE_VEHICLE, 1, 1))
        pkt[1] = 2  # future version
        self.assertIsNone(mr.decode(bytes(pkt)))

    def test_limits(self):
        self.assertEqual(mr.MAX_PAYLOAD, 1172)
        self.assertEqual(len(mr.encode(KEY_V, mr.DATA, 1, 1, 1, bytes(mr.MAX_PAYLOAD))), mr.MAX_DATAGRAM)

    def test_link_status(self):
        st = mr.LinkStatus.unpack(mr.STATUS_BODY.pack(1, 7, 85, 3, 0, -71, 120))
        self.assertTrue(st.online)
        self.assertEqual(st.describe(), "vehicle: online, LTE -71 dBm, rtt to server 85 ms, loss up 0.3% down 0.0%")
        self.assertIn("not connected", mr.LinkStatus.unpack(mr.NO_VEHICLE_STATUS).describe())
        self.assertIn("OFFLINE", mr.LinkStatus.unpack(mr.STATUS_BODY.pack(0, 7, 85, 3, 0, -71, 4200)).describe())
        short = mr.LinkStatus.unpack(b"\x01")  # an older server sending fewer fields
        self.assertTrue(short.online)
        self.assertEqual(short.rssi_dbm, mr.RSSI_UNKNOWN)

    def test_silence_counts_on_past_65_5_s(self):
        """STATUS's idle_ms tops out at 65.5 s; the relay keeps the session for 120 s. The 1.5.2 review found
        MavLTE showing "last heard 66 s ago" all that time."""
        status = lambda idle: mr.LinkStatus.unpack(mr.STATUS_BODY.pack(0, 7, 85, 3, 0, -71, idle))  # noqa: E731
        self.assertEqual(status(mr.IDLE_CAPPED).describe(), "vehicle: OFFLINE, last heard over a minute ago")
        agent = mr.GcsAgent(("127.0.0.1", 9), KEY_G)
        with mock.patch.object(mr.time, "monotonic", return_value=1000.0):
            agent._got_status(status(60000))
            self.assertAlmostEqual(agent.vehicle_silence(), 60.0)
        with mock.patch.object(mr.time, "monotonic", return_value=1030.0):
            agent._got_status(status(mr.IDLE_CAPPED))  # 90 s silent; STATUS says 65.5
            self.assertAlmostEqual(agent.vehicle_silence(), 90.0)
        with mock.patch.object(mr.time, "monotonic", return_value=1061.0):
            agent._got_status(mr.LinkStatus.unpack(mr.NO_VEHICLE_STATUS))  # the relay has dropped it
            self.assertIsNone(agent.vehicle_silence())
        late = mr.GcsAgent(("127.0.0.1", 9), KEY_G)  # connects when the aircraft is already long silent
        late._got_status(status(mr.IDLE_CAPPED))
        self.assertIsNone(late.vehicle_silence())  # only "over a minute" is known


class ReplayWindowTest(unittest.TestCase):
    def test_window(self):
        w = mr.ReplayWindow()
        self.assertFalse(w.accept(0))
        self.assertTrue(w.accept(1))
        self.assertFalse(w.accept(1))
        self.assertTrue(w.accept(5))
        self.assertTrue(w.accept(3))  # late but new
        self.assertFalse(w.accept(3))
        self.assertTrue(w.accept(2))
        self.assertTrue(w.accept(4))
        self.assertTrue(w.accept(100))
        self.assertTrue(w.accept(37))  # 63 behind the top: still inside the window
        self.assertFalse(w.accept(36))  # 64 behind: too old
        self.assertFalse(w.accept(5))
        self.assertTrue(w.accept(100 + 1000))  # big jump
        self.assertFalse(w.accept(100))

    def test_random_order_accepts_each_once(self):
        rng = random.Random(1)
        seqs = list(range(1, 2000))
        # shuffle locally so that nothing is more than 30 packets late
        for i in range(0, len(seqs), 30):
            chunk = seqs[i:i + 30]
            rng.shuffle(chunk)
            seqs[i:i + 30] = chunk
        w = mr.ReplayWindow()
        self.assertTrue(all(w.accept(s) for s in seqs))
        self.assertFalse(any(w.accept(s) for s in seqs[-40:]))


class LossMeterTest(unittest.TestCase):
    def test_loss(self):
        m = mr.LossMeter()
        self.assertIsNone(m.permille())
        for seq in range(1, 101):
            if seq % 10:
                m.packet(seq)
        m.roll()
        self.assertEqual(m.permille(), 91)  # 9 of 99 lost (100 is lost too, but nothing above it arrived)
        m.packet(10)  # arrives late: counts as received after all
        m.roll()
        self.assertEqual(m.permille(), 81)
        for _ in range(20):
            m.packet(10)  # can never push the loss below zero
        m.roll()
        self.assertEqual(m.permille(), 0)


class FramerTest(unittest.TestCase):
    def test_boundaries(self):
        frames = [v2_frame(0, b"\xfd" * 9), v1_frame(0, b"\xfe" * 9), v2_frame(33, bytes(28), signed=True),
                  v2_frame(1, b"")]
        stream = b"".join(frames)
        ends, pos = [], 0
        for f in frames:
            pos += len(f)
            ends.append(pos)
        self.assertEqual(boundaries(stream), ends)

    def test_stray_bytes_and_bad_flags(self):
        self.assertEqual(boundaries(b"\x00\x01\x02"), [1, 2, 3])
        # 0xFD followed by an unknown incompatibility flag is not a frame
        self.assertEqual(boundaries(b"\xfd\x05\x80" + v2_frame()), [3, 3 + len(v2_frame())])


class BatcherTest(unittest.TestCase):
    def assert_whole_frames(self, chunk):
        framer = mr.MavFramer()
        for b in chunk:
            framer.push(b)
        self.assertFalse(framer.in_frame, "chunk ends inside a frame")

    def test_stream_is_preserved_and_cut_on_boundaries(self):
        rng = random.Random(7)
        stream = b"".join(random_frames(rng, 300))
        batcher = mr.Batcher()
        out, t, pos = [], 0.0, 0
        while pos < len(stream):
            n = rng.randrange(1, 200)
            out += batcher.feed(stream[pos:pos + n], t)
            pos += n
            t += 0.01
            chunk = batcher.poll(t, 0.05)
            if chunk:
                out.append(chunk)
        t += 1.0
        chunk = batcher.poll(t, 0.05)
        if chunk:
            out.append(chunk)
        self.assertEqual(b"".join(out), stream)
        for chunk in out:
            self.assertLessEqual(len(chunk), mr.MAX_PAYLOAD)
            self.assert_whole_frames(chunk)

    def test_waits_for_max_age(self):
        b = mr.Batcher()
        self.assertEqual(b.feed(v2_frame(), 10.0), [])
        self.assertIsNone(b.poll(10.049, 0.05))
        self.assertEqual(b.poll(10.05, 0.05), v2_frame())
        self.assertIsNone(b.poll(11.0, 0.05))

    def test_holds_partial_frame(self):
        frame = v2_frame(payload=bytes(40))
        b = mr.Batcher()
        b.feed(v2_frame() + frame[:10], 0.0)
        self.assertEqual(b.poll(0.1, 0.05), v2_frame())
        self.assertIsNone(b.poll(0.2, 0.05))
        b.feed(frame[10:], 0.3)
        self.assertEqual(b.poll(0.3, 0.05), frame)  # age counts from the frame's first byte

    def test_incomplete_frame_times_out(self):
        b = mr.Batcher(frame_timeout=0.5)
        b.feed(b"\xfe\xff\x00", 0.0)  # claims a 263-byte frame that never comes
        self.assertIsNone(b.poll(0.4, 0.0))
        self.assertEqual(b.poll(0.5, 0.0), b"\xfe\xff\x00")
        b.feed(v2_frame(), 0.6)
        self.assertEqual(b.poll(0.6, 0.0), v2_frame())

    def test_full_buffer_is_cut_at_last_boundary(self):
        frame = v2_frame(payload=bytes(200))  # 212 bytes
        b = mr.Batcher()
        out = b.feed(frame * 6, 0.0)  # 1272 bytes > 1172
        self.assertEqual(out, [frame * 5])
        self.assertEqual(b.poll(1.0, 0.05), frame)

    def test_garbage_longer_than_a_datagram(self):
        b = mr.Batcher()
        junk = bytes(3000)
        out = b.feed(junk, 0.0)
        tail = b.poll(0.0, 0.0)
        self.assertEqual(b"".join(out) + (tail or b""), junk)
        self.assertTrue(all(len(c) <= mr.MAX_PAYLOAD for c in out))


class ConfigTest(unittest.TestCase):
    def test_parse_hostport(self):
        self.assertEqual(mr.parse_hostport("example.com:14650"), ("example.com", 14650))
        self.assertEqual(mr.parse_hostport("[::1]:5760"), ("::1", 5760))
        self.assertEqual(mr.parse_hostport(":14650", "0.0.0.0"), ("0.0.0.0", 14650))
        with self.assertRaises(ValueError):
            mr.parse_hostport("example.com")

    def test_parse_key(self):
        self.assertEqual(mr.parse_key(KEY_V.hex()), KEY_V)
        with self.assertRaises(ValueError):
            mr.parse_key("xyz")
        with self.assertRaises(ValueError):
            mr.parse_key("00ff")

    def test_server_config_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "relay.ini")
            with open(path, "w") as f:
                f.write("[server]\nlisten = 0.0.0.0:15000\nvehicle_key = %s\ngcs_key = %s  # comment\n"
                        "tcp_listen = 0.0.0.0:5760\ntcp_allow = 192.0.2.0/24\n" % (KEY_V.hex(), KEY_G.hex()))
            args = mr.build_parser().parse_args(["server", "--config", path, "--listen", ":16000"])
            opts = mr.resolve_options(args)
        self.assertEqual(opts.listen, ("", 16000))  # command line wins over the file
        self.assertEqual((opts.vehicle_key, opts.gcs_key), (KEY_V, KEY_G))
        self.assertEqual(opts.tcp_listen, ("0.0.0.0", 5760))
        self.assertEqual([str(n) for n in opts.tcp_allow], ["192.0.2.0/24"])

    def test_gcs_and_vehicle_config_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            with open(path, "w") as f:
                f.write("[gcs]\nserver = relay.example.com:14650\nkey = %s\ntcp = off\n\n"
                        "[vehicle]\nserver = relay.example.com:14650\nkey = %s\ntcp = 127.0.0.1:5762\n"
                        "always_send = yes\n" % (KEY_G.hex(), KEY_V.hex()))
            gcs = mr.resolve_options(mr.build_parser().parse_args(["gcs", "--config", path]))
            vehicle = mr.resolve_options(mr.build_parser().parse_args(["vehicle", "--config", path]))
        self.assertEqual(gcs.server, ("relay.example.com", 14650))
        self.assertEqual(gcs.key, KEY_G)
        self.assertEqual(gcs.udp, ("127.0.0.1", 14550))  # default
        self.assertIsNone(gcs.tcp)
        self.assertEqual((vehicle.key, vehicle.tcp, vehicle.always_send), (KEY_V, "127.0.0.1:5762", True))
        self.assertEqual(vehicle.batch_ms, 50)

    def test_update_config_keeps_the_rest(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            with open(path, "w") as f:
                f.write("# notes\n[server]\nlisten = 0.0.0.0:14650  # port\n\n[gcs]\n# the relay\nserver = old:1\n"
                        "udp = off\n\n[vehicle]\nkey = abc\n")
            mr.update_config(path, "gcs", {"server": "new:2", "key": "k1"})
            mr.update_config(os.path.join(tmp, "fresh.ini"), "gcs", {"key": "k2"})
            with open(path) as f:
                text = f.read()
            with open(os.path.join(tmp, "fresh.ini")) as f:
                fresh = f.read()
        self.assertEqual(text, "# notes\n[server]\nlisten = 0.0.0.0:14650  # port\n\n[gcs]\n# the relay\n"
                               "server = new:2\nudp = off\nkey = k1\n\n[vehicle]\nkey = abc\n")
        self.assertEqual(fresh, "[gcs]\nkey = k2\n")

    def test_a_header_with_a_comment(self):
        """The 1.5.2 review found update_config adding a second [gcs] after "[gcs]  # ..." (read fine), after
        which MavLTE did not start any more."""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            with open(path, "w") as f:
                f.write("[gcs]  # laptop agent\nserver = a:1\n\n[vehicle] ; the plane\nkey = abc\n")
            mr.update_config(path, "gcs", {"tcp": "127.0.0.1:5760"})
            mr.update_config(path, "vehicle", {"key": "def"})
            with open(path) as f:
                text = f.read()
            conf = mr.load_config(path, "gcs")
        self.assertEqual(text, "[gcs]  # laptop agent\nserver = a:1\ntcp = 127.0.0.1:5760\n\n[vehicle] ; the plane\n"
                               "key = def\n")
        self.assertEqual(conf, {"server": "a:1", "tcp": "127.0.0.1:5760"})
        self.assertEqual([mr.ini_section(line) for line in ("[gcs]", "  [gcs] # x", "[gcs]#x", "# [gcs]", "key = [x]")],
                         ["gcs", "gcs", "gcs", None, None])  # as configparser reads them

    def test_a_section_written_twice_is_read(self):  # as 1.5.2 could leave it: the later value wins
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            with open(path, "w") as f:
                f.write("[gcs]  # laptop agent\nserver = a:1\ntcp = 10.1.1.1:1\n\n[gcs]\ntcp = 127.0.0.1:5760\n")
            self.assertEqual(mr.load_config(path, "gcs"), {"server": "a:1", "tcp": "127.0.0.1:5760"})

    def test_an_unreadable_file_says_why(self):
        with tempfile.TemporaryDirectory() as tmp:
            broken, ansi = os.path.join(tmp, "broken.ini"), os.path.join(tmp, "ansi.ini")
            with open(broken, "w") as f:
                f.write("[gcs]\nserver = a:1\nthis line means nothing\n")
            with open(ansi, "wb") as f:
                f.write("[gcs]\nname = Kuş\n".encode("cp1254"))  # saved as Turkish ANSI, not UTF-8
            for path in (broken, ansi):
                with self.assertRaises(SystemExit) as caught:
                    mr.load_config(path, "gcs")
                self.assertIn(f"cannot read config file {path}: ", str(caught.exception))

    def test_values_are_read_as_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            mr.update_config(path, "gcs", {"name": "33% Cub", "udp": "[fe80::1%12]:14550"})
            conf = mr.load_config(path, "gcs")
        self.assertEqual((conf["name"], conf["udp"]), ("33% Cub", "[fe80::1%12]:14550"))  # no % interpolation

    def test_vehicle_source_on_the_command_line_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "mavrelay.ini")
            with open(path, "w") as f:
                f.write("[vehicle]\nserver = relay.example.com:14650\nkey = %s\ntcp = 127.0.0.1:5762\n" % KEY_V.hex())
            given = mr.resolve_options(mr.build_parser().parse_args(["vehicle", "--config", path,
                                                                     "--udp", "0.0.0.0:14560"]))
            from_file = mr.resolve_options(mr.build_parser().parse_args(["vehicle", "--config", path]))
        self.assertEqual((given.serial, given.tcp, given.udp), (None, None, "0.0.0.0:14560"))
        self.assertEqual((from_file.tcp, from_file.udp), ("127.0.0.1:5762", None))

    def test_tcp_allow_default_is_loopback(self):
        args = mr.build_parser().parse_args(["server", "--vehicle-key", KEY_V.hex(), "--gcs-key", KEY_G.hex()])
        port = mr.TcpGcsPort(None, mr.resolve_options(args).tcp_allow)
        self.assertTrue(port.allowed("127.0.0.1"))
        self.assertTrue(port.allowed("::1"))
        self.assertTrue(port.allowed("::ffff:127.0.0.1"))
        self.assertFalse(port.allowed("203.0.113.9"))


class UdpPlanTest(unittest.TestCase):
    """What the agent's UDP port does with each kind of address: (send to, bind)."""

    def test_loopback_sends_to_gcs_software_on_this_computer(self):  # the default, as ever
        self.assertEqual(mr.udp_plan("127.0.0.1", 14550, own=True), (("127.0.0.1", 14550), ("127.0.0.1", 0)))
        self.assertEqual(mr.udp_plan("", 14550), (("127.0.0.1", 14550), ("127.0.0.1", 0)))
        self.assertEqual(mr.udp_plan("::1", 14550), (("::1", 14550), ("::1", 0)))

    def test_all_addresses_or_one_of_this_computer_listen(self):  # as the TCP port does
        self.assertEqual(mr.udp_plan("0.0.0.0", 14550), (None, ("0.0.0.0", 14550)))
        self.assertEqual(mr.udp_plan("::", 14550), (None, ("::", 14550)))
        self.assertEqual(mr.udp_plan("192.168.2.178", 14550, own=True), (None, ("192.168.2.178", 14550)))

    def test_another_computer_is_sent_to(self):
        self.assertEqual(mr.udp_plan("192.168.2.50", 14550), (("192.168.2.50", 14550), ("0.0.0.0", 0)))

    def test_own_addresses(self):
        self.assertTrue(mr.is_own_address("127.0.0.1"))
        self.assertFalse(mr.is_own_address("192.0.2.1"))  # TEST-NET-1: nobody's
        self.assertFalse(mr.is_own_address("relay.example.com"))  # names are resolved before
        lan = mr.lan_address()
        if lan is not None:
            self.assertTrue(mr.is_own_address(lan))


class VersionTest(unittest.TestCase):
    def test_one_version_for_everything(self):
        # the firmware and the PC and server side always carry the same number
        path = os.path.join(os.path.dirname(__file__), "..", "..", "firmware", "main", "version.h")
        with open(path, encoding="utf-8") as f:
            firmware = re.search(r'#define FIRMWARE_VERSION "([^"]+)"', f.read()).group(1)
        self.assertEqual(firmware, mr.__version__)


class HelloReplayTest(unittest.TestCase):
    """A HELLO carries no sequence number: captured ones replayed or flooded from one address must
    not keep anyone else from getting a session (driven packet by packet, no timing involved)."""

    PLANE = ("203.0.113.5", 5000)
    ATTACKER = ("198.51.100.7", 4000)

    def setUp(self):
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.sent = []
        sent = self.sent

        class Transport:
            def sendto(self, data, addr):
                sent.append((mr.decode(data), addr))

        self.relay.connection_made(Transport())

    def hello(self, key, role, nonce, addr):
        self.relay.datagram_received(mr.encode(key, mr.HELLO, role, 0, 0, nonce + b"test"), addr)

    def welcomes_to(self, addr):
        return [p.session for p, a in self.sent if a == addr and p.type == mr.WELCOME]

    def test_replays_and_floods_do_not_block_a_new_session(self):
        self.hello(KEY_V, mr.ROLE_VEHICLE, b"plane-01", self.PLANE)
        [sid] = self.welcomes_to(self.PLANE)
        self.hello(KEY_V, mr.ROLE_VEHICLE, b"plane-01", self.PLANE)  # a retry gets the same session
        self.assertEqual(self.welcomes_to(self.PLANE), [sid, sid])

        captured = mr.encode(KEY_G, mr.HELLO, mr.ROLE_GCS, 0, 0, b"laptop01" + b"agent")
        for _ in range(100):
            self.relay.datagram_received(captured, self.ATTACKER)
        self.assertEqual(sum(s.nonce == b"laptop01" for s in self.relay.sessions.values()), 1)
        for i in range(100):  # many different captured HELLOs, all from one address
            self.hello(KEY_G, mr.ROLE_GCS, i.to_bytes(8, "big"), self.ATTACKER)
        self.assertIn(sid, self.relay.sessions)  # they only pushed out each other
        self.assertLessEqual(sum(not s.active for s in self.relay.sessions.values()), self.relay.MAX_PENDING)

        body = mr.PING_BODY.pack(0, mr.U16_UNKNOWN, mr.U16_UNKNOWN, mr.RSSI_UNKNOWN, mr.RAT_UNKNOWN, 0)
        self.relay.datagram_received(mr.encode(KEY_V, mr.PING, mr.ROLE_VEHICLE, sid, 1, body), self.PLANE)
        self.assertIs(self.relay.vehicle, self.relay.sessions[sid])

        before = (len(self.sent), len(self.relay.sessions))
        self.hello(KEY_V, mr.ROLE_VEHICLE, b"plane-01", self.ATTACKER)  # the plane's HELLO, replayed
        self.assertEqual((len(self.sent), len(self.relay.sessions)), before)  # no answer, no new session


class LiveTest(unittest.IsolatedAsyncioTestCase):
    """Server and clients on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop  # same event loop as mavrelay uses on Windows

    async def asyncSetUp(self):
        self.tasks = []
        self.port = await self.start_server()

    async def asyncTearDown(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.server_transport.close()

    async def start_server(self, port=0):
        loop = asyncio.get_running_loop()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.server_transport, _ = await loop.create_datagram_endpoint(lambda: self.relay,
                                                                       local_addr=("127.0.0.1", port))
        relay = self.relay

        async def ticker():
            while True:
                await asyncio.sleep(0.2)
                relay.tick(time.monotonic())

        self.tasks.append(asyncio.ensure_future(ticker()))
        return self.server_transport.get_extra_info("sockname")[1]

    def client(self, role, key=None):
        inbox = []
        status = []
        c = mr.TunnelClient(role, key or (KEY_V if role == mr.ROLE_VEHICLE else KEY_G), "127.0.0.1", self.port,
                            on_data=inbox.append, on_status=status.append, info="test")
        c.inbox, c.status = inbox, status
        c.task = asyncio.ensure_future(c.run())
        self.tasks.append(c.task)
        return c

    async def until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail("condition not met in time")
            await asyncio.sleep(0.02)

    async def connected_pair(self):
        vehicle = self.client(mr.ROLE_VEHICLE)
        gcs = self.client(mr.ROLE_GCS)
        await self.until(lambda: vehicle.is_connected and gcs.is_connected)
        await self.until(lambda: self.relay.vehicle is not None and len(self.relay.gcs_sessions()) == 1)
        return vehicle, gcs

    async def test_data_both_ways(self):
        vehicle, gcs = await self.connected_pair()
        telemetry = [v2_frame(0, bytes([i]) * 9, i) for i in range(50)]
        for frame in telemetry:
            self.assertTrue(vehicle.send_data(frame))
        await self.until(lambda: len(gcs.inbox) == 50)
        self.assertEqual(gcs.inbox, telemetry)
        gcs.send_data(v2_frame(76, bytes(33)))
        await self.until(lambda: vehicle.inbox == [v2_frame(76, bytes(33))])
        self.assertEqual(self.relay.vehicle.info, "test")

    async def test_a_burst_to_the_aircraft_goes_in_few_packets(self):
        # the aircraft's modem holds only about 10 packets: a burst of small messages from GCS software goes
        # packed, in order; a message after a quiet moment goes at once, by itself
        vehicle, gcs = await self.connected_pair()
        burst = [v2_frame(76, bytes([i]) * 33, i) for i in range(57)]  # as Mission Planner sends them at times
        for frame in burst:
            self.assertTrue(gcs.send_data(frame))
        await self.until(lambda: b"".join(vehicle.inbox) == b"".join(burst))
        self.assertLess(len(vehicle.inbox), 10)
        await asyncio.sleep(0.05)
        vehicle.inbox.clear()
        start = time.monotonic()
        gcs.send_data(v2_frame(0, bytes(9), 99))
        await self.until(lambda: vehicle.inbox == [v2_frame(0, bytes(9), 99)])
        self.assertLess(time.monotonic() - start, 1.0)

    async def test_status_and_gcs_presence(self):
        vehicle = self.client(mr.ROLE_VEHICLE)
        await self.until(lambda: vehicle.is_connected)
        await self.until(lambda: vehicle.rtt_ms != mr.U16_UNKNOWN)
        self.assertFalse(vehicle.gcs_present)
        gcs = self.client(mr.ROLE_GCS)
        await self.until(lambda: vehicle.gcs_present, timeout=5)
        # the vehicle reports its round-trip time in the PING after it first measured it
        await self.until(lambda: any(s.online and s.rtt_ms < 1000 and s.up_loss == 0 for s in gcs.status))

    async def test_watching_gcs_gets_status_but_no_telemetry(self):
        vehicle, gcs = await self.connected_pair()
        await self.until(lambda: vehicle.gcs_present)
        gcs.ping_flags = mr.PING_FLAG_WATCHING
        gcs.ping_now()
        await self.until(lambda: self.relay.gcs_sessions()[0].watching)
        await self.until(lambda: not vehicle.gcs_present)  # a watcher is not a GCS for the vehicle
        vehicle.send_data(v2_frame(0, bytes(9), 1))
        await self.until(lambda: any(s.online for s in gcs.status))  # it is still told about the vehicle
        await asyncio.sleep(0.3)
        self.assertEqual(gcs.inbox, [])  # but gets none of its telemetry
        gcs.ping_flags = 0
        gcs.ping_now()
        await self.until(lambda: vehicle.gcs_present)
        vehicle.send_data(v2_frame(0, bytes(9), 2))
        await self.until(lambda: gcs.inbox == [v2_frame(0, bytes(9), 2)])

    async def test_agent_only_watches_without_ports(self):
        vehicle = self.client(mr.ROLE_VEHICLE)
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks.append(asyncio.ensure_future(agent.run()))
        self.assertTrue(agent.watching)
        await self.until(lambda: agent.vehicle_status() is not None and agent.vehicle_status().online)
        await self.until(lambda: not vehicle.gcs_present)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        self.addCleanup(probe.close)
        await agent.start_udp(probe.getsockname())
        self.assertFalse(agent.watching)
        await self.until(lambda: vehicle.gcs_present)
        agent.stop_udp()
        self.assertTrue(agent.watching)
        await self.until(lambda: not vehicle.gcs_present)

    async def test_mistyped_host_keeps_the_client_retrying(self):
        # an empty label makes getaddrinfo raise UnicodeError, not OSError
        client = mr.TunnelClient(mr.ROLE_GCS, KEY_G, "10.0.0..1", self.port, on_data=lambda data: None)
        task = asyncio.ensure_future(client.run())
        self.tasks.append(task)
        await asyncio.sleep(0.3)
        self.assertFalse(task.done())  # still running, and will look the name up again in 5 s
        self.assertIsNone(client.server_addr)

    async def test_forged_and_replayed_packets_are_ignored(self):
        vehicle, gcs = await self.connected_pair()
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(probe.close)
        server = ("127.0.0.1", self.port)
        # right session, wrong key
        probe.sendto(mr.encode(KEY_G, mr.DATA, mr.ROLE_VEHICLE, vehicle.session, 999, b"evil"), server)
        # correct packet, replayed from another address
        vehicle.send_data(b"once")
        await self.until(lambda: gcs.inbox == [b"once"])
        replay = mr.encode(KEY_V, mr.DATA, mr.ROLE_VEHICLE, vehicle.session, vehicle.tx_seq, b"once")
        probe.sendto(replay, server)
        probe.sendto(b"\xfd\x09" + bytes(30), server)  # raw MAVLink is not accepted either
        await asyncio.sleep(0.3)
        self.assertEqual(gcs.inbox, [b"once"])
        self.assertGreaterEqual(self.relay.counters["bad_tag"], 1)
        self.assertGreaterEqual(self.relay.counters["replayed"], 1)
        self.assertEqual(self.relay.vehicle.addr[1], vehicle.transport.get_extra_info("sockname")[1])

    async def test_hello_with_wrong_key_gets_no_answer(self):
        vehicle = self.client(mr.ROLE_VEHICLE, key=KEY_G)
        await asyncio.sleep(1.5)
        self.assertFalse(vehicle.is_connected)
        self.assertEqual(self.relay.sessions, {})

    async def test_vehicle_restart_replaces_session(self):
        old, gcs = await self.connected_pair()
        old_sid = old.session
        old.task.cancel()  # the ESP32 reboots...
        new = self.client(mr.ROLE_VEHICLE)  # ...and comes back with a new session
        await self.until(lambda: new.is_connected and self.relay.vehicle is not None
                         and self.relay.vehicle.sid == new.session)
        self.assertNotIn(old_sid, self.relay.sessions)
        new.send_data(b"from new")
        await self.until(lambda: gcs.inbox == [b"from new"])

    async def test_server_restart_clients_reconnect(self):
        vehicle, gcs = await self.connected_pair()
        self.server_transport.close()
        await asyncio.sleep(0.1)
        await self.start_server(self.port)  # fresh server, same port, no sessions
        await self.until(lambda: self.relay.vehicle is not None and len(self.relay.gcs_sessions()) == 1, timeout=6)
        vehicle.send_data(b"after restart")
        await self.until(lambda: b"after restart" in gcs.inbox)

    async def test_client_recovers_from_silent_server(self):
        original = mr.LINK_TIMEOUT
        mr.LINK_TIMEOUT = 1.0
        self.addCleanup(setattr, mr, "LINK_TIMEOUT", original)
        vehicle = self.client(mr.ROLE_VEHICLE)
        await self.until(lambda: vehicle.is_connected)
        first = vehicle.session
        self.relay.sessions.clear()  # the server forgets without telling anyone...
        self.relay.vehicle = None
        self.relay.datagram_received = lambda data, addr: None  # ...and stops answering
        await self.until(lambda: not vehicle.is_connected, timeout=3)
        del self.relay.datagram_received
        await self.until(lambda: vehicle.is_connected and vehicle.session != first, timeout=4)

    async def test_plain_tcp_port(self):
        self.relay.tcp = mr.TcpGcsPort(self.relay, [mr.ipaddress.ip_network("127.0.0.0/8")])
        await self.relay.tcp.start("127.0.0.1", 0)
        tcp_port = self.relay.tcp.server.sockets[0].getsockname()[1]
        vehicle = self.client(mr.ROLE_VEHICLE)
        await self.until(lambda: vehicle.is_connected)
        reader, writer = await asyncio.open_connection("127.0.0.1", tcp_port)
        await self.until(lambda: len(self.relay.tcp.clients) == 1)
        await self.until(lambda: vehicle.gcs_present)
        vehicle.send_data(v2_frame(0, bytes(9), 1))
        self.assertEqual(await asyncio.wait_for(reader.readexactly(21), 3), v2_frame(0, bytes(9), 1))
        # a command split over two TCP writes arrives at the vehicle as one whole frame
        cmd = v2_frame(76, bytes(33), 5)
        writer.write(cmd[:7])
        await writer.drain()
        await asyncio.sleep(0.05)
        writer.write(cmd[7:])
        await writer.drain()
        await self.until(lambda: vehicle.inbox == [cmd])
        writer.close()
        self.relay.tcp.server.close()

    async def test_plain_tcp_port_refuses_other_addresses(self):
        self.relay.tcp = mr.TcpGcsPort(self.relay, [mr.ipaddress.ip_network("192.0.2.0/24")])
        await self.relay.tcp.start("127.0.0.1", 0)
        tcp_port = self.relay.tcp.server.sockets[0].getsockname()[1]
        reader, writer = await asyncio.open_connection("127.0.0.1", tcp_port)
        self.assertEqual(await asyncio.wait_for(reader.read(), 3), b"")
        self.assertEqual(self.relay.tcp.clients, set())
        writer.close()
        self.relay.tcp.server.close()

    def gcs_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        s.setblocking(False)
        self.addCleanup(s.close)
        return s

    async def telemetry_flows(self, vehicle):
        """Until the relay passes the vehicle's telemetry on to a GCS agent: one not only watching."""
        await self.until(lambda: vehicle.is_connected and any(not s.watching for s in self.relay.gcs_sessions()))

    async def received(self, sock, timeout=3.0):
        end = time.monotonic() + timeout
        while True:
            try:
                return sock.recvfrom(4096)
            except BlockingIOError:
                if time.monotonic() > end:
                    self.fail("nothing received")
                await asyncio.sleep(0.02)

    async def test_agent_listens_on_udp_for_gcs_software_on_any_computer(self):
        vehicle = self.client(mr.ROLE_VEHICLE)
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks.append(asyncio.ensure_future(agent.run()))
        self.addCleanup(agent.close_ports)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        await agent.start_udp(("0.0.0.0", port))  # 0.0.0.0: it listens, as the TCP port does
        udp = agent.udp
        self.assertEqual((udp.target, udp.local), (None, ("0.0.0.0", port)))
        self.assertFalse(agent.watching)
        thief = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(thief.close)
        with self.assertRaises(OSError):
            thief.bind(("127.0.0.1", port))  # the port is the agent's alone

        laptop, phone = self.gcs_socket(), self.gcs_socket()  # GCS software (Mission Planner: UDPCl) on two computers
        for gcs in (laptop, phone):
            gcs.sendto(v2_frame(0, bytes(9), 1), ("127.0.0.1", port))  # a heartbeat says it is there
        await self.until(lambda: udp.gcs_count() == 2)
        await self.telemetry_flows(vehicle)
        vehicle.send_data(v2_frame(30, bytes(28), 7))
        for gcs in (laptop, phone):
            data, addr = await self.received(gcs)
            self.assertEqual((data, addr[1]), (v2_frame(30, bytes(28), 7), port))  # from the port it sends to
        phone.sendto(v2_frame(76, bytes(33), 2), ("127.0.0.1", port))
        await self.until(lambda: v2_frame(76, bytes(33), 2) in vehicle.inbox)

        udp.PEER_TIMEOUT = 0.5  # the laptop goes quiet; the phone does not
        await asyncio.sleep(0.6)
        phone.sendto(v2_frame(0, bytes(9), 3), ("127.0.0.1", port))
        await self.until(lambda: udp.gcs_count(within=0.3) == 1)
        vehicle.send_data(v2_frame(30, bytes(28), 8))
        self.assertEqual((await self.received(phone))[0], v2_frame(30, bytes(28), 8))
        await asyncio.sleep(0.2)
        with self.assertRaises(BlockingIOError):
            laptop.recvfrom(4096)  # nothing for a GCS that left
        self.assertEqual(len(udp.peers), 1)

        await agent.start_udp(("0.0.0.0", port))  # on again at once: the same port is free for it
        self.assertEqual(agent.udp.local, ("0.0.0.0", port))

    async def test_agent_udp_port_problems_are_reported(self):
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        busy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        busy.bind(("127.0.0.1", 0))  # another program has the port
        self.addCleanup(busy.close)
        with self.assertRaises(OSError):
            await agent.start_udp(("0.0.0.0", busy.getsockname()[1]))
        with self.assertRaises(socket.gaierror):
            await agent.start_udp(("10.0.0..1", 14550))  # a mistyped address
        self.assertIsNone(agent.udp)
        self.assertTrue(agent.watching)
        with self.assertRaises(socket.gaierror):  # the same on TCP (it raised UnicodeEncodeError until 1.5.3)
            await agent.start_tcp(("10.0.0..1", 5760))
        self.assertIsNone(agent.tcp)
        self.assertTrue(agent.watching)
        lan = mr.lan_address()
        if lan is not None:  # this computer's own address: it listens there
            await agent.start_udp((lan, 0))
            self.addCleanup(agent.close_ports)
            self.assertEqual((agent.udp.target, agent.udp.local[0]), (None, lan))

    async def test_agent_udp_to_gcs_software_on_this_computer(self):  # the default: Mission Planner on 14550
        vehicle = self.client(mr.ROLE_VEHICLE)
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.tasks.append(asyncio.ensure_future(agent.run()))
        self.addCleanup(agent.close_ports)
        planner, stranger = self.gcs_socket(), self.gcs_socket()
        await agent.start_udp(("localhost", planner.getsockname()[1]))
        self.assertEqual(agent.udp.target, ("127.0.0.1", planner.getsockname()[1]))  # as its answers show it
        await self.telemetry_flows(vehicle)
        vehicle.send_data(v2_frame(30, bytes(28), 1))
        data, agent_addr = await self.received(planner)
        planner.sendto(v2_frame(0, bytes(9), 1), agent_addr)
        stranger.sendto(v2_frame(0, bytes(9), 2), agent_addr)
        await self.until(lambda: len(vehicle.inbox) == 2)
        vehicle.send_data(v2_frame(30, bytes(28), 2))
        self.assertEqual((await self.received(planner))[0], v2_frame(30, bytes(28), 2))  # once, not twice
        await asyncio.sleep(0.2)
        for sock in (planner, stranger):  # telemetry goes to the address set, and to no one else
            with self.assertRaises(BlockingIOError):
                sock.recvfrom(4096)


if __name__ == "__main__":
    unittest.main()

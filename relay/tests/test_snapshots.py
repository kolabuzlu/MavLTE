"""Tests for snapshots, photos from the aircraft on request. Run from the relay directory:
python -m unittest -v"""

import asyncio
import json
import os
import random
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mavrelay as mr  # noqa: E402

KEY_V = bytes(range(32))
KEY_G = bytes(range(100, 132))
WHERE = (411234567, 289876543, 120_000, 4500)  # over Istanbul, 120 m up, heading 45 degrees


def photo_bytes(size, seed=1):
    rng = random.Random(seed)
    return bytes(rng.randrange(256) for _ in range(size))


class PhotoPartsTest(unittest.TestCase):
    def test_info_roundtrip(self):
        info = mr.PhotoInfo(1790000000, 45000, 640, 480, *WHERE, mr.SNAP_OK, 1790000003)
        self.assertEqual(mr.SNAP_INFO_BODY.size, 31)
        self.assertEqual(mr.PhotoInfo.unpack(info.pack()), info)
        self.assertEqual(info.chunks, 44)
        self.assertEqual(mr.PhotoInfo(1).chunks, 0)
        self.assertEqual(mr.PhotoInfo.unpack(mr.PhotoInfo(7, status=mr.SNAP_BUSY).pack()).lat, mr.UNKNOWN_I32)

    def test_largest_packets_fit(self):
        self.assertLessEqual(mr.SNAP_DATA_HEAD.size + mr.SNAP_CHUNK, mr.MAX_PAYLOAD)
        bitmap = (mr.SNAP_MAX_BYTES // mr.SNAP_CHUNK + 7) // 8
        self.assertLessEqual(mr.SNAP_ACK_HEAD.size + bitmap, mr.MAX_PAYLOAD)

    def test_receiver_in_any_order(self):
        data = photo_bytes(2500)  # chunks of 1024, 1024 and 452
        receiver = mr.PhotoReceiver(5)
        receiver.on_data(2, data[2048:], 0.0)  # before its SNAP_INFO
        receiver.on_data(1, b"too short", 0.0)  # kept for now: the length is not known yet
        receiver.on_info(mr.PhotoInfo(5, len(data)), 0.0)
        self.assertEqual(sorted(receiver.parts), [2])  # the short one went when the size came
        receiver.on_data(3, data[:10], 0.0)  # no chunk 3 in 2500 bytes
        receiver.on_data(0, data[:1000], 0.0)  # wrong length
        self.assertFalse(receiver.complete)
        receiver.on_data(0, data[:1024], 0.0)
        receiver.on_data(1, data[1024:2048], 0.0)
        receiver.on_data(1, b"x" * 1024, 0.0)  # a copy that came late changes nothing
        self.assertTrue(receiver.complete)
        self.assertEqual(receiver.data(), data)
        self.assertEqual(receiver.ack(), mr.SNAP_ACK_HEAD.pack(5, mr.ACK_DONE | mr.ACK_HAVE_INFO) + b"\x07")

    def test_ack_bitmap(self):
        receiver = mr.PhotoReceiver(9)
        for i in (0, 3, 9):
            receiver.on_data(i, bytes(1024), 0.0)
        self.assertEqual(receiver.ack(), mr.SNAP_ACK_HEAD.pack(9, 0) + bytes([0b00001001, 0b00000010]))
        self.assertEqual(mr.PhotoReceiver(9).ack(), mr.SNAP_ACK_HEAD.pack(9, 0))

    def test_too_large_or_empty_photo_is_refused(self):
        for size in (0, mr.SNAP_MAX_BYTES + 1):
            receiver = mr.PhotoReceiver(1)
            receiver.on_info(mr.PhotoInfo(1, size), 0.0)
            self.assertIsNone(receiver.info)


class RateControlTest(unittest.TestCase):
    def test_backs_off_when_a_queue_builds_up(self):
        control = mr.RateControl(32768, start=32768)
        control.rtt_sample(0.10, 0.0)
        control.rtt_sample(0.20, 1.0)  # 100 ms more: within the allowance of 150 ms
        self.assertEqual(control.rate, 32768)
        control.rtt_sample(0.40, 2.0)
        self.assertEqual(control.rate, 16384)
        control.rtt_sample(0.40, 2.5)  # at most once a second
        self.assertEqual(control.rate, 16384)
        for t in range(3, 20):
            control.rtt_sample(2.0, float(t))
        self.assertEqual(control.rate, control.FLOOR)
        for t in range(20, 40):  # the queue drains: back up, a tenth of the cap each time
            control.rtt_sample(0.10, float(t))
        self.assertEqual(control.rate, 32768)

    def test_slow_link_allowance_grows_with_its_round_trip(self):
        control = mr.RateControl(2048, start=2048)
        control.rtt_sample(1.0, 0.0)  # 2G: a second is normal
        control.rtt_sample(1.45, 1.0)  # half the base more is still fine
        self.assertEqual(control.rate, 2048)
        control.rtt_sample(1.6, 2.0)
        self.assertEqual(control.rate, 1024)

    def test_budget(self):
        control = mr.RateControl(10000, start=10000)
        self.assertEqual(control.budget(0.0), 0)
        self.assertAlmostEqual(control.budget(0.1), 1000)
        control.spend(1000)
        self.assertAlmostEqual(control.budget(10.0), 2 * mr.SNAP_CHUNK)  # no big burst after a pause
        control.set_cap(1000)
        self.assertEqual(control.rate, 1000)


class TransferTest(unittest.TestCase):
    """A PhotoSender and a PhotoReceiver over a wire that loses and reorders packets, on a simulated
    clock (the receiver ACKs as the relay and the agent do: twice a second while something arrives)."""

    def transfer(self, data, loss, rate=16384.0, seed=3, limit=300.0):
        rng = random.Random(seed)
        info = mr.PhotoInfo(1000, len(data), 640, 480, *WHERE)
        receiver = mr.PhotoReceiver(1000)
        wire = []
        clock = [0.0]
        self.sent_bytes = 0

        def send(ptype, body):
            self.sent_bytes += len(body)
            if rng.random() >= loss:
                wire.append((clock[0] + 0.05 + rng.random() * 0.1, "receiver", ptype, body))

        sender = mr.PhotoSender(info, mr.BytesSource(data), send, 0.0)
        control = mr.RateControl(rate, start=rate)
        while not sender.done and clock[0] < limit:
            now = clock[0]
            for item in sorted(i for i in wire if i[0] <= now):
                wire.remove(item)
                _, to, ptype, body = item
                if to == "sender":
                    photo_id, flags = mr.SNAP_ACK_HEAD.unpack_from(body)
                    sender.on_ack(flags, body[mr.SNAP_ACK_HEAD.size:], now)
                elif ptype == mr.SNAP_INFO:
                    receiver.on_info(mr.PhotoInfo.unpack(body), now)
                else:
                    _, index = mr.SNAP_DATA_HEAD.unpack_from(body)
                    receiver.on_data(index, body[mr.SNAP_DATA_HEAD.size:], now)
            if receiver.news and (receiver.complete or now - receiver.acked_at >= 0.5):
                receiver.news, receiver.acked_at = False, now
                if rng.random() >= loss:
                    wire.append((now + 0.05 + rng.random() * 0.1, "sender", mr.SNAP_ACK, receiver.ack()))
            sender.pump(now, control, mr.rto_for(200))
            clock[0] = round(now + 0.01, 2)
        return sender, receiver, clock[0]

    def test_whole_photo_despite_loss(self):
        data = photo_bytes(50_000)
        sender, receiver, took = self.transfer(data, loss=0.3)
        self.assertTrue(sender.done)
        self.assertEqual(receiver.data(), data)
        self.assertLess(self.sent_bytes, 2.5 * len(data))  # what got lost, and not much more, went again

    def test_each_chunk_once_without_loss(self):
        data = photo_bytes(20_000)
        sender, receiver, took = self.transfer(data, loss=0.0)
        self.assertEqual(receiver.data(), data)
        self.assertEqual(self.sent_bytes, len(data) + 20 * mr.SNAP_DATA_HEAD.size + mr.SNAP_INFO_BODY.size)

    def test_keeps_to_the_rate(self):
        sender, receiver, took = self.transfer(photo_bytes(40_000), loss=0.0, rate=4096)
        self.assertTrue(sender.done)
        self.assertGreater(took, 40_000 / 4096 - 1)

    def test_restart_sends_everything_again(self):
        data = photo_bytes(3000)
        sent = []
        sender = mr.PhotoSender(mr.PhotoInfo(1, len(data)), mr.BytesSource(data), lambda t, b: sent.append(t), 0.0)
        control = mr.RateControl(1e6, start=1e6)
        control.budget(0.0)
        sender.pump(1.0, control, 1.0)
        sender.on_ack(mr.ACK_HAVE_INFO, b"\x07", 1.1)
        sender.pump(1.2, control, 1.0)
        self.assertEqual(sent, [mr.SNAP_INFO] + [mr.SNAP_DATA] * 3)  # nothing more: all acknowledged
        sender.restart(2.0)
        sender.pump(2.0, control, 1.0)
        self.assertEqual(sent[4:], [mr.SNAP_INFO] + [mr.SNAP_DATA] * 3)

    def test_gives_up_without_acks(self):
        sender = mr.PhotoSender(mr.PhotoInfo(1, 100), mr.BytesSource(bytes(100)), lambda t, b: None, 0.0)
        sender.pump(59.0, mr.RateControl(1000), 2.0)
        self.assertFalse(sender.failed)
        sender.pump(61.0, mr.RateControl(1000), 2.0)
        self.assertTrue(sender.failed)


class RelayPhotoTest(unittest.TestCase):
    """The relay's PhotoStore, driven packet by packet (no sockets, no waiting)."""

    PLANE = ("203.0.113.5", 5000)
    LAPTOP = ("198.51.100.7", 4000)
    TABLET = ("198.51.100.8", 4001)

    def setUp(self):
        self.folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.folder, True)
        self.sent = []
        self.relay = self.new_relay()
        self.seq = {}

    def new_relay(self, **kw):
        relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G}, photo_folder=self.folder, **kw)
        sent = self.sent

        class Transport:
            def sendto(self, data, addr):
                sent.append((mr.decode(data), addr))

        relay.connection_made(Transport())
        return relay

    def connect(self, role, addr):
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        nonce = os.urandom(mr.NONCE_LEN)
        self.relay.datagram_received(mr.encode(key, mr.HELLO, role, 0, 0, nonce + b"test"), addr)
        sid = [p.session for p, a in self.sent if a == addr and p.type == mr.WELCOME][-1]
        self.seq[sid] = 0
        body = mr.PING_BODY.pack(0, 50, mr.U16_UNKNOWN, mr.RSSI_UNKNOWN, mr.RAT_UNKNOWN, 0)
        self.packet(role, sid, addr, mr.PING, body)
        return sid

    def packet(self, role, sid, addr, ptype, body):
        self.seq[sid] += 1
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, ptype, role, sid, self.seq[sid], body), addr)

    def to(self, addr, ptype):
        return [p.body for p, a in self.sent if a == addr and p.type == ptype]

    def infos_to(self, addr):
        return [mr.PhotoInfo.unpack(b) for b in self.to(addr, mr.SNAP_INFO)]

    def aircraft_sends(self, sid, photo_id, data, where=WHERE):
        info = mr.PhotoInfo(photo_id, len(data), 640, 480, *where)
        self.packet(mr.ROLE_VEHICLE, sid, self.PLANE, mr.SNAP_INFO, info.pack())
        for i in range(info.chunks):
            chunk = data[i * mr.SNAP_CHUNK:(i + 1) * mr.SNAP_CHUNK]
            self.packet(mr.ROLE_VEHICLE, sid, self.PLANE, mr.SNAP_DATA, mr.SNAP_DATA_HEAD.pack(photo_id, i) + chunk)

    def take_photo(self, data):
        """A GCS agent asks, the aircraft answers; returns the ids of the photo and the sessions."""
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.SNAP_REQ, mr.SNAP_REQ_BODY.pack(0, 1))
        [req] = self.to(self.PLANE, mr.SNAP_REQ)
        photo_id, size = mr.SNAP_REQ_BODY.unpack(req)
        self.assertEqual(size, 1)
        self.aircraft_sends(plane, photo_id, data)
        self.pump()
        return photo_id, plane, laptop

    def pump(self, seconds=2.0):
        start = time.monotonic()
        for step in range(int(seconds / 0.05) + 1):
            self.relay.photos.pump(start + step * 0.05)

    def delivered(self, addr):
        """What reached `addr` of photo data, as {photo id: {chunk: bytes}}."""
        photos = {}
        for body in self.to(addr, mr.SNAP_DATA):
            photo_id, index = mr.SNAP_DATA_HEAD.unpack_from(body)
            photos.setdefault(photo_id, {})[index] = body[mr.SNAP_DATA_HEAD.size:]
        return photos

    def test_no_aircraft(self):
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.SNAP_REQ, mr.SNAP_REQ_BODY.pack(0, 1))
        [info] = self.infos_to(self.LAPTOP)
        self.assertEqual(info.status, mr.SNAP_NO_AIRCRAFT)

    def test_asks_again_then_gives_up(self):
        self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.SNAP_REQ, mr.SNAP_REQ_BODY.pack(0, 9))
        now = time.monotonic()
        self.relay.photos.pump(now + 1.0)
        self.relay.photos.pump(now + 2.5)
        reqs = [mr.SNAP_REQ_BODY.unpack(b) for b in self.to(self.PLANE, mr.SNAP_REQ)]
        self.assertEqual(len(reqs), 2)
        self.assertEqual(reqs[0], reqs[1])
        self.assertEqual(reqs[0][1], 2)  # an unknown size: the largest
        self.relay.photos.pump(now + 21.0)
        [info] = self.infos_to(self.LAPTOP)
        self.assertEqual((info.photo_id, info.status), (reqs[0][0], mr.SNAP_NO_ANSWER))

    def test_photo_is_kept_and_passed_on(self):
        data = photo_bytes(5000)
        photo_id, plane, laptop = self.take_photo(data)
        acks = self.to(self.PLANE, mr.SNAP_ACK)
        self.assertEqual(mr.SNAP_ACK_HEAD.unpack_from(acks[-1]), (photo_id, mr.ACK_DONE | mr.ACK_HAVE_INFO))
        with open(os.path.join(self.folder, f"{photo_id}.jpg"), "rb") as f:
            self.assertEqual(f.read(), data)
        with open(os.path.join(self.folder, f"{photo_id}.json")) as f:
            meta = json.load(f)
        self.assertEqual((meta["lat"], meta["vehicle"]), (WHERE[0], "test"))
        self.assertAlmostEqual(meta["time"], time.time(), delta=5)
        # the agent that asked got it while it was arriving, with the relay's time on it (its SNAP_INFO
        # goes again every second until the agent ACKs)
        info = self.infos_to(self.LAPTOP)[0]
        self.assertEqual(set(self.infos_to(self.LAPTOP)), {info})
        self.assertEqual((info.photo_id, info.size, info.lat, info.time), (photo_id, 5000, WHERE[0], meta["time"]))
        chunks = self.delivered(self.LAPTOP)[photo_id]
        self.assertEqual(b"".join(chunks[i] for i in range(5)), data)
        # the aircraft sends a chunk again (it missed the relay's last ACK): it is told it is all there
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.SNAP_DATA, mr.SNAP_DATA_HEAD.pack(photo_id, 2) + data[2048:3072])
        self.assertEqual(mr.SNAP_ACK_HEAD.unpack_from(self.to(self.PLANE, mr.SNAP_ACK)[-1])[1], mr.ACK_DONE | mr.ACK_HAVE_INFO)
        # a relay started later finds it in its folder
        relay = self.new_relay()
        self.assertEqual(relay.photos.stored[photo_id].size, 5000)
        self.assertEqual(relay.photos.source(photo_id).chunk(4), data[4096:])
        self.assertGreater(relay.photos.new_id(), photo_id)

    def test_sync_sends_what_the_agent_misses(self):
        first, plane, laptop = self.take_photo(photo_bytes(3000, seed=1))
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.SNAP_REQ, mr.SNAP_REQ_BODY.pack(0, 0))
        second = mr.SNAP_REQ_BODY.unpack(self.to(self.PLANE, mr.SNAP_REQ)[-1])[0]
        self.aircraft_sends(plane, second, photo_bytes(2000, seed=2))
        self.pump()
        self.assertGreater(second, first)
        tablet = self.connect(mr.ROLE_GCS, self.TABLET)
        self.packet(mr.ROLE_GCS, tablet, self.TABLET, mr.SNAP_SYNC, mr.SNAP_SYNC_BODY.pack(first))
        self.pump()
        self.assertEqual(list(self.delivered(self.TABLET)), [second])
        self.assertEqual({i.photo_id for i in self.infos_to(self.TABLET)}, {second})

    def test_problems_go_to_the_asker_once(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        laptop = self.connect(mr.ROLE_GCS, self.LAPTOP)
        tablet = self.connect(mr.ROLE_GCS, self.TABLET)
        self.packet(mr.ROLE_GCS, tablet, self.TABLET, mr.SNAP_SYNC, mr.SNAP_SYNC_BODY.pack(0))
        self.packet(mr.ROLE_GCS, laptop, self.LAPTOP, mr.SNAP_REQ, mr.SNAP_REQ_BODY.pack(0, 1))
        photo_id = mr.SNAP_REQ_BODY.unpack(self.to(self.PLANE, mr.SNAP_REQ)[-1])[0]
        answer = mr.PhotoInfo(photo_id, status=mr.SNAP_NO_CAMERA).pack()
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.SNAP_INFO, answer)
        self.packet(mr.ROLE_VEHICLE, plane, self.PLANE, mr.SNAP_INFO, answer)  # its answer to a repeated request
        self.assertEqual([i.status for i in self.infos_to(self.LAPTOP)], [mr.SNAP_NO_CAMERA])
        self.assertEqual(self.infos_to(self.TABLET), [])

    def test_unasked_photos(self):
        plane = self.connect(mr.ROLE_VEHICLE, self.PLANE)
        self.aircraft_sends(plane, int(time.time()) - 3600, photo_bytes(1000))  # old and never asked for
        # a recent one: asked for by this relay before it restarted
        recent = int(time.time()) - 5
        self.aircraft_sends(plane, recent, photo_bytes(1000))
        self.relay.photos.pump(time.monotonic())
        self.assertEqual(list(self.relay.photos.stored), [recent])

    def test_old_photos_are_deleted(self):
        store = self.relay.photos
        old, new = int(time.time()) - 8 * 86400, int(time.time()) - 86400
        store._keep(mr.PhotoInfo(old, 3, time=old), b"old")
        store._keep(mr.PhotoInfo(new, 3, time=new), b"new")
        self.assertEqual(sorted(os.listdir(self.folder)), [f"{new}.jpg", f"{new}.json"])
        self.assertEqual(list(store.stored), [new])
        store.MAX_PHOTOS = 2
        for i in range(1, 4):
            store._keep(mr.PhotoInfo(new + i, 3, time=new + i), b"abc")
        self.assertEqual(sorted(store.stored), [new + 2, new + 3])
        self.assertEqual(len(os.listdir(self.folder)), 4)

    def test_agent_back_after_many_photos(self):
        store = self.relay.photos
        first = int(time.time()) - 3600
        for i in range(store.SYNC_MOST + 10):
            store._keep(mr.PhotoInfo(first + i, 1500, time=first + i), bytes([i]) * 1500)
        store.memory.clear()  # as after a relay restart: they come from the folder, a chunk at a time
        tablet = self.connect(mr.ROLE_GCS, self.TABLET)
        self.packet(mr.ROLE_GCS, tablet, self.TABLET, mr.SNAP_SYNC, mr.SNAP_SYNC_BODY.pack(0))
        deliveries = self.relay.sessions[tablet].deliveries
        self.assertEqual([s.info.photo_id for s in deliveries], list(range(first + 10, first + store.SYNC_MOST + 10)))
        self.assertEqual(store.memory, {})
        self.pump(0.5)
        chunks = self.delivered(self.TABLET)[first + 10]
        self.assertEqual(chunks, {0: bytes([10]) * 1024, 1: bytes([10]) * 476})

    def test_without_a_folder(self):
        relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G}, photo_folder=None)
        now = int(time.time())
        for i in range(mr.PhotoStore.IN_MEMORY + 5):
            relay.photos._keep(mr.PhotoInfo(now + i, 3, time=now), b"abc")
        self.assertEqual(len(relay.photos.stored), mr.PhotoStore.IN_MEMORY)
        self.assertEqual(relay.photos.source(now + 24).chunk(0), b"abc")


class LiveSnapshotTest(unittest.IsolatedAsyncioTestCase):
    """Relay, aircraft and GCS agent on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop  # same event loop as mavrelay uses on Windows

    async def asyncSetUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.tasks = []
        self.port = await self.start_server()

    async def asyncTearDown(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.server_transport.close()

    def spawn(self, coro):
        task = asyncio.ensure_future(coro)
        self.tasks.append(task)
        return task

    async def start_server(self, port=0):
        loop = asyncio.get_running_loop()
        self.relay = relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G},
                                            photo_folder=os.path.join(self.tmp, "relay"))
        self.server_transport, _ = await loop.create_datagram_endpoint(lambda: relay, local_addr=("127.0.0.1", port))

        async def ticker():
            last = 0.0
            while True:
                await asyncio.sleep(0.05)
                now = time.monotonic()
                relay.photos.pump(now)
                if now - last >= 0.2:
                    relay.tick(now)
                    last = now

        self.ticker = self.spawn(ticker())
        return self.server_transport.get_extra_info("sockname")[1]

    async def restart_server(self):
        self.ticker.cancel()
        self.server_transport.close()
        await asyncio.sleep(0.1)
        await self.start_server(self.port)

    def aircraft(self, capture, cap=64 * 1024):
        client = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", self.port, on_data=lambda data: None)
        outbox = mr.PhotoOutbox(client, capture=capture, where=lambda: WHERE, cap=lambda: cap)
        client.on_packet, client.on_session = outbox.on_packet, outbox.on_session

        async def pump():
            while True:
                await asyncio.sleep(0.01)
                outbox.pump(time.monotonic())

        self.spawn(client.run())
        self.spawn(pump())
        return client, outbox

    def agent(self, name):
        photos = []
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G, photo_dir=os.path.join(self.tmp, name),
                            on_photo=lambda path, info: photos.append((path, info)))
        agent.photos_saved = photos
        self.spawn(agent.run())
        return agent

    async def until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail("condition not met in time")
            await asyncio.sleep(0.02)

    async def test_photo_on_request(self):
        data = photo_bytes(30_000)
        sizes = []
        plane, _ = self.aircraft(lambda w, h: sizes.append((w, h)) or data)
        agent = self.agent("laptop")
        await self.until(lambda: plane.is_connected and agent.client.is_connected)
        await self.until(lambda: self.relay.vehicle is not None and self.relay.vehicle.last_rx > 0)
        self.assertTrue(agent.photos.request(2))
        await self.until(lambda: agent.photos_saved)
        [(path, info)] = agent.photos_saved
        self.assertEqual(sizes, [(1024, 768)])
        with open(path, "rb") as f:
            self.assertEqual(f.read(), data)
        with open(path[:-4] + ".json") as f:
            meta = json.load(f)
        self.assertEqual((meta["latitude"], meta["longitude"], meta["altitude_m"]), (41.1234567, 28.9876543, 120.0))
        self.assertEqual(os.path.basename(path), time.strftime("MavLTE_%Y-%m-%d_%H-%M-%S_", time.localtime(info.time))
                         + f"{info.photo_id}.jpg")
        self.assertEqual(agent.photos.newest(), info.photo_id)
        self.assertIn(f"{info.photo_id}.jpg", os.listdir(os.path.join(self.tmp, "relay")))
        self.assertEqual(agent.photos.problem, "")

        # another agent, started later, gets it without asking
        tablet = self.agent("tablet")
        await self.until(lambda: tablet.photos_saved)
        self.assertEqual(tablet.photos_saved[0][1].photo_id, info.photo_id)

    async def test_no_camera(self):
        plane, _ = self.aircraft(None)
        agent = self.agent("laptop")
        await self.until(lambda: plane.is_connected and agent.client.is_connected)
        await self.until(lambda: self.relay.vehicle is not None)
        agent.photos.request(1)
        await self.until(lambda: agent.photos.problem)
        self.assertEqual(agent.photos.problem, mr.SNAP_PROBLEMS[mr.SNAP_NO_CAMERA])
        self.assertIsNone(agent.photos.asked_at)

    async def test_relay_restart_in_the_middle_of_a_photo(self):
        data = photo_bytes(40_000)
        plane, outbox = self.aircraft(lambda w, h: data, cap=8192)  # 5 s or more on the way
        agent = self.agent("laptop")
        await self.until(lambda: plane.is_connected and agent.client.is_connected)
        await self.until(lambda: self.relay.vehicle is not None)
        agent.photos.request(1)
        await self.until(lambda: agent.photos.arriving and agent.photos.arriving[1] > 4000)
        await self.restart_server()  # a new relay, same folder: it knows nothing of this photo
        await self.until(lambda: agent.photos_saved, timeout=20)
        with open(agent.photos_saved[0][0], "rb") as f:
            self.assertEqual(f.read(), data)


if __name__ == "__main__":
    unittest.main()

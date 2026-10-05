"""Tests for logs: the aircraft's log files, listed and downloaded through the relay (docs/PROTOCOL.md, "Logs").
Run from the relay directory:  python -m unittest -v"""

import asyncio
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
HEADER = b"time_utc,uptime_s,gnss_fix\n"


def log_bytes(lines, first_time=1790000000, untimed=0, seed=1):
    """A log file: its header, `untimed` lines without a time, then `lines` lines a second apart."""
    rng = random.Random(seed)
    rows = [b",%d,0" % i for i in range(untimed)]
    for i in range(lines):
        stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(first_time + i)).encode()
        rows.append(stamp + b",%d,3,%s" % (untimed + i, bytes(rng.choice(b"abcdef0123456789") for _ in range(40))))
    return HEADER + b"\n".join(rows) + b"\n"


class LogFilesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def write(self, name, data):
        with open(os.path.join(self.tmp, name), "wb") as f:
            f.write(data)

    def test_times(self):
        self.assertEqual(mr.log_time(b"2026-10-02T09:35:12Z"), 1790933712)
        for bad in (b"", b"time_utc", b"2026-10-02T09:35", b"\xff\xfe"):
            self.assertEqual(mr.log_time(bad), 0)
        # the first 20 lines before the board knew the time: the start reckoned back to its first line
        self.write("LOG00001.CSV", log_bytes(500, untimed=20))  # longer than what is read of each end
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "LOG00001.CSV")), (1789999980, 1790000499))
        self.write("LOG00003.CSV", log_bytes(50, untimed=200))  # no time at all in what is read of its start
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "LOG00003.CSV")), (1789999800, 1790000049))
        self.write("LOG00004.CSV", log_bytes(3))  # shorter than what is read
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "LOG00004.CSV")), (1790000000, 1790000002))
        self.write("LOG00002.CSV", HEADER + b",1,0\n,2,0\n")  # no GNSS and no relay yet: no time at all
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "LOG00002.CSV")), (0, 0))
        self.write("LOG00005.CSV", HEADER)  # only the header: the power went within a second
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "LOG00005.CSV")), (0, 0))
        self.assertEqual(mr.log_times(os.path.join(self.tmp, "none.csv")), (0, 0))

    def test_entries_newest_first(self):
        self.write("LOG00002.CSV", log_bytes(3, 1790000100))
        self.write("log00010.csv", log_bytes(2, 1790000200))  # any case on disk
        self.write("LOG00001.CSV", b"")
        self.write("notes.txt", b"not a log")
        self.write("LOG1.CSV", b"not one of ours")
        entries = mr.log_entries(self.tmp)
        self.assertEqual([e[0] for e in entries], ["LOG00010.CSV", "LOG00002.CSV", "LOG00001.CSV"])
        self.assertEqual(entries[0][2:], (1790000200, 1790000201))
        self.assertEqual(entries[2], ("LOG00001.CSV", 0, 0, 0))
        self.assertEqual(mr.log_entries(os.path.join(self.tmp, "none")), [])

    def test_largest_packets_fit(self):
        self.assertLessEqual(mr.FILE_DATA_HEAD.size + mr.FILE_CHUNK, mr.MAX_PAYLOAD)
        self.assertLessEqual(mr.FILE_LIST_HEAD.size + mr.FILE_LIST_MOST * mr.FILE_ENTRY.size, mr.MAX_PAYLOAD)


class RelayRoutesTest(unittest.TestCase):
    """The relay's part, packet by packet: it passes requests on under ids of its own, and answers back."""

    def setUp(self):
        self.sent = []
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        sent = self.sent

        class Transport:
            def sendto(self, data, addr):
                sent.append((mr.decode(data), addr))

        self.relay.connection_made(Transport())
        self.seq = {}

    def connect(self, role, addr):
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.relay.datagram_received(mr.encode(key, mr.HELLO, role, 0, 0, os.urandom(8) + b"test"), addr)
        sid = [p.session for p, a in self.sent if a == addr and p.type == mr.WELCOME][-1]
        self.seq[sid] = 0
        self.send(role, sid, addr, mr.PING, mr.PING_BODY.pack(0, mr.U16_UNKNOWN, mr.U16_UNKNOWN, mr.RSSI_UNKNOWN,
                                                             mr.RAT_UNKNOWN, 0))
        return sid

    def send(self, role, sid, addr, ptype, body):
        key = KEY_V if role == mr.ROLE_VEHICLE else KEY_G
        self.seq[sid] += 1
        self.relay.datagram_received(mr.encode(key, ptype, role, sid, self.seq[sid], body), addr)

    def got(self, addr, ptype):
        return [p.body for p, a in self.sent if a == addr and p.type == ptype]

    def test_welcome_carries_the_relays_clock(self):
        before = int(time.time() * 1000)
        self.connect(mr.ROLE_VEHICLE, ("203.0.113.5", 5000))
        [welcome] = [p.body for p, a in self.sent if p.type == mr.WELCOME]
        clock = mr.WELCOME_TIME.unpack_from(welcome, mr.NONCE_LEN)[0]
        self.assertTrue(before <= clock <= int(time.time() * 1000))

    def test_no_aircraft(self):
        laptop = ("198.51.100.7", 4000)
        sid = self.connect(mr.ROLE_GCS, laptop)
        self.send(mr.ROLE_GCS, sid, laptop, mr.FILE_REQ, mr.FILE_REQ_BODY.pack(7, mr.FILE_OP_LIST, 0, b""))
        self.send(mr.ROLE_GCS, sid, laptop, mr.FILE_REQ, mr.FILE_REQ_BODY.pack(8, mr.FILE_OP_GET, 100, b"LOG00001.CSV"))
        self.assertEqual(self.got(laptop, mr.FILE_LIST), [mr.FILE_LIST_HEAD.pack(7, mr.FILE_NO_AIRCRAFT, 0, 0, 0)])
        self.assertEqual(self.got(laptop, mr.FILE_DATA), [mr.FILE_DATA_HEAD.pack(8, mr.FILE_NO_AIRCRAFT, 100, 0)])

    def test_requests_and_answers_pass_through(self):
        plane, laptop, tablet = ("203.0.113.5", 5000), ("198.51.100.7", 4000), ("198.51.100.8", 4001)
        vid = self.connect(mr.ROLE_VEHICLE, plane)
        lid = self.connect(mr.ROLE_GCS, laptop)
        tid = self.connect(mr.ROLE_GCS, tablet)
        for sid, addr in ((lid, laptop), (tid, tablet)):  # both agents use request id 5
            self.send(mr.ROLE_GCS, sid, addr, mr.FILE_REQ, mr.FILE_REQ_BODY.pack(5, mr.FILE_OP_GET, 0, b"LOG00001.CSV"))
        asked = [mr.FILE_REQ_BODY.unpack(b) for b in self.got(plane, mr.FILE_REQ)]
        self.assertEqual(len({a[0] for a in asked}), 2)  # under two ids of the relay's own
        self.assertEqual({a[1:] for a in asked}, {(mr.FILE_OP_GET, 0, b"LOG00001.CSV")})
        laptop_id, tablet_id = asked[0][0], asked[1][0]
        self.send(mr.ROLE_VEHICLE, vid, plane, mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(tablet_id, 0, 0, 3) + b"abc")
        self.assertEqual(self.got(tablet, mr.FILE_DATA), [mr.FILE_DATA_HEAD.pack(5, 0, 0, 3) + b"abc"])
        self.assertEqual(self.got(laptop, mr.FILE_DATA), [])
        self.send(mr.ROLE_GCS, lid, laptop, mr.FILE_ACK, mr.FILE_ACK_BODY.pack(5, 1024))
        self.assertEqual(self.got(plane, mr.FILE_ACK), [mr.FILE_ACK_BODY.pack(laptop_id, 1024)])
        self.send(mr.ROLE_GCS, lid, laptop, mr.FILE_ACK, mr.FILE_ACK_BODY.pack(99, 1024))  # no such request
        self.send(mr.ROLE_VEHICLE, vid, plane, mr.FILE_LIST, mr.FILE_LIST_HEAD.pack(4321, 0, 0, 0, 0))
        self.assertEqual(len(self.got(plane, mr.FILE_ACK)), 1)
        self.assertEqual(self.got(laptop, mr.FILE_LIST) + self.got(tablet, mr.FILE_LIST), [])
        self.send(mr.ROLE_VEHICLE, vid, plane, mr.FILE_REQ, mr.FILE_REQ_BODY.pack(1, 1, 0, b""))  # not the vehicle's
        self.relay.files.forget(time.monotonic() + mr.FileRoutes.FORGET + 1)  # two minutes later
        self.assertEqual(self.relay.files.routes, {})
        self.send(mr.ROLE_VEHICLE, vid, plane, mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(tablet_id, 0, 3, 3))
        self.assertEqual(len(self.got(tablet, mr.FILE_DATA)), 1)


class LiveLogsTest(unittest.IsolatedAsyncioTestCase):
    """Relay, aircraft (the Python vehicle's FileOutbox) and GCS agents on real UDP sockets on localhost."""

    if sys.platform == "win32" and sys.version_info >= (3, 13):
        loop_factory = asyncio.SelectorEventLoop  # same event loop as mavrelay uses on Windows

    async def asyncSetUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.card = os.path.join(self.tmp, "card")
        os.makedirs(self.card)
        self.tasks = []
        loop = asyncio.get_running_loop()
        self.relay = relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.transport, _ = await loop.create_datagram_endpoint(lambda: relay, local_addr=("127.0.0.1", 0))
        self.port = self.transport.get_extra_info("sockname")[1]

        async def ticker():
            while True:
                await asyncio.sleep(0.2)
                relay.tick(time.monotonic())

        self.spawn(ticker())

    async def asyncTearDown(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.transport.close()

    def spawn(self, coro):
        task = asyncio.ensure_future(coro)
        self.tasks.append(task)
        return task

    def write(self, name, data):
        with open(os.path.join(self.card, name), "wb") as f:
            f.write(data)

    def aircraft(self, folder="card", cap=64 * 1024):
        client = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", self.port, on_data=lambda data: None)
        outbox = mr.FileOutbox(client, os.path.join(self.tmp, folder) if folder else None, cap=lambda: cap)
        client.on_packet = outbox.on_packet

        async def pump():
            while True:
                await asyncio.sleep(0.01)
                outbox.pump(time.monotonic())

        self.spawn(client.run())
        self.spawn(pump())
        return client, outbox

    def agent(self):
        agent = mr.GcsAgent(("127.0.0.1", self.port), KEY_G)
        self.spawn(agent.run())
        return agent

    async def until(self, cond, timeout=8.0):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail("condition not met in time")
            await asyncio.sleep(0.02)

    async def connected(self, *clients):
        await self.until(lambda: all(c.is_connected for c in clients))
        await self.until(lambda: self.relay.vehicle is not None and self.relay.vehicle.last_rx > 0)

    async def listing(self, agent, first=0):
        result = []
        self.assertTrue(agent.files.list(first, lambda *answer: result.append(answer)))
        await self.until(lambda: result)
        return result[0]

    async def download(self, agent, name, offset=0, timeout=15.0):
        """The file as downloaded from `offset`: (ok, problem, bytes, progress calls)."""
        got, result, progress = bytearray(), [], []

        def write(at, data):
            self.assertEqual(at, offset + len(got))  # in order, nothing twice
            got.extend(data)

        self.assertTrue(agent.files.get(name, offset, write, lambda have, size: progress.append((have, size)),
                                        lambda ok, problem: result.append((ok, problem))))
        await self.until(lambda: result, timeout)
        return result[0][0], result[0][1], bytes(got), progress

    async def test_list_and_download(self):
        old, new = log_bytes(900, 1790000000), log_bytes(30, 1790100000, seed=2)
        self.write("LOG00001.CSV", old)
        self.write("LOG00002.CSV", new)
        plane, outbox = self.aircraft()
        agent = self.agent()
        await self.connected(plane, agent.client)
        entries, files, problem = await self.listing(agent)
        self.assertEqual((files, problem), (2, ""))
        self.assertEqual(entries, [("LOG00002.CSV", len(new), 1790100000, 1790100029),
                                   ("LOG00001.CSV", len(old), 1790000000, 1790000899)])
        self.assertEqual(await self.listing(agent, first=1), ([entries[1]], 2, ""))
        ok, problem, data, progress = await self.download(agent, "LOG00001.CSV")
        self.assertEqual((ok, problem), (True, ""))
        self.assertEqual(data, old)
        self.assertEqual(progress[-1], (len(old), len(old)))
        await self.until(lambda: outbox.get is None)  # the aircraft heard the last ACK

        # a download that stopped goes on where it did
        ok, _, rest, _ = await self.download(agent, "LOG00001.CSV", offset=10_000)
        self.assertTrue(ok)
        self.assertEqual(rest, old[10_000:])
        ok, _, nothing, _ = await self.download(agent, "LOG00001.CSV", offset=len(old))  # all there already
        self.assertEqual((ok, nothing), (True, b""))

    async def test_problems(self):
        plane, _ = self.aircraft()
        agent = self.agent()
        await self.connected(plane, agent.client)
        ok, problem, _, _ = await self.download(agent, "LOG00099.CSV")
        self.assertEqual((ok, problem), (False, mr.FILE_PROBLEMS[mr.FILE_NOT_FOUND]))
        ok, problem, _, _ = await self.download(agent, "../secret.txt")  # only the card's logs, not even asked for
        self.assertEqual((ok, problem), (False, "not a log file's name: '../secret.txt'"))
        ok, problem, _, _ = await self.download(agent, "log00099.csv")  # (the card's names, in any case)
        self.assertEqual((ok, problem), (False, mr.FILE_PROBLEMS[mr.FILE_NOT_FOUND]))
        self.assertEqual(await self.listing(agent), ([], 0, ""))

    async def test_no_card(self):
        plane, _ = self.aircraft(folder=None)
        agent = self.agent()
        await self.connected(plane, agent.client)
        self.assertEqual(await self.listing(agent), (None, 0, mr.FILE_PROBLEMS[mr.FILE_NO_CARD]))
        ok, problem, _, _ = await self.download(agent, "LOG00001.CSV")
        self.assertEqual((ok, problem), (False, mr.FILE_PROBLEMS[mr.FILE_NO_CARD]))

    async def test_without_an_aircraft(self):
        agent = self.agent()
        await self.until(lambda: agent.client.is_connected)
        self.assertEqual(await self.listing(agent), (None, 0, mr.FILE_PROBLEMS[mr.FILE_NO_AIRCRAFT]))

    async def test_lost_packets(self):
        data = log_bytes(1500, seed=5)  # about 75 KB
        self.write("LOG00003.CSV", data)
        plane, _ = self.aircraft()
        agent = self.agent()
        await self.connected(plane, agent.client)
        rng = random.Random(7)
        send = self.relay._send

        def lossy(sess, ptype, body):  # the relay loses a tenth of the data and of the ACKs
            if ptype in (mr.FILE_DATA, mr.FILE_ACK) and rng.random() < 0.1:
                return
            send(sess, ptype, body)

        self.relay._send = lossy
        ok, problem, got, _ = await self.download(agent, "LOG00003.CSV", timeout=40)
        self.assertEqual((ok, problem), (True, ""))
        self.assertEqual(got, data)

    async def test_a_new_download_takes_over(self):
        self.write("LOG00004.CSV", log_bytes(3000, seed=4))
        self.write("LOG00005.CSV", log_bytes(10, seed=6))
        plane, outbox = self.aircraft(cap=4096)  # slow: the first is still going when the second comes
        laptop, tablet = self.agent(), self.agent()
        await self.connected(plane, laptop.client, tablet.client)
        first = []
        laptop.files.get("LOG00004.CSV", 0, lambda at, data: None, lambda have, size: None,
                         lambda ok, problem: first.append((ok, problem)))
        await self.until(lambda: outbox.get is not None)
        ok, _, data, _ = await self.download(tablet, "LOG00005.CSV")
        self.assertTrue(ok)
        self.assertEqual(data, log_bytes(10, seed=6))
        await self.until(lambda: first)
        self.assertEqual(first, [(False, mr.FILE_PROBLEMS[mr.FILE_STOPPED])])

    async def test_a_growing_file(self):
        self.write("LOG00006.CSV", log_bytes(100))
        plane, _ = self.aircraft()
        agent = self.agent()
        await self.connected(plane, agent.client)
        ok, _, start, _ = await self.download(agent, "LOG00006.CSV")
        with open(os.path.join(self.card, "LOG00006.CSV"), "ab") as f:  # the aircraft logs on
            f.write(b"2026-10-02T09:35:12Z,100,3,more\n")
        ok, _, rest, _ = await self.download(agent, "LOG00006.CSV", offset=len(start))
        self.assertTrue(ok)
        self.assertEqual(rest, b"2026-10-02T09:35:12Z,100,3,more\n")


class FetcherTest(unittest.TestCase):
    """The agent's side alone, packet by packet."""

    class Client:
        is_connected = True

        def __init__(self):
            self.sent = []

        def send_packet(self, ptype, body):
            self.sent.append((ptype, body))
            return True

    def test_asks_again_then_gives_up(self):
        client = self.Client()
        fetcher = mr.FileFetcher(client)
        result = []
        fetcher.get("LOG00001.CSV", 0, lambda at, data: None, lambda have, size: None,
                    lambda ok, problem: result.append((ok, problem)))
        start = time.monotonic()
        for t in range(1, 40):
            fetcher.pump(start + t)
        gets = [b for p, b in client.sent if p == mr.FILE_REQ]
        self.assertGreaterEqual(len(gets), 5)  # its GET again, every few seconds
        self.assertEqual(result, [(False, "no answer from the aircraft (firmware before 1.8.0?)")])

    def test_drops_what_comes_out_of_order(self):
        client = self.Client()
        fetcher = mr.FileFetcher(client)
        got = []
        fetcher.get("LOG00001.CSV", 0, lambda at, data: got.append((at, data)), lambda have, size: None,
                    lambda ok, problem: None)
        req = mr.FILE_REQ_BODY.unpack(client.sent[0][1])[0]
        fetcher.on_packet(mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(req, 0, 1024, 3000) + bytes(1024))  # after a gap
        fetcher.on_packet(mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(req, 0, 0, 3000) + bytes(1024))
        fetcher.on_packet(mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(req, 0, 0, 3000) + bytes(1024))  # a copy
        fetcher.on_packet(mr.FILE_DATA, mr.FILE_DATA_HEAD.pack(req + 1, 0, 1024, 3000) + bytes(1024))  # not ours
        self.assertEqual([at for at, _ in got], [0])
        self.assertEqual(fetcher.download["next"], 1024)
        fetcher.pump(time.monotonic() + 1)
        self.assertEqual(client.sent[-1], (mr.FILE_ACK, mr.FILE_ACK_BODY.pack(req, 1024)))


if __name__ == "__main__":
    unittest.main()

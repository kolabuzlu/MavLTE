"""Runs the firmware's C tunnel code (firmware/test/host/tunnel_harness) against this relay.

Needs make and a C compiler (Linux, macOS or WSL); skipped otherwise.
"""

import asyncio
import os
import shutil
import subprocess
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import mavrelay as mr  # noqa: E402
from test_mavrelay import KEY_G, KEY_V, v2_frame  # noqa: E402

HOST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "firmware", "test", "host")


def build_harness():
    if sys.platform == "win32" or not shutil.which("make") or not (shutil.which("cc") or shutil.which("gcc")):
        return None
    result = subprocess.run(["make", "-s", "harness"], cwd=HOST_DIR, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError("building tunnel_harness failed:\n" + result.stdout + result.stderr)
    return os.path.join(HOST_DIR, "tunnel_harness")


def split_frames(data):
    frames, start = [], 0
    framer = mr.MavFramer()
    for i, b in enumerate(data):
        if framer.push(b):
            frames.append(data[start:i + 1])
            start = i + 1
    return frames


class CVehicleTest(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        cls.harness = build_harness()
        if not cls.harness:
            raise unittest.SkipTest("needs make and a C compiler")

    async def start_server(self, port=0):
        loop = asyncio.get_running_loop()
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G})
        self.transport, _ = await loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", port))
        return self.transport.get_extra_info("sockname")[1]

    async def until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail("condition not met in time")
            await asyncio.sleep(0.02)

    async def test_c_vehicle_through_relay(self):
        port = await self.start_server()
        received = bytearray()
        gcs = mr.TunnelClient(mr.ROLE_GCS, KEY_G, "127.0.0.1", port, on_data=received.extend)
        gcs_task = asyncio.ensure_future(gcs.run())
        self.addCleanup(gcs_task.cancel)
        await self.until(lambda: gcs.is_connected)

        proc = await asyncio.create_subprocess_exec(self.harness, "127.0.0.1", str(port), KEY_V.hex(), "6",
                                                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        await self.until(lambda: self.relay.vehicle is not None)
        self.assertEqual(self.relay.vehicle.info, "tunnel_harness")
        await asyncio.sleep(1.0)

        # a command from the GCS reaches the C vehicle, which echoes it back
        command = v2_frame(76, bytes(range(33)), 9)
        gcs.send_data(command)
        await self.until(lambda: command in received)
        restart_at = len(received)

        # the relay restarts; the C vehicle must find its way back on its own
        self.transport.close()
        await asyncio.sleep(0.2)
        await self.start_server(port)
        await self.until(lambda: self.relay.vehicle is not None and gcs.is_connected, timeout=6)
        await self.until(lambda: len(received) > restart_at + 21 * 50, timeout=5)

        out, err = await asyncio.wait_for(proc.communicate(), 15)
        self.assertEqual(proc.returncode, 0, err.decode())
        stats = dict(item.split("=") for item in out.decode().split())
        self.assertEqual(stats["sessions"], "2")
        self.assertEqual(stats["bad"], "0")

        counters, gaps = [], 0
        for frame in split_frames(bytes(received)):
            if frame[7:10] == b"\x00\x00\x00":  # msgid 0: the harness's own frames
                self.assertEqual(len(frame), 21)
                counters.append(int.from_bytes(frame[10:14], "little"))
        for a, b in zip(counters, counters[1:]):
            self.assertGreater(b, a)
            gaps += b != a + 1
        self.assertLessEqual(gaps, 1, "frames lost other than around the relay restart")
        self.assertGreater(len(counters), 500)


if __name__ == "__main__":
    unittest.main()

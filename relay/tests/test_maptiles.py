"""The moving map's tiles (maptiles.py): the Web Mercator arithmetic, Esri's tile addresses, and the loader,
against a fake Esri (no internet). Run from the relay directory:  python -m unittest -v"""

import math
import os
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import maptiles  # noqa: E402


def wait_for(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            raise AssertionError("not in time")
        time.sleep(0.01)


class TileMathTest(unittest.TestCase):
    def test_pixels(self):
        self.assertEqual(maptiles.to_pixel(0, 0, 0), (128.0, 128.0))  # the middle of the one tile
        self.assertEqual(maptiles.to_pixel(0, -180, 1), (0.0, 256.0))
        self.assertAlmostEqual(maptiles.to_pixel(maptiles.MAX_LAT, 0, 4)[1], 0, places=3)  # the top edge
        for lat, lon in ((41.123457, 28.987654), (-33.8568, 151.2153), (64.1466, -21.9426)):
            x, y = maptiles.to_pixel(lat, lon, 16)
            # the formula every slippy map uses, written the other way
            size = 256 * 2 ** 16
            self.assertAlmostEqual(y, (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * size, places=6)
            for zoom in (3, 16, 19):
                back = maptiles.to_latlon(*maptiles.to_pixel(lat, lon, zoom), zoom)
                self.assertAlmostEqual(back[0], lat, places=9)
                self.assertAlmostEqual(back[1], lon, places=9)

    def test_tiles_around(self):
        self.assertEqual(sorted(maptiles.tiles_around(600, 600, 400, 400, 3)),
                         [(col, row) for col in (1, 2, 3) for row in (1, 2, 3)])
        rows = {row for _, row in maptiles.tiles_around(1024, 100, 400, 400, 3)}
        self.assertEqual(rows, {0, 1})  # none above the world
        cols = {col for col, _ in maptiles.tiles_around(100, 1024, 400, 400, 3)}
        self.assertEqual(cols, {-1, 0, 1})  # beyond the date line, on the left
        self.assertEqual((maptiles.tile_key(3, -1, 2), maptiles.tile_key(3, 8, 2)), ((3, 7, 2), (3, 0, 2)))

    def test_url(self):
        self.assertEqual(maptiles.tile_url((16, 38227, 24538)), "https://server.arcgisonline.com/ArcGIS/rest/services/"
                                                                 "World_Imagery/MapServer/tile/16/24538/38227")


class TileLoaderTest(unittest.TestCase):
    def test_fetches_once_and_keeps(self):
        asked = []
        loader = maptiles.TileLoader(lambda url: asked.append(url) or b"jpeg " + url.encode())
        key = (16, 1, 2)
        self.assertIsNone(loader.get(key))  # on its way
        wait_for(lambda: loader.get(key) is not None)
        self.assertEqual(loader.get(key), b"jpeg " + maptiles.tile_url(key).encode())
        self.assertTrue(loader.arrived())
        self.assertFalse(loader.arrived())  # nothing new since
        self.assertEqual(len(asked), 1)

    def test_a_failure_waits(self):
        calls = []

        def fetch(url):
            calls.append(url)
            raise OSError("no internet")

        loader = maptiles.TileLoader(fetch)
        key = (16, 1, 2)
        with self.assertLogs("mavrelay.map", "WARNING") as logs:
            loader.get(key)
            wait_for(lambda: key in loader.failed)
        self.assertIn("cannot load the map's tiles from Esri: no internet", logs.output[0])
        self.assertIsNone(loader.get(key))  # not again for a while
        time.sleep(0.1)
        self.assertEqual(len(calls), 1)
        loader.failed[key] -= loader.RETRY_S  # half a minute later
        loader.get(key)
        wait_for(lambda: len(calls) == 2)

    def test_keeps_the_newest(self):
        with mock.patch.object(maptiles.TileLoader, "KEEP", 3):
            loader = maptiles.TileLoader(lambda url: b"jpeg")
            for col in range(5):
                loader.get((16, col, 0))
                wait_for(lambda: loader.get((16, col, 0)) is not None)
            self.assertEqual(list(loader.tiles), [(16, 2, 0), (16, 3, 0), (16, 4, 0)])

    def test_the_newest_wishes_first(self):
        started, release = threading.Event(), threading.Event()

        def fetch(url):
            started.set()
            release.wait(5)
            return b"jpeg"

        with mock.patch.object(maptiles.TileLoader, "WORKERS", 1), \
                mock.patch.object(maptiles.TileLoader, "WANTED_MOST", 2):
            loader = maptiles.TileLoader(fetch)
            loader.get((16, 0, 0))
            started.wait(5)  # busy with that one
            for col in (1, 2, 3):
                loader.get((16, col, 0))
            self.assertEqual(list(loader.wanted), [(16, 2, 0), (16, 3, 0)])  # the map moved on from (16, 1, 0)
            release.set()
            wait_for(lambda: len(loader.tiles) == 3)
            self.assertNotIn((16, 1, 0), loader.tiles)


if __name__ == "__main__":
    unittest.main()

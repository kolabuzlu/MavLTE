"""The moving map's tiles, for MavLTE's window: Esri World Imagery, in the Web Mercator grid every web map uses
(256-pixel tiles; zoom 0 is the whole world in one tile, and each zoom level doubles it). The web page does the
same in web/app.js. Standard library: the tiles come as JPEG, which the window turns into pictures with Pillow.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import urllib.request
from collections import OrderedDict, deque
from typing import Callable, Deque, Dict, Iterator, List, Optional, Set, Tuple

import mavrelay as mr

TILE = 256
MIN_ZOOM, MAX_ZOOM = 3, 19
URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
ATTRIBUTION = "Esri, Vantor, Earthstar Geographics, and the GIS User Community"  # the service's own credit line
MAX_LAT = 85.05112878  # the Web Mercator square ends here

Key = Tuple[int, int, int]  # zoom, column, row

log = logging.getLogger("mavrelay.map")


def to_pixel(lat: float, lon: float, zoom: int) -> Tuple[float, float]:
    """Where a point lies on the world map at `zoom`, in pixels from its top left corner."""
    size = TILE * 2 ** zoom
    s = math.sin(math.radians(max(-MAX_LAT, min(MAX_LAT, lat))))
    return (lon + 180.0) / 360.0 * size, (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * size


def to_latlon(x: float, y: float, zoom: int) -> Tuple[float, float]:
    size = TILE * 2 ** zoom
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / size)))), x / size * 360.0 - 180.0


def tiles_around(cx: float, cy: float, width: int, height: int, zoom: int) -> Iterator[Tuple[int, int]]:
    """The tiles a view of width x height pixels centred on world pixel (cx, cy) shows, as (column, row): columns
    may lie beyond the date line (tile_key wraps them), rows only where there are tiles."""
    last = 2 ** zoom - 1
    for row in range(max(0, int((cy - height / 2) // TILE)), min(last, int((cy + height / 2) // TILE)) + 1):
        for col in range(int((cx - width / 2) // TILE), int((cx + width / 2) // TILE) + 1):
            yield col, row


def tile_key(zoom: int, col: int, row: int) -> Key:
    return zoom, col % 2 ** zoom, row


def tile_url(key: Key) -> str:
    zoom, col, row = key
    return URL.format(z=zoom, y=row, x=col)


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={
        "User-Agent": f"MavLTE/{mr.__version__} (+https://github.com/kolabuzlu/MavLTE)"})
    with urllib.request.urlopen(request, timeout=15) as response:
        return response.read()


class TileLoader:
    """Fetches tiles on a few threads of its own, the newest wishes first, and keeps their JPEG bytes: the most
    recently used KEEP. get() answers at once, with the bytes or None (then the tile is on its way, or failed
    less than RETRY_S ago). arrived() says whether tiles came since it was last asked: the map draws again."""

    KEEP = 600  # tiles: about 15 MB
    WORKERS = 4
    WANTED_MOST = 64  # waiting at most: the newest (a map that moved on needs the older ones no more)
    RETRY_S = 30.0

    def __init__(self, fetch: Callable[[str], bytes] = fetch) -> None:
        self.fetch = fetch
        self.lock = threading.Condition()
        self.tiles: "OrderedDict[Key, bytes]" = OrderedDict()
        self.wanted: Deque[Key] = deque()
        self.busy: Set[Key] = set()
        self.failed: Dict[Key, float] = {}
        self.fresh = False
        self.warned = False
        self.threads: List[threading.Thread] = []

    def get(self, key: Key) -> Optional[bytes]:
        with self.lock:
            data = self.tiles.get(key)
            if data is not None:
                self.tiles.move_to_end(key)
                return data
            if key in self.busy or key in self.wanted or time.monotonic() - self.failed.get(key, -1e9) < self.RETRY_S:
                return None
            self.wanted.append(key)
            while len(self.wanted) > self.WANTED_MOST:
                self.wanted.popleft()
            if len(self.threads) < self.WORKERS:
                thread = threading.Thread(target=self._work, name="map tiles", daemon=True)
                self.threads.append(thread)
                thread.start()
            self.lock.notify()
        return None

    def arrived(self) -> bool:
        with self.lock:
            fresh, self.fresh = self.fresh, False
            return fresh

    def _work(self) -> None:
        while True:
            with self.lock:
                while not self.wanted:
                    self.lock.wait()
                key = self.wanted.pop()
                self.busy.add(key)
            try:
                data, problem = self.fetch(tile_url(key)), None
            except Exception as exc:  # no internet, a timeout, an HTTP error: the map shows no tile there
                data, problem = None, exc
            with self.lock:
                self.busy.discard(key)
                if data:
                    self.tiles[key] = data
                    while len(self.tiles) > self.KEEP:
                        self.tiles.popitem(last=False)
                    self.failed.pop(key, None)
                    self.fresh, self.warned = True, False
                else:
                    self.failed[key] = time.monotonic()
                    if not self.warned:
                        self.warned = True
                        log.warning("cannot load the map's tiles from Esri: %s", problem or "an empty answer")

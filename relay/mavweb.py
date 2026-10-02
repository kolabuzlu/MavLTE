#!/usr/bin/env python3
"""mavweb - MavLTE's Aircraft card as a web page, for a phone: the aircraft's link, where it is, its locator
voice and a photo from its camera, with no computer at hand.

    phone ==HTTPS==> Caddy ==HTTP 127.0.0.1:8090==> mavweb ==UDP==> mavrelay server <==4G== aircraft

It runs on the relay's server beside `mavrelay.py server`, as a GCS agent that only watches: the aircraft holds
its telemetry back for it, as for MavLTE with both switches off. It finds the relay's port and the GCS key in
the relay's config file, and its own settings in a [web] section there (install-web.sh sets it all up):

    [web]
    listen = 127.0.0.1:8090      plain HTTP, for the HTTPS proxy in front of it: keep it on 127.0.0.1
    name = My UAV                the aircraft's name on the page
    photo_dir = ...              the aircraft's photos (default: web-photos in /var/lib/mavrelay)
    password = pbkdf2_sha256$... both written by `mavweb.py password`; a new password signs every phone out
    secret = ...

    sudo python3 mavweb.py password --config /etc/mavrelay/mavrelay.ini     sets the page's password
    python3 mavweb.py --config /etc/mavrelay/mavrelay.ini                   the page itself (mavweb.service)

The page is the web folder beside this file. Its moving map loads Esri World Imagery tiles straight from Esri
into the phone; mavweb keeps the aircraft's track for it. Standard library only (3.8+), like mavrelay.py.
"""

from __future__ import annotations

import argparse
import asyncio
import getpass
import hashlib
import hmac
import http.cookies
import http.server
import json
import logging
import math
import os
import re
import secrets
import signal
import socket
import socketserver
import sys
import threading
import time
import urllib.parse
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

import aircraft_card
import mavrelay as mr

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
# the page's files, open to anyone: nothing of the aircraft is in them
FILES = {
    "/": (os.path.join(WEB, "index.html"), "text/html; charset=utf-8"),
    "/app.css": (os.path.join(WEB, "app.css"), "text/css; charset=utf-8"),
    "/app.js": (os.path.join(WEB, "app.js"), "text/javascript; charset=utf-8"),
    "/manifest.webmanifest": (os.path.join(WEB, "manifest.webmanifest"), "application/manifest+json"),
    "/icon.png": (os.path.join(HERE, "mavlte.png"), "image/png"),
}
HEADERS = (
    ("Content-Security-Policy", "default-src 'none'; script-src 'self'; style-src 'self'; "
                                "img-src 'self' https://server.arcgisonline.com; "  # the moving map's tiles
                                "connect-src 'self'; manifest-src 'self'; base-uri 'none'; form-action 'self'; "
                                "frame-ancestors 'none'"),
    ("X-Content-Type-Options", "nosniff"),
    ("Referrer-Policy", "no-referrer"),
    ("X-Frame-Options", "DENY"),
)

COOKIE = "mavlte"
SIGNED_IN_S = 180 * 86400  # a phone stays signed in this long after its last visit
ROUNDS = 600_000  # PBKDF2-SHA256, for the password
KEEP_PHOTOS = 200  # the newest photos kept in photo_dir
PHOTO_NAME = re.compile(r"MavLTE_.*_(\d+)\.jpg")

wlog = mr.log.getChild("web")


# ---------------------------------------------------------------------------------------------
# The password, and who is signed in


def hash_password(password: str, rounds: Optional[int] = None) -> str:
    rounds = rounds or ROUNDS
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, rounds)
    return f"pbkdf2_sha256${rounds}${salt.hex()}${digest.hex()}"


def check_password(password: str, stored: str) -> bool:
    try:
        scheme, rounds, salt, digest = stored.split("$")
        if scheme != "pbkdf2_sha256":
            return False
        got = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), int(rounds))
        return hmac.compare_digest(got, bytes.fromhex(digest))
    except ValueError:
        return False


def make_password() -> str:
    """Easy to type on a phone: three groups of four letters and digits, none that look alike (59 bits)."""
    alphabet = "abcdefghjkmnpqrstuvwxyz23456789"
    return "-".join("".join(secrets.choice(alphabet) for _ in range(4)) for _ in range(3))


def set_password(path: str, password: str, rounds: Optional[int] = None) -> None:
    """Keeps the password's hash in [web], with a new secret: every phone signed in has to sign in again."""
    mr.update_config(path, "web", {"password": hash_password(password, rounds), "secret": secrets.token_hex(32)})


def cookie_header(value: str, max_age: int = SIGNED_IN_S) -> str:
    return f"{COOKIE}={value}; Max-Age={max_age}; Path=/; HttpOnly; Secure; SameSite=Strict"


class Access:
    """Who may see the page: the password, sign-ins kept in cookies signed with the secret, and a pause after
    wrong passwords. Both come from [web] in the config file, read again when the file changes (`mavweb.py
    password` while the page runs). Used from the HTTP server's threads."""

    TRIES = 5  # wrong passwords from one address within TRIES_S; then it waits
    TRIES_ALL = 30  # ... from all addresses together
    TRIES_S = 600.0
    CHECK_EVERY = 2.0  # seconds between looks at the config file

    def __init__(self, path: str) -> None:
        self.path = path
        self.lock = threading.Lock()
        self.hashing = threading.Semaphore(2)  # PBKDF2 takes a moment: a few at once at most
        self.password = ""
        self.secret = b""
        self.seen: Optional[bytes] = None  # the config file as last read
        self.checked = -1e9
        self.fails: Dict[str, Deque[float]] = {}
        self.all_fails: Deque[float] = deque()
        self._reload(time.monotonic(), first=True)

    def _reload(self, now: float, first: bool = False) -> None:
        """Takes a new password from the config file (holding the lock, or from __init__)."""
        if now - self.checked < self.CHECK_EVERY:
            return
        self.checked = now
        try:
            with open(self.path, "rb") as f:
                seen = f.read()
        except OSError as exc:
            if first:
                raise SystemExit(f"cannot read config file {self.path}: {exc.strerror or exc}") from None
            return
        if seen == self.seen:
            return
        self.seen = seen  # a broken file is not read again until it changes
        try:
            web = mr.load_config(self.path, "web")
        except SystemExit as exc:
            if first:
                raise
            wlog.warning("%s; the page keeps its password", exc)
            return
        password, secret = web.get("password", "").strip(), web.get("secret", "").strip()
        if not password.startswith("pbkdf2_sha256$") or len(secret) < 32:
            problem = (f"no password for the web page in {self.path}: set one with  "
                       f"sudo python3 {os.path.abspath(__file__)} password --config {self.path}")
            if first:
                raise SystemExit(problem)
            wlog.warning("%s; the page keeps its password", problem)
            return
        if not first and (password, secret.encode()) != (self.password, self.secret):
            wlog.info("the page has a new password: every phone signs in again")
        self.password, self.secret = password, secret.encode()

    def _tag(self, expires: int) -> bytes:
        return hmac.new(self.secret, f"signed in until {expires}".encode(), hashlib.sha256).hexdigest().encode()

    def cookie(self, now_unix: float) -> str:
        """A new sign-in, for SIGNED_IN_S."""
        with self.lock:
            expires = int(now_unix) + SIGNED_IN_S
            return f"{expires}.{self._tag(expires).decode()}"

    def signed_in(self, value: str, now_unix: float) -> Optional[int]:
        """When the sign-in in the cookie runs out, or None if it is not one of ours, or has run out."""
        expires, _, tag = value.partition(".")
        if not (expires.isdigit() and len(expires) < 12) or int(expires) <= now_unix:
            return None
        with self.lock:
            self._reload(time.monotonic())
            good = hmac.compare_digest(tag.encode("utf-8", "replace"), self._tag(int(expires)))
        return int(expires) if good else None

    def _wait(self, who: str, now: float) -> float:
        """Seconds before `who` may try a password again (holding the lock)."""
        for fails in (self.fails.get(who), self.all_fails):
            while fails and now - fails[0] > self.TRIES_S:
                fails.popleft()
        if who in self.fails and not self.fails[who]:
            del self.fails[who]
        mine = self.fails.get(who, ())
        if len(mine) >= self.TRIES:
            return mine[0] + self.TRIES_S - now
        if len(self.all_fails) >= self.TRIES_ALL:
            return self.all_fails[0] + self.TRIES_S - now
        return 0.0

    def sign_in(self, password: str, who: str, now: float) -> Tuple[int, str]:
        """(204, '') for the right password, else the HTTP status and why: 401 wrong, 429 too many tries."""
        with self.lock:
            self._reload(now)
            wait = self._wait(who, now)
            stored = self.password
        if wait > 0:
            return 429, f"Too many wrong passwords: try again in {math.ceil(wait / 60)} min"
        with self.hashing:
            right = check_password(password, stored)
        with self.lock:
            if right:
                self.fails.pop(who, None)
                return 204, ""
            if len(self.fails) > 1000:  # many addresses: forget the ones that have waited long enough
                for other in list(self.fails):
                    self._wait(other, now)
            self.fails.setdefault(who, deque()).append(now)
            self.all_fails.append(now)
        return 401, "Wrong password"


# ---------------------------------------------------------------------------------------------
# The agent, and what the page shows of it


def photo_entry(path: str, short: bool = False) -> dict:
    """A photo for the page; its time in unix seconds, which the page writes in the phone's own time zone."""
    meta = aircraft_card.photo_meta(path)
    return {"id": int(PHOTO_NAME.fullmatch(os.path.basename(path)).group(1)), "time": aircraft_card.photo_time(meta),
            "caption": aircraft_card.photo_caption(meta, short=short, clock=False)}


class WebPage:
    """The page's server side: a GCS agent on an asyncio loop of its own, in a thread (as in MavLTE), and what
    the page reads from it and asks of it, done on that loop."""

    def __init__(self, opts: argparse.Namespace) -> None:
        self.opts = opts
        self.access = Access(opts.config)
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, name="agent", daemon=True)
        self.agent: Optional[mr.GcsAgent] = None
        self.task: Optional[asyncio.Future] = None
        self.tracking: Optional[asyncio.Future] = None
        self.failed = False
        self.on_failure: Optional[Callable[[], None]] = None
        self.voice = aircraft_card.Voice()
        self.voice_shown = (False, aircraft_card.Shown("", aircraft_card.DIM))
        self.track = aircraft_card.Track()  # for the moving map: kept on the agent's loop
        self.by_id: Dict[int, str] = {}  # the photos kept
        self.newest: Optional[dict] = None
        self._photos_changed()

    def start(self) -> None:
        self.thread.start()

        def go() -> None:
            self.agent = mr.GcsAgent(self.opts.relay, self.opts.key, info=f"mavlte-web/{mr.__version__}",
                                     photo_dir=self.opts.photo_dir, on_photo=lambda path, info: self._photos_changed())
            self.task = asyncio.ensure_future(self.agent.run())
            self.task.add_done_callback(self._ended)
            self.tracking = asyncio.ensure_future(self._track())

        self.call(go)

    async def _track(self) -> None:
        """Every fix onto the track, also while no phone looks: the map then shows where the aircraft went."""
        while True:
            self.track.add(self.agent.last_fix)
            await asyncio.sleep(0.5)

    def _ended(self, task: asyncio.Future) -> None:
        if not task.cancelled() and task.exception() is not None:
            wlog.error("the agent stopped: %r", task.exception())
            self.failed = True
            if self.on_failure is not None:
                self.on_failure()

    def stop(self) -> None:
        async def go() -> None:
            tasks = [task for task in (self.task, self.tracking) if task is not None]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        if self.thread.is_alive():
            try:
                asyncio.run_coroutine_threadsafe(go(), self.loop).result(5)
            except Exception:  # going away all the same
                pass
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(5)
        self.loop.close()

    def call(self, fn: Callable, timeout: float = 5.0):
        """fn() on the agent's loop, from one of the HTTP server's threads."""
        async def go():
            return fn()

        return asyncio.run_coroutine_threadsafe(go(), self.loop).result(timeout)

    def _photos_changed(self) -> None:
        """A photo came in (or the page starts): forget the oldest beyond KEEP_PHOTOS."""
        paths = aircraft_card.photo_files(self.opts.photo_dir)
        for old in paths[:-KEEP_PHOTOS]:
            for name in (old, old[:-4] + ".json"):
                try:
                    os.remove(name)
                except OSError:
                    pass
        paths = paths[-KEEP_PHOTOS:]
        self.by_id = {int(PHOTO_NAME.fullmatch(os.path.basename(p)).group(1)): p for p in paths}
        self.newest = photo_entry(paths[-1], short=True) if paths else None

    def photo_path(self, photo_id: int) -> Optional[str]:
        return self.by_id.get(photo_id)

    def photos(self) -> List[dict]:
        """The photos kept, newest first, for the viewer."""
        return [photo_entry(path) for _, path in sorted(self.by_id.items(), reverse=True)]

    def track_points(self) -> List[list]:
        """The track for the moving map: [latitude, longitude, unix time], oldest first."""
        return self.call(lambda: [[round(p.lat / 1e7, 7), round(p.lon / 1e7, 7), p.time] for p in self.track.fixes])

    def state(self, size: int) -> dict:
        """The Aircraft card, as MavLTE shows it (size: the photo size the phone has chosen)."""
        return self.call(lambda: self._state(size))

    def _state(self, size: int) -> dict:
        agent = self.agent
        now, now_unix = time.monotonic(), time.time()
        status = agent.vehicle_status() if agent.client.session else None
        online = aircraft_card.is_online(status)
        relay_led, relay_text = aircraft_card.relay(agent.client, rtt=False)  # it is on this server
        led, headline = aircraft_card.headline(agent, status, switches=False)
        where = aircraft_card.position(agent, online, now_unix)
        module, temp = aircraft_card.module(agent, online, now_unix)
        shown = self.voice.show(agent, status, now)
        if shown is not None:
            self.voice_shown = shown
        voice_on, voice = self.voice_shown
        camera = aircraft_card.camera(agent, online, size, now)
        fix = agent.last_fix  # the moving map's aircraft
        self.track.add(fix)
        live = aircraft_card.live(agent, online, now_unix)
        return {
            "name": self.opts.name,
            "version": mr.__version__,
            "relay": {"led": relay_led, "text": relay_text},
            "aircraft": {"led": led, "text": headline, "bars": aircraft_card.bars(status)},
            "link": aircraft_card.link(status),
            "loss": aircraft_card.loss(status),
            "position": {"text": where.text, "color": where.color,
                         "map": aircraft_card.map_link(where.fix) if where.fix else None,
                         "copy": aircraft_card.coordinates(where.fix) if where.fix else None},
            "module": {"text": module.text, "color": module.color},
            "temp": {"text": temp.text, "color": temp.color},
            "fix": None if fix is None else {
                "lat": round(fix.lat / 1e7, 7), "lon": round(fix.lon / 1e7, 7), "time": fix.time,
                "heading": aircraft_card.heading(fix), "live": live is not None and live.has_fix},
            "voice": {"on": voice_on, "text": voice.text, "color": voice.color, "can": bool(agent.client.session)},
            "camera": camera._asdict(),
            "photo": self.newest,
        }

    def set_voice(self, on: bool) -> bool:
        """False while there is no session with the relay."""
        return self.call(lambda: self.agent.set_voice(on))

    def snapshot(self, size: int) -> Optional[str]:
        """Asks the aircraft for a photo; None, or why not (as the card says it)."""
        def go() -> Optional[str]:
            agent = self.agent
            status = agent.vehicle_status() if agent.client.session else None
            camera = aircraft_card.camera(agent, aircraft_card.is_online(status), size, time.monotonic())
            if not camera.ready:
                return camera.text
            return None if agent.photos.request(size) else "Not connected to the relay"

        problem = self.call(go)
        if problem is None:
            wlog.info("asking the aircraft for a photo (%s, %d×%d)", aircraft_card.SIZE_NAMES[size], *mr.SNAP_SIZES[size])
        return problem


# ---------------------------------------------------------------------------------------------
# HTTP


class Handler(http.server.BaseHTTPRequestHandler):
    server_version = "mavweb"
    sys_version = ""
    timeout = 30

    @property
    def page(self) -> WebPage:
        return self.server.page

    def log_message(self, format, *args) -> None:  # a phone asks every second: the log would be nothing else
        pass

    def who(self) -> str:
        """The phone's address: from the HTTPS proxy in front (on this server), which puts it last."""
        peer = self.client_address[0]
        forwarded = self.headers.get("X-Forwarded-For", "").split(",")[-1].strip()
        return forwarded if forwarded and mr.is_loopback(peer) else peer

    def send(self, code: int, body: bytes = b"", ctype: str = "", cache: str = "no-store", headers=()) -> None:
        self.send_response(code)
        if ctype:
            self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        for name, value in (*HEADERS, *headers):
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, code: int, data, headers=()) -> None:
        self.send(code, json.dumps(data).encode(), "application/json", headers=headers)

    def signed_in(self) -> Optional[int]:
        jar = http.cookies.SimpleCookie()
        try:
            jar.load(self.headers.get("Cookie", ""))
        except http.cookies.CookieError:
            return None
        morsel = jar.get(COOKIE)
        return self.page.access.signed_in(morsel.value, time.time()) if morsel is not None else None

    def from_the_page(self) -> bool:
        """A POST from the page itself, not a form on another site: the page's own header (no other site can
        send it without asking first, and nothing here says yes), and its origin."""
        if self.headers.get("X-MavLTE") != "1":
            return False
        origin = self.headers.get("Origin")
        return origin is None or urllib.parse.urlsplit(origin).netloc == self.headers.get("Host", "")

    def read_json(self) -> Optional[dict]:
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return None
        if not 0 <= n <= 4096:
            return None
        try:
            data = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return None
        return data if isinstance(data, dict) else None

    def do_GET(self) -> None:
        self._answer(self._get)

    def do_POST(self) -> None:
        self._answer(self._post)

    def _answer(self, handler: Callable[[], None]) -> None:
        try:
            handler()
        except Exception as exc:  # the agent's loop not answering, say: the page says so, and asks again
            wlog.error("%s %s: %r", self.command, urllib.parse.urlsplit(self.path).path, exc)
            self.send_json(500, {"error": "The page's server has a problem: see journalctl -u mavweb"})

    def _get(self) -> None:
        url = urllib.parse.urlsplit(self.path)
        if url.path in FILES:
            path, ctype = FILES[url.path]
            try:
                with open(path, "rb") as f:
                    body = f.read()
            except OSError:
                return self.send_json(404, {"error": "Not found"})
            return self.send(200, body, ctype, cache="no-cache")  # always the newest after an update
        photo = re.fullmatch(r"/photo/(\d{1,10})\.jpg", url.path)
        if url.path not in ("/api/state", "/api/photos", "/api/track") and photo is None:
            return self.send_json(404, {"error": "Not found"})
        expires = self.signed_in()
        if expires is None:
            return self.send_json(401, {"signin": True})
        if url.path == "/api/state":
            size = urllib.parse.parse_qs(url.query).get("size", ["1"])[0]
            state = self.page.state(int(size) if size in ("0", "1", "2") else 1)
            now = time.time()
            renew = []  # signed in for SIGNED_IN_S from the last visit
            if expires < now + SIGNED_IN_S - 86400:
                renew.append(("Set-Cookie", cookie_header(self.page.access.cookie(now))))
            return self.send_json(200, state, renew)
        if url.path == "/api/photos":
            return self.send_json(200, self.page.photos())
        if url.path == "/api/track":
            return self.send_json(200, {"fixes": self.page.track_points()})
        path = self.page.photo_path(int(photo.group(1)))
        try:
            with open(path or "", "rb") as f:
                body = f.read()
        except OSError:
            return self.send_json(404, {"error": "Not found"})
        self.send(200, body, "image/jpeg", cache="private, max-age=31536000, immutable")  # an id is one photo

    def _post(self) -> None:
        path = urllib.parse.urlsplit(self.path).path
        if not self.from_the_page():
            return self.send_json(403, {"error": "Not from the page"})
        data = self.read_json()
        if data is None:
            return self.send_json(400, {"error": "Bad request"})
        if path == "/api/signin":
            password = data.get("password")
            if not isinstance(password, str) or len(password) > 256:
                return self.send_json(400, {"error": "Bad request"})
            code, why = self.page.access.sign_in(password, self.who(), time.monotonic())
            if code != 204:
                if code == 401:
                    wlog.warning("wrong password for the page, from %s", self.who())
                return self.send_json(code, {"error": why})
            wlog.info("signed in to the page from %s", self.who())
            return self.send(204, headers=[("Set-Cookie", cookie_header(self.page.access.cookie(time.time())))])
        if path == "/api/signout":
            return self.send(204, headers=[("Set-Cookie", cookie_header("", 0))])
        if path not in ("/api/voice", "/api/snapshot"):
            return self.send_json(404, {"error": "Not found"})
        if self.signed_in() is None:
            return self.send_json(401, {"signin": True})
        if path == "/api/voice":
            on = data.get("on")
            if not isinstance(on, bool):
                return self.send_json(400, {"error": "Bad request"})
            if not self.page.set_voice(on):
                return self.send_json(503, {"error": "Not connected to the relay"})
            wlog.info("the locator voice switched %s on the page, from %s", "on" if on else "off", self.who())
            return self.send_json(200, {"ok": True})
        size = data.get("size")
        if isinstance(size, bool) or size not in (0, 1, 2):
            return self.send_json(400, {"error": "Bad request"})
        problem = self.page.snapshot(size)
        if problem:
            return self.send_json(409, {"error": problem})
        self.send_json(200, {"ok": True})


class WebServer(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, listen: Tuple[str, int], page: WebPage) -> None:
        self.address_family = socket.AF_INET6 if ":" in listen[0] else socket.AF_INET
        self.page = page
        super().__init__(listen, Handler)

    def server_bind(self) -> None:  # without HTTPServer's reverse lookup of its own address
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


# ---------------------------------------------------------------------------------------------
# Command line


def load_options(path: str) -> argparse.Namespace:
    server = mr.load_config(path, "server")
    web = mr.load_config(path, "web")
    opts = argparse.Namespace(config=path)
    key = web.get("key", "").strip() or server.get("gcs_key", "").strip()
    if not key:
        raise SystemExit(f"{path}: no GCS key (gcs_key in [server], or key in [web])")
    try:
        if web.get("relay", "").strip():
            opts.relay = mr.parse_hostport(web["relay"])
        else:  # the relay on this server
            host, port = mr.parse_hostport(server.get("listen", "").strip() or "0.0.0.0:14650")
            opts.relay = ("127.0.0.1" if host in ("", "0.0.0.0") else "::1" if host == "::" else host, port)
        opts.key = mr.parse_key(key)
        opts.listen = mr.parse_hostport(web.get("listen", "").strip() or "127.0.0.1:8090")
    except ValueError as exc:
        raise SystemExit(f"{path}: {exc}") from None
    opts.name = web.get("name", "").strip() or "My UAV"
    home = os.environ.get("STATE_DIRECTORY") or os.path.dirname(os.path.abspath(path))
    opts.photo_dir = web.get("photo_dir", "").strip() or os.path.join(home, "web-photos")
    opts.log_level = web.get("log_level", "").strip() or server.get("log_level", "").strip() or "info"
    return opts


def password_command(args: argparse.Namespace) -> None:
    if not os.path.exists(args.config):
        raise SystemExit(f"no config file {args.config}: install the relay first (install-server.sh)")
    if args.random:
        password = make_password()
    else:
        password = getpass.getpass("New password for the web page: ")
        if len(password) < 8:
            raise SystemExit("too short: 8 characters at least")
        if getpass.getpass("Again: ") != password:
            raise SystemExit("not the same: the password stays as it was")
    try:
        set_password(args.config, password)
    except OSError as exc:
        raise SystemExit(f"cannot write {args.config}: {exc.strerror or exc}") from None
    if args.random:
        print(f"The web page's password: {password}")
    print(f"Kept in {args.config} (as a hash). Phones signed in before sign in again with it.")


def run(opts: argparse.Namespace) -> None:
    page = WebPage(opts)
    httpd = WebServer(opts.listen, page)
    page.on_failure = lambda: threading.Thread(target=httpd.shutdown, daemon=True).start()
    page.start()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # systemctl stop
    wlog.info("MavLTE web page %s for %s: http://%s (behind the HTTPS proxy), relay %s", mr.__version__, opts.name,
              mr.fmt_addr(opts.listen), mr.fmt_addr(opts.relay))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        page.stop()
    if page.failed:
        raise SystemExit(1)  # systemd starts it again


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="mavweb", description="MavLTE's Aircraft card as a web page, for a phone. "
                                                           "Runs on the relay's server, behind an HTTPS proxy.")
    p.add_argument("--version", action="version", version=mr.__version__)
    p.add_argument("command", nargs="?", choices=("run", "password"), default="run",
                   help="run the page (the default), or set its password")
    p.add_argument("--config", default="/etc/mavrelay/mavrelay.ini",
                   help="the relay's config file (default /etc/mavrelay/mavrelay.ini)")
    p.add_argument("--random", action="store_true", help="password: make one up, and print it")
    return p


def main(argv: Optional[List[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    if args.command == "password":
        password_command(args)
        return
    opts = load_options(args.config)
    logging.basicConfig(
        level=getattr(logging, str(opts.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    run(opts)


if __name__ == "__main__":
    main()

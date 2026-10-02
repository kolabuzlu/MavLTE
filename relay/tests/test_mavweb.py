"""The web page (mavweb.py): signing in, the Aircraft card as the phone gets it, the locator voice and a photo,
against a real relay and a fake aircraft. The page's server runs in this process, on a free port.
Run from the relay directory:  python -m unittest -v"""

import asyncio
import contextlib
import http.client
import io
import json
import os
import re
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import aircraft_card  # noqa: E402
import mavrelay as mr  # noqa: E402
import mavweb  # noqa: E402
from test_mavrelay import KEY_G, KEY_V  # noqa: E402

PASSWORD = "wheat-field-7"
ROUNDS = 1000  # quick; the page itself uses mavweb.ROUNDS
JPEG = b"\xff\xd8" + bytes(range(256)) * 20 + b"\xff\xd9"


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def fake_photo(folder, photo_id):
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"MavLTE_2026-10-02_07-18-01_{photo_id}.jpg")
    with open(path, "wb") as f:
        f.write(JPEG)
    with open(path[:-4] + ".json", "w") as f:
        json.dump(dict(mr.PhotoInfo(photo_id, len(JPEG), 320, 240, time=photo_id)._asdict()), f)
    return path


class WebTest(unittest.TestCase):
    def setUp(self):
        # the relay and the aircraft, on their own asyncio loop
        self.loop = asyncio.SelectorEventLoop() if sys.platform == "win32" else asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.home = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.home, True)
        self.relay = mr.RelayServer({mr.ROLE_VEHICLE: KEY_V, mr.ROLE_GCS: KEY_G}, state_dir=self.home)
        self.modem = {"voice": "speaks"}

        async def start():
            transport, _ = await self.loop.create_datagram_endpoint(lambda: self.relay, local_addr=("127.0.0.1", 0))
            self.vehicle = mr.TunnelClient(mr.ROLE_VEHICLE, KEY_V, "127.0.0.1", transport.get_extra_info("sockname")[1],
                                           on_data=lambda data: None)
            self.outbox = outbox = mr.PhotoOutbox(self.vehicle, capture=lambda width, height: JPEG,
                                                  where=lambda: (411234567, 289876543, 120_000, 4500),
                                                  cap=lambda: 64 * 1024)
            self.vehicle.on_packet, self.vehicle.on_session = outbox.on_packet, outbox.on_session
            self.vehicle_task = asyncio.ensure_future(self.vehicle.run())

            async def ticks():  # and the aircraft's speaker, as the firmware reports it
                while True:
                    await asyncio.sleep(0.05)
                    now = time.monotonic()
                    self.relay.tick(now)
                    self.relay.photos.pump(now)
                    outbox.pump(now)
                    flags = mr.PING_FLAG_SPEAKING if self.vehicle.voice_on and self.modem["voice"] == "speaks" else 0
                    if flags != self.vehicle.ping_flags:
                        self.vehicle.ping_flags = flags
                        self.vehicle.ping_now()

            asyncio.ensure_future(ticks())
            return transport.get_extra_info("sockname")[1]

        relay_port = asyncio.run_coroutine_threadsafe(start(), self.loop).result(5)
        self.photos = os.path.join(self.home, "web-photos")
        self.config = os.path.join(self.home, "mavrelay.ini")
        with open(self.config, "w", encoding="utf-8") as f:
            f.write(f"[server]\nlisten = 0.0.0.0:{relay_port}\nvehicle_key = {KEY_V.hex()}\ngcs_key = {KEY_G.hex()}\n\n"
                    f"[web]\nlisten = 127.0.0.1:{free_port()}\nname = Test UAV\nphoto_dir = {self.photos}\n")
        mavweb.set_password(self.config, PASSWORD, rounds=ROUNDS)
        opts = mavweb.load_options(self.config)
        self.assertEqual(opts.relay, ("127.0.0.1", relay_port))  # [server]'s 0.0.0.0: the relay on this server
        self.page = mavweb.WebPage(opts)
        self.httpd = mavweb.WebServer(opts.listen, self.page)
        self.port = opts.listen[1]
        self.page.start()
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.page.stop()

        async def cancel_all():
            self.relay.transport.close()
            tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        asyncio.run_coroutine_threadsafe(cancel_all(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()

    # -- a phone

    def request(self, method, path, body=None, cookie=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        sent = dict(headers or {})
        if cookie:
            sent["Cookie"] = f"mavlte={cookie}"
        data = None
        if body is not None:
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            sent.setdefault("Content-Type", "application/json")
            sent.setdefault("X-MavLTE", "1")
        try:
            conn.request(method, path, body=data, headers=sent)
            r = conn.getresponse()
            return r.status, r, r.read()
        finally:
            conn.close()

    def sign_in(self, password=PASSWORD, who=None):
        status, r, body = self.request("POST", "/api/signin", {"password": password},
                                       headers={"X-Forwarded-For": who} if who else None)
        cookie = r.getheader("Set-Cookie")
        return status, cookie.split(";")[0].split("=", 1)[1] if cookie else json.loads(body)["error"]

    def signed_in(self):
        status, cookie = self.sign_in()
        self.assertEqual(status, 204)
        return cookie

    def state(self, cookie, size=None):
        status, _, body = self.request("GET", "/api/state" + (f"?size={size}" if size is not None else ""),
                                       cookie=cookie)
        self.assertEqual(status, 200)
        return json.loads(body)

    def wait(self, cookie, cond, what, timeout=8.0, size=None):
        end = time.time() + timeout
        while True:
            s = self.state(cookie, size)
            if cond(s):
                return s
            if time.time() > end:
                self.fail(f"{what}: not in time, the card says {s}")
            time.sleep(0.1)

    def report(self, **fields):
        fields = {**dict(gnss_time=int(time.time()), lat=411234567, lon=289876543, alt=150_000, sats=11,
                         fix=mr.FIX_3D, fc_silent=0, battery_mv=4298, battery_pct=100, temp=45), **fields}
        self.loop.call_soon_threadsafe(self.vehicle.send_packet, mr.POSITION, mr.Position(**fields).pack())

    def aircraft_gone(self):
        self.relay.ONLINE_TIMEOUT = 0.5  # quicker than the real 3 s
        self.loop.call_soon_threadsafe(self.vehicle_task.cancel)

    # -- the tests

    def test_the_page_is_open_and_the_aircraft_is_not(self):
        status, r, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"<title>MavLTE</title>", body)
        self.assertIn("script-src 'self'", r.getheader("Content-Security-Policy"))
        self.assertEqual((r.getheader("X-Frame-Options"), r.getheader("X-Content-Type-Options")), ("DENY", "nosniff"))
        for path, ctype in (("/app.js", "text/javascript"), ("/app.css", "text/css"), ("/icon.png", "image/png"),
                            ("/manifest.webmanifest", "application/manifest+json")):
            status, r, _ = self.request("GET", path)
            self.assertEqual((status, r.getheader("Content-Type").split(";")[0]), (200, ctype), path)
        for path in ("/api/state", "/api/photos", "/photo/1790000000.jpg"):
            self.assertEqual(self.request("GET", path)[0], 401, path)
        for path in ("/mavweb.py", "/../mavrelay.ini", "/web/app.js", "/photo/../mavrelay.ini", "/index.html"):
            self.assertEqual(self.request("GET", path)[0], 404, path)
        self.assertEqual(self.request("POST", "/api/voice", {"on": True})[0], 401)
        self.assertFalse(self.relay.voice.on)

    def test_signing_in(self):
        with self.assertLogs("mavrelay.web", "WARNING"):
            self.assertEqual(self.sign_in("wheat-field-8"), (401, "Wrong password"))
        status, r, _ = self.request("POST", "/api/signin", {"password": PASSWORD})
        self.assertEqual(status, 204)
        cookie = r.getheader("Set-Cookie")
        self.assertRegex(cookie, r"^mavlte=\d+\.[0-9a-f]{64}; Max-Age=15552000; Path=/; HttpOnly; Secure; "
                                 r"SameSite=Strict$")
        value = cookie.split(";")[0].split("=", 1)[1]
        s = self.state(value)
        self.assertEqual((s["name"], s["version"]), ("Test UAV", mr.__version__))
        status, r, _ = self.request("GET", "/api/state", cookie=value)
        self.assertIsNone(r.getheader("Set-Cookie"))  # a new sign-in: nothing to renew

        access = self.page.access
        tampered = value[:-1] + ("0" if value[-1] != "0" else "1")
        expired = f"{int(time.time()) - 5}.{access._tag(int(time.time()) - 5).decode()}"
        for bad in (tampered, expired, "x", "1.", f"{10 ** 20}.ab", "1999999999.é"):
            self.assertEqual(self.request("GET", "/api/state", cookie=bad)[0], 401, bad)

        # signed in two days ago: a visit renews it
        two_days = int(time.time()) + mavweb.SIGNED_IN_S - 2 * 86400
        status, r, _ = self.request("GET", "/api/state", cookie=f"{two_days}.{access._tag(two_days).decode()}")
        self.assertEqual(status, 200)
        self.assertTrue(r.getheader("Set-Cookie", "").startswith("mavlte="))

        status, r, _ = self.request("POST", "/api/signout", {})
        self.assertEqual(status, 204)
        self.assertIn("Max-Age=0", r.getheader("Set-Cookie"))

    def test_wrong_passwords_wait(self):
        with self.assertLogs("mavrelay.web", "WARNING") as logs:
            for _ in range(mavweb.Access.TRIES):
                self.assertEqual(self.sign_in("guess", who="203.0.113.9")[0], 401)
        self.assertIn("wrong password for the page, from 203.0.113.9", logs.output[0])  # the phone's, from the proxy
        status, why = self.sign_in(who="203.0.113.9")  # even the right one, now
        self.assertEqual((status, why), (429, "Too many wrong passwords: try again in 10 min"))
        self.assertEqual(self.sign_in(who="198.51.100.4")[0], 204)  # another phone is not kept waiting
        with mock.patch.object(mavweb.Access, "TRIES_ALL", mavweb.Access.TRIES + 2), self.assertLogs("mavrelay.web"):
            for who in ("192.0.2.1", "192.0.2.2"):
                self.assertEqual(self.sign_in("guess", who=who)[0], 401)
            self.assertEqual(self.sign_in(who="198.51.100.4")[0], 429)  # many wrong ones from all over

    def test_posts_come_from_the_page_only(self):
        cookie = self.signed_in()
        host = f"127.0.0.1:{self.port}"
        for headers in ({"X-MavLTE": "0"}, {"X-MavLTE": "1", "Origin": "https://example.com"},
                        {"X-MavLTE": "1", "Origin": "null"}):
            self.assertEqual(self.request("POST", "/api/voice", {"on": True}, cookie, headers)[0], 403, headers)
        self.assertFalse(self.relay.voice.on)
        for body in (b"{", b"[]", {"on": "yes"}, {"on": 1}):
            self.assertEqual(self.request("POST", "/api/voice", body, cookie)[0], 400, body)
        for size in (3, -1, True, "1", None):
            self.assertEqual(self.request("POST", "/api/snapshot", {"size": size}, cookie)[0], 400, size)
        self.assertEqual(self.request("POST", "/api/voice", b"x" * 5000, cookie)[0], 400)
        self.wait(cookie, lambda s: s["voice"]["can"], "connected to the relay")
        status, _, body = self.request("POST", "/api/voice", {"on": True}, cookie, {"Origin": f"https://{host}"})
        self.assertEqual((status, json.loads(body)), (200, {"ok": True}))

    def test_the_card_follows_the_aircraft(self):
        cookie = self.signed_in()
        s = self.wait(cookie, lambda s: s["aircraft"]["text"] == "Online", "online")
        self.assertEqual(s["aircraft"]["led"], aircraft_card.GREEN)  # it only watches, as MavLTE with both off
        self.assertEqual(s["relay"], {"led": "green", "text": "Connected to the relay"})  # no round trip: it is here
        self.assertEqual(s["loss"][:3], "up ")
        self.assertEqual((s["position"]["text"], s["position"]["map"], s["module"]["text"]), ("-", None, "-"))

        self.report()
        s = self.wait(cookie, lambda s: s["position"]["text"] != "-", "a position")
        self.assertEqual(s["position"], {"text": "41.12346, 28.98765 · 11 satellites", "color": "text",
                                         "map": "https://www.google.com/maps/search/?api=1&query=41.123457,28.987654",
                                         "copy": "41.123457, 28.987654"})
        self.assertEqual(s["module"], {"text": "flight controller talking · external power", "color": "text"})
        self.assertEqual(s["temp"], {"text": " · 45 °C", "color": "text"})
        self.report(flags=mr.POS_FC_SILENT, fc_silent=130, battery_mv=3950, battery_pct=78, temp=85)  # it came down
        s = self.wait(cookie, lambda s: s["module"]["color"] == "red", "the flight controller silent")
        self.assertEqual(s["module"]["text"], "flight controller silent for 2 min · battery 78%")
        self.assertEqual(s["temp"], {"text": " · 85 °C", "color": "red"})

        self.aircraft_gone()
        s = self.wait(cookie, lambda s: s["aircraft"]["text"].startswith("Offline, last heard "), "offline")
        self.assertEqual((s["aircraft"]["led"], s["aircraft"]["bars"], s["link"]), ("red", None, "-"))
        self.assertTrue(s["position"]["text"].startswith("last known 41.12346, 28.98765, "))
        self.assertEqual((s["position"]["color"], s["module"]["text"]), ("amber", "-"))
        self.assertEqual(s["position"]["copy"], "41.123457, 28.987654")  # still there to look for it

    def test_locator_voice(self):
        cookie = self.signed_in()
        s = self.wait(cookie, lambda s: s["voice"]["text"].startswith("Off:"), "voice off")
        self.assertEqual((s["voice"]["on"], s["voice"]["color"]), (False, "dim"))
        self.assertEqual(self.request("POST", "/api/voice", {"on": True}, cookie)[0], 200)
        s = self.wait(cookie, lambda s: s["voice"]["text"] == "On: the aircraft's speaker is sounding", "sounding")
        self.assertEqual((s["voice"]["on"], s["voice"]["color"]), (True, "green"))
        self.assertTrue(self.relay.voice.on and self.vehicle.voice_on)
        self.assertEqual(self.request("POST", "/api/voice", {"on": False}, cookie)[0], 200)
        self.wait(cookie, lambda s: s["voice"]["text"].startswith("Off:"), "off again")
        self.assertFalse(self.relay.voice.on)

    def test_snapshot(self):
        cookie = self.signed_in()
        s = self.wait(cookie, lambda s: s["camera"]["ready"], "Snapshot ready")
        self.assertEqual(s["camera"]["text"], "640×480, about 10-30 KB")  # the size the phone chose: Medium
        self.assertEqual(self.state(cookie, size=0)["camera"]["text"], "320×240, about 5-10 KB")
        self.assertIsNone(s["photo"])
        self.assertEqual(self.request("POST", "/api/snapshot", {"size": 0}, cookie)[0], 200)
        s = self.wait(cookie, lambda s: s["photo"] is not None, "the photo")
        photo = s["photo"]
        self.assertEqual(photo["caption"], "120 m · heading 45°")  # the time is the phone's to write
        self.assertGreater(photo["time"], time.time() - 60)
        status, r, body = self.request("GET", f"/photo/{photo['id']}.jpg", cookie=cookie)
        self.assertEqual((status, r.getheader("Content-Type"), body), (200, "image/jpeg", JPEG))
        self.assertEqual(self.request("GET", f"/photo/{photo['id']}.jpg")[0], 401)
        self.assertEqual(self.request("GET", f"/photo/{photo['id'] + 1}.jpg", cookie=cookie)[0], 404)
        status, _, body = self.request("GET", "/api/photos", cookie=cookie)
        [listed] = json.loads(body)
        self.assertEqual(listed["id"], photo["id"])
        self.assertTrue(listed["caption"].startswith("41.123457, 28.987654 · 120 m above home · heading 45° · 320×240"))

        self.wait(cookie, lambda s: s["camera"]["ready"], "ready again")
        self.outbox.capture = None  # its CAM switch off
        self.assertEqual(self.request("POST", "/api/snapshot", {"size": 1}, cookie)[0], 200)
        s = self.wait(cookie, lambda s: s["camera"]["text"].startswith("No photo"), "the aircraft's answer")
        self.assertEqual((s["camera"]["text"], s["camera"]["color"]),
                         ("No photo: " + mr.SNAP_PROBLEMS[mr.SNAP_NO_CAMERA], "amber"))

        self.aircraft_gone()
        self.wait(cookie, lambda s: s["aircraft"]["text"].startswith("Offline"), "offline")
        status, _, body = self.request("POST", "/api/snapshot", {"size": 1}, cookie)
        self.assertEqual((status, json.loads(body)["error"]), (409, "No photo: " + mr.SNAP_PROBLEMS[mr.SNAP_NO_CAMERA]))

    def test_a_new_password_signs_every_phone_out(self):
        cookie = self.signed_in()
        mavweb.set_password(self.config, "barley-field-9", rounds=ROUNDS)  # mavweb.py password, while it runs
        self.page.access.checked = -1e9  # it looks every 2 s
        with self.assertLogs("mavrelay.web", "INFO") as logs:
            self.assertEqual(self.request("GET", "/api/state", cookie=cookie)[0], 401)
            self.assertEqual(self.sign_in()[0], 401)
        self.assertIn("the page has a new password", logs.output[0])
        self.assertEqual(self.sign_in("barley-field-9")[0], 204)
        with open(self.config, "a", encoding="utf-8") as f:  # a broken file: the page keeps the password it has
            f.write("this line is broken\n")
        self.page.access.checked = -1e9
        with self.assertLogs("mavrelay.web", "WARNING") as logs:
            self.assertEqual(self.sign_in("barley-field-9")[0], 204)
        self.assertIn("the page keeps its password", logs.output[0])


class OptionsTest(unittest.TestCase):
    """mavweb's settings and its password, without the network."""

    def setUp(self):
        self.home = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.home, True)
        self.config = os.path.join(self.home, "mavrelay.ini")

    def write(self, text):
        with open(self.config, "w", encoding="utf-8") as f:
            f.write(text)

    def test_options(self):
        self.write(f"[server]\nlisten = [::]:14651\ngcs_key = {KEY_G.hex()}\n")
        with mock.patch.dict(os.environ, {"STATE_DIRECTORY": "/var/lib/mavrelay"}):
            opts = mavweb.load_options(self.config)
        self.assertEqual((opts.relay, opts.key, opts.listen), (("::1", 14651), KEY_G, ("127.0.0.1", 8090)))
        self.assertEqual((opts.name, opts.photo_dir), ("My UAV", os.path.join("/var/lib/mavrelay", "web-photos")))
        self.write(f"[web]\nrelay = relay.example.com:14650\nkey = {KEY_G.hex()}\nlisten = 127.0.0.1:8091\nname = UAV#2\n")
        opts = mavweb.load_options(self.config)
        self.assertEqual((opts.relay, opts.listen, opts.name), (("relay.example.com", 14650), ("127.0.0.1", 8091),
                                                                "UAV#2"))
        self.write("[server]\nlisten = 0.0.0.0:14650\n")
        with self.assertRaisesRegex(SystemExit, "no GCS key"):
            mavweb.load_options(self.config)
        self.write("[server]\ngcs_key = 12ab\n")
        with self.assertRaisesRegex(SystemExit, "too short"):
            mavweb.load_options(self.config)

    def test_password(self):
        stored = mavweb.hash_password("mavlte", rounds=ROUNDS)
        self.assertRegex(stored, r"^pbkdf2_sha256\$1000\$[0-9a-f]{32}\$[0-9a-f]{64}$")
        self.assertTrue(mavweb.check_password("mavlte", stored))
        for wrong, against in (("mavlt", stored), ("mavlte", "plain"), ("mavlte", "pbkdf2_sha256$x$y$z"),
                               ("mavlte", stored.replace("pbkdf2_sha256", "md5"))):
            self.assertFalse(mavweb.check_password(wrong, against))
        made = {mavweb.make_password() for _ in range(50)}
        self.assertEqual(len(made), 50)
        for password in made:
            self.assertRegex(password, r"^[a-hjkmnp-z2-9]{4}-[a-hjkmnp-z2-9]{4}-[a-hjkmnp-z2-9]{4}$")

    def test_password_command(self):
        self.write(f"# the relay\n[server]\ngcs_key = {KEY_G.hex()}\n")
        out = io.StringIO()
        with mock.patch.object(mavweb, "ROUNDS", ROUNDS), contextlib.redirect_stdout(out):
            mavweb.main(["password", "--random", "--config", self.config])
        password = re.search(r"password: (\S+)", out.getvalue()).group(1)
        web = mr.load_config(self.config, "web")
        self.assertTrue(mavweb.check_password(password, web["password"]))
        self.assertEqual(len(web["secret"]), 64)
        self.assertEqual(mr.load_config(self.config, "server")["gcs_key"], KEY_G.hex())  # the rest as it was
        with open(self.config, encoding="utf-8") as f:
            self.assertTrue(f.read().startswith("# the relay\n[server]\n"))
        with self.assertRaisesRegex(SystemExit, "no config file"):
            mavweb.main(["password", "--random", "--config", os.path.join(self.home, "none.ini")])

    def test_no_password_no_page(self):
        self.write(f"[server]\ngcs_key = {KEY_G.hex()}\n[web]\nname = Test UAV\n")
        with self.assertRaisesRegex(SystemExit, "no password for the web page .* password --config"):
            mavweb.Access(self.config)

    def test_old_photos_go(self):
        folder = os.path.join(self.home, "web-photos")
        paths = [fake_photo(folder, photo_id) for photo_id in (1790000003, 1790000001, 1790000002)]
        self.write(f"[server]\ngcs_key = {KEY_G.hex()}\n[web]\nphoto_dir = {folder}\n")
        mavweb.set_password(self.config, PASSWORD, rounds=ROUNDS)
        with mock.patch.object(mavweb, "KEEP_PHOTOS", 2):
            page = mavweb.WebPage(mavweb.load_options(self.config))
        self.addCleanup(page.loop.close)
        self.assertEqual(sorted(page.by_id), [1790000002, 1790000003])
        self.assertFalse(os.path.exists(paths[1]) or os.path.exists(paths[1][:-4] + ".json"))
        self.assertEqual(page.newest["id"], 1790000003)
        self.assertEqual([p["id"] for p in page.photos()], [1790000003, 1790000002])  # newest first


if __name__ == "__main__":
    unittest.main()

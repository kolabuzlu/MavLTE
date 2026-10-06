"""The Aircraft card's words (aircraft_card.py), which MavLTE's window and its web page both show: worked out
from an agent's state, without a window or a network. Run from the relay directory:  python -m unittest -v"""

import os
import sys
import time
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import aircraft_card as card  # noqa: E402
import mavrelay as mr  # noqa: E402

ONLINE = mr.STATUS_VEHICLE_ONLINE


def status(flags=ONLINE, rat=7, rtt=85, up=3, down=0, rssi=-71, idle=200, sinr=None, net=None):
    """A STATUS as relays send it: with the LTE signal's quality since 1.8.7 (sinr), and the network since 1.8.8
    (net, the byte), without them before."""
    body = mr.STATUS_BODY.pack(flags, rat, rtt, up, down, rssi, idle)
    if sinr is not None or net is not None:
        body += mr.QUALITY.pack(mr.SINR_UNKNOWN if sinr is None else sinr)
    return mr.LinkStatus.unpack(body if net is None else body + mr.NET_REPORT.pack(net))


def net(chosen, vehicle=None, fallback=False):
    """STATUS's network byte: the network chosen at the relay, and the one the vehicle reports (None: none)."""
    if vehicle is None:
        return chosen
    return (chosen | mr.STATUS_NET_REPORTED | vehicle << mr.STATUS_NET_SHIFT
            | (mr.STATUS_NET_FALLBACK if fallback else 0))


def agent(session=1, watching=True, silence=None, position=None, last_fix=None, voice_request=None, photos=None,
          network_request=None):
    return SimpleNamespace(client=SimpleNamespace(session=session, hellos=0, rtt_ms=42), watching=watching,
                           vehicle_silence=lambda: silence, position=position, last_fix=last_fix,
                           voice_request=voice_request, photos=photos, network_request=network_request)


def report(**fields):
    fields = {**dict(gnss_time=1, lat=411234567, lon=289876543, sats=11, fix=mr.FIX_3D, fc_silent=0,
                     time=int(time.time())), **fields}
    return mr.Position(**fields)


class CardTest(unittest.TestCase):
    def test_headline(self):
        self.assertEqual(card.headline(None, None), (card.OFF, "Not connected to the relay"))
        self.assertEqual(card.headline(agent(session=0), None), (card.OFF, "Not connected to the relay"))
        self.assertEqual(card.headline(agent(), None), (card.OFF, "Waiting for news from the relay…"))
        self.assertEqual(card.headline(agent(), status()), (card.GREEN, "Available: switch TCP or UDP on"))
        self.assertEqual(card.headline(agent(), status(), switches=False), (card.GREEN, "Online"))  # the web page
        self.assertEqual(card.headline(agent(watching=False), status()), (card.BLUE, "Online"))
        self.assertEqual(card.headline(agent(), status(0, idle=mr.U16_UNKNOWN)),
                         (card.OFF, "Not connected to the relay (yet)"))
        self.assertEqual(card.headline(agent(silence=72.4), status(0, idle=mr.IDLE_CAPPED)),
                         (card.RED, "Offline, last heard 72 s ago"))
        self.assertEqual(card.headline(agent(), status(0, idle=mr.IDLE_CAPPED)),
                         (card.RED, "Offline, last heard over a minute ago"))

    def test_relay(self):
        self.assertEqual(card.relay(agent().client), (card.GREEN, "Connected to the relay · 42 ms"))
        self.assertEqual(card.relay(agent().client, rtt=False), (card.GREEN, "Connected to the relay"))
        client = SimpleNamespace(session=0, hellos=5, rtt_ms=mr.U16_UNKNOWN)
        self.assertEqual(card.relay(client), (card.RED, "No answer from the relay: check server, UDP port and key"))
        client.hellos = 1
        self.assertEqual(card.relay(client), (card.AMBER, "Connecting to the relay…"))

    def test_link(self):
        self.assertEqual((card.link(status()), card.loss(status())), ("LTE -71 dBm · round trip 85 ms",
                                                                      "up 0.3%, down 0.0%"))
        self.assertEqual(card.link(status(rat=mr.RAT_UNKNOWN, rssi=mr.RSSI_UNKNOWN, rtt=mr.U16_UNKNOWN)), "-")
        self.assertEqual(card.link(status(rssi=mr.RSSI_UNKNOWN)), "LTE · round trip 85 ms")
        for offline in (None, status(0)):
            self.assertEqual((card.link(offline), card.loss(offline), card.bars(offline)), ("-", "-", None))
        self.assertEqual([card.bars(status(rssi=r)) for r in (-110, -100, -90, -80, -70)], [0, 1, 2, 3, 4])
        self.assertEqual([card.bars(status(rssi=mr.RSSI_UNKNOWN, up=up)) for up in (5, 30, 100, 400)], [4, 3, 2, 1])

    def test_link_quality(self):
        """1.8.7: on LTE the bars and the Link line follow the signal's quality (SINR) too: in the air the signal
        stays strong while the quality falls, and the link with it (first flight: -51 dBm, four bars, link down)."""
        flight = status(rssi=-53, sinr=-14)
        self.assertEqual(card.link(flight), "LTE -53 dBm · quality -14 dB · round trip 85 ms")
        self.assertEqual((card.bars(flight), card.link_color(flight)), (0, card.RED))
        self.assertEqual([card.bars(status(rssi=-53, sinr=q)) for q in (-14, -13, -12, -9, -8, -1, 0, 9, 10, 25)],
                         [0, 0, 1, 1, 2, 2, 3, 3, 4, 4])
        self.assertEqual([card.link_color(status(rssi=-53, sinr=q)) for q in (-13, -12, -9, -8, 10)],
                         [card.RED, card.AMBER, card.AMBER, card.TEXT, card.TEXT])
        self.assertEqual(card.bars(status(rssi=-100, sinr=20)), 1)  # never more than the signal's strength allows
        self.assertEqual(card.bars(status(rssi=mr.RSSI_UNKNOWN, sinr=-5)), 2)
        # a relay before 1.8.7 (no quality in STATUS), and 2G (no quality): as before
        self.assertEqual(status().sinr_db, mr.SINR_UNKNOWN)
        self.assertEqual((card.bars(status(rssi=-53)), card.link_color(status(rssi=-53))), (4, card.TEXT))
        edge = status(rat=3, rssi=-60, sinr=mr.SINR_UNKNOWN)
        self.assertEqual((card.link(edge), card.bars(edge), card.link_color(edge)),
                         ("EDGE -60 dBm · round trip 85 ms", 4, card.TEXT))
        for offline in (None, status(0, sinr=-14)):
            self.assertEqual((card.link_color(offline), card.bars(offline)), (card.TEXT, None))

    def test_network(self):
        """1.8.8: the network chosen for the aircraft (automatic, 2G, LTE): the selector, in amber while the aircraft
        has not taken the choice; and on the link line, the aircraft on 2G because LTE failed."""
        self.assertEqual(card.NETWORK_NAMES, ("Auto", "2G", "LTE"))
        self.assertEqual([mr.NET_AUTO, mr.NET_2G, mr.NET_LTE], [0, 1, 2])  # their segments
        shown = card.NetworkShown
        # greyed out without the relay; no news: as it was
        self.assertEqual(card.network(None, None), shown(None, False, False))
        self.assertEqual(card.network(agent(session=0), status(net=net(mr.NET_2G))), shown(None, False, False))
        self.assertIsNone(card.network(agent(), None))
        # the relay's choice, in green once the aircraft runs it
        self.assertEqual(card.network(agent(), status(net=net(mr.NET_2G, mr.NET_2G))), shown(mr.NET_2G, True, False))
        self.assertEqual(card.network(agent(), status(net=net(mr.NET_AUTO, mr.NET_AUTO, fallback=True))),
                         shown(mr.NET_AUTO, True, False))
        # in amber while it does not: asked for (not yet at the relay), switching, the aircraft away, or its firmware
        # before 1.8.8 (which does not say, and stays on automatic)
        self.assertEqual(card.network(agent(network_request=mr.NET_LTE), status(net=net(mr.NET_2G, mr.NET_2G))),
                         shown(mr.NET_LTE, True, True))
        self.assertEqual(card.network(agent(), status(net=net(mr.NET_2G, mr.NET_AUTO))), shown(mr.NET_2G, True, True))
        self.assertEqual(card.network(agent(), status(0, idle=mr.U16_UNKNOWN, net=net(mr.NET_LTE))),
                         shown(mr.NET_LTE, True, True))
        self.assertEqual(card.network(agent(), status(net=net(mr.NET_LTE))), shown(mr.NET_LTE, True, True))
        self.assertEqual(card.network(agent(), status(net=net(mr.NET_AUTO))), shown(mr.NET_AUTO, True, False))
        self.assertEqual(card.network(agent(), status()), shown(mr.NET_AUTO, True, False))  # a relay before 1.8.8

        # the link line: as before, but on 2G because LTE failed: short (MavLTE's window has no width to spare), amber
        lte = status(rssi=-53, sinr=12, net=net(mr.NET_AUTO, mr.NET_AUTO))
        self.assertEqual((card.link(lte), card.link_color(lte)), ("LTE -53 dBm · quality 12 dB · round trip 85 ms",
                                                                  card.TEXT))
        two = status(rat=3, rssi=-60, rtt=290, net=net(mr.NET_2G, mr.NET_2G))
        self.assertEqual((card.link(two), card.link_color(two)), ("EDGE -60 dBm · round trip 290 ms", card.TEXT))
        fell = status(rat=3, rssi=-60, rtt=290, net=net(mr.NET_AUTO, mr.NET_AUTO, fallback=True))
        self.assertTrue(fell.fallback)
        self.assertEqual((card.link(fell), card.link_color(fell), card.network_note(fell)),
                         ("EDGE -60 dBm · round trip 290 ms · LTE failed", card.AMBER, "LTE failed"))
        for other in (status(rssi=-53, net=net(mr.NET_2G, mr.NET_AUTO)), status(rssi=-53, net=net(mr.NET_LTE))):
            self.assertEqual((card.link(other), card.link_color(other)), ("LTE -53 dBm · round trip 85 ms", card.TEXT))
        # ... offline: nothing (the relay keeps the choice for when it is back)
        away = status(0, net=net(mr.NET_2G, mr.NET_AUTO, fallback=True))
        self.assertEqual((card.link(away), card.link_color(away), card.network_note(away)), ("-", card.TEXT, ""))

    def test_position_and_module(self):
        now = time.time()
        live = report(battery_mv=4298, battery_pct=100, temp=45)
        a = agent(position=live, last_fix=live)
        where = card.position(a, True, now)
        self.assertEqual((where.text, where.color, where.fix), ("41.12346, 28.98765 · 11 satellites", card.TEXT, live))
        self.assertEqual((card.coordinates(live), card.map_link(live)),
                         ("41.123457, 28.987654", "https://www.google.com/maps/search/?api=1&query=41.123457,28.987654"))
        self.assertEqual(card.module(a, True, now), (card.Shown("flight controller talking · external power"),
                                                     card.Shown(" · 45 °C")))
        where = card.position(a, False, now + 7 * 3600 + 40 * 60)  # offline: where it was last
        self.assertEqual((where.text, where.color), ("last known 41.12346, 28.98765, 7 h 40 min ago", card.AMBER))
        self.assertEqual(card.module(a, False, now), (card.Shown("-"), card.Shown("")))

        searching = report(fix=mr.FIX_NONE, lat=mr.UNKNOWN_I32, lon=mr.UNKNOWN_I32, sats=3, fc_silent=mr.U16_UNKNOWN,
                           battery_mv=3950, battery_pct=78, temp=85)
        a = agent(position=searching)
        self.assertEqual(card.position(a, True, now), ("GNSS searching (3 satellites)", card.MUTED, None))
        self.assertEqual(card.module(a, True, now), (card.Shown("flight controller not heard yet · battery 78%, 3.95 V",
                                                                card.AMBER), card.Shown(" · 85 °C", card.RED)))
        self.assertEqual(card.position(agent(position=report(flags=mr.POS_NO_GNSS, fix=mr.FIX_NONE)), True, now).text,
                         "the LTE module cannot read its GNSS")
        silent = report(flags=mr.POS_FC_SILENT, fc_silent=130)
        self.assertEqual(card.module(agent(position=silent), True, now)[0],
                         card.Shown("flight controller silent for 2 min", card.RED))
        self.assertEqual(card.position(agent(position=report(time=int(now) - 20)), True, now).text, "-")  # not live

    def test_voice(self):
        voice = card.Voice()
        self.assertEqual(voice.show(agent(session=0), None, 0), (False, card.Shown("Not connected to the relay", card.DIM)))
        self.assertIsNone(voice.show(agent(), None, 0))  # no news: as it was
        self.assertEqual(voice.show(agent(voice_request=True), None, 0), (True, card.Shown("Switching on…")))
        on = ONLINE | mr.STATUS_VOICE_ON
        self.assertEqual(voice.show(agent(), status(), 0), (False, card.Shown("Off: switch on to hear the aircraft",
                                                                             card.DIM)))
        self.assertEqual(voice.show(agent(), status(on), 1.0), (True, card.Shown("On: waiting for the aircraft…")))
        self.assertEqual(voice.show(agent(), status(on), 1.0 + card.Voice.ANSWER_S)[1],
                         card.Shown("On, but no answer: firmware before 1.5.0?", card.AMBER))
        self.assertEqual(voice.show(agent(), status(on | mr.STATUS_SPEAKING), 30)[1],
                         card.Shown("On: the aircraft's speaker is sounding", card.GREEN))
        self.assertEqual(voice.show(agent(), status(on | mr.STATUS_VOICE_FAILED), 31)[1],
                         card.Shown("On, but the aircraft cannot play it", card.RED))
        self.assertEqual(voice.show(agent(), status(mr.STATUS_VOICE_ON | mr.STATUS_SPEAKING, idle=9000), 32)[1],
                         card.Shown("On: it was sounding when last heard", card.AMBER))
        self.assertEqual(voice.show(agent(), status(mr.STATUS_VOICE_ON, idle=9000), 33)[1],
                         card.Shown("On: sounds once the aircraft is back", card.AMBER))

    def test_camera(self):
        photos = SimpleNamespace(arriving=None, asked_at=None, problem="", problem_at=0.0)
        a = agent(photos=photos)
        self.assertEqual(card.camera(a, True, 2, 100.0), card.Camera(card.SIZE_HINTS[2], card.DIM, None, True, False))
        self.assertEqual(card.camera(a, False, 1, 100.0), ("The aircraft is offline", card.DIM, None, False, False))
        self.assertEqual(card.camera(agent(session=0, photos=photos), True, 1, 100.0).text, "Not connected to the relay")
        photos.problem, photos.problem_at = "no photo came", 50.0
        self.assertEqual(card.camera(a, True, 1, 100.0), ("No photo: no photo came", card.AMBER, None, True, False))
        self.assertEqual(card.camera(a, True, 1, 111.0).text, card.SIZE_HINTS[1])  # a minute later: forgotten
        photos.asked_at = 99.0
        self.assertEqual(card.camera(a, True, 1, 100.0), ("Asking the aircraft…", card.TEXT, None, False, True))
        photos.arriving = (mr.PhotoInfo(1, 40 * 1024, 1024, 768), 10 * 1024)
        self.assertEqual(card.camera(a, True, 1, 100.0),
                         ("Arriving: 10 of 40 KB (1024×768)", card.TEXT, 0.25, False, True))

    def test_track(self):
        track = card.Track()
        self.assertIsNone(track.newest)
        first = report(time=1790000100)
        self.assertTrue(track.add(first))
        self.assertFalse(track.add(first))  # the same fix again: the app looks four times a second
        self.assertFalse(track.add(report(time=1790000099)))  # older: the relay's last known, after a newer one
        self.assertFalse(track.add(report(time=1790000200, fix=mr.FIX_NONE, lat=mr.UNKNOWN_I32, lon=mr.UNKNOWN_I32)))
        self.assertFalse(track.add(None))
        self.assertTrue(track.add(report(time=1790000105, lat=411244567)))
        self.assertEqual(([p.time for p in track.fixes], track.newest.lat), ([1790000100, 1790000105], 411244567))
        with mock.patch.object(card.Track, "MOST", 3):
            short = card.Track()
        for t in range(5):
            short.add(report(time=1790000000 + t))
        self.assertEqual([p.time - 1790000000 for p in short.fixes], [2, 3, 4])  # the oldest go

    def test_heading(self):
        self.assertIsNone(card.heading(report()))  # no speed, no course
        self.assertIsNone(card.heading(report(speed=150, course=9000)))  # too slow to tell
        self.assertIsNone(card.heading(report(speed=1500)))
        self.assertEqual(card.heading(report(speed=1500, course=27050)), 270.5)

    def test_photo_caption(self):
        meta = dict(mr.PhotoInfo(1790000000, 40 * 1024, 1024, 768, 411234567, 289876543, 120_000, 4500,
                                 time=1790000003)._asdict(), latitude=41.1234567, longitude=28.9876543, altitude_m=120.0)
        self.assertEqual(card.photo_time(meta), 1790000003)
        self.assertEqual(card.photo_caption(meta, clock=False),
                         "41.123457, 28.987654 · 120 m above home · heading 45° · 1024×768, 40 KB")
        self.assertEqual(card.photo_caption(meta, short=True, clock=False), "120 m · heading 45°")
        self.assertTrue(card.photo_caption(meta, short=True).endswith(" · 120 m · heading 45°"))
        self.assertEqual(card.photo_time({}), 0)
        # the exposure it was taken with (1.8.9), when not as the camera would
        meta["exposure"] = 2
        self.assertTrue(card.photo_caption(meta, clock=False).endswith(" · 1024×768, 40 KB · EV +2"))
        self.assertEqual(card.photo_caption(meta, short=True, clock=False), "120 m · heading 45° · EV +2")
        meta["exposure"] = -1
        self.assertTrue(card.photo_caption(meta, short=True, clock=False).endswith(" · EV −1"))
        for nothing in (0, None, True, "2"):
            meta["exposure"] = nothing
            self.assertNotIn("EV", card.photo_caption(meta, clock=False))

    def test_exposure(self):
        self.assertEqual(card.EXPOSURE_MOST, 3)
        self.assertEqual([card.exposure_text(s) for s in (-3, -1, 0, 2)], ["EV −3", "EV −1", "EV 0", "EV +2"])
        self.assertEqual(card.exposure_hint(0), "EV 0: as the camera sets it")
        self.assertEqual(card.exposure_hint(2), "EV +2: brighter than the camera would take it")
        self.assertEqual(card.exposure_hint(-1), "EV −1: darker than the camera would take it")


if __name__ == "__main__":
    unittest.main()

"""The Aircraft card's words (aircraft_card.py), which MavLTE's window and its web page both show: worked out
from an agent's state, without a window or a network. Run from the relay directory:  python -m unittest -v"""

import os
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import aircraft_card as card  # noqa: E402
import mavrelay as mr  # noqa: E402

ONLINE = mr.STATUS_VEHICLE_ONLINE


def status(flags=ONLINE, rat=7, rtt=85, up=3, down=0, rssi=-71, idle=200):
    return mr.LinkStatus.unpack(mr.STATUS_BODY.pack(flags, rat, rtt, up, down, rssi, idle))


def agent(session=1, watching=True, silence=None, position=None, last_fix=None, voice_request=None, photos=None):
    return SimpleNamespace(client=SimpleNamespace(session=session, hellos=0, rtt_ms=42), watching=watching,
                           vehicle_silence=lambda: silence, position=position, last_fix=last_fix,
                           voice_request=voice_request, photos=photos)


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
        self.assertEqual(card.module(a, True, now), (card.Shown("flight controller not heard yet · battery 78%",
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

    def test_photo_caption(self):
        meta = dict(mr.PhotoInfo(1790000000, 40 * 1024, 1024, 768, 411234567, 289876543, 120_000, 4500,
                                 time=1790000003)._asdict(), latitude=41.1234567, longitude=28.9876543, altitude_m=120.0)
        self.assertEqual(card.photo_time(meta), 1790000003)
        self.assertEqual(card.photo_caption(meta, clock=False),
                         "41.123457, 28.987654 · 120 m above home · heading 45° · 1024×768, 40 KB")
        self.assertEqual(card.photo_caption(meta, short=True, clock=False), "120 m · heading 45°")
        self.assertTrue(card.photo_caption(meta, short=True).endswith(" · 120 m · heading 45°"))
        self.assertEqual(card.photo_time({}), 0)


if __name__ == "__main__":
    unittest.main()

# MavLTE tunnel protocol (version 1)

The vehicle (ESP32 + 4G modem) and the GCS agent (the MavLTE app, or `mavrelay.py gcs`) are *clients*.
The relay (`mavrelay.py server`) is the *server*. Clients always send first, so both work
behind carrier-grade NAT; the server replies to the address each packet came from.

Everything travels over UDP. MAVLink bytes are carried unmodified, so MAVLink 1, MAVLink 2
and MAVLink 2 signing pass through end to end.

## Datagram

```
offset  size  field
0       1     magic    0xA5
1       1     version  1
2       1     type     see below
3       1     role     role of the sender: 0 = server, 1 = vehicle, 2 = gcs
4       4     session  u32 little endian (0 in HELLO)
8       4     seq      u32 little endian
12      n     body
12+n    16    tag      first 16 bytes of HMAC-SHA256(key, bytes 0 .. 12+n-1)
```

The key is the one belonging to the *client role* on that leg: the vehicle key between
vehicle and server, the GCS key between a GCS agent and the server. Keys are 32 random
bytes, written as 64 hex characters.

A datagram is at most 1200 bytes, so the IP packet stays under 1280 bytes and is never
fragmented. That leaves at most 1172 bytes of MAVLink per DATA packet. Senders only cut
the MAVLink stream at frame boundaries, so losing a datagram loses whole frames and never
corrupts the frames around it.

The server sends MAVLink for the vehicle at once after a quiet moment, and packs whatever
follows within 5 ms into one DATA (since 1.8.4). GCS software sends bursts of dozens of small
messages at times (Mission Planner: 57 within 3 ms when a screen reads its parameters), while
the vehicle's modem holds only about ten packets as its serial line passes them on (the
A7670E at 921600 baud: one small packet per 1.2-1.5 ms). Sent one packet each, most of such a
burst was lost.

## Packet types

| type | name    | direction | session / seq | body |
|------|---------|-----------|---------------|------|
| 1    | HELLO   | C → S | 0 / 0 | nonce (8 bytes), then optional UTF-8 info text (≤ 64 bytes) |
| 2    | WELCOME | S → C | new session / 0 | the 8-byte nonce from the HELLO, then (since 1.8.0) the server's clock, `unix_ms u64` |
| 3    | DATA    | C ↔ S | session / seq | MAVLink bytes (whole frames), ≤ 1172 bytes |
| 4    | PING    | C → S | session / seq | `t_ms u32, rtt_ms u16, rx_loss_permille u16, rssi_dbm i16, rat u8, flags u8` (GCS: bit 0 watching; vehicle: bit 1 sounding, bit 2 cannot sound) |
| 5    | PONG    | S → C | session / seq | `t_ms u32` (echo), `flags u8` (bit 0: a GCS is connected; to the vehicle, bit 1: sound the speaker) |
| 6    | REJECT  | S → C | the rejected session / 0 | `reason u8` (1 = unknown or expired session) |
| 7    | STATUS  | S → GCS | session / seq | `flags u8, rat u8, rtt_ms u16, up_loss_permille u16, down_loss_permille u16, rssi_dbm i16, idle_ms u16` (flags: bit 0 vehicle online, bits 1-3 the [locator voice](#locator-voice)) |
| 8    | SNAP_REQ  | GCS → S → V | session / seq | `photo_id u32, size u8` (see [Snapshots](#snapshots)) |
| 9    | SNAP_INFO | V → S → GCS | session / seq | `photo_id u32, bytes u32, width u16, height u16, lat i32, lon i32, alt_mm i32, heading_cdeg u16, status u8, time u32` |
| 10   | SNAP_DATA | V → S → GCS | session / seq | `photo_id u32, chunk u16`, then the chunk (1024 bytes, the last one shorter) |
| 11   | SNAP_ACK  | GCS → S → V | session / seq | `photo_id u32, flags u8` (bit 0: all there, bit 1: have the SNAP_INFO), then a bitmap of the chunks received |
| 12   | SNAP_SYNC | GCS → S | session / seq | `newest_photo_id u32` |
| 13   | POSITION  | V → S → GCS | session / seq | `gnss_time u32, lat i32, lon i32, alt_mm i32, speed_cms u16, course_cdeg u16, hdop u16, sats u8, fix u8, flags u8, fc_silent_s u16, battery_mv u16, battery_pct u8, time u32, chip_c i8` (see [Locator](#locator)) |
| 14   | VOICE     | GCS → S | session / seq | `flags u8` (bit 0: on) (see [Locator voice](#locator-voice)) |
| 15   | FILE_REQ  | GCS → S → V | session / seq | `req_id u16, op u8, offset u32, name` (12 bytes) (see [Logs](#logs)) |
| 16   | FILE_LIST | V → S → GCS | session / seq | `req_id u16, status u8, files u16, first u16, count u8`, then `count` × (`name` (12 bytes), `bytes u32, start u32, end u32`) |
| 17   | FILE_DATA | V → S → GCS | session / seq | `req_id u16, status u8, offset u32, size u32`, then up to 1024 bytes of the file |
| 18   | FILE_ACK  | GCS → S → V | session / seq | `req_id u16, next u32` |

Unknown values are `0xFFFF` for u16 fields, `0x7FFF` for `rssi_dbm` and `0xFF` for `rat`.
`rat` uses the 3GPP TS 27.007 access technology numbers (0 GSM, 3 EDGE, 7 LTE, ...).
Receivers ignore extra trailing bytes in a body and treat missing trailing fields as unknown,
so fields can be appended in later versions.

STATUS describes the vehicle's link: flag bit 0 means the vehicle was heard within the last
3 s, `rtt_ms`, `down_loss_permille`, `rssi_dbm` and `rat` are what the vehicle reported in its
last PING, `up_loss_permille` is measured by the server, and `idle_ms` is the time since the
server last heard from the vehicle. It tops out at 65534 (65.5 s or more): a GCS agent that saw it
count up to there goes on counting by itself, until the server drops the session (120 s by default).

## Session setup

1. The client sends HELLO with a random nonce once a second until it gets a WELCOME. It
   keeps the same nonce for all retries of one attempt.
2. The server checks the tag with the key for the claimed role, creates a *pending* session
   with a random non-zero id and answers WELCOME, echoing the nonce.
3. The client accepts a WELCOME only if it carries the nonce of its current attempt. It then
   starts its sequence numbers at 1 and sends a PING straight away.
4. The first valid DATA or PING makes the session *active*. A new active vehicle session
   replaces the previous one; up to 8 GCS sessions can be active at once.

Since 1.8.0 the WELCOME also carries the server's clock (unix milliseconds) after the nonce. A
vehicle without a GNSS time sets its own clock from it, for the times in its log. Clients that do
not need it ignore it, as older ones do.

Clients PING once a second. A client that hears nothing valid from the server for 10 s drops
its session and starts over with HELLO. Servers drop active sessions after 120 s of silence
and pending ones after 15 s. A DATA or PING for a session the server does not know (for
example after a server restart) is answered with REJECT, at most once a second per session,
and the client then starts over with HELLO right away.

The server always sends to the address it last received a valid, new packet from on that
session, so a client whose IP address or NAT mapping changes keeps its session.

## Replay protection

Every receiver keeps a 64-packet sliding window (as in IPsec, RFC 4303) per session and
direction. Packets with a sequence number of 0, one already seen, or one more than 64 below
the highest seen are dropped. The window is only updated after the tag has been verified.
Sessions are chosen by the server at random, so old packets from earlier sessions are
rejected as unknown sessions.

HELLO carries no sequence number, so a captured HELLO can be replayed. Clients use a new
random nonce for every attempt, and the server gives one nonce at most one session: a HELLO
whose nonce belongs to a pending session gets that session's WELCOME again (a client
retrying), and one whose nonce belongs to an active session is ignored (a replay). When the
capped pool of pending sessions is full, the server first drops the oldest pending session
from the new HELLO's own IP address, so replayed HELLOs from one address cannot push out other
clients' sessions before they can answer. WELCOME is bound to the client's current nonce and
REJECT to the client's current session.

## Link state hints

- PONG flag bit 0 tells the vehicle whether any GCS is connected. The ESP32 firmware stops
  sending telemetry while no GCS is connected (configurable) and resumes within about a
  second when one connects. PINGs continue either way, which keeps the session and the
  carrier NAT mapping alive.
- The vehicle reports its measured round-trip time, downlink loss and modem signal in PING.
  The server passes these on to GCS agents in STATUS once a second.
- A GCS agent with no GCS software attached sets PING flag bit 0 (*watching*). The server
  still sends it STATUS, so it can show whether the vehicle is available, but sends it no DATA
  and does not count it as a connected GCS for the PONG flag, so the vehicle keeps its
  telemetry back. The flag holds until the agent's next PING; agents PING at once when it
  changes. Vehicles send 0; servers that predate the flag treat a watcher as a GCS.

## Snapshots

A photo from the aircraft's camera, on request only: the aircraft never takes one by itself.
Each leg (aircraft to relay, relay to GCS agent) carries it the same way, and each receiver
acknowledges what it has.

1. A GCS agent sends SNAP_REQ with `photo_id` 0 and a size: 0 = 320×240, 1 = 640×480,
   2 = 1024×768 (larger values mean the largest). The server picks the photo id, its own unix
   time (and one more than the last if that is not larger), so ids grow across server restarts.
   Without an online vehicle it answers at once with SNAP_INFO status NO_AIRCRAFT; otherwise it
   passes SNAP_REQ with the id on to the vehicle, and again every 2 s until the vehicle answers.
   After 20 s without an answer it tells the agent NO_ANSWER.
2. The vehicle answers each SNAP_REQ with a SNAP_INFO: a problem (status NO_CAMERA, FAILED, or
   BUSY while another photo is on its way), or the photo's size in bytes, its width and height,
   and where the aircraft was (1e-7 degrees, mm above home, centidegrees; unknown: `0x80000000`
   and `0xFFFF`). A SNAP_REQ for an id it has answered before gets the same answer, never a
   second photo.
3. The vehicle then sends the photo as SNAP_DATA chunks of 1024 bytes (up to 1024 chunks, 1 MB).
   It sends its SNAP_INFO again until an ACK has flag bit 1, and any chunk again that the ACKs
   do not show after `max(1 s, 2.5 × round trip)` (2 s while the round trip is unknown). It
   gives up after 60 s without an ACK. A chunk that comes before its SNAP_INFO is kept.
4. Each receiver ACKs twice a second while something arrives, and at once when it has it all
   (flag bit 0). A sender that hears "all there" stops. A receiver that already finished a
   photo answers any more of its packets with "all there".
5. The server keeps finished photos (by default for 7 days, in a folder), sets `time` in the
   SNAP_INFO it passes on to its unix time, and passes a photo on to every GCS agent that sent
   SNAP_REQ or SNAP_SYNC during its session, starting while the photo is still arriving. A
   problem goes only to the agent that asked.
6. A GCS agent sends SNAP_SYNC with the newest photo id it has at the start of each session. The
   server then sends it every photo it keeps with a larger id: photos taken while the agent was
   away arrive by themselves.

**Speed.** A photo goes only as fast as the link carries it without delaying the telemetry:
senders pace their chunks, start at 4 KB/s, and follow the round trip (a queue building up in
the network makes it grow): when it rises more than `max(150 ms, half its lowest value of the
last minute)` above that lowest value, they halve their pace (at most once a second, not below
256 B/s); otherwise they add a tenth of their cap each second, as LEDBAT does (RFC 6817). The
vehicle's cap is 32 KB/s on LTE and 2 KB/s on 2G, the server's 64 KB/s to each agent. When the
vehicle gets a new session (perhaps with a restarted server that knows nothing of the photo), it
sends everything again; the first ACK tells what the server still has. A server takes a photo
it did not ask for only if its id is within 10 minutes of its clock (asked for before it
restarted).

Status values: 0 OK, 1 NO_AIRCRAFT, 2 NO_CAMERA, 3 FAILED, 4 BUSY, 5 NO_ANSWER.

## Locator

Where the aircraft is, from the LTE module's own GNSS receiver, whatever the flight controller
does: a crashed aircraft whose flight controller is dead still reports its position as long as
the module has power.

- The vehicle sends POSITION every few seconds (5 s by default) while it has a session: the
  GNSS time (unix seconds, 0 if unknown), latitude and longitude (1e-7 degrees), altitude above
  mean sea level (mm), ground speed (cm/s), course (centidegrees), HDOP (×100), satellites, and
  `fix` (0 none, 2 2D, 3 3D). Without a fix, the position fields are unknown (`0x80000000`,
  `0xFFFF`). `flags` bit 0: the flight controller has sent nothing for 10 s or more; bit 1: the
  module cannot read its GNSS. `fc_silent_s` is the time since the flight controller last sent
  anything (`0xFFFF`: nothing since the module started). The module's battery: millivolts and
  percent (`0xFFFF`, `0xFF` unknown), as its fuel gauge reads them. On V2 boards the gauge sits on
  the board's supply rail: 4250 mV or more (more than a Li-ion cell holds) means external power
  (USB or the 5V pin), and the percent then says nothing about the cell. `time` is 0. `chip_c` is the temperature of the module's
  ESP32-S3, from its own sensor, in °C (`-128` unknown). It reads warmer than the air around
  the board; MavLTE shows it in amber from 70 °C and in red from 80 °C.
- Before version 1.4.0 the body ended after `time` (34 bytes). Each side reads the fields it
  knows and ignores any that follow, so both lengths work everywhere: a 34-byte body has no
  temperature.
- The server sets `time` to its unix time and passes the POSITION on to every active GCS
  session, watching ones too. It keeps the last POSITION with a fix (on disk, so that it
  survives a restart) and sends it to each GCS session when that becomes active: an agent that
  connects later still learns the last known position.
- A GCS agent treats a POSITION whose `time` is 15 s old or more as the last known position, not
  a live one.

## Locator voice

For the last metres to a crashed aircraft: while the voice is on, the speaker on the aircraft's board
sounds again and again (the ESP32 firmware: the modem plays a two-tone alarm stored in its flash, or
says a phrase with its text-to-speech).

1. A GCS agent switches it with VOICE (bit 0: on). The server keeps the switch, on disk so that it
   survives a restart, until an agent switches it again: the aircraft does not have to be online.
2. The server sets PONG flag bit 1 in every PONG to the vehicle while the voice is on. The vehicle
   keeps the last state it was told while it has no session, so an aircraft whose voice is on keeps
   sounding where it has no coverage, and one switched on while it was away starts at its first PONG.
3. The vehicle says in each PING what it makes of it: flag bit 1, it sounds; bit 2, it was asked to
   but cannot (its modem refuses). The server passes these on in STATUS, as they were in the
   vehicle's last PING (also while it is offline), together with its own switch: STATUS flag bit 1,
   the voice is on; bit 2, the vehicle sounds; bit 3, it cannot.
4. A GCS agent sends VOICE again, once a second, until STATUS shows the switch it asked for, for at
   most 10 s (a server older than 1.5.0 ignores VOICE and never shows it).

## Logs

The aircraft's own log files: the ESP32 firmware writes one to its SD card per power-on, a line a
second (README, *Flight log*). GCS agents list and download them through the server, which keeps
nothing of them: it passes each request to the vehicle, and the answers back to the agent that asked.

1. A GCS agent sends FILE_REQ. `op` 1, LIST: the list of files, newest first, from entry `offset`
   on. `op` 2, GET: the file `name` from byte `offset` on. `op` 3, STOP: the end of a GET. `name`
   is the file's name on the card (ASCII, NUL-padded to 12 bytes, as `LOG00012.CSV`). The server
   passes the request to the vehicle under a `req_id` of its own (two agents may use the same
   one) and the vehicle's answers back under the agent's; it forgets a request 120 s after its
   last packet. Without an online vehicle it answers at once, with status NO_AIRCRAFT.
2. LIST: the vehicle answers with one FILE_LIST: how many files the card has, the index of the
   first entry in this packet, and up to 40 entries: the name, the size in bytes, and the times
   of the file's first and last line (unix seconds, 0 if unknown). For more, the agent asks again
   from the next index.
3. GET: the vehicle answers with FILE_DATA, each with the file's size, the offset and up to 1024
   bytes of the file from there. The size is the one when the GET came: a file still being
   written grows, and a later GET from there brings the rest. The vehicle sends at most 32 KB
   beyond the last offset an ACK showed, paced as photos are and never while a photo goes; with
   no new ACK for `max(1 s, 2.5 × round trip)` it starts again from the last offset an ACK
   showed. A GET from the size or beyond gets one FILE_DATA without bytes. A new GET, from any
   agent, ends the one before, which gets status STOPPED. The vehicle gives up after 30 s
   without an ACK.
4. The agent answers FILE_DATA with FILE_ACK `next`: every byte before that offset has come. It
   ACKs at least five times a second while bytes arrive, keeps only the bytes at `next` and drops
   the others (go-back-N), and sends its GET again from `next` when nothing has come for a few
   seconds.
5. A status other than OK comes without bytes or entries, and ends the request.

Status values: 0 OK, 1 NO_CARD (no SD card, or one the vehicle cannot read), 2 NOT_FOUND,
3 NO_AIRCRAFT, 4 CARD_ERROR (reading the file failed), 5 STOPPED.

A vehicle or server older than 1.8.0 ignores these packets: the agent hears nothing.

The board also offers the same files on its USB serial port, for MavLTE on a PC beside it: text
commands and lines, each line of the file's bytes with its CRC-32 (`firmware/main/usbproto.h`,
`relay/boardusb.py`). That is not part of this protocol.

## Not provided

Packets are authenticated, not encrypted: the MAVLink content is visible to anyone on the
path (for example your mobile carrier). Use MAVLink 2 signing on the flight controller so
that only your GCS can command the aircraft, whatever happens on the network.

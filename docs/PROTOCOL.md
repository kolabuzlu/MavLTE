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

## Packet types

| type | name    | direction | session / seq | body |
|------|---------|-----------|---------------|------|
| 1    | HELLO   | C → S | 0 / 0 | nonce (8 bytes), then optional UTF-8 info text (≤ 64 bytes) |
| 2    | WELCOME | S → C | new session / 0 | the 8-byte nonce from the HELLO |
| 3    | DATA    | C ↔ S | session / seq | MAVLink bytes (whole frames), ≤ 1172 bytes |
| 4    | PING    | C → S | session / seq | `t_ms u32, rtt_ms u16, rx_loss_permille u16, rssi_dbm i16, rat u8, flags u8` (bit 0, GCS only: watching) |
| 5    | PONG    | S → C | session / seq | `t_ms u32` (echo), `flags u8` (bit 0: a GCS is connected) |
| 6    | REJECT  | S → C | the rejected session / 0 | `reason u8` (1 = unknown or expired session) |
| 7    | STATUS  | S → GCS | session / seq | `flags u8, rat u8, rtt_ms u16, up_loss_permille u16, down_loss_permille u16, rssi_dbm i16, idle_ms u16` |

Unknown values are `0xFFFF` for u16 fields, `0x7FFF` for `rssi_dbm` and `0xFF` for `rat`.
`rat` uses the 3GPP TS 27.007 access technology numbers (0 GSM, 3 EDGE, 7 LTE, ...).
Receivers ignore extra trailing bytes in a body and treat missing trailing fields as unknown,
so fields can be appended in later versions.

STATUS describes the vehicle's link: flag bit 0 means the vehicle was heard within the last
3 s, `rtt_ms`, `down_loss_permille`, `rssi_dbm` and `rat` are what the vehicle reported in its
last PING, `up_loss_permille` is measured by the server, and `idle_ms` is the time since the
server last heard from the vehicle.

## Session setup

1. The client sends HELLO with a random nonce once a second until it gets a WELCOME. It
   keeps the same nonce for all retries of one attempt.
2. The server checks the tag with the key for the claimed role, creates a *pending* session
   with a random non-zero id and answers WELCOME, echoing the nonce.
3. The client accepts a WELCOME only if it carries the nonce of its current attempt. It then
   starts its sequence numbers at 1 and sends a PING straight away.
4. The first valid DATA or PING makes the session *active*. A new active vehicle session
   replaces the previous one; up to 8 GCS sessions can be active at once.

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

HELLO carries no sequence number: replaying one only makes the server create a pending
session that nobody can use, and the number of pending sessions is capped. WELCOME is bound
to the client's current nonce and REJECT to the client's current session.

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

## Not provided

Packets are authenticated, not encrypted: the MAVLink content is visible to anyone on the
path (for example your mobile carrier). Use MAVLink 2 signing on the flight controller so
that only your GCS can command the aircraft, whatever happens on the network.

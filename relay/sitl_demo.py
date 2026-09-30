#!/usr/bin/env python3
"""Try MavLTE with ArduPilot SITL on one PC, before the ESP32 board arrives.

    SITL plane --tcp 5762--> vehicle --(emulated 4G)--> relay --> GCS agent --udp 14550--> Mission Planner
    (flight controller)      (plays the ESP32)

One command starts all of it. Then, in Mission Planner, choose UDP, click Connect and accept port
14550: you see and command the simulated plane only through the tunnel, just as in the aircraft.

    python sitl_demo.py                               clean link
    python sitl_demo.py --delay 80 --jitter 30 --loss 3    a mediocre mobile link
    python sitl_demo.py --server your-server:14650 --vehicle-key ... --gcs-key ...   through your relay

Finds Mission Planner's copy of ArduPlane SITL by itself (or give --sitl). Runs SITL in its own
folder, so Mission Planner's simulator settings are not touched. Stop with Ctrl+C.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import random
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Optional

import mavrelay as mr

log = logging.getLogger("4g-link")

IP_UDP_HEADERS = 28  # bytes a mobile operator counts on top of each UDP payload
USB_PORT = 5780  # SITL SERIAL0, the plane's "USB port"
CONFIG = Path(__file__).resolve().parent / "mavrelay.ini"


def gcs_key_from_config(config: Path, relay_port: int) -> bytes:
    """The GCS key in mavrelay.ini, so that the MavLTE app can connect as it is; created if missing."""
    conf = mr.load_config(str(config), "gcs") if config.exists() else {}
    try:
        key = mr.parse_key(conf.get("key", ""))
        log.info("GCS key: the one in %s", config.name)
        return key
    except ValueError:
        pass
    key = secrets.token_bytes(32)
    values = {"key": key.hex()}
    if not conf.get("server"):
        values["server"] = f"127.0.0.1:{relay_port}"
    mr.update_config(str(config), "gcs", values)
    log.info("saved a new GCS key for this test in %s", config)
    return key


def find_sitl() -> Optional[Path]:
    home = Path.home()
    if sys.platform == "win32":
        candidates = [docs / "Mission Planner" / "sitl" / "ArduPlane.exe"
                      for docs in (home / "Documents", home / "OneDrive" / "Documents")]
    else:
        candidates = [home / "ardupilot" / "build" / "sitl" / "bin" / "arduplane"]
        found = shutil.which("arduplane")
        if found:
            candidates.insert(0, Path(found))
    return next((c for c in candidates if c.is_file()), None)


def start_sitl(args) -> subprocess.Popen:
    exe = Path(args.sitl) if args.sitl else find_sitl()
    if exe is None or not exe.is_file():
        raise SystemExit("ArduPlane SITL not found. Start a plane simulation in Mission Planner once (it downloads "
                         "SITL), or give its path with --sitl.")
    workdir = Path(tempfile.gettempdir()) / "mavrelay-sitl"  # keeps SITL's parameters between runs
    workdir.mkdir(exist_ok=True)
    # SERIAL0 is the plane's "USB port": on TCP 5780 so that 5760 stays free for the GCS agent, and
    # without waiting for a connection. SERIAL1 (TCP 5762) plays the TELEM port wired to the ESP32.
    cmd = [str(exe), "--model", "plane", "--speedup", str(args.speedup), "--serial0", f"tcp:{USB_PORT}"]
    if args.home:
        cmd += ["--home", args.home]
    if args.wipe:
        cmd += ["--wipe"]
    logfile = open(workdir / "sitl.log", "w")
    log.info("starting %s (log: %s)", exe.name, workdir / "sitl.log")
    return subprocess.Popen(cmd, cwd=workdir, stdin=subprocess.DEVNULL, stdout=logfile, stderr=subprocess.STDOUT)


class LinkEmulator:
    """Plays the mobile network between the vehicle and the relay.

    Delays, reorders and drops UDP packets, and counts the traffic including IP and UDP headers,
    which is what a SIM card is billed for.
    """

    def __init__(self, relay_addr, delay_ms: float, jitter_ms: float, loss_pct: float) -> None:
        self.relay_addr = relay_addr
        self.delay = delay_ms / 1000
        self.jitter = jitter_ms / 1000
        self.loss = loss_pct / 100
        self.vehicle_addr = None
        self.up_bytes = self.down_bytes = self.up_packets = self.dropped = 0
        self.front = self.back = None
        self.address = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self.front, _ = await loop.create_datagram_endpoint(lambda: _Side(self._from_vehicle),
                                                            local_addr=("127.0.0.1", 0))
        v6 = ":" in self.relay_addr[0]
        self.back, _ = await loop.create_datagram_endpoint(lambda: _Side(self._from_relay),
                                                           local_addr=("::" if v6 else "0.0.0.0", 0))
        self.address = self.front.get_extra_info("sockname")[:2]

    def _from_vehicle(self, data: bytes, addr) -> None:
        self.vehicle_addr = addr
        self.up_bytes += len(data) + IP_UDP_HEADERS
        self.up_packets += 1
        self._forward(self.back, data, self.relay_addr)

    def _from_relay(self, data: bytes, addr) -> None:
        self.down_bytes += len(data) + IP_UDP_HEADERS
        if self.vehicle_addr is not None:
            self._forward(self.front, data, self.vehicle_addr)

    def _forward(self, transport, data: bytes, addr) -> None:
        if self.loss and random.random() < self.loss:
            self.dropped += 1
            return
        delay = max(0.0, self.delay + random.uniform(-self.jitter, self.jitter))
        if delay:
            asyncio.get_running_loop().call_later(delay, transport.sendto, data, addr)
        else:
            transport.sendto(data, addr)

    def describe(self) -> str:
        if not (self.delay or self.jitter or self.loss):
            return "no delay or loss added"
        return f"{self.delay * 1000:.0f} ms ± {self.jitter * 1000:.0f} ms each way, {self.loss * 100:g}% loss"


class _Side(asyncio.DatagramProtocol):
    def __init__(self, on_datagram) -> None:
        self.on_datagram = on_datagram

    def datagram_received(self, data: bytes, addr) -> None:
        self.on_datagram(data, addr)

    def error_received(self, exc) -> None:
        pass


async def report(link: LinkEmulator, interval: float) -> None:
    up = down = packets = 0
    while True:
        await asyncio.sleep(interval)
        du, dd, dp = link.up_bytes - up, link.down_bytes - down, link.up_packets - packets
        up, down, packets = link.up_bytes, link.down_bytes, link.up_packets
        if not (du or dd):
            continue
        per_hour = (du + dd) / interval * 3600 / 1e6
        log.info("up %.1f KB/s in %.0f packets/s, down %.1f KB/s: about %.0f MB per hour%s",
                 du / interval / 1000, dp / interval, dd / interval / 1000, per_hour,
                 f" ({link.dropped} packets dropped so far)" if link.loss else "")


async def run(args, keys) -> None:
    vehicle_key, gcs_key = keys
    tasks = []
    if args.server:
        host, port = mr.parse_hostport(args.server)
        infos = await asyncio.get_running_loop().getaddrinfo(host, port, type=socket.SOCK_DGRAM)
        relay_addr = infos[0][4][:2]
    else:
        relay_addr = ("127.0.0.1", args.relay_port)
        tasks.append(mr.run_server(argparse.Namespace(
            listen=relay_addr, vehicle_key=vehicle_key, gcs_key=gcs_key, tcp_listen=None, tcp_allow=[],
            session_timeout=120.0)))
    link = LinkEmulator(relay_addr, args.delay, args.jitter, args.loss)
    await link.start()
    tasks.append(mr.run_vehicle(argparse.Namespace(
        server=link.address, key=vehicle_key, serial=None, tcp=args.fc, udp=None, batch_ms=args.batch_ms,
        always_send=False)))
    if not args.no_agent:
        tasks.append(mr.run_gcs(argparse.Namespace(
            server=relay_addr, key=gcs_key, udp=mr.parse_hostport(args.udp), tcp=mr.parse_hostport(args.tcp),
            status_interval=10.0)))
    tasks.append(report(link, 10.0))

    udp_port, tcp_port = mr.parse_hostport(args.udp)[1], mr.parse_hostport(args.tcp)[1]
    print("-" * 100)
    print(f" SITL plane -> vehicle (plays the ESP32) -> emulated 4G ({link.describe()}) -> relay -> GCS agent")
    if args.no_agent and args.server:
        print(f" Start your GCS agent as usual, with relay server {args.server} and your GCS key.")
    elif args.no_agent:
        script = str(Path(mr.__file__).resolve())
        script = f'"{script}"' if " " in script else script
        print(f" Start the GCS agent yourself: open the MavLTE app (relay server 127.0.0.1:{args.relay_port},")
        print(f" key from {Path(args.config).name}), or paste this into another window:")
        print(f"   python {script} gcs --server 127.0.0.1:{args.relay_port} --key {gcs_key.hex()}")
    if args.no_agent:
        print(f" Mission Planner then: UDP, port {udp_port}, or TCP 127.0.0.1 port {tcp_port}")
    else:
        print(f" Mission Planner: choose UDP, click Connect, port {udp_port}   (or TCP 127.0.0.1 port {tcp_port})")
    print(f" The plane's USB port (SITL SERIAL0) is TCP 127.0.0.1 port {USB_PORT}, e.g. to set up MAVLink signing.")
    print(" Stop with Ctrl+C.")
    print("-" * 100, flush=True)
    await asyncio.gather(*(asyncio.ensure_future(t) for t in tasks))


class RoleFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        record.role = record.name.split(".")[-1]
        return super().format(record)


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="Try MavLTE with ArduPilot SITL on one PC.")
    p.add_argument("--sitl", help="ArduPlane SITL program (default: Mission Planner's copy)")
    p.add_argument("--no-sitl", action="store_true", help="SITL is already running: only start the link")
    p.add_argument("--fc", default="127.0.0.1:5762",
                   help="SITL serial port that plays the TELEM port the ESP32 is wired to (default: SERIAL1)")
    p.add_argument("--speedup", type=float, default=1.0, help="SITL speed-up factor (default 1)")
    p.add_argument("--home", help="SITL start position: lat,lon,alt,heading")
    p.add_argument("--wipe", action="store_true", help="start SITL with default parameters")
    p.add_argument("--delay", type=float, default=0.0, help="emulated 4G: one-way delay in ms")
    p.add_argument("--jitter", type=float, default=0.0, help="emulated 4G: random delay variation in ms")
    p.add_argument("--loss", type=float, default=0.0, help="emulated 4G: percent of packets lost each way")
    p.add_argument("--no-agent", action="store_true",
                   help="do not start a GCS agent: you run your own (the demo prints the command)")
    p.add_argument("--config", default=str(CONFIG),
                   help="with --no-agent: settings file whose [gcs] key the local relay uses (default mavrelay.ini)")
    p.add_argument("--server", help="use your relay server (host:port) instead of one on this PC")
    p.add_argument("--vehicle-key", help="vehicle key (of your server; default: a new random one)")
    p.add_argument("--gcs-key", help="GCS key (of your server; default: a new random one)")
    p.add_argument("--relay-port", type=int, default=14650, help="UDP port of the local relay (default 14650)")
    p.add_argument("--udp", default="127.0.0.1:14550", help="where the GCS agent sends MAVLink (default 127.0.0.1:14550)")
    p.add_argument("--tcp", default="127.0.0.1:5760", help="TCP port of the GCS agent (default 127.0.0.1:5760)")
    p.add_argument("--batch-ms", type=float, default=50.0, help="telemetry batching, like the firmware (default 50)")
    args = p.parse_args(argv)

    if args.server and not args.vehicle_key:
        p.error("--server needs --vehicle-key")
    if args.server and not args.no_agent and not args.gcs_key:
        p.error("--server needs --gcs-key (or --no-agent)")
    handler = logging.StreamHandler()
    handler.setFormatter(RoleFormatter("%(asctime)s %(role)-8s %(message)s", datefmt="%H:%M:%S"))
    logging.basicConfig(level=logging.INFO, handlers=[handler])
    try:
        vehicle_key = mr.parse_key(args.vehicle_key) if args.vehicle_key else secrets.token_bytes(32)
        if args.gcs_key:
            gcs_key = mr.parse_key(args.gcs_key)
        elif args.no_agent and not args.server:
            gcs_key = gcs_key_from_config(Path(args.config), args.relay_port)  # the MavLTE app finds it there
        else:
            gcs_key = secrets.token_bytes(32)
    except ValueError as exc:
        p.error(str(exc))
    keys = (vehicle_key, gcs_key)
    if hasattr(signal, "SIGBREAK"):  # Ctrl+Break on Windows: stop cleanly as with Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)

    sitl = None if args.no_sitl else start_sitl(args)
    try:
        mr.run_async(run(args, keys))
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        log.error("%s (is another copy of this demo, or Mission Planner's simulator, running?)", exc)
    finally:
        if sitl is not None and sitl.poll() is None:
            sitl.terminate()
            try:
                sitl.wait(5)
            except subprocess.TimeoutExpired:
                sitl.kill()
        log.info("stopped")


if __name__ == "__main__":
    main()

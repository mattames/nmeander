#!/usr/bin/env python3
"""
nmeander - NMEA 0183 GPS simulator that writes to a serial port
(e.g. a USB-to-serial adaptor), or to stdout for testing.

Generates RMC, VTG, GGA, GSA, GSV, GLL and ZDA sentences with real UTC
time, valid checksums and a moving (or stationary) position.

Requires: pip install pyserial

Examples
--------
  # Find your adaptor (FTDI / CP210x / CH340 etc.)
  python3 nmeander.py --list

  # Stationary fix at the default position, 9600 baud, 1 Hz  (macOS: use /dev/cu.*)
  python3 nmeander.py --port /dev/cu.usbserial-A10KXYZ

  # Moving at 12 kn on a 045 deg course, 4800 baud (classic NMEA rate), echo to screen
  python3 nmeander.py --port /dev/ttyUSB0 --baud 4800 --speed 12 --course 45 --echo

  # Loop around waypoints at 60 kn, 5 Hz, GN talker, 3 m position noise
  python3 nmeander.py --port COM5 --rate 5 --talker GN --speed 60 --jitter 3 \\
      --waypoints "-33.8568,151.2153;-33.8523,151.2108;-33.8688,151.2093"

  # No port: just print sentences
  python3 nmeander.py --stdout --speed 5

  # Pretend a cold start: no fix for the first 20 s
  python3 nmeander.py --port /dev/ttyUSB0 --acquire 20
"""

import argparse
import math
import random
import sys
import time
from datetime import datetime, timezone

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # allow --stdout use without pyserial
    serial = None
    list_ports = None

EARTH_R = 6371008.8      # mean Earth radius, metres
KNOT_MS = 1852.0 / 3600  # 1 knot in m/s

ALL_SENTENCES = ("RMC", "VTG", "GGA", "GSA", "GSV", "GLL", "ZDA")


# --------------------------------------------------------------------------
# NMEA formatting helpers
# --------------------------------------------------------------------------

def checksum(body: str) -> str:
    """XOR of every byte between '$' and '*'."""
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return f"{cs:02X}"


def sentence(body: str) -> str:
    return f"${body}*{checksum(body)}\r\n"


def _deg_to_dm(value: float, deg_width: int) -> str:
    """Decimal degrees -> [d]ddmm.mmmm without the 59.99995 -> 60.0000 rounding bug."""
    total = round(abs(value) * 60 * 10000)        # in 1/10000 minute
    d, rem = divmod(total, 60 * 10000)
    m_int, m_frac = divmod(rem, 10000)
    return f"{d:0{deg_width}d}{m_int:02d}.{m_frac:04d}"


def fmt_lat(lat: float):
    return _deg_to_dm(lat, 2), ("N" if lat >= 0 else "S")


def fmt_lon(lon: float):
    return _deg_to_dm(lon, 3), ("E" if lon >= 0 else "W")


def fmt_time(t: datetime) -> str:
    return t.strftime("%H%M%S.") + f"{t.microsecond // 10000:02d}"


# --------------------------------------------------------------------------
# Geodesy
# --------------------------------------------------------------------------

def destination(lat, lon, bearing_deg, dist_m):
    """Point reached from (lat, lon) travelling dist_m on initial bearing (great circle)."""
    p1, l1, th = math.radians(lat), math.radians(lon), math.radians(bearing_deg)
    d = dist_m / EARTH_R
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(th))
    l2 = l1 + math.atan2(math.sin(th) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), (math.degrees(l2) + 540) % 360 - 180


def distance_bearing(lat1, lon1, lat2, lon2):
    """Haversine distance (m) and initial bearing (deg true) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    dist = 2 * EARTH_R * math.asin(math.sqrt(a))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return dist, (math.degrees(math.atan2(y, x)) + 360) % 360


# --------------------------------------------------------------------------
# Simulated receiver state
# --------------------------------------------------------------------------

class Satellite:
    def __init__(self, prn):
        self.prn = prn
        self.elev = random.uniform(5, 85)
        self.az = random.uniform(0, 360)
        self.snr = random.uniform(28, 46)
        self.az_rate = random.uniform(-0.01, 0.01)   # deg/s - slow sky drift

    def step(self, dt):
        self.az = (self.az + self.az_rate * dt) % 360
        self.snr = min(50, max(20, self.snr + random.gauss(0, 0.3)))


class Receiver:
    def __init__(self, args):
        self.lat, self.lon = args.lat, args.lon
        self.alt = args.alt
        self.speed_kn = args.speed
        self.course = args.course % 360
        self.waypoints = args.waypoints
        self.wp_idx = 0
        self.loop = not args.no_loop
        self.jitter = args.jitter
        self.elapsed = 0.0
        self.acquire = args.acquire
        self.force_nofix = args.no_fix
        self.hdop, self.pdop, self.vdop = args.hdop, args.pdop, args.vdop
        self.geoid = args.geoid

        n_view = max(args.sats, args.sats_in_view)
        prns = random.sample(range(1, 33), min(n_view, 32))
        self.sats = [Satellite(p) for p in prns]
        self.sats.sort(key=lambda s: -s.elev)
        self.n_used = min(args.sats, len(self.sats))

        if self.waypoints:
            # start at the first waypoint and head for the second
            self.lat, self.lon = self.waypoints[0]
            self.wp_idx = 1 % len(self.waypoints)

    @property
    def has_fix(self):
        return not self.force_nofix and self.elapsed >= self.acquire

    def step(self, dt):
        self.elapsed += dt
        for s in self.sats:
            s.step(dt)

        if self.speed_kn <= 0:
            return
        travel = self.speed_kn * KNOT_MS * dt

        if self.waypoints and len(self.waypoints) > 1:
            while travel > 0 and self.speed_kn > 0:
                tlat, tlon = self.waypoints[self.wp_idx]
                dist, brg = distance_bearing(self.lat, self.lon, tlat, tlon)
                self.course = brg
                if dist > travel:
                    self.lat, self.lon = destination(self.lat, self.lon, brg, travel)
                    travel = 0
                else:
                    self.lat, self.lon = tlat, tlon
                    travel -= dist
                    nxt = self.wp_idx + 1
                    if nxt >= len(self.waypoints):
                        if self.loop:
                            nxt = 0
                        else:
                            self.speed_kn = 0   # arrived; stop
                            break
                    self.wp_idx = nxt
        else:
            self.lat, self.lon = destination(self.lat, self.lon, self.course, travel)

    def reported_position(self):
        """True position plus optional Gaussian noise."""
        if self.jitter <= 0:
            return self.lat, self.lon, self.alt
        lat, lon = destination(self.lat, self.lon, random.uniform(0, 360),
                               abs(random.gauss(0, self.jitter)))
        return lat, lon, self.alt + random.gauss(0, self.jitter * 1.5)


# --------------------------------------------------------------------------
# Sentence builders
# --------------------------------------------------------------------------

def build_epoch(rx: Receiver, utc: datetime, talker: str, wanted) -> str:
    lat, lon, alt = rx.reported_position()
    la, ns = fmt_lat(lat)
    lo, ew = fmt_lon(lon)
    tstr = fmt_time(utc)
    dstr = utc.strftime("%d%m%y")
    fix = rx.has_fix
    spd = rx.speed_kn if fix else 0.0
    cog = rx.course
    mode = "A" if fix else "N"        # NMEA 2.3+ mode indicator
    used = rx.sats[: rx.n_used] if fix else []

    out = []
    for name in wanted:
        if name == "RMC":
            if fix:
                out.append(sentence(f"{talker}RMC,{tstr},A,{la},{ns},{lo},{ew},"
                                    f"{spd:.2f},{cog:.2f},{dstr},,,{mode}"))
            else:
                out.append(sentence(f"{talker}RMC,{tstr},V,,,,,,,{dstr},,,N"))

        elif name == "VTG":
            if fix:
                out.append(sentence(f"{talker}VTG,{cog:.2f},T,,M,{spd:.2f},N,"
                                    f"{spd * 1.852:.2f},K,{mode}"))
            else:
                out.append(sentence(f"{talker}VTG,,T,,M,,N,,K,N"))

        elif name == "GGA":
            if fix:
                out.append(sentence(f"{talker}GGA,{tstr},{la},{ns},{lo},{ew},1,"
                                    f"{len(used):02d},{rx.hdop:.2f},{alt:.1f},M,"
                                    f"{rx.geoid:.1f},M,,"))
            else:
                out.append(sentence(f"{talker}GGA,{tstr},,,,,0,00,99.99,,,,,,"))

        elif name == "GSA":
            prns = [f"{s.prn:02d}" for s in used[:12]]
            prns += [""] * (12 - len(prns))
            if fix:
                out.append(sentence(f"{talker}GSA,A,3,{','.join(prns)},"
                                    f"{rx.pdop:.2f},{rx.hdop:.2f},{rx.vdop:.2f}"))
            else:
                out.append(sentence(f"{talker}GSA,A,1,{','.join(prns)},99.99,99.99,99.99"))

        elif name == "GSV":
            # Satellites are GPS PRNs, so GSV always uses the GP talker.
            sats = rx.sats
            total = max(1, math.ceil(len(sats) / 4))
            for i in range(total):
                chunk = sats[i * 4:(i + 1) * 4]
                fields = []
                for s in chunk:
                    snr = f"{s.snr:02.0f}" if fix else ""
                    fields.append(f"{s.prn:02d},{s.elev:02.0f},{s.az:03.0f},{snr}")
                body = f"GPGSV,{total},{i + 1},{len(sats):02d}"
                if fields:
                    body += "," + ",".join(fields)
                out.append(sentence(body))

        elif name == "GLL":
            if fix:
                out.append(sentence(f"{talker}GLL,{la},{ns},{lo},{ew},{tstr},A,{mode}"))
            else:
                out.append(sentence(f"{talker}GLL,,,,,{tstr},V,N"))

        elif name == "ZDA":
            out.append(sentence(f"{talker}ZDA,{tstr},{utc:%d},{utc:%m},{utc:%Y},00,00"))

    return "".join(out)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_waypoints(text):
    pts = []
    for chunk in text.replace("\n", ";").split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        a, b = chunk.split(",")
        pts.append((float(a), float(b)))
    if len(pts) < 2:
        raise argparse.ArgumentTypeError("need at least two 'lat,lon' waypoints")
    return pts


def parse_sentences(text):
    names = [s.strip().upper() for s in text.split(",") if s.strip()]
    bad = [n for n in names if n not in ALL_SENTENCES]
    if bad:
        raise argparse.ArgumentTypeError(f"unknown sentence(s): {', '.join(bad)}; "
                                         f"choose from {', '.join(ALL_SENTENCES)}")
    return names


def main():
    p = argparse.ArgumentParser(description="NMEA 0183 GPS simulator",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("Examples", 1)[1])
    out = p.add_argument_group("output")
    out.add_argument("--port", help="serial device, e.g. /dev/cu.usbserial-XXXX, /dev/ttyUSB0, COM5")
    out.add_argument("--baud", type=int, default=9600, help="baud rate (default 9600; classic NMEA is 4800)")
    out.add_argument("--stdout", action="store_true", help="write to stdout instead of a serial port")
    out.add_argument("--echo", action="store_true", help="also print sentences to the terminal")
    out.add_argument("--list", action="store_true", help="list serial ports and exit")
    out.add_argument("--rate", type=float, default=1.0, help="update rate in Hz (default 1)")
    out.add_argument("--talker", default="GP", help="talker ID: GP, GN, GL, GA... (default GP)")
    out.add_argument("--sentences", type=parse_sentences, default=list(ALL_SENTENCES[:6]),
                     help="comma list, in output order (default RMC,VTG,GGA,GSA,GSV,GLL; ZDA also available)")
    out.add_argument("--duration", type=float, default=0, help="stop after N seconds (default: run forever)")

    mv = p.add_argument_group("position & motion")
    mv.add_argument("--lat", type=float, default=-33.8523, help="start latitude, decimal deg (default -33.8523)")
    mv.add_argument("--lon", type=float, default=151.2108, help="start longitude, decimal deg (default 151.2108)")
    mv.add_argument("--alt", type=float, default=50.0, help="altitude MSL, metres (default 50)")
    mv.add_argument("--geoid", type=float, default=22.0, help="geoid separation, metres (default 22, ~Sydney)")
    mv.add_argument("--speed", type=float, default=0.0, help="ground speed, knots (default 0 = stationary)")
    mv.add_argument("--course", type=float, default=0.0, help="course over ground, deg true (default 0)")
    mv.add_argument("--waypoints", type=parse_waypoints,
                    help='route "lat,lon;lat,lon;..." - starts at the first, overrides --lat/--lon/--course')
    mv.add_argument("--no-loop", action="store_true", help="stop at the last waypoint instead of looping")
    mv.add_argument("--jitter", type=float, default=0.0, help="position noise, 1-sigma metres (default 0)")

    fx = p.add_argument_group("fix quality")
    fx.add_argument("--sats", type=int, default=8, help="satellites used in fix (default 8)")
    fx.add_argument("--sats-in-view", type=int, default=11, help="satellites in view (default 11)")
    fx.add_argument("--hdop", type=float, default=0.9)
    fx.add_argument("--pdop", type=float, default=1.6)
    fx.add_argument("--vdop", type=float, default=1.3)
    fx.add_argument("--acquire", type=float, default=0.0, help="report no fix for the first N seconds")
    fx.add_argument("--no-fix", action="store_true", help="never report a fix")
    fx.add_argument("--seed", type=int, help="random seed for repeatable satellites/noise")
    # "--waypoints -33.8,151.2;..." would look like an option to argparse; glue it on.
    argv = sys.argv[1:]
    for i, a in enumerate(argv[:-1]):
        if a == "--waypoints":
            argv[i:i + 2] = [f"--waypoints={argv[i + 1]}"]
            break
    args = p.parse_args(argv)

    if args.list:
        if list_ports is None:
            sys.exit("pyserial is not installed: pip install pyserial")
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for port in ports:
            print(f"{port.device:<32} {port.description}  [{port.hwid}]")
        return

    if args.seed is not None:
        random.seed(args.seed)
    talker = args.talker.upper()[:2]
    if args.rate <= 0:
        sys.exit("--rate must be > 0")
    period = 1.0 / args.rate

    if args.stdout:
        ser = None
    else:
        if not args.port:
            p.error("give --port (see --list) or use --stdout")
        if serial is None:
            sys.exit("pyserial is not installed: pip install pyserial")
        try:
            ser = serial.Serial(args.port, args.baud, bytesize=8, parity="N",
                                stopbits=1, timeout=0, write_timeout=2,
                                xonxoff=False, rtscts=False, dsrdtr=False)
        except serial.SerialException as e:
            sys.exit(f"Could not open {args.port}: {e}")
        print(f"Sending NMEA on {args.port} @ {args.baud} 8N1, {args.rate:g} Hz. Ctrl-C to stop.",
              file=sys.stderr)

    rx = Receiver(args)
    warned = False

    # Align epochs to whole multiples of the period on the UTC clock,
    # like a real receiver (e.g. 093015.00, 093015.20 at 5 Hz).
    start = math.ceil(time.time() * args.rate) / args.rate
    k = 0
    try:
        while True:
            epoch = start + k * period
            delay = epoch - time.time()
            if delay > 0:
                time.sleep(delay)
            elif delay < -period:
                # fell well behind (laptop slept, port blocked...) - resync
                start = math.ceil(time.time() * args.rate) / args.rate
                k = 0
                continue

            if k:
                rx.step(period)
            utc = datetime.fromtimestamp(epoch, tz=timezone.utc)
            data = build_epoch(rx, utc, talker, args.sentences)

            if ser is not None:
                if not warned and len(data) * 10 > args.baud * period:
                    print(f"Warning: {len(data)} bytes per epoch exceeds what {args.baud} baud "
                          f"can carry at {args.rate:g} Hz - raise --baud, lower --rate or drop "
                          f"sentences.", file=sys.stderr)
                    warned = True
                try:
                    ser.write(data.encode("ascii"))
                except serial.SerialTimeoutException:
                    print("Serial write timed out (flow control / unplugged?)", file=sys.stderr)
                except serial.SerialException as e:
                    sys.exit(f"Serial error: {e}")
            if ser is None or args.echo:
                sys.stdout.write(data)
                sys.stdout.flush()

            k += 1
            if args.duration and k * period >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:          # e.g. piped into `head`
        sys.stderr.close()
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()

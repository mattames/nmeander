# nmeander

A small NMEA 0183 GPS simulator. It writes realistic GPS sentences to a serial port
(e.g. a USB-to-serial adaptor) so you can feed a chartplotter, autopilot, radio,
microcontroller or app under test without a real GPS receiver.

- Sentences: RMC, VTG, GGA, GSA, GSV, GLL, ZDA (choose which, and their order)
- Real UTC time, epochs aligned to the second like a real receiver
- Stationary, constant course/speed, or a looping waypoint route
- Optional position noise, fake cold-start (no fix for N seconds), or permanent no-fix
- Configurable talker ID (GP, GN, ...), update rate, baud rate, DOPs and satellite count
- Warns if the sentences won't fit through the chosen baud rate

## Install

```sh
python3 -m pip install -r requirements.txt
```

## Usage

```sh
# List serial ports to find the adaptor (FTDI / CP210x / CH340 / PL2303 ...)
python3 nmeander.py --list

# Stationary fix, 9600 8N1, 1 Hz   (macOS: use the /dev/cu.* device, not /dev/tty.*)
python3 nmeander.py --port /dev/cu.usbserial-XXXX

# 12 kn on a 045° course at the classic 4800 baud, echoing to the terminal
python3 nmeander.py --port /dev/ttyUSB0 --baud 4800 --speed 12 --course 45 \
    --sentences RMC,GGA,GSA,VTG --echo

# Loop a route at 60 kn, 5 Hz, GN talker, 3 m of position noise
python3 nmeander.py --port COM5 --rate 5 --talker GN --speed 60 --jitter 3 \
    --waypoints "-33.8568,151.2153;-33.8523,151.2108;-33.8688,151.2093"

# No hardware: print to stdout
python3 nmeander.py --stdout --speed 5
```

Run `python3 nmeander.py --help` for every option.

| Option | Default | Meaning |
|---|---|---|
| `--port` | – | Serial device |
| `--baud` | 9600 | Baud rate (8N1, no flow control) |
| `--rate` | 1 | Updates per second |
| `--talker` | GP | Talker ID (GSV always uses GP) |
| `--sentences` | RMC,VTG,GGA,GSA,GSV,GLL | Sentences to send, in order |
| `--lat` / `--lon` | -33.8523 / 151.2108 | Start position |
| `--alt` / `--geoid` | 50 / 22 | Altitude MSL and geoid separation (m) |
| `--speed` / `--course` | 0 / 0 | Knots, degrees true |
| `--waypoints` | – | `"lat,lon;lat,lon;..."` route (loops unless `--no-loop`) |
| `--jitter` | 0 | Position noise, 1σ metres |
| `--sats` / `--sats-in-view` | 8 / 11 | Satellites used / visible |
| `--acquire` | 0 | Seconds of no-fix before the fix appears |
| `--no-fix` | off | Never report a fix |
| `--duration` | forever | Stop after N seconds |
| `--seed` | random | Repeatable satellites and noise |
| `--echo` / `--stdout` | off | Also print / only print |

## Wiring notes

- **Baud budget:** a full default set is ~480 bytes per epoch, which is right at the limit of
  4800 baud at 1 Hz. At 4800, drop GSV/GLL; for higher update rates, raise the baud rate.
- **Signal levels:** NMEA 0183 is formally RS-422, but most listeners accept single-ended RS-232.
  Use an RS-232 (DB9) adaptor for marine gear, connecting adaptor TX → listener's NMEA IN+ (or RX)
  and ground → NMEA IN− / GND. Use a TTL (3.3 V / 5 V) adaptor only for inputs that expect a
  bare GPS module, and match the logic voltage.

## References

The official NMEA 0183 standard is sold by the NMEA. Free references include
Eric S. Raymond's *NMEA Revealed* (gpsd project) and GPS-chip vendor protocol manuals
(u-blox, Quectel/MediaTek, SiRF).

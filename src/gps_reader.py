#!/usr/bin/env python3
"""
SPIRE GPS Reader Process
Reads NMEA sentences from Waveshare L76K GNSS module over UART.
Publishes position, altitude, time, speed, and satellite info to shared memory.

Parsed sentences:
  - GNGGA: position, altitude, fix quality, satellites, HDOP
  - GNRMC: UTC time/date, speed, heading, validity
  - GNGSA: PDOP, VDOP, fix mode

Usage:
  python3 gps_reader.py                          # Default: /dev/ttyAMA0
  python3 gps_reader.py -p /dev/serial0          # Custom port
  python3 gps_reader.py --set-time               # Sync system clock from GPS
  python3 gps_reader.py --log data/gps_test      # Log to CSV
  python3 gps_reader.py --rate 5                 # 5 Hz update rate
"""

import time
import sys
import os
import csv
import json
import signal
import logging
import argparse
import subprocess
from datetime import datetime, timezone

import serial

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GPS_SHM_NAME = "spire_gps_state"
GPS_SHM_SIZE = 512
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 9600

# ---------------------------------------------------------------------------
# PMTK Commands (L76K / Quectel L76)
# ---------------------------------------------------------------------------

# Startup modes
PMTK_HOT_START = "$PMTK101"
PMTK_WARM_START = "$PMTK102"
PMTK_COLD_START = "$PMTK103"
PMTK_FULL_COLD_START = "$PMTK104"

# Standby
PMTK_STANDBY = "$PMTK161"

# Power modes
PMTK_NORMAL_MODE = "$PMTK225,0"

# Position fix intervals
PMTK_FIX_1HZ = "$PMTK220,1000"
PMTK_FIX_2HZ = "$PMTK220,500"
PMTK_FIX_5HZ = "$PMTK220,200"
PMTK_FIX_10HZ = "$PMTK220,100"

# NMEA output config — enable GGA, GSA, RMC
PMTK_NMEA_OUTPUT = "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0"

# Restore defaults
PMTK_RESTORE_DEFAULTS = "$PMTK314,-1"

# Baud rates
PMTK_BAUD_9600 = "$PMTK251,9600"
PMTK_BAUD_115200 = "$PMTK251,115200"

# Enable GNSS constellations: GPS + GLONASS + BeiDou
PMTK_GNSS_GPS_GLONASS_BEIDOU = "$PCAS04,7"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.gps")


def setup_logging(verbose=False):
    level = logging.DEBUG if verbose else logging.INFO
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(fmt)
    log.setLevel(logging.DEBUG)
    log.addHandler(console)


# ---------------------------------------------------------------------------
# PMTK command sender
# ---------------------------------------------------------------------------

def send_pmtk(ser, command):
    """Send a PMTK command with checksum to the GPS module.

    Args:
        ser: Serial port instance
        command: PMTK command string (e.g. "$PMTK220,1000")
    """
    # Compute XOR checksum of characters between $ and *
    checksum = 0
    for ch in command[1:]:
        checksum ^= ord(ch)
    msg = f"{command}*{checksum:02X}\r\n"
    ser.write(msg.encode())
    log.debug(f"Sent: {msg.strip()}")


# ---------------------------------------------------------------------------
# NMEA Parser
# ---------------------------------------------------------------------------

def verify_checksum(sentence):
    """Verify NMEA sentence checksum.

    Args:
        sentence: Raw NMEA sentence string

    Returns:
        True if checksum valid or no checksum present
    """
    if "*" not in sentence:
        return False
    try:
        body, expected = sentence.split("*")
        body = body[1:]  # Remove leading $
        computed = 0
        for ch in body:
            computed ^= ord(ch)
        return computed == int(expected[:2], 16)
    except (ValueError, IndexError):
        return False


def parse_nmea_coordinate(raw, hemisphere):
    """Convert NMEA coordinate format to decimal degrees.

    NMEA format: DDDMM.MMMM (longitude) or DDMM.MMMM (latitude)

    Args:
        raw: Raw coordinate string from NMEA
        hemisphere: 'N', 'S', 'E', or 'W'

    Returns:
        float: Decimal degrees (negative for S/W)
    """
    if not raw or not hemisphere:
        return 0.0
    try:
        # Find the decimal point, degrees are everything before last 2 digits
        dot = raw.index(".")
        degrees = float(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
        result = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            result = -result
        return result
    except (ValueError, IndexError):
        return 0.0


def parse_gngga(fields):
    """Parse $GNGGA sentence — position, altitude, fix info.

    Fields:
        0: $GNGGA
        1: UTC time (HHMMSS.SS)
        2: Latitude (DDMM.MMMM)
        3: N/S
        4: Longitude (DDDMM.MMMM)
        5: E/W
        6: Fix quality (0=invalid, 1=GPS, 2=DGPS)
        7: Satellites in use
        8: HDOP
        9: Altitude (meters)
        10: Altitude unit (M)
    """
    data = {}
    try:
        data["utc_time"] = fields[1] if len(fields) > 1 else ""
        data["latitude"] = parse_nmea_coordinate(
            fields[2], fields[3]
        ) if len(fields) > 3 and fields[2] else 0.0
        data["longitude"] = parse_nmea_coordinate(
            fields[4], fields[5]
        ) if len(fields) > 5 and fields[4] else 0.0
        data["fix_quality"] = int(fields[6]) if len(fields) > 6 and fields[6] else 0
        data["satellites"] = int(fields[7]) if len(fields) > 7 and fields[7] else 0
        data["hdop"] = float(fields[8]) if len(fields) > 8 and fields[8] else 0.0
        data["altitude_m"] = float(fields[9]) if len(fields) > 9 and fields[9] else 0.0
    except (ValueError, IndexError) as e:
        log.debug(f"GGA parse error: {e}")
    return data


def parse_gnrmc(fields):
    """Parse $GNRMC sentence — time, date, speed, heading.

    Fields:
        0: $GNRMC
        1: UTC time (HHMMSS.SS)
        2: Status (A=active, V=void)
        3: Latitude
        4: N/S
        5: Longitude
        6: E/W
        7: Speed (knots)
        8: Heading (degrees)
        9: Date (DDMMYY)
    """
    data = {}
    try:
        data["utc_time"] = fields[1] if len(fields) > 1 else ""
        data["fix_valid"] = fields[2] == "A" if len(fields) > 2 else False
        data["speed_knots"] = float(fields[7]) if len(fields) > 7 and fields[7] else 0.0
        data["speed_kmh"] = data["speed_knots"] * 1.852
        data["heading"] = float(fields[8]) if len(fields) > 8 and fields[8] else 0.0
        data["utc_date"] = fields[9] if len(fields) > 9 else ""

        # Build ISO datetime string
        if data["utc_time"] and data["utc_date"]:
            t = data["utc_time"]
            d = data["utc_date"]
            try:
                data["utc_datetime"] = (
                    f"20{d[4:6]}-{d[2:4]}-{d[0:2]}T"
                    f"{t[0:2]}:{t[2:4]}:{t[4:6]}Z"
                )
            except IndexError:
                data["utc_datetime"] = ""
    except (ValueError, IndexError) as e:
        log.debug(f"RMC parse error: {e}")
    return data


def parse_gngsa(fields):
    """Parse $GNGSA sentence — DOP values and fix type.

    Fields:
        0: $GNGSA
        1: Mode (A=auto, M=manual)
        2: Fix type (1=no fix, 2=2D, 3=3D)
        3-14: Satellite PRNs
        15: PDOP
        16: HDOP
        17: VDOP
    """
    data = {}
    try:
        data["fix_type"] = int(fields[2]) if len(fields) > 2 and fields[2] else 1
        data["pdop"] = float(fields[15]) if len(fields) > 15 and fields[15] else 0.0
        data["hdop"] = float(fields[16]) if len(fields) > 16 and fields[16] else 0.0
        data["vdop_raw"] = fields[17].split("*")[0] if len(fields) > 17 else ""
        data["vdop"] = float(data["vdop_raw"]) if data["vdop_raw"] else 0.0
    except (ValueError, IndexError) as e:
        log.debug(f"GSA parse error: {e}")
    return data


# ---------------------------------------------------------------------------
# Shared Memory Publisher
# ---------------------------------------------------------------------------

class GPSSharedMemory:
    """Publish GPS state to shared memory for other processes."""

    def __init__(self, name=GPS_SHM_NAME, size=GPS_SHM_SIZE):
        from multiprocessing import shared_memory, resource_tracker
        self.name = name
        self.size = size

        # Clean up stale block
        try:
            old = shared_memory.SharedMemory(name=name, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass

        self.shm = shared_memory.SharedMemory(
            name=name, create=True, size=size
        )
        log.info(f"Shared memory created: {name} ({size} bytes)")

    def publish(self, state):
        """Write state dict to shared memory as JSON."""
        data = json.dumps(state).encode("utf-8")
        self.shm.buf[:self.size] = b'\x00' * self.size
        self.shm.buf[:len(data)] = data

    def close(self):
        try:
            self.shm.close()
            self.shm.unlink()
            log.info("GPS shared memory released")
        except Exception as e:
            log.debug(f"GPS shared memory cleanup: {e}")


# ---------------------------------------------------------------------------
# CSV Logger
# ---------------------------------------------------------------------------

class GPSLogger:
    """Log GPS data to CSV."""

    def __init__(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        path = os.path.join(output_dir, f"gps_log_{timestamp}.csv")

        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp_mono_ns", "utc_datetime",
            "latitude", "longitude", "altitude_m",
            "speed_knots", "speed_kmh", "heading",
            "satellites", "fix_quality", "fix_valid",
            "hdop", "pdop", "vdop",
        ])
        self.path = path
        log.info(f"GPS log: {path}")

    def write(self, state):
        self.writer.writerow([
            state.get("timestamp_mono_ns", ""),
            state.get("utc_datetime", ""),
            state.get("latitude", ""),
            state.get("longitude", ""),
            state.get("altitude_m", ""),
            state.get("speed_knots", ""),
            state.get("speed_kmh", ""),
            state.get("heading", ""),
            state.get("satellites", ""),
            state.get("fix_quality", ""),
            state.get("fix_valid", ""),
            state.get("hdop", ""),
            state.get("pdop", ""),
            state.get("vdop", ""),
        ])

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.close()
        log.info(f"GPS log closed: {self.path}")


# ---------------------------------------------------------------------------
# System time sync
# ---------------------------------------------------------------------------

def sync_system_time(utc_datetime_str):
    """Set system clock from GPS UTC time.

    Args:
        utc_datetime_str: ISO format string (e.g. "2026-06-15T10:30:00Z")

    Returns:
        True if successful
    """
    try:
        result = subprocess.run(
            ["sudo", "date", "-s", utc_datetime_str],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            log.info(f"System clock synced to GPS: {utc_datetime_str}")
            return True
        else:
            log.warning(f"Time sync failed: {result.stderr.strip()}")
            return False
    except Exception as e:
        log.warning(f"Time sync error: {e}")
        return False


# ---------------------------------------------------------------------------
# Main GPS loop
# ---------------------------------------------------------------------------

def gps_loop(ser, publisher, logger, set_time, duration_s):
    """Main GPS reading loop.

    Args:
        ser: Serial port instance
        publisher: GPSSharedMemory instance
        logger: GPSLogger instance or None
        set_time: Sync system clock on first valid fix
        duration_s: Run duration (0 = infinite)
    """
    running = True
    time_synced = False
    fix_count = 0
    no_fix_count = 0

    # Accumulated state from multiple NMEA sentences
    state = {
        "fix_valid": False,
        "fix_quality": 0,
        "fix_type": 1,
        "satellites": 0,
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude_m": 0.0,
        "speed_knots": 0.0,
        "speed_kmh": 0.0,
        "heading": 0.0,
        "utc_time": "",
        "utc_date": "",
        "utc_datetime": "",
        "hdop": 0.0,
        "pdop": 0.0,
        "vdop": 0.0,
        "timestamp_mono_ns": 0,
    }

    def handle_signal(signum, frame):
        nonlocal running
        log.info(f"Signal {signum} received, stopping...")
        running = False

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    log.info("GPS reading started")
    start_time = time.monotonic()

    # Report interval
    last_report = time.monotonic()
    report_interval = 5.0

    try:
        while running:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
            except (serial.SerialException, OSError) as e:
                log.error(f"Serial read error: {e}")
                time.sleep(1)
                continue

            if not line or not line.startswith("$"):
                continue

            if not verify_checksum(line):
                log.debug(f"Checksum failed: {line}")
                continue

            fields = line.split(",")
            sentence_type = fields[0]

            # Parse known sentence types
            if sentence_type in ("$GNGGA", "$GPGGA"):
                gga = parse_gngga(fields)
                state.update(gga)

            elif sentence_type in ("$GNRMC", "$GPRMC"):
                rmc = parse_gnrmc(fields)
                state.update(rmc)
                state["timestamp_mono_ns"] = time.monotonic_ns()

                # Publish after RMC (last in typical NMEA cycle)
                publisher.publish(state)

                # Log to CSV
                if logger:
                    logger.write(state)
                    if fix_count % 10 == 0:
                        logger.flush()

                # Track fixes
                if state.get("fix_valid"):
                    fix_count += 1
                    no_fix_count = 0

                    # Sync system time on first valid fix
                    if set_time and not time_synced and state.get("utc_datetime"):
                        if sync_system_time(state["utc_datetime"]):
                            time_synced = True
                else:
                    no_fix_count += 1

            elif sentence_type in ("$GNGSA", "$GPGSA"):
                gsa = parse_gngsa(fields)
                state.update(gsa)

            # Periodic status report
            now = time.monotonic()
            if now - last_report >= report_interval:
                if state.get("fix_valid"):
                    log.info(
                        f"FIX: lat={state['latitude']:.6f} "
                        f"lon={state['longitude']:.6f} "
                        f"alt={state['altitude_m']:.1f}m | "
                        f"sats={state['satellites']} "
                        f"hdop={state['hdop']:.1f} | "
                        f"speed={state['speed_kmh']:.1f}km/h "
                        f"heading={state['heading']:.0f}° | "
                        f"UTC={state.get('utc_datetime', 'N/A')}"
                    )
                else:
                    log.info(
                        f"NO FIX | sats={state['satellites']} "
                        f"quality={state['fix_quality']} | "
                        f"Waiting for satellite lock..."
                    )
                last_report = now

            # Duration check
            if duration_s > 0 and (now - start_time) >= duration_s:
                log.info("Duration reached, stopping.")
                break

    except Exception as e:
        log.error(f"GPS loop error: {e}", exc_info=True)

    finally:
        total = time.monotonic() - start_time
        log.info(f"Stopped. Fixes: {fix_count}, duration: {total:.1f}s")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE GPS Reader — Waveshare L76K",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                              # Default /dev/ttyAMA0 @ 9600
  %(prog)s -p /dev/serial0              # Custom port
  %(prog)s --set-time                   # Sync system clock from GPS
  %(prog)s --log data/gps_test          # Log to CSV
  %(prog)s --rate 5 -d 60              # 5 Hz for 60 seconds
        """
    )

    parser.add_argument("-p", "--port", type=str, default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--rate", type=int, default=1,
                        choices=[1, 2, 5, 10],
                        help="GPS update rate in Hz (default: 1)")
    parser.add_argument("--set-time", action="store_true",
                        help="Sync system clock from GPS on first fix")
    parser.add_argument("--log", type=str, default=None,
                        help="Log GPS data to CSV in given directory")
    parser.add_argument("-d", "--duration", type=float, default=0,
                        help="Run duration in seconds (0 = infinite)")
    parser.add_argument("--cold-start", action="store_true",
                        help="Force cold start (full satellite search)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 40)
    log.info("SPIRE GPS Reader — L76K")
    log.info("=" * 40)

    # Open serial port
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=1.0,
        )
        log.info(f"Serial port: {args.port} @ {args.baud} baud")
    except serial.SerialException as e:
        log.error(f"Cannot open {args.port}: {e}")
        sys.exit(1)

    # Configure GPS module
    time.sleep(0.5)

    if args.cold_start:
        send_pmtk(ser, PMTK_COLD_START)
        log.info("Cold start initiated (may take 30-60s for first fix)")
        time.sleep(1.0)

    # Set update rate
    rate_cmds = {1: PMTK_FIX_1HZ, 2: PMTK_FIX_2HZ,
                 5: PMTK_FIX_5HZ, 10: PMTK_FIX_10HZ}
    send_pmtk(ser, rate_cmds[args.rate])
    log.info(f"Update rate: {args.rate} Hz")

    # Enable GPS + GLONASS + BeiDou
    send_pmtk(ser, PMTK_GNSS_GPS_GLONASS_BEIDOU)

    # Configure NMEA output (GGA + RMC + GSA only)
    send_pmtk(ser, PMTK_NMEA_OUTPUT)
    time.sleep(0.5)

    # Shared memory
    publisher = GPSSharedMemory()

    # CSV logger
    logger = None
    if args.log:
        logger = GPSLogger(args.log)

    try:
        gps_loop(ser, publisher, logger, args.set_time, args.duration)
    finally:
        if logger:
            logger.close()
        publisher.close()
        ser.close()
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()
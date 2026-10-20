#!/usr/bin/env python3
"""
Live I2C continuity monitor for SPIRE.

Polls every expected sensor with a real register read (not just an address
probe) and reports OK/FAIL live, plus a running error count per device.
Use it while wiggling the harness to find intermittent contacts before flight.

Usage:
    python3 i2c_watch.py            # poll all known sensors, ~5 Hz
    python3 i2c_watch.py -r 20      # 20 Hz
    python3 i2c_watch.py -b 1       # explicit bus number

Ctrl+C prints a summary: total polls, error count and worst dropout streak.
"""

import argparse
import time
from smbus2 import SMBus, i2c_msg

# addr -> (name, probe_kind)
#   'reg'  : plain register read (most sensors)
#   'mprls': MPR pressure sensor, needs i2c_rdwr 4-byte read
SENSORS = {
    0x18: ("MPRLS  pressure", "mprls"),
    0x1E: ("LSM9DS1 mag    ", "reg"),
    0x41: ("0x41   device  ", "reg"),
    0x69: ("ICM-20948 imu  ", "reg"),
    0x6B: ("LSM9DS1 acc/gyr", "reg"),
}

# WHO_AM_I-ish register that is safe to read on each device
PROBE_REG = {
    0x1E: 0x0F,   # LSM9DS1 mag WHO_AM_I
    0x41: 0x00,   # INA-style config register
    0x69: 0x00,   # ICM-20948 WHO_AM_I (bank 0)
    0x6B: 0x0F,   # LSM9DS1 ag WHO_AM_I
}

GREEN = "\033[32m"
RED = "\033[31m"
DIM = "\033[2m"
RESET = "\033[0m"
CLEAR = "\033[2J\033[H"


def probe(bus, addr, kind):
    """Return True if a real read succeeds."""
    try:
        if kind == "mprls":
            read = i2c_msg.read(addr, 4)
            bus.i2c_rdwr(read)
            return len(list(read)) == 4
        bus.read_byte_data(addr, PROBE_REG.get(addr, 0x00))
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bus", type=int, default=1)
    ap.add_argument("-r", "--rate", type=float, default=5.0,
                    help="polls per second (default 5)")
    args = ap.parse_args()

    stats = {a: {"err": 0, "streak": 0, "worst": 0} for a in SENSORS}
    polls = 0
    start = time.monotonic()
    period = 1.0 / args.rate

    print("Wiggle the harness. Ctrl+C to stop.\n")
    try:
        with SMBus(args.bus) as bus:
            while True:
                polls += 1
                lines = []
                for addr, (name, kind) in sorted(SENSORS.items()):
                    ok = probe(bus, addr, kind)
                    s = stats[addr]
                    if ok:
                        s["streak"] = 0
                        mark = f"{GREEN}OK  {RESET}"
                    else:
                        s["err"] += 1
                        s["streak"] += 1
                        s["worst"] = max(s["worst"], s["streak"])
                        mark = f"{RED}FAIL{RESET}"
                    rate = 100.0 * (polls - s["err"]) / polls
                    lines.append(
                        f"  0x{addr:02X}  {name}  {mark}   "
                        f"errors {s['err']:<5d} uptime {rate:6.2f}%  "
                        f"worst gap {s['worst']}"
                    )

                elapsed = time.monotonic() - start
                print(CLEAR, end="")
                print(f"SPIRE I2C continuity  bus {args.bus}  "
                      f"{args.rate:g} Hz  polls {polls}  "
                      f"{elapsed:6.1f}s\n")
                print("\n".join(lines))
                print(f"\n{DIM}Any FAIL or rising error count means an "
                      f"intermittent contact.{RESET}")
                time.sleep(period)

    except KeyboardInterrupt:
        elapsed = time.monotonic() - start
        print("\n\n=== summary ===")
        print(f"duration {elapsed:.1f}s, {polls} polls per device\n")
        clean = True
        for addr, (name, _) in sorted(SENSORS.items()):
            s = stats[addr]
            rate = 100.0 * (polls - s["err"]) / polls if polls else 0.0
            flag = "" if s["err"] == 0 else "   <-- INTERMITTENT"
            if s["err"]:
                clean = False
            print(f"  0x{addr:02X}  {name}  errors {s['err']:<5d} "
                  f"uptime {rate:6.2f}%  worst gap {s['worst']}{flag}")
        print()
        print("All devices solid." if clean else
              "Fix the flagged connections before flight.")


if __name__ == "__main__":
    main()
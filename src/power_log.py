#!/usr/bin/env python3
"""
SPIRE power logger (Test A and Test E).

Samples, at a fixed rate, everything the Pi can measure about its own supply
and writes one CSV row per sample:

  ext5v_v          Pi 5 input voltage from the PMIC (vcgencmd pmic_read_adc EXT5V_V)
  throttled_hex    vcgencmd get_throttled (bit 0 under-voltage now, bit 2
                   throttled now, bit 16 under-voltage since boot, bit 18
                   throttled since boot)
  soc_temp_c       SoC temperature from /sys/class/thermal
  pack_v           UPS battery pack voltage, INA219 bus voltage (0x41, reg 0x02)
  shunt_mv         INA219 shunt voltage (reg 0x01), signed
  pack_a, pack_w   current and power, only if --shunt-ohm is given (the shunt
                   value of the UPS board; read the marking on the resistor,
                   e.g. R010 = 0.010 ohm)

The INA219 is only read, never configured, so a vendor UPS script that set it
up is not disturbed. The servo rail cannot be measured by the Pi; film the
multimeter instead (Test A1).

Rows are flushed every sample and fsynced once a second, so a power cut loses
at most the last second (Test E-9). Ctrl+C prints a summary.

Usage:
  python3 src/power_log.py                       # 5 Hz, file on the USB drive
  python3 src/power_log.py --shunt-ohm 0.01 --tag A1
  python3 src/power_log.py --rate 2 --duration 1800 --tag A2_idle
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage  # noqa: E402

INA219_ADDR = 0x41
REG_SHUNT = 0x01
REG_BUS = 0x02
THERMAL = "/sys/class/thermal/thermal_zone0/temp"

FIELDS = ["utc", "mono_s", "ext5v_v", "throttled_hex", "uv_now", "uv_since_boot",
          "thr_now", "thr_since_boot", "soc_temp_c", "pack_v", "shunt_mv",
          "pack_a", "pack_w"]


def vcgencmd(*args):
    try:
        out = subprocess.run(["vcgencmd", *args], capture_output=True,
                             text=True, timeout=1.0)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def read_ext5v():
    m = re.search(r"=\s*([\d.]+)\s*V", vcgencmd("pmic_read_adc", "EXT5V_V"))
    return float(m.group(1)) if m else None


def read_throttled():
    m = re.search(r"0x[0-9a-fA-F]+", vcgencmd("get_throttled"))
    return int(m.group(0), 16) if m else None


def read_temp():
    try:
        with open(THERMAL) as fh:
            return int(fh.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


class INA219Reader:
    """Read-only access to the UPS INA219 (bus and shunt voltage)."""

    def __init__(self, bus=1, addr=INA219_ADDR):
        self.addr = addr
        self.bus = None
        try:
            from smbus2 import SMBus
            self.bus = SMBus(bus)
        except Exception as e:
            print(f"INA219 unavailable ({e}); pack columns stay empty")

    def _word(self, reg):
        raw = self.bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)      # INA219 is big-endian

    def read(self):
        """(pack_v, shunt_mv) or (None, None) on an I2C error."""
        if self.bus is None:
            return None, None
        try:
            bus_v = (self._word(REG_BUS) >> 3) * 0.004
            sh = self._word(REG_SHUNT)
            if sh & 0x8000:
                sh -= 1 << 16
            return bus_v, sh * 0.01                  # 10 uV per bit -> mV
        except OSError:
            return None, None


def fmt(v, f):
    return "" if v is None else format(v, f)


def main():
    ap = argparse.ArgumentParser(description="SPIRE power logger")
    ap.add_argument("-r", "--rate", type=float, default=5.0,
                    help="Samples per second (default 5)")
    ap.add_argument("-d", "--duration", type=float, default=0,
                    help="Stop after this many seconds (0 = until Ctrl+C)")
    ap.add_argument("--shunt-ohm", type=float, default=None,
                    help="UPS INA219 shunt resistance in ohm, enables current "
                         "and power columns")
    ap.add_argument("--tag", default="",
                    help="Short label added to the file name, e.g. A1 or A2_idle")
    ap.add_argument("-o", "--output", default=None,
                    help="Output folder (default: /mnt/spire_data/power)")
    ap.add_argument("--allow-local", action="store_true",
                    help="Write to ./data/power if the USB drive is missing")
    args = ap.parse_args()

    out_dir = args.output or storage.data_dir("power", allow_local=args.allow_local)
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    name = f"power_{stamp}{'_' + args.tag if args.tag else ''}.csv"
    path = os.path.join(out_dir, name)

    ina = INA219Reader()
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    period = 1.0 / args.rate
    t0 = time.monotonic()
    last_sync = t0
    n = 0
    stats = {"ext5v": [], "temp": [], "pack_v": [], "pack_w": [],
             "uv_now": 0, "thr_now": 0, "thr_first": None, "thr_last": None}
    first_pack = last_pack = None

    print(f"Logging to {path} at {args.rate:g} Hz. Ctrl+C to stop.", flush=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(FIELDS)
        next_t = t0
        while running:
            now = time.monotonic()
            ext5v = read_ext5v()
            thr = read_throttled()
            temp = read_temp()
            pack_v, shunt_mv = ina.read()
            pack_a = pack_w = None
            if args.shunt_ohm and shunt_mv is not None:
                pack_a = shunt_mv / 1000.0 / args.shunt_ohm
                if pack_v is not None:
                    pack_w = pack_a * pack_v
            bits = lambda b: "" if thr is None else int(bool(thr & (1 << b)))
            w.writerow([datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                        f"{now - t0:.3f}", fmt(ext5v, ".4f"),
                        "" if thr is None else f"0x{thr:x}",
                        bits(0), bits(16), bits(2), bits(18),
                        fmt(temp, ".1f"), fmt(pack_v, ".3f"), fmt(shunt_mv, ".2f"),
                        fmt(pack_a, ".4f"), fmt(pack_w, ".3f")])
            fh.flush()
            if now - last_sync >= 1.0:
                os.fsync(fh.fileno())
                last_sync = now
            n += 1

            if ext5v is not None:
                stats["ext5v"].append(ext5v)
            if temp is not None:
                stats["temp"].append(temp)
            if pack_v is not None:
                stats["pack_v"].append(pack_v)
                first_pack = first_pack if first_pack is not None else pack_v
                last_pack = pack_v
            if pack_w is not None:
                # Sign depends on how the shunt sits on the board; the CSV
                # keeps it, the summary uses the magnitude.
                stats["pack_w"].append(abs(pack_w))
            if thr is not None:
                stats["uv_now"] += bool(thr & 0x1)
                stats["thr_now"] += bool(thr & 0x4)
                if stats["thr_first"] is None:
                    stats["thr_first"] = thr
                stats["thr_last"] = thr

            if n % max(1, int(args.rate)) == 0:
                print(f"\r{now - t0:7.1f} s  EXT5V {fmt(ext5v, '.3f')} V  "
                      f"throttled {'' if thr is None else hex(thr)}  "
                      f"SoC {fmt(temp, '.1f')} C  pack {fmt(pack_v, '.2f')} V"
                      f"{'  ' + fmt(pack_w, '.2f') + ' W' if pack_w is not None else ''}   ",
                      end="", flush=True)

            if args.duration and now - t0 >= args.duration:
                break
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
        os.fsync(fh.fileno())

    dur = time.monotonic() - t0
    print(f"\n\nSummary ({path})")
    print(f"  duration            {dur / 60:.1f} min, {n} samples")
    if stats["ext5v"]:
        e = stats["ext5v"]
        print(f"  EXT5V_V             min {min(e):.3f}  mean {sum(e) / len(e):.3f} V"
              f"   (A1: min >= 4.95 V)")
    if stats["thr_first"] is not None:
        print(f"  under-voltage now   {stats['uv_now']} samples, throttled now "
              f"{stats['thr_now']} samples   (A1: 0)")
        print(f"  get_throttled       0x{stats['thr_first']:x} at start -> "
              f"0x{stats['thr_last']:x} at end   (A1/E-2: 0x0 from a fresh boot)")
    if stats["temp"]:
        print(f"  SoC temperature     max {max(stats['temp']):.1f} C   (E-6: < 80 C)")
    if first_pack is not None:
        print(f"  UPS pack            {first_pack:.2f} V -> {last_pack:.2f} V")
    if stats["pack_w"]:
        p = sum(stats["pack_w"]) / len(stats["pack_w"])
        print(f"  UPS pack power      mean {p:.2f} W, energy {p * dur / 3600:.2f} Wh "
              f"({p:.2f} Wh per hour)")
    elif args.shunt_ohm is None:
        print("  UPS pack power      n/a (pass --shunt-ohm to get current and power)")


if __name__ == "__main__":
    main()

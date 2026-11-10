#!/usr/bin/env python3
"""
SPIRE Battery Monitor
Waveshare UPS Module 3S, INA219 over I2C.

Live, in-place view in the terminal (one compact block that refreshes,
no scrolling): charge bar, voltage, current with direction, power, and the
time to empty (discharging) or to full (charging).

Sign convention: positive current = charging, negative = discharging
(matches the INA219 shunt sign seen in power_log.py: Pi on battery gave a
negative shunt voltage).

Time estimates:
  - Current is averaged over the last --avg-window seconds, so short load
    spikes (camera, servo) do not make the estimate jump.
  - State of charge is coulomb-counted from a starting value taken from the
    rest-voltage table (3S Li-ion); the voltage-only value is shown next to
    it. While charging, the voltage reads high (charger pushes it up), so
    the voltage-only value overestimates; the counted one does not.
  - Time to full assumes constant current to 100 %. The final constant-
    voltage phase (current tapering above ~12.45 V) is not modelled, so the
    real time is longer; the view says so when the pack is in that phase.
  - Capacity is the nominal value (--capacity); estimates scale with it.

Usage:
  python3 battery_monitor.py                     # live view, 1 s refresh
  python3 battery_monitor.py --capacity 3000     # your cells' capacity
  python3 battery_monitor.py --log data/battery  # also write a CSV
  python3 battery_monitor.py --plain             # one line per sample (no live view)
  python3 battery_monitor.py --calibrate         # find the shunt value
"""

import argparse
import csv
import logging
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime, timezone

import smbus2

INA219_ADDR = 0x41
I2C_BUS = 1

REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

# 3S Li-ion rest voltage -> state of charge (typical curve, moderate load)
VOLTAGE_SOC_TABLE = [
    (12.60, 100), (12.45, 95), (12.33, 90), (12.18, 85), (12.06, 80),
    (11.94, 75), (11.82, 70), (11.70, 65), (11.58, 60), (11.46, 55),
    (11.34, 50), (11.22, 45), (11.10, 40), (10.98, 35), (10.86, 30),
    (10.74, 25), (10.50, 20), (10.20, 15), (9.90, 10), (9.60, 5), (9.00, 0),
]
CV_PHASE_V = 12.45          # above this while charging: constant-voltage taper
IDLE_MA = 30                # |current| below this: neither charging nor discharging

log = logging.getLogger("spire.battery")


def setup_logging(verbose=False):
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
    h = logging.StreamHandler(sys.stdout)
    h.setLevel(logging.DEBUG if verbose else logging.INFO)
    h.setFormatter(fmt)
    log.setLevel(logging.DEBUG)
    log.addHandler(h)


def voltage_to_soc(v):
    if v >= VOLTAGE_SOC_TABLE[0][0]:
        return 100.0
    if v <= VOLTAGE_SOC_TABLE[-1][0]:
        return 0.0
    for (vh, sh), (vl, sl) in zip(VOLTAGE_SOC_TABLE, VOLTAGE_SOC_TABLE[1:]):
        if vl <= v <= vh:
            return sl + (v - vl) / (vh - vl) * (sh - sl)
    return 0.0


class INA219:
    def __init__(self, bus_num=I2C_BUS, address=INA219_ADDR, shunt_ohms=0.01, max_current_a=5.0):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        config = 0x2000 | 0x1800 | 0x0078 | 0x0780 | 0x0007   # 32 V, ±320 mV, 128x avg, continuous
        self._write(REG_CONFIG, config)
        self.current_lsb = max_current_a / 32768.0
        self._write(REG_CALIBRATION, int(0.04096 / (self.current_lsb * shunt_ohms)))

    def _read(self, reg):
        raw = self.bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def _write(self, reg, value):
        self.bus.write_word_data(self.addr, reg, ((value & 0xFF) << 8) | (value >> 8))

    def voltage(self):
        return (self._read(REG_BUS_VOLTAGE) >> 3) * 0.004

    def shunt_mv(self):
        raw = self._read(REG_SHUNT_VOLTAGE)
        return (raw - 65536 if raw > 32767 else raw) * 0.01

    def current_ma(self):
        """Signed current from the shunt voltage (+ charging, - discharging)."""
        return self.shunt_mv() / self.shunt_ohms

    def close(self):
        self.bus.close()


class BatteryState:
    """Averaged current, coulomb-counted charge and time estimates."""

    def __init__(self, capacity_mah, avg_window_s):
        self.capacity = capacity_mah
        self.window = avg_window_s
        self.samples = deque()                 # (t, current_ma)
        self.t_start = self.t_last = None
        self.soc = None                        # coulomb-counted, %
        self.mah = 0.0                         # charge moved since start (+ in)
        self.mwh = 0.0

    def update(self, t, v, i_ma):
        if self.t_start is None:
            self.t_start = self.t_last = t
            self.soc = voltage_to_soc(v)
        dt_h = (t - self.t_last) / 3600.0
        self.t_last = t
        self.mah += i_ma * dt_h
        self.mwh += v * i_ma * dt_h
        self.soc = min(100.0, max(0.0, self.soc + 100.0 * i_ma * dt_h / self.capacity))
        self.samples.append((t, i_ma))
        while self.samples and t - self.samples[0][0] > self.window:
            self.samples.popleft()

    def avg_ma(self):
        return sum(i for _, i in self.samples) / len(self.samples) if self.samples else 0.0

    def estimate(self, v):
        """(state, minutes or None, note)."""
        i = self.avg_ma()
        if i > IDLE_MA:
            note = "constant-voltage phase: current tapering, real time longer" if v >= CV_PHASE_V \
                else "to 100 % at this current (final taper not included)"
            return "CHARGING", 60.0 * self.capacity * (100.0 - self.soc) / 100.0 / i, note
        if i < -IDLE_MA:
            return "DISCHARGING", 60.0 * self.capacity * self.soc / 100.0 / -i, "to 0 % at this load"
        return "IDLE", None, "current too small to estimate"


def fmt_minutes(m):
    if m is None:
        return "--"
    m = int(round(m))
    return f"{m // 60} h {m % 60:02d} min" if m >= 60 else f"{m} min"


def bar(pct, width=24):
    n = int(round(pct / 100.0 * width))
    return "█" * n + "░" * (width - n)


class LiveView:
    """Redraws a fixed block of lines in place (ANSI cursor movement)."""

    def __init__(self):
        self.lines = 0

    def draw(self, lines):
        out = sys.stdout
        if self.lines:
            out.write(f"\033[{self.lines}F")          # cursor up to the block start
        for ln in lines:
            out.write("\033[2K" + ln + "\n")          # clear line, write
        for _ in range(self.lines - len(lines)):
            out.write("\033[2K\n")
        self.lines = max(self.lines, len(lines))
        out.flush()


def render(addr, v, i_inst, st):
    state, minutes, note = st.estimate(v)
    i_avg = st.avg_ma()
    arrow = {"CHARGING": "▲ charging", "DISCHARGING": "▼ discharging", "IDLE": "■ idle"}[state]
    label = {"CHARGING": "Full in ", "DISCHARGING": "Empty in", "IDLE": "Time    "}[state]
    elapsed = int(st.t_last - st.t_start)
    warn = ""
    if state == "DISCHARGING" and st.soc < 10:
        warn = "  !! CRITICAL: shut down now"
    elif state == "DISCHARGING" and st.soc < 20:
        warn = "  ! LOW: plan shutdown"
    return [
        f"SPIRE battery · UPS 3S · INA219 0x{addr:02X} · {datetime.now():%H:%M:%S}",
        f"[{bar(st.soc)}] {st.soc:5.1f} %   {arrow}{warn}",
        f"V {v:6.2f} V   I {i_avg / 1000:+6.2f} A (now {i_inst / 1000:+.2f})   "
        f"P {abs(v * i_avg) / 1000:5.1f} W",
        f"{label} {fmt_minutes(minutes):>13s}   ({note})",
        f"SoC by voltage {voltage_to_soc(v):5.1f} %   session {elapsed // 60} min {elapsed % 60:02d} s   "
        f"{st.mah:+.0f} mAh  {st.mwh / 1000:+.2f} Wh",
    ]


class CsvLog:
    def __init__(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(
            out_dir, f"battery_log_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.csv")
        self.fh = open(self.path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["utc", "voltage_v", "current_ma", "current_avg_ma", "soc_counted_pct",
                         "soc_voltage_pct", "state", "minutes_left", "mah_moved", "mwh_moved"])

    def write(self, v, i, st):
        state, minutes, _ = st.estimate(v)
        self.w.writerow([datetime.now(timezone.utc).isoformat(), f"{v:.3f}", f"{i:.1f}",
                         f"{st.avg_ma():.1f}", f"{st.soc:.2f}", f"{voltage_to_soc(v):.2f}", state,
                         "" if minutes is None else f"{minutes:.1f}", f"{st.mah:.1f}",
                         f"{st.mwh:.1f}"])
        self.fh.flush()

    def close(self):
        self.fh.close()


def calibrate_shunt(address, bus_num=I2C_BUS):
    ina = INA219(bus_num, address)
    mv = ina.shunt_mv()
    print(f"Shunt voltage: {mv:.3f} mV")
    try:
        actual = float(input("Current measured with a multimeter (mA, positive): "))
        if actual > 0:
            print(f"Shunt resistance: {abs(mv) / actual:.4f} ohm  "
                  f"-> python3 battery_monitor.py --shunt {abs(mv) / actual:.4f}")
    except ValueError:
        print("Invalid input.")
    ina.close()


def main():
    ap = argparse.ArgumentParser(description="SPIRE battery monitor (Waveshare UPS 3S)")
    ap.add_argument("-i", "--interval", type=float, default=1.0, help="refresh period [s]")
    ap.add_argument("-d", "--duration", type=float, default=0, help="stop after [s], 0 = never")
    ap.add_argument("--avg-window", type=float, default=60.0,
                    help="current averaging window for the time estimate [s]")
    ap.add_argument("--capacity", type=int, default=3500, help="pack capacity [mAh]")
    ap.add_argument("--shunt", type=float, default=0.01, help="shunt resistance [ohm]")
    ap.add_argument("--address", type=lambda x: int(x, 0), default=INA219_ADDR)
    ap.add_argument("--log", help="write a CSV into this directory")
    ap.add_argument("--plain", action="store_true", help="one line per sample, no live view")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    setup_logging(args.verbose)

    if args.calibrate:
        calibrate_shunt(args.address)
        return

    ina = INA219(address=args.address, shunt_ohms=args.shunt)
    st = BatteryState(args.capacity, args.avg_window)
    logger = CsvLog(args.log) if args.log else None
    live = None if (args.plain or not sys.stdout.isatty()) else LiveView()

    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if live:
        sys.stdout.write("\033[?25l")                 # hide cursor
    t0 = time.monotonic()
    try:
        while running:
            t = time.monotonic()
            v, i = ina.voltage(), ina.current_ma()
            st.update(t, v, i)
            if logger:
                logger.write(v, i, st)
            if live:
                live.draw(render(args.address, v, i, st))
            else:
                state, minutes, _ = st.estimate(v)
                log.info(f"{st.soc:5.1f} % {state:11s} {v:.2f} V {st.avg_ma():+.0f} mA "
                         f"{'full in' if state == 'CHARGING' else 'empty in'} {fmt_minutes(minutes)}")
            if args.duration and t - t0 >= args.duration:
                break
            time.sleep(max(0.0, args.interval - (time.monotonic() - t)))
    finally:
        if live:
            sys.stdout.write("\033[?25h")             # show cursor again
            sys.stdout.flush()
        if logger:
            logger.close()
            print(f"CSV: {logger.path}")
        ina.close()


if __name__ == "__main__":
    main()
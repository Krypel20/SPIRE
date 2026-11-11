#!/usr/bin/env python3
"""
I2C bus clear for SPIRE (Raspberry Pi 5, bus 1: SDA = GPIO2 / pin 3, SCL = GPIO3 / pin 5).

When a glitch on SCL desynchronises a sensor in the middle of a read, the
sensor keeps SDA low while it waits for clock pulses that never come. The
kernel then reports "SDA stuck at low" / "controller timed out" and the whole
bus is dead until the sensor is unplugged or power cycled.

The standard fix is the I2C "bus clear" (NXP UM10204, I2C-bus specification,
section 3.1.16): the master sends up to 9 clock pulses so the stuck device can
finish its byte and release SDA, then a STOP condition.

This script does that by taking GPIO2/3 away from the I2C block for a moment
with `pinctrl`, then gives them back (alt function a3, pull-up), so the kernel
driver can use the bus again. Lines are only ever pulled LOW actively; HIGH is
"input + pull-up" (open-drain emulation), so the Pi never drives a line high
against a device that holds it low.

Usage:
  python3 src/i2c_bus_clear.py          # clear only if SDA or SCL reads low
  python3 src/i2c_bus_clear.py --force  # always send the 9 clocks + STOP
"""

import argparse
import subprocess
import sys
import time

SDA = 2   # GPIO2, physical pin 3
SCL = 3   # GPIO3, physical pin 5
HALF_PERIOD_S = 0.0005   # clock speed is irrelevant here; pinctrl calls are slower anyway


def pinctrl(*args):
    r = subprocess.run(["pinctrl", *map(str, args)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"pinctrl {' '.join(map(str, args))} failed: "
                           f"{r.stderr.strip() or r.stdout.strip()} (try with sudo)")
    return r.stdout


def level(gpio):
    """Return 'hi' or 'lo' as read on the pad."""
    out = pinctrl("get", gpio)
    return "lo" if "| lo" in out else "hi"


def release(gpio):
    pinctrl("set", gpio, "ip", "pu")       # let the pull-up take the line high


def pull_low(gpio):
    pinctrl("set", gpio, "op", "dl")       # actively drive low


def bus_clear(force=False, verbose=True):
    """Run the bus clear. Returns True if both lines read high afterwards."""
    sda, scl = level(SDA), level(SCL)
    if verbose:
        print(f"before: SDA {sda}  SCL {scl}")
    if sda == "hi" and scl == "hi" and not force:
        if verbose:
            print("both lines high: nothing to clear")
        return True
    try:
        release(SDA)
        release(SCL)
        time.sleep(HALF_PERIOD_S)
        for i in range(9):
            if level(SDA) == "hi" and i > 0:
                break                      # device let go of SDA
            pull_low(SCL)
            time.sleep(HALF_PERIOD_S)
            release(SCL)
            time.sleep(HALF_PERIOD_S)
        # STOP: SDA low -> high while SCL is high
        pull_low(SCL)
        time.sleep(HALF_PERIOD_S)
        pull_low(SDA)
        time.sleep(HALF_PERIOD_S)
        release(SCL)
        time.sleep(HALF_PERIOD_S)
        release(SDA)
        time.sleep(HALF_PERIOD_S)
    finally:
        # hand the pins back to the I2C block
        pinctrl("set", SDA, "a3", "pu")
        pinctrl("set", SCL, "a3", "pu")
    sda, scl = level(SDA), level(SCL)
    if verbose:
        print(f"after:  SDA {sda}  SCL {scl}")
    return sda == "hi" and scl == "hi"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true",
                    help="send the clocks even if both lines already read high")
    args = ap.parse_args()
    try:
        ok = bus_clear(force=args.force)
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(2)
    print("bus released" if ok else "line still low: a device holds it (unplug / power cycle)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

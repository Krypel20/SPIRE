#!/usr/bin/env python3
"""
MPRLS (Honeywell MPR, Adafruit 3965, 0-25 PSI / 0-172 kPa) test and accuracy check.

The MPR series does NOT use register addressing. Communication is:
  1. Write the 3-byte measurement command [0xAA, 0x00, 0x00] in one transaction.
  2. Wait for the conversion to finish (poll the busy bit in the status byte).
  3. Read 4 bytes: status + 24-bit raw pressure count.

This is why a plain smbus2 write_byte() fails with Remote I/O error: it sends
a single byte and the sensor NACKs it. We use i2c_rdwr / i2c_msg for proper
multi-byte transactions instead.

Wiring (RPi 5, I2C bus 1):
  VIN -> 3.3V (pin 1)      GND -> GND (pin 6)
  SDA -> GPIO 2 (pin 3)    SCL -> GPIO 3 (pin 5)
  EOC, RST -> not connected (not needed)
"""

import time
import statistics
from smbus2 import SMBus, i2c_msg

I2C_BUS = 1
MPRLS_ADDR = 0x18

# Status byte bits (Honeywell MPR datasheet)
STATUS_POWER = 0x40      # device powered
STATUS_BUSY = 0x20       # measurement in progress
STATUS_INTEGRITY = 0x04  # memory integrity test failed
STATUS_MATH_SAT = 0x01   # math saturation (pressure out of range)

# Transfer-function constants for the 0-25 PSI variant, 10%-90% calibration.
OUTPUT_MIN = 0x19999A    # 10% of 2^24  = 1677722 counts
OUTPUT_MAX = 0xE66666    # 90% of 2^24  = 15099494 counts
PSI_MIN = 0.0
PSI_MAX = 25.0
PSI_TO_HPA = 68.947572932  # 1 psi in hectopascal


def read_pressure(bus, addr=MPRLS_ADDR, timeout_s=0.1, retries=3):
    """Trigger one measurement and return (status, raw_counts).

    Raises RuntimeError on integrity/saturation fault or persistent zero data.

    The busy bit can read low immediately after the command, before the sensor
    has latched it, so we wait the minimum conversion time first and then poll.
    A raw count of 0 is never a physical reading (0 PSI maps to OUTPUT_MIN, not
    0), so it signals a timing glitch and we retry.
    """
    for _ in range(retries):
        # 1. Start measurement: command 0xAA + two 0x00 argument bytes.
        bus.i2c_rdwr(i2c_msg.write(addr, [0xAA, 0x00, 0x00]))

        # 2. Wait the minimum conversion time, then poll until busy clears.
        time.sleep(0.005)
        deadline = time.monotonic() + timeout_s
        while True:
            read = i2c_msg.read(addr, 4)
            bus.i2c_rdwr(read)
            data = list(read)
            status = data[0]

            if status & STATUS_INTEGRITY:
                raise RuntimeError("MPRLS integrity test failed (status 0x%02X)" % status)
            if status & STATUS_MATH_SAT:
                raise RuntimeError("MPRLS math saturation (pressure out of range)")
            if not (status & STATUS_BUSY):
                break
            if time.monotonic() > deadline:
                raise RuntimeError("MPRLS conversion timeout (status 0x%02X)" % status)
            time.sleep(0.001)

        # 3. Assemble 24-bit pressure count.
        raw = (data[1] << 16) | (data[2] << 8) | data[3]
        if raw != 0:
            return status, raw
        time.sleep(0.005)  # zero data: timing glitch, retry

    raise RuntimeError("MPRLS returned zero data after %d attempts" % retries)


def counts_to_hpa(raw):
    """Convert raw 24-bit count to absolute pressure in hPa."""
    psi = (raw - OUTPUT_MIN) * (PSI_MAX - PSI_MIN) / (OUTPUT_MAX - OUTPUT_MIN) + PSI_MIN
    return psi * PSI_TO_HPA


def pressure_to_altitude(hpa, sea_level_hpa=1013.25):
    """Pressure altitude via the international barometric formula (metres).

    Uses the standard sea-level reference by default, so this is pressure
    altitude, not GPS altitude. Set sea_level_hpa to the local QNH to compare
    against true elevation. Returns NaN for non-physical (<=0) pressure.
    """
    if hpa <= 0:
        return float("nan")
    return 44330.0 * (1.0 - (hpa / sea_level_hpa) ** (1.0 / 5.255))


def main():
    print("MPRLS Pressure Sensor Test")
    print("=" * 60)

    with SMBus(I2C_BUS) as bus:
        # Single measurement.
        status, raw = read_pressure(bus)
        hpa = counts_to_hpa(raw)
        print("Single measurement:")
        print("  status      : 0x%02X (power=%d busy=%d)"
              % (status, bool(status & STATUS_POWER), bool(status & STATUS_BUSY)))
        print("  raw counts  : %d" % raw)
        print("  pressure    : %.2f hPa  (%.3f kPa, %.3f psi)"
              % (hpa, hpa / 10.0, hpa / PSI_TO_HPA))
        print("  pressure alt: %.1f m (vs 1013.25 hPa reference)"
              % pressure_to_altitude(hpa))
        print()

        # Repeatability / noise: take N samples and report statistics.
        n = 50
        print("Precision check: %d samples..." % n)
        samples = []
        for _ in range(n):
            _, raw_i = read_pressure(bus)
            samples.append(counts_to_hpa(raw_i))
            time.sleep(0.02)

        mean = statistics.mean(samples)
        sd = statistics.pstdev(samples)
        print("  mean        : %.2f hPa" % mean)
        print("  std dev     : %.3f hPa  (repeatability / noise floor)" % sd)
        print("  min / max   : %.2f / %.2f hPa (spread %.2f)"
              % (min(samples), max(samples), max(samples) - min(samples)))
        print("  alt noise   : ~%.1f m at this pressure"
              % abs(pressure_to_altitude(mean) - pressure_to_altitude(mean + sd)))
        print()
        print("Note: this is the absolute (station) pressure, not sea-level.")
        print("To judge accuracy, compare 'mean' against your local station")
        print("pressure (a nearby weather station / METAR, corrected for your")
        print("elevation). Datasheet accuracy is +/-1.5%% FSS (~26 hPa worst case);")
        print("the std dev above is the short-term precision, which is far tighter.")


if __name__ == "__main__":
    main()
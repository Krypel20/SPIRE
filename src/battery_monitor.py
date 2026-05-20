#!/usr/bin/env python3
"""
SPIRE Battery Monitor
Monitors Waveshare UPS Module 3S via INA219 over I2C.
Tracks voltage, current, power, and estimates remaining battery time.

Can run standalone or be integrated into data_logger process.

Usage:
  python3 battery_monitor.py                    # Monitor every 10s
  python3 battery_monitor.py -i 5               # Monitor every 5s
  python3 battery_monitor.py --log data/battery  # Log to CSV
  python3 battery_monitor.py --calibrate         # Find correct shunt value
"""

import time
import sys
import os
import csv
import json
import signal
import logging
import argparse
import smbus2
import struct
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

INA219_ADDR = 0x41
I2C_BUS = 1

# INA219 Registers
REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

# 3S Li-ion voltage-to-SOC lookup table (State of Charge)
# Based on typical Li-ion discharge curve at moderate load
VOLTAGE_SOC_TABLE = [
    (12.60, 100),
    (12.45, 95),
    (12.33, 90),
    (12.18, 85),
    (12.06, 80),
    (11.94, 75),
    (11.82, 70),
    (11.70, 65),
    (11.58, 60),
    (11.46, 55),
    (11.34, 50),
    (11.22, 45),
    (11.10, 40),
    (10.98, 35),
    (10.86, 30),
    (10.74, 25),
    (10.50, 20),
    (10.20, 15),
    (9.90, 10),
    (9.60, 5),
    (9.00, 0),
]

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.battery")


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
# INA219 Driver (direct smbus2, no external library)
# ---------------------------------------------------------------------------

class BatteryMonitor:
    """Read battery voltage, current, and power from INA219 on UPS 3S."""

    def __init__(self, bus_num=I2C_BUS, address=INA219_ADDR,
                 shunt_ohms=0.01, max_current_a=5.0,
                 battery_capacity_mah=3500):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.shunt_ohms = shunt_ohms
        self.battery_capacity_mah = battery_capacity_mah

        # Configure INA219
        # 32V bus range, shunt voltage range based on max current
        # Averaging: 128 samples for stable readings
        # Bus and shunt ADC: 12-bit, 128 samples averaging
        config = (0x2000 |  # 32V bus range
                  0x1800 |  # Shunt ±320mV range
                  0x0078 |  # Bus ADC 128 samples
                  0x0780 |  # Shunt ADC 128 samples
                  0x0007)   # Continuous shunt and bus
        self._write_register(REG_CONFIG, config)

        # Calibration
        # cal = trunc(0.04096 / (current_lsb * shunt_ohms))
        self.current_lsb = max_current_a / 32768.0
        cal = int(0.04096 / (self.current_lsb * shunt_ohms))
        self._write_register(REG_CALIBRATION, cal)

        # Tracking for time estimation
        self.start_time = time.monotonic()
        self.energy_used_mwh = 0.0
        self.last_power_time = time.monotonic()

        log.info(f"INA219 at 0x{address:02X}, shunt={shunt_ohms}Ω, "
                 f"capacity={battery_capacity_mah}mAh")

    def _read_register(self, reg):
        """Read 16-bit register (big-endian)."""
        raw = self.bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def _write_register(self, reg, value):
        """Write 16-bit register (big-endian)."""
        swapped = ((value & 0xFF) << 8) | (value >> 8)
        self.bus.write_word_data(self.addr, reg, swapped)

    def read_voltage(self):
        """Read bus voltage in volts."""
        raw = self._read_register(REG_BUS_VOLTAGE)
        # Bits [15:3] contain voltage, LSB = 4mV
        return (raw >> 3) * 0.004

    def read_shunt_voltage(self):
        """Read shunt voltage in millivolts."""
        raw = self._read_register(REG_SHUNT_VOLTAGE)
        # Signed 16-bit, LSB = 10µV
        if raw > 32767:
            raw -= 65536
        return raw * 0.01

    def read_current(self):
        """Read current in milliamps."""
        raw = self._read_register(REG_CURRENT)
        if raw > 32767:
            raw -= 65536
        return raw * self.current_lsb * 1000

    def read_power(self):
        """Read power in milliwatts."""
        raw = self._read_register(REG_POWER)
        return raw * self.current_lsb * 20 * 1000

    def voltage_to_soc(self, voltage):
        """Estimate State of Charge from voltage.

        Returns:
            SOC in percent (0-100)
        """
        if voltage >= VOLTAGE_SOC_TABLE[0][0]:
            return 100
        if voltage <= VOLTAGE_SOC_TABLE[-1][0]:
            return 0

        # Linear interpolation between table entries
        for i in range(len(VOLTAGE_SOC_TABLE) - 1):
            v_high, soc_high = VOLTAGE_SOC_TABLE[i]
            v_low, soc_low = VOLTAGE_SOC_TABLE[i + 1]
            if v_low <= voltage <= v_high:
                ratio = (voltage - v_low) / (v_high - v_low)
                return soc_low + ratio * (soc_high - soc_low)
        return 0

    def estimate_remaining_time(self, voltage, current_ma):
        """Estimate remaining battery time.

        Args:
            voltage: Current bus voltage
            current_ma: Current draw in mA (positive = discharging)

        Returns:
            Estimated remaining time in minutes, or -1 if cannot estimate
        """
        if current_ma <= 10:
            return -1  # Not enough current to estimate

        soc = self.voltage_to_soc(voltage)
        remaining_mah = self.battery_capacity_mah * (soc / 100.0)

        if current_ma > 0:
            remaining_hours = remaining_mah / current_ma
            return remaining_hours * 60
        return -1

    def update_energy(self, power_mw):
        """Track cumulative energy usage."""
        now = time.monotonic()
        dt_hours = (now - self.last_power_time) / 3600.0
        self.energy_used_mwh += power_mw * dt_hours
        self.last_power_time = now

    def read_all(self):
        """Read all battery parameters.

        Returns:
            dict with voltage, current, power, soc, remaining_min
        """
        voltage = self.read_voltage()
        shunt_mv = self.read_shunt_voltage()
        current_ma = self.read_current()
        power_mw = self.read_power()

        # Use shunt voltage as fallback for current
        current_from_shunt = shunt_mv / (self.shunt_ohms * 1000) * 1000

        soc = self.voltage_to_soc(voltage)
        remaining = self.estimate_remaining_time(voltage, abs(current_ma))

        self.update_energy(abs(power_mw))

        return {
            "voltage_v": round(voltage, 3),
            "shunt_mv": round(shunt_mv, 3),
            "current_ma": round(current_ma, 1),
            "current_from_shunt_ma": round(current_from_shunt, 1),
            "power_mw": round(power_mw, 0),
            "soc_percent": round(soc, 1),
            "remaining_min": round(remaining, 1) if remaining > 0 else -1,
            "energy_used_mwh": round(self.energy_used_mwh, 1),
            "timestamp_mono": time.monotonic(),
        }

    def close(self):
        self.bus.close()


# ---------------------------------------------------------------------------
# Calibration mode
# ---------------------------------------------------------------------------

def calibrate_shunt(address=INA219_ADDR, bus_num=I2C_BUS):
    """Help determine correct shunt resistor value.

    Reads raw shunt voltage and asks user to input known current
    to calculate actual shunt resistance.
    """
    log.info("=== Shunt Calibration Mode ===")
    log.info("Connect a known load to the battery output.")
    log.info("Measure actual current with a multimeter.\n")

    bus = smbus2.SMBus(bus_num)

    # Read shunt voltage
    raw = bus.read_word_data(address, REG_SHUNT_VOLTAGE)
    raw = ((raw & 0xFF) << 8) | (raw >> 8)
    if raw > 32767:
        raw -= 65536
    shunt_uv = raw * 10  # µV
    shunt_mv = shunt_uv / 1000

    log.info(f"Shunt voltage: {shunt_mv:.3f} mV ({shunt_uv} µV)")

    try:
        actual_ma = float(input("\nEnter actual current from multimeter (mA): "))
        if actual_ma > 0:
            actual_a = actual_ma / 1000
            shunt_ohms = (shunt_mv / 1000) / actual_a
            log.info(f"\nCalculated shunt resistance: {shunt_ohms:.4f} Ω")
            log.info(f"Use: python3 battery_monitor.py --shunt {shunt_ohms:.4f}")
        else:
            log.warning("Current must be positive.")
    except ValueError:
        log.error("Invalid input.")

    bus.close()


# ---------------------------------------------------------------------------
# CSV Logger
# ---------------------------------------------------------------------------

class BatteryLogger:
    def __init__(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        path = os.path.join(output_dir, f"battery_log_{timestamp}.csv")

        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp_mono", "voltage_v", "current_ma",
            "power_mw", "soc_percent", "remaining_min",
            "energy_used_mwh"
        ])
        self.path = path
        log.info(f"Battery log: {path}")

    def write(self, data):
        self.writer.writerow([
            data["timestamp_mono"],
            data["voltage_v"],
            data["current_ma"],
            data["power_mw"],
            data["soc_percent"],
            data["remaining_min"],
            data["energy_used_mwh"],
        ])
        self.file.flush()

    def close(self):
        self.file.close()
        log.info(f"Battery log closed: {self.path}")


# ---------------------------------------------------------------------------
# Main monitoring loop
# ---------------------------------------------------------------------------

def monitor_loop(monitor, logger, interval_s, duration_s):
    running = True

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    log.info(f"Monitoring every {interval_s}s")
    log.info("Ctrl+C to stop.\n")

    start = time.monotonic()

    while running:
        data = monitor.read_all()

        # Status bar
        soc = data["soc_percent"]
        bars = int(soc / 5)
        bar_str = "█" * bars + "░" * (20 - bars)

        remaining_str = (f"{data['remaining_min']:.0f} min"
                         if data["remaining_min"] > 0 else "N/A")

        log.info(
            f"[{bar_str}] {soc:.0f}% | "
            f"{data['voltage_v']:.2f}V | "
            f"{data['current_ma']:.0f}mA | "
            f"{data['power_mw']:.0f}mW | "
            f"Remaining: {remaining_str}"
        )

        if logger:
            logger.write(data)

        # Low battery warning
        if soc < 20:
            log.warning("LOW BATTERY — consider safe shutdown")
        if soc < 10:
            log.warning("CRITICAL — shutdown imminent")

        # Duration check
        if duration_s > 0 and (time.monotonic() - start) >= duration_s:
            break

        time.sleep(interval_s)

    # Final summary
    elapsed = time.monotonic() - start
    log.info(f"\nSession: {elapsed/60:.1f} min, "
             f"energy used: {data['energy_used_mwh']:.0f} mWh")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Battery Monitor — Waveshare UPS 3S",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Monitor every 10s
  %(prog)s -i 5 --log data/battery  # Every 5s with CSV logging
  %(prog)s --calibrate              # Determine shunt resistance
  %(prog)s --shunt 0.005            # Use calibrated shunt value
        """
    )

    parser.add_argument("-i", "--interval", type=float, default=10,
                        help="Monitoring interval in seconds (default: 10)")
    parser.add_argument("-d", "--duration", type=float, default=0,
                        help="Duration in seconds, 0=infinite (default: 0)")
    parser.add_argument("--log", type=str, default=None,
                        help="Log to CSV in given directory")
    parser.add_argument("--address", type=lambda x: int(x, 0),
                        default=INA219_ADDR,
                        help=f"INA219 I2C address (default: 0x{INA219_ADDR:02X})")
    parser.add_argument("--shunt", type=float, default=0.01,
                        help="Shunt resistance in ohms (default: 0.01)")
    parser.add_argument("--capacity", type=int, default=3500,
                        help="Battery capacity in mAh (default: 3500)")
    parser.add_argument("--calibrate", action="store_true",
                        help="Calibration mode — determine shunt resistance")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 40)
    log.info("SPIRE Battery Monitor")
    log.info("=" * 40)

    if args.calibrate:
        calibrate_shunt(address=args.address)
        return

    monitor = BatteryMonitor(
        address=args.address,
        shunt_ohms=args.shunt,
        battery_capacity_mah=args.capacity,
    )

    logger = None
    if args.log:
        logger = BatteryLogger(args.log)

    try:
        monitor_loop(monitor, logger, args.interval, args.duration)
    finally:
        if logger:
            logger.close()
        monitor.close()
        log.info("Done.")


if __name__ == "__main__":
    main()

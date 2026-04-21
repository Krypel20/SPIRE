#!/usr/bin/env python3
"""
SPIRE IMU Reader Process
Continuous high-frequency IMU sampling from ICM-20948 sensor.
Writes current state to shared memory for capture and servo processes.

The ICM-20948 is a 9-axis IMU (accel + gyro + magnetometer) communicating
over I2C. This module handles register-level communication, sensor
configuration, and shared memory publishing.

Usage:
  python3 imu_reader.py                    # Default: 500 Hz, bus 1, addr 0x69
  python3 imu_reader.py -r 1000 -b 1      # 1 kHz on bus 1
  python3 imu_reader.py --mag              # Enable magnetometer (slower)
  python3 imu_reader.py --log data/imu     # Log raw data to CSV
"""

import time
import sys
import os
import csv
import json
import struct
import signal
import logging
import argparse
from datetime import datetime, timezone

import smbus2

# ---------------------------------------------------------------------------
# ICM-20948 Register Map
# ---------------------------------------------------------------------------

# Bank 0 registers
ICM_WHO_AM_I        = 0x00
ICM_USER_CTRL       = 0x03
ICM_LP_CONFIG       = 0x05
ICM_PWR_MGMT_1      = 0x06
ICM_PWR_MGMT_2      = 0x07
ICM_INT_PIN_CFG      = 0x0F
ICM_INT_ENABLE_1     = 0x11
ICM_ACCEL_XOUT_H     = 0x2D
ICM_GYRO_XOUT_H      = 0x33
ICM_TEMP_OUT_H       = 0x39
ICM_REG_BANK_SEL     = 0x7F

# Bank 2 registers
ICM_GYRO_SMPLRT_DIV  = 0x00
ICM_GYRO_CONFIG_1    = 0x01
ICM_ACCEL_SMPLRT_DIV_1 = 0x10
ICM_ACCEL_SMPLRT_DIV_2 = 0x11
ICM_ACCEL_CONFIG     = 0x14

# Magnetometer (AK09916) registers — accessed via I2C master or bypass
AK_I2C_ADDR          = 0x0C
AK_WHO_AM_I          = 0x01
AK_ST1               = 0x10
AK_HXL               = 0x11
AK_CNTL2             = 0x31
AK_CNTL3             = 0x32

# Expected WHO_AM_I value
ICM_WHO_AM_I_VAL     = 0xEA

# Sensitivity scale factors
ACCEL_SCALE = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}  # LSB/g
GYRO_SCALE  = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}          # LSB/(°/s)
MAG_SCALE   = 0.15  # µT/LSB for AK09916

# Shared memory config
IMU_SHM_NAME = "spire_imu_state"
IMU_SHM_SIZE = 512  # bytes — JSON-encoded state

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.imu")


def setup_logging(verbose=False):
    """Configure console logging."""
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
# ICM-20948 Driver
# ---------------------------------------------------------------------------

class ICM20948:
    """Low-level driver for ICM-20948 9-axis IMU over I2C."""

    def __init__(self, bus_num=1, address=0x69):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = address
        self.current_bank = -1
        self.gyro_fs = 1     # ±500 °/s default
        self.accel_fs = 1    # ±4g default

    def _select_bank(self, bank):
        """Switch register bank (0-3)."""
        if bank != self.current_bank:
            self.bus.write_byte_data(self.addr, ICM_REG_BANK_SEL, bank << 4)
            self.current_bank = bank

    def _read_byte(self, bank, reg):
        """Read single byte from register in specified bank."""
        self._select_bank(bank)
        return self.bus.read_byte_data(self.addr, reg)

    def _write_byte(self, bank, reg, val):
        """Write single byte to register in specified bank."""
        self._select_bank(bank)
        self.bus.write_byte_data(self.addr, reg, val)

    def _read_bytes(self, bank, reg, count):
        """Read multiple bytes from register in specified bank."""
        self._select_bank(bank)
        return self.bus.read_i2c_block_data(self.addr, reg, count)

    def check_id(self):
        """Verify ICM-20948 WHO_AM_I register."""
        who = self._read_byte(0, ICM_WHO_AM_I)
        if who != ICM_WHO_AM_I_VAL:
            raise RuntimeError(
                f"ICM-20948 not found. WHO_AM_I=0x{who:02X}, "
                f"expected 0x{ICM_WHO_AM_I_VAL:02X}"
            )
        log.info(f"ICM-20948 detected at 0x{self.addr:02X} "
                 f"(WHO_AM_I=0x{who:02X})")

    def reset(self):
        """Software reset the sensor."""
        self._write_byte(0, ICM_PWR_MGMT_1, 0x81)  # DEVICE_RESET + SLEEP
        time.sleep(0.1)
        self._write_byte(0, ICM_PWR_MGMT_1, 0x01)  # Auto-select clock
        time.sleep(0.05)

    def configure(self, gyro_fs=1, accel_fs=1, gyro_rate_div=0,
                  accel_rate_div=0):
        """Configure sensor ranges and sample rates.

        Args:
            gyro_fs: 0=±250, 1=±500, 2=±1000, 3=±2000 °/s
            accel_fs: 0=±2g, 1=±4g, 2=±8g, 3=±16g
            gyro_rate_div: Gyro sample rate divider (rate = 1125/(1+div) Hz)
            accel_rate_div: Accel sample rate divider (rate = 1125/(1+div) Hz)
        """
        self.gyro_fs = gyro_fs
        self.accel_fs = accel_fs

        # Enable all accel and gyro axes
        self._write_byte(0, ICM_PWR_MGMT_2, 0x00)

        # Gyro config (Bank 2)
        self._write_byte(2, ICM_GYRO_SMPLRT_DIV, gyro_rate_div)
        # GYRO_FS_SEL | GYRO_FCHOICE (enable DLPF)
        self._write_byte(2, ICM_GYRO_CONFIG_1, (gyro_fs << 1) | 0x01)

        # Accel config (Bank 2)
        self._write_byte(2, ICM_ACCEL_SMPLRT_DIV_1,
                         (accel_rate_div >> 8) & 0x0F)
        self._write_byte(2, ICM_ACCEL_SMPLRT_DIV_2,
                         accel_rate_div & 0xFF)
        self._write_byte(2, ICM_ACCEL_CONFIG, (accel_fs << 1) | 0x01)

        # Back to bank 0 for data reads
        self._select_bank(0)

        gyro_rate = 1125.0 / (1 + gyro_rate_div)
        accel_rate = 1125.0 / (1 + accel_rate_div)
        gyro_range = [250, 500, 1000, 2000][gyro_fs]
        accel_range = [2, 4, 8, 16][accel_fs]

        log.info(f"Gyro:  ±{gyro_range} °/s @ {gyro_rate:.0f} Hz")
        log.info(f"Accel: ±{accel_range} g @ {accel_rate:.0f} Hz")

    def enable_magnetometer(self):
        """Enable AK09916 magnetometer via I2C bypass mode."""
        # Enable I2C bypass to access magnetometer directly
        self._write_byte(0, ICM_INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        # Check magnetometer WHO_AM_I
        try:
            who = self.bus.read_byte_data(AK_I2C_ADDR, AK_WHO_AM_I)
            log.info(f"AK09916 magnetometer detected (WHO_AM_I=0x{who:02X})")
        except OSError:
            log.warning("AK09916 magnetometer not found on I2C bypass")
            return False

        # Reset magnetometer
        self.bus.write_byte_data(AK_I2C_ADDR, AK_CNTL3, 0x01)
        time.sleep(0.01)

        # Set continuous measurement mode 4 (100 Hz)
        self.bus.write_byte_data(AK_I2C_ADDR, AK_CNTL2, 0x08)
        time.sleep(0.01)

        log.info("Magnetometer: continuous mode @ 100 Hz")
        return True

    def read_accel_gyro(self):
        """Read accelerometer and gyroscope data.

        Returns:
            tuple: (ax, ay, az, gx, gy, gz) in g and °/s
        """
        # Read 12 bytes: accel (6) + gyro (6), starting at ACCEL_XOUT_H
        # Accel: 0x2D-0x32, Gyro: 0x33-0x38
        raw = self._read_bytes(0, ICM_ACCEL_XOUT_H, 12)

        # Parse signed 16-bit big-endian values
        ax_raw = struct.unpack(">h", bytes(raw[0:2]))[0]
        ay_raw = struct.unpack(">h", bytes(raw[2:4]))[0]
        az_raw = struct.unpack(">h", bytes(raw[4:6]))[0]
        gx_raw = struct.unpack(">h", bytes(raw[6:8]))[0]
        gy_raw = struct.unpack(">h", bytes(raw[8:10]))[0]
        gz_raw = struct.unpack(">h", bytes(raw[10:12]))[0]

        # Convert to physical units
        a_scale = ACCEL_SCALE[self.accel_fs]
        g_scale = GYRO_SCALE[self.gyro_fs]

        ax = ax_raw / a_scale
        ay = ay_raw / a_scale
        az = az_raw / a_scale
        gx = gx_raw / g_scale
        gy = gy_raw / g_scale
        gz = gz_raw / g_scale

        return ax, ay, az, gx, gy, gz

    def read_magnetometer(self):
        """Read magnetometer data from AK09916.

        Returns:
            tuple: (mx, my, mz) in µT, or None if not ready
        """
        try:
            st1 = self.bus.read_byte_data(AK_I2C_ADDR, AK_ST1)
            if not (st1 & 0x01):
                return None  # Data not ready

            raw = self.bus.read_i2c_block_data(AK_I2C_ADDR, AK_HXL, 8)
            # 6 bytes mag data + ST2 (must read to clear)

            mx_raw = struct.unpack("<h", bytes(raw[0:2]))[0]
            my_raw = struct.unpack("<h", bytes(raw[2:4]))[0]
            mz_raw = struct.unpack("<h", bytes(raw[4:6]))[0]

            mx = mx_raw * MAG_SCALE
            my = my_raw * MAG_SCALE
            mz = mz_raw * MAG_SCALE

            return mx, my, mz
        except OSError:
            return None

    def read_temperature(self):
        """Read die temperature in °C."""
        raw = self._read_bytes(0, ICM_TEMP_OUT_H, 2)
        temp_raw = struct.unpack(">h", bytes(raw))[0]
        return (temp_raw - 21.0) / 333.87 + 21.0

    def close(self):
        """Close I2C bus."""
        self.bus.close()


# ---------------------------------------------------------------------------
# Shared Memory Publisher
# ---------------------------------------------------------------------------

class SharedMemoryPublisher:
    """Publish IMU state to shared memory for other processes."""

    def __init__(self, name=IMU_SHM_NAME, size=IMU_SHM_SIZE):
        from multiprocessing import shared_memory
        self.name = name
        self.size = size
        try:
            # Try to attach to existing block (clean up stale)
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
        # Clear buffer and write
        self.shm.buf[:self.size] = b'\x00' * self.size
        self.shm.buf[:len(data)] = data

    def close(self):
        """Close and unlink shared memory."""
        try:
            self.shm.close()
            self.shm.unlink()
            log.info("Shared memory released")
        except Exception as e:
            log.debug(f"Shared memory cleanup: {e}")


# ---------------------------------------------------------------------------
# CSV Logger
# ---------------------------------------------------------------------------

class IMULogger:
    """Log raw IMU data to CSV file."""

    def __init__(self, output_dir, enable_mag=False):
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        path = os.path.join(output_dir, f"imu_log_{timestamp}.csv")

        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)

        header = [
            "sample_id", "timestamp_mono_ns",
            "gyro_x", "gyro_y", "gyro_z",
            "accel_x", "accel_y", "accel_z",
        ]
        if enable_mag:
            header += ["mag_x", "mag_y", "mag_z"]

        self.writer.writerow(header)
        self.enable_mag = enable_mag
        self.path = path
        log.info(f"IMU log: {path}")

    def write(self, sample_id, t_mono, ax, ay, az, gx, gy, gz,
              mx=None, my=None, mz=None):
        """Write one sample to CSV."""
        row = [sample_id, t_mono,
               f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
               f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}"]
        if self.enable_mag:
            row += [
                f"{mx:.2f}" if mx is not None else "",
                f"{my:.2f}" if my is not None else "",
                f"{mz:.2f}" if mz is not None else "",
            ]
        self.writer.writerow(row)

    def flush(self):
        """Flush CSV buffer to disk."""
        self.file.flush()

    def close(self):
        """Close CSV file."""
        self.file.close()
        log.info(f"IMU log closed: {self.path}")


# ---------------------------------------------------------------------------
# Main sampling loop
# ---------------------------------------------------------------------------

def sampling_loop(imu, publisher, logger, target_rate_hz, enable_mag,
                  duration_s):
    """High-frequency IMU sampling loop.

    Args:
        imu: ICM20948 driver instance
        publisher: SharedMemoryPublisher instance
        logger: IMULogger instance or None
        target_rate_hz: Target sampling rate in Hz
        enable_mag: Read magnetometer data
        duration_s: Run duration in seconds (0 = infinite)
    """
    interval = 1.0 / target_rate_hz
    sample_id = 0
    running = True
    mag_data = (0.0, 0.0, 0.0)

    # Performance tracking
    perf_start = time.monotonic()
    perf_samples = 0
    perf_interval = 5.0  # Report every 5 seconds

    def handle_signal(signum, frame):
        nonlocal running
        log.info(f"Signal {signum} received, stopping...")
        running = False

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    log.info(f"Sampling started @ {target_rate_hz} Hz target")
    if duration_s > 0:
        log.info(f"Duration: {duration_s} s")

    start_time = time.monotonic()

    try:
        while running:
            t_loop_start = time.monotonic_ns()

            # Read accel + gyro (always)
            ax, ay, az, gx, gy, gz = imu.read_accel_gyro()

            # Read magnetometer (if enabled, ~100 Hz max)
            if enable_mag and sample_id % max(1, target_rate_hz // 100) == 0:
                mag_result = imu.read_magnetometer()
                if mag_result:
                    mag_data = mag_result

            mx, my, mz = mag_data

            t_mono = time.monotonic_ns()

            # Publish to shared memory
            state = {
                "gyro_x": round(gx, 4),
                "gyro_y": round(gy, 4),
                "gyro_z": round(gz, 4),
                "accel_x": round(ax, 4),
                "accel_y": round(ay, 4),
                "accel_z": round(az, 4),
                "mag_x": round(mx, 2),
                "mag_y": round(my, 2),
                "mag_z": round(mz, 2),
                "timestamp_mono_ns": t_mono,
                "sample_id": sample_id,
            }
            publisher.publish(state)

            # Log to CSV (if enabled)
            if logger:
                logger.write(
                    sample_id, t_mono, ax, ay, az, gx, gy, gz,
                    mx if enable_mag else None,
                    my if enable_mag else None,
                    mz if enable_mag else None,
                )
                # Flush every 100 samples
                if sample_id % 100 == 0:
                    logger.flush()

            sample_id += 1
            perf_samples += 1

            # Performance report
            now = time.monotonic()
            if now - perf_start >= perf_interval:
                actual_rate = perf_samples / (now - perf_start)
                log.info(
                    f"Rate: {actual_rate:.1f} Hz | "
                    f"Samples: {sample_id} | "
                    f"Gyro: ({gx:.1f}, {gy:.1f}, {gz:.1f}) °/s | "
                    f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) g"
                )
                perf_start = now
                perf_samples = 0

            # Duration check
            if duration_s > 0 and (now - start_time) >= duration_s:
                log.info("Duration reached, stopping.")
                break

            # Timing: sleep to maintain target rate
            elapsed_ns = time.monotonic_ns() - t_loop_start
            sleep_ns = int(interval * 1e9) - elapsed_ns
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

    except Exception as e:
        log.error(f"Sampling error: {e}", exc_info=True)

    finally:
        total_time = time.monotonic() - start_time
        avg_rate = sample_id / total_time if total_time > 0 else 0
        log.info(f"Stopped. Total samples: {sample_id}, "
                 f"avg rate: {avg_rate:.1f} Hz, "
                 f"duration: {total_time:.1f} s")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE IMU Reader — ICM-20948",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # 500 Hz, bus 1, addr 0x69
  %(prog)s -r 1000                  # 1 kHz sampling
  %(prog)s --mag                    # Enable magnetometer
  %(prog)s --log data/imu_test      # Log to CSV
  %(prog)s -r 200 -d 10            # 200 Hz for 10 seconds
        """
    )

    parser.add_argument("-b", "--bus", type=int, default=1,
                        help="I2C bus number (default: 1)")
    parser.add_argument("-a", "--address", type=lambda x: int(x, 0),
                        default=0x69,
                        help="I2C address (default: 0x69)")
    parser.add_argument("-r", "--rate", type=int, default=500,
                        help="Target sampling rate in Hz (default: 500)")
    parser.add_argument("--gyro-fs", type=int, default=1,
                        choices=[0, 1, 2, 3],
                        help="Gyro range: 0=±250, 1=±500, 2=±1000, "
                             "3=±2000 °/s (default: 1)")
    parser.add_argument("--accel-fs", type=int, default=1,
                        choices=[0, 1, 2, 3],
                        help="Accel range: 0=±2g, 1=±4g, 2=±8g, "
                             "3=±16g (default: 1)")
    parser.add_argument("--mag", action="store_true",
                        help="Enable magnetometer readout")
    parser.add_argument("--log", type=str, default=None,
                        help="Log raw IMU data to CSV in given directory")
    parser.add_argument("-d", "--duration", type=float, default=0,
                        help="Run duration in seconds (0 = infinite)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 40)
    log.info("SPIRE IMU Reader — ICM-20948")
    log.info("=" * 40)

    # Initialize sensor
    imu = ICM20948(bus_num=args.bus, address=args.address)

    try:
        imu.check_id()
    except RuntimeError as e:
        log.error(str(e))
        sys.exit(1)

    imu.reset()

    # Compute sample rate divider for target rate
    # ICM-20948 internal rate = 1125 Hz
    # Output rate = 1125 / (1 + divider)
    if args.rate >= 1125:
        divider = 0
    else:
        divider = max(0, int(1125.0 / args.rate) - 1)
    actual_rate = 1125.0 / (1 + divider)

    imu.configure(
        gyro_fs=args.gyro_fs,
        accel_fs=args.accel_fs,
        gyro_rate_div=divider,
        accel_rate_div=divider,
    )

    # Enable magnetometer if requested
    mag_ok = False
    if args.mag:
        mag_ok = imu.enable_magnetometer()

    # Setup shared memory
    publisher = SharedMemoryPublisher()

    # Setup CSV logger
    logger = None
    if args.log:
        logger = IMULogger(args.log, enable_mag=args.mag and mag_ok)

    log.info(f"Target rate: {args.rate} Hz "
             f"(sensor divider: {divider}, actual: {actual_rate:.0f} Hz)")

    try:
        sampling_loop(
            imu, publisher, logger,
            target_rate_hz=args.rate,
            enable_mag=args.mag and mag_ok,
            duration_s=args.duration,
        )
    finally:
        if logger:
            logger.close()
        publisher.close()
        imu.close()
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()
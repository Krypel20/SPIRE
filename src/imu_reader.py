#!/usr/bin/env python3
"""
SPIRE IMU Reader Process
Continuous high-frequency IMU sampling with hardware-abstracted driver.
Writes current state to shared memory for capture and servo processes.

Supports multiple IMU sensors via pluggable drivers:
  - ICM-20948 (9-axis)
  - MPU6886  (6-axis)
  - LSM9DS1  (9-axis)

Usage:
  python3 imu_reader.py                          # ICM-20948, 500 Hz
  python3 imu_reader.py --sensor mpu6886         # MPU6886
  python3 imu_reader.py --sensor lsm9ds1 -a 0x6B # LSM9DS1
  python3 imu_reader.py -r 1000 --mag --log data/imu
"""

import time
import sys
import os
import csv
import json
import signal
import logging
import argparse
from datetime import datetime, timezone

# Add parent dir to path for imu_drivers package
sys.path.insert(0, os.path.dirname(__file__))
from imu_drivers import create_driver, list_drivers

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

IMU_SHM_NAME = "spire_imu_state"
IMU_SHM_SIZE = 512

DEFAULT_CAL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "imu_calibration.json"
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.imu")


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
# Shared Memory Publisher
# ---------------------------------------------------------------------------

class SharedMemoryPublisher:
    """Publish IMU state to shared memory for other processes."""

    def __init__(self, name=IMU_SHM_NAME, size=IMU_SHM_SIZE):
        from multiprocessing import shared_memory
        self.name = name
        self.size = size

        # Clean up stale block if exists
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
        self.file.flush()

    def close(self):
        self.file.close()
        log.info(f"IMU log closed: {self.path}")


# ---------------------------------------------------------------------------
# Calibration loader
# ---------------------------------------------------------------------------

def load_calibration(path):
    """Load calibration from JSON file.

    Returns:
        dict or None
    """
    try:
        with open(path, "r") as f:
            cal = json.load(f)
        log.info(f"Calibration loaded: {path}")
        return cal
    except FileNotFoundError:
        log.warning(f"No calibration file at {path} — "
                    "run imu_calibrate.py first")
        return None
    except json.JSONDecodeError as e:
        log.error(f"Invalid calibration file: {e}")
        return None


# ---------------------------------------------------------------------------
# Main sampling loop
# ---------------------------------------------------------------------------

def sampling_loop(imu, publisher, logger, target_rate_hz, enable_mag,
                  duration_s, cal=None):
    """High-frequency IMU sampling loop.

    Args:
        imu: IMUDriver instance (any supported sensor)
        publisher: SharedMemoryPublisher instance
        logger: IMULogger instance or None
        target_rate_hz: Target sampling rate in Hz
        enable_mag: Read magnetometer data
        duration_s: Run duration in seconds (0 = infinite)
        cal: Calibration dict or None
    """
    interval = 1.0 / target_rate_hz
    sample_id = 0
    running = True
    mag_data = (0.0, 0.0, 0.0)

    # Pre-extract calibration values for speed
    if cal:
        gb = cal["gyro_bias_dps"]
        ao = cal["accel_offset_g"]
        gbx, gby, gbz = gb["x"], gb["y"], gb["z"]
        aox, aoy, aoz = ao["x"], ao["y"], ao["z"]
        log.info(f"Calibration active — gyro bias: "
                 f"({gbx:+.3f}, {gby:+.3f}, {gbz:+.3f}) °/s")
    else:
        gbx = gby = gbz = 0.0
        aox = aoy = aoz = 0.0
        log.info("No calibration — using raw values")

    # Performance tracking
    perf_start = time.monotonic()
    perf_samples = 0
    perf_interval = 5.0

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

            # Read accel + gyro (hardware-independent call)
            ax, ay, az, gx, gy, gz = imu.read_accel_gyro()

            # Apply calibration correction
            gx -= gbx; gy -= gby; gz -= gbz
            ax -= aox; ay -= aoy; az -= aoz

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

            # Log to CSV
            if logger:
                logger.write(
                    sample_id, t_mono, ax, ay, az, gx, gy, gz,
                    mx if enable_mag else None,
                    my if enable_mag else None,
                    mz if enable_mag else None,
                )
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

            # Timing
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
    available = ", ".join(list_drivers())

    parser = argparse.ArgumentParser(
        description="SPIRE IMU Reader — Multi-sensor support",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Supported sensors: {available}

Examples:
  %(prog)s                                # ICM-20948, 500 Hz
  %(prog)s --sensor mpu6886               # MPU6886
  %(prog)s --sensor lsm9ds1 -a 0x6B       # LSM9DS1
  %(prog)s -r 1000 --mag --log data/imu    # 1 kHz + magnetometer + CSV
  %(prog)s --sensor icm20948 --no-cal      # Skip calibration
        """
    )

    parser.add_argument("--sensor", type=str, default="icm20948",
                        help=f"Sensor type ({available}) "
                             "(default: icm20948)")
    parser.add_argument("-b", "--bus", type=int, default=1,
                        help="I2C bus number (default: 1)")
    parser.add_argument("-a", "--address", type=lambda x: int(x, 0),
                        default=None,
                        help="I2C address (default: sensor-specific)")
    parser.add_argument("-r", "--rate", type=int, default=500,
                        help="Target sampling rate in Hz (default: 500)")
    parser.add_argument("--gyro-fs", type=int, default=1,
                        choices=[0, 1, 2, 3],
                        help="Gyro range index (default: 1)")
    parser.add_argument("--accel-fs", type=int, default=1,
                        choices=[0, 1, 2, 3],
                        help="Accel range index (default: 1)")
    parser.add_argument("--mag", action="store_true",
                        help="Enable magnetometer readout")
    parser.add_argument("--log", type=str, default=None,
                        help="Log raw IMU data to CSV in given directory")
    parser.add_argument("-d", "--duration", type=float, default=0,
                        help="Run duration in seconds (0 = infinite)")
    parser.add_argument("--cal", type=str, default=DEFAULT_CAL_PATH,
                        help="Calibration JSON path")
    parser.add_argument("--no-cal", action="store_true",
                        help="Disable calibration (use raw values)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 40)
    log.info("SPIRE IMU Reader")
    log.info("=" * 40)

    # Create hardware driver
    try:
        imu = create_driver(
            args.sensor,
            bus=args.bus,
            address=args.address,
        )
    except ValueError as e:
        log.error(str(e))
        sys.exit(1)

    # Initialize sensor
    try:
        imu.check_id()
    except RuntimeError as e:
        log.error(str(e))
        sys.exit(1)

    # Compute rate divider
    info = {"actual_rate_hz": 0}
    if args.rate >= 1125:
        divider = 0
    else:
        divider = max(0, int(1125.0 / args.rate) - 1)

    imu.initialize(
        gyro_fs=args.gyro_fs,
        accel_fs=args.accel_fs,
        rate_div=divider,
    )

    info = imu.get_sensor_info()
    log.info(f"Sensor: {info['name']}")

    # Enable magnetometer
    mag_ok = False
    if args.mag:
        if info["has_magnetometer"]:
            mag_ok = imu.enable_magnetometer()
        else:
            log.warning(f"{info['name']} has no magnetometer")

    # Shared memory
    publisher = SharedMemoryPublisher()

    # CSV logger
    logger = None
    if args.log:
        logger = IMULogger(args.log, enable_mag=args.mag and mag_ok)

    log.info(f"Target rate: {args.rate} Hz "
             f"(actual: {info['actual_rate_hz']:.0f} Hz)")

    # Load calibration
    cal = None
    if not args.no_cal:
        cal = load_calibration(os.path.abspath(args.cal))

    try:
        sampling_loop(
            imu, publisher, logger,
            target_rate_hz=args.rate,
            enable_mag=args.mag and mag_ok,
            duration_s=args.duration,
            cal=cal,
        )
    finally:
        if logger:
            logger.close()
        publisher.close()
        imu.close()
        log.info("Shutdown complete.")


if __name__ == "__main__":
    main()

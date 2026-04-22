#!/usr/bin/env python3
"""
SPIRE IMU Calibration Module
Determines gyroscope bias and accelerometer offset at rest.
Saves calibration to JSON for use by imu_reader.

Usage:
  python3 imu_calibrate.py                    # 5s calibration, bus 1, addr 0x69
  python3 imu_calibrate.py -t 10              # 10s calibration
  python3 imu_calibrate.py -o config/cal.json # Custom output path
  python3 imu_calibrate.py --verify           # Verify existing calibration
"""

import time
import sys
import os
import json
import logging
import argparse
import math
from datetime import datetime, timezone

import smbus2

# Import driver abstraction
sys.path.insert(0, os.path.dirname(__file__))
from imu_drivers import create_driver, list_drivers

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.calibrate")


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
# Calibration
# ---------------------------------------------------------------------------

DEFAULT_CAL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "imu_calibration.json"
)


def collect_samples(imu, duration_s, rate_hz=200):
    """Collect IMU samples at rest for calibration.

    Args:
        imu: ICM20948 driver instance
        duration_s: Collection duration in seconds
        rate_hz: Sampling rate during calibration

    Returns:
        dict with lists of gyro and accel samples
    """
    interval = 1.0 / rate_hz
    samples = {
        "gx": [], "gy": [], "gz": [],
        "ax": [], "ay": [], "az": [],
    }

    total = int(duration_s * rate_hz)
    log.info(f"Collecting {total} samples over {duration_s}s "
             f"@ {rate_hz} Hz...")
    log.info("Keep the sensor PERFECTLY STILL on a flat surface.")
    log.info("")

    # Countdown
    for i in range(3, 0, -1):
        log.info(f"  Starting in {i}...")
        time.sleep(1.0)

    log.info("  Collecting...")

    for i in range(total):
        t_start = time.monotonic()

        ax, ay, az, gx, gy, gz = imu.read_accel_gyro()

        samples["gx"].append(gx)
        samples["gy"].append(gy)
        samples["gz"].append(gz)
        samples["ax"].append(ax)
        samples["ay"].append(ay)
        samples["az"].append(az)

        # Progress report every 20%
        if (i + 1) % (total // 5) == 0:
            pct = 100 * (i + 1) / total
            log.info(f"  {pct:.0f}% ({i+1}/{total})")

        elapsed = time.monotonic() - t_start
        wait = interval - elapsed
        if wait > 0:
            time.sleep(wait)

    return samples


def compute_calibration(samples):
    """Compute calibration parameters from collected samples.

    Gyro bias: mean of all samples (should be near zero at rest)
    Accel offset: difference from expected gravity vector

    Assumes sensor is stationary on a flat surface with one axis
    aligned to gravity. Detects which axis has gravity automatically.

    Returns:
        dict with calibration parameters
    """
    n = len(samples["gx"])

    # Gyro bias — simple mean
    gyro_bias = {
        "x": sum(samples["gx"]) / n,
        "y": sum(samples["gy"]) / n,
        "z": sum(samples["gz"]) / n,
    }

    # Gyro noise — standard deviation
    gyro_std = {
        "x": _std(samples["gx"], gyro_bias["x"]),
        "y": _std(samples["gy"], gyro_bias["y"]),
        "z": _std(samples["gz"], gyro_bias["z"]),
    }

    # Accel mean
    accel_mean = {
        "x": sum(samples["ax"]) / n,
        "y": sum(samples["ay"]) / n,
        "z": sum(samples["az"]) / n,
    }

    # Accel noise
    accel_std = {
        "x": _std(samples["ax"], accel_mean["x"]),
        "y": _std(samples["ay"], accel_mean["y"]),
        "z": _std(samples["az"], accel_mean["z"]),
    }

    # Detect gravity axis (largest absolute mean)
    abs_means = {
        "x": abs(accel_mean["x"]),
        "y": abs(accel_mean["y"]),
        "z": abs(accel_mean["z"]),
    }
    gravity_axis = max(abs_means, key=abs_means.get)
    gravity_sign = 1.0 if accel_mean[gravity_axis] > 0 else -1.0

    # Accel offset — subtract expected gravity
    accel_offset = {
        "x": accel_mean["x"],
        "y": accel_mean["y"],
        "z": accel_mean["z"],
    }
    accel_offset[gravity_axis] -= gravity_sign * 1.0

    # Total magnitude check
    g_magnitude = math.sqrt(
        accel_mean["x"]**2 + accel_mean["y"]**2 + accel_mean["z"]**2
    )

    return {
        "gyro_bias": gyro_bias,
        "gyro_noise_std": gyro_std,
        "accel_offset": accel_offset,
        "accel_noise_std": accel_std,
        "gravity_axis": gravity_axis,
        "gravity_sign": gravity_sign,
        "gravity_magnitude_g": round(g_magnitude, 6),
        "num_samples": n,
    }


def _std(values, mean):
    """Compute standard deviation."""
    n = len(values)
    variance = sum((x - mean) ** 2 for x in values) / n
    return math.sqrt(variance)


def save_calibration(cal, path):
    """Save calibration data to JSON file."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    output = {
        "calibration_date": datetime.now(timezone.utc).isoformat(),
        "sensor": "ICM-20948",
        "gyro_bias_dps": {
            "x": round(cal["gyro_bias"]["x"], 6),
            "y": round(cal["gyro_bias"]["y"], 6),
            "z": round(cal["gyro_bias"]["z"], 6),
        },
        "gyro_noise_dps": {
            "x": round(cal["gyro_noise_std"]["x"], 6),
            "y": round(cal["gyro_noise_std"]["y"], 6),
            "z": round(cal["gyro_noise_std"]["z"], 6),
        },
        "accel_offset_g": {
            "x": round(cal["accel_offset"]["x"], 6),
            "y": round(cal["accel_offset"]["y"], 6),
            "z": round(cal["accel_offset"]["z"], 6),
        },
        "accel_noise_g": {
            "x": round(cal["accel_noise_std"]["x"], 6),
            "y": round(cal["accel_noise_std"]["y"], 6),
            "z": round(cal["accel_noise_std"]["z"], 6),
        },
        "gravity_axis": cal["gravity_axis"],
        "gravity_sign": cal["gravity_sign"],
        "gravity_magnitude_g": cal["gravity_magnitude_g"],
        "num_samples": cal["num_samples"],
    }

    with open(path, "w") as f:
        json.dump(output, f, indent=2)

    log.info(f"Calibration saved: {path}")
    return output


def load_calibration(path):
    """Load calibration from JSON file.

    Returns:
        dict with calibration data, or None if not found
    """
    try:
        with open(path, "r") as f:
            cal = json.load(f)
        log.info(f"Calibration loaded: {path} "
                 f"(date: {cal.get('calibration_date', 'unknown')})")
        return cal
    except FileNotFoundError:
        log.warning(f"No calibration file found: {path}")
        return None
    except json.JSONDecodeError as e:
        log.error(f"Invalid calibration file: {e}")
        return None


def apply_calibration(ax, ay, az, gx, gy, gz, cal):
    """Apply calibration correction to raw IMU readings.

    Args:
        ax, ay, az: Raw accel in g
        gx, gy, gz: Raw gyro in °/s
        cal: Calibration dict from load_calibration()

    Returns:
        tuple: (ax, ay, az, gx, gy, gz) corrected
    """
    gb = cal["gyro_bias_dps"]
    ao = cal["accel_offset_g"]

    gx -= gb["x"]
    gy -= gb["y"]
    gz -= gb["z"]

    ax -= ao["x"]
    ay -= ao["y"]
    az -= ao["z"]

    return ax, ay, az, gx, gy, gz


def print_calibration_report(cal):
    """Print human-readable calibration summary."""
    log.info("")
    log.info("=" * 50)
    log.info("CALIBRATION RESULTS")
    log.info("=" * 50)
    log.info(f"Samples: {cal['num_samples']}")
    log.info("")

    gb = cal["gyro_bias"]
    gs = cal["gyro_noise_std"]
    log.info("Gyroscope bias (°/s):")
    log.info(f"  X: {gb['x']:+.4f}  (noise σ: {gs['x']:.4f})")
    log.info(f"  Y: {gb['y']:+.4f}  (noise σ: {gs['y']:.4f})")
    log.info(f"  Z: {gb['z']:+.4f}  (noise σ: {gs['z']:.4f})")

    total_bias = math.sqrt(gb["x"]**2 + gb["y"]**2 + gb["z"]**2)
    log.info(f"  Total bias magnitude: {total_bias:.4f} °/s")
    log.info("")

    ao = cal["accel_offset"]
    an = cal["accel_noise_std"]
    log.info("Accelerometer offset (g):")
    log.info(f"  X: {ao['x']:+.6f}  (noise σ: {an['x']:.6f})")
    log.info(f"  Y: {ao['y']:+.6f}  (noise σ: {an['y']:.6f})")
    log.info(f"  Z: {ao['z']:+.6f}  (noise σ: {an['z']:.6f})")
    log.info("")

    log.info(f"Gravity axis: {cal['gravity_axis'].upper()} "
             f"({'+'if cal['gravity_sign']>0 else '-'})")
    log.info(f"Gravity magnitude: {cal['gravity_magnitude_g']:.4f} g "
             f"(expected: 1.0000 g)")
    log.info("=" * 50)


# ---------------------------------------------------------------------------
# Verify mode
# ---------------------------------------------------------------------------

def verify_calibration(imu, cal_path, duration_s=3):
    """Verify calibration by reading corrected data for a few seconds."""
    cal = load_calibration(cal_path)
    if cal is None:
        log.error("Cannot verify — no calibration file.")
        return

    log.info("Verifying calibration (keep sensor still)...")
    time.sleep(1.0)

    samples = 0
    sum_gx = sum_gy = sum_gz = 0.0
    sum_ax = sum_ay = sum_az = 0.0

    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
        ax, ay, az, gx, gy, gz = apply_calibration(
            ax, ay, az, gx, gy, gz, cal
        )
        sum_gx += gx; sum_gy += gy; sum_gz += gz
        sum_ax += ax; sum_ay += ay; sum_az += az
        samples += 1
        time.sleep(0.005)

    log.info("")
    log.info("Verification (post-calibration averages):")
    log.info(f"  Gyro:  ({sum_gx/samples:+.3f}, {sum_gy/samples:+.3f}, "
             f"{sum_gz/samples:+.3f}) °/s  [expect ~0.0]")
    log.info(f"  Accel: ({sum_ax/samples:+.3f}, {sum_ay/samples:+.3f}, "
             f"{sum_az/samples:+.3f}) g  [expect gravity on one axis]")

    # Check quality
    gyro_residual = math.sqrt(
        (sum_gx/samples)**2 + (sum_gy/samples)**2 + (sum_gz/samples)**2
    )
    if gyro_residual < 0.1:
        log.info(f"  Gyro residual: {gyro_residual:.4f} °/s — GOOD")
    elif gyro_residual < 0.3:
        log.info(f"  Gyro residual: {gyro_residual:.4f} °/s — ACCEPTABLE")
    else:
        log.warning(f"  Gyro residual: {gyro_residual:.4f} °/s — "
                    "POOR (recalibrate)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE IMU Calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # 5s calibration, ICM-20948
  %(prog)s --sensor mpu6886         # Calibrate MPU6886
  %(prog)s -t 10                    # 10s for better accuracy
  %(prog)s --verify                 # Check existing calibration
  %(prog)s -o config/custom.json    # Custom output path
        """
    )

    parser.add_argument("--sensor", type=str, default="icm20948",
                        help=f"Sensor type ({', '.join(list_drivers())}) "
                             "(default: icm20948)")
    parser.add_argument("-b", "--bus", type=int, default=1,
                        help="I2C bus number (default: 1)")
    parser.add_argument("-a", "--address", type=lambda x: int(x, 0),
                        default=None,
                        help="I2C address (default: sensor-specific)")
    parser.add_argument("-t", "--time", type=float, default=5.0,
                        help="Calibration duration in seconds (default: 5)")
    parser.add_argument("-o", "--output", type=str, default=DEFAULT_CAL_PATH,
                        help="Output calibration JSON path")
    parser.add_argument("--verify", action="store_true",
                        help="Verify existing calibration")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 50)
    log.info("SPIRE IMU Calibration")
    log.info("=" * 50)

    # Initialize sensor via driver abstraction
    try:
        imu = create_driver(args.sensor, bus=args.bus, address=args.address)
    except ValueError as e:
        log.error(str(e))
        sys.exit(1)

    try:
        imu.check_id()
    except RuntimeError as e:
        log.error(str(e))
        sys.exit(1)

    imu.initialize(gyro_fs=1, accel_fs=1)

    if args.verify:
        verify_calibration(imu, args.output)
        imu.close()
        return

    # Collect and compute
    samples = collect_samples(imu, args.time)
    cal = compute_calibration(samples)
    print_calibration_report(cal)

    # Save
    save_calibration(cal, args.output)

    # Auto-verify
    log.info("")
    verify_calibration(imu, args.output)

    imu.close()
    log.info("Done.")


if __name__ == "__main__":
    main()

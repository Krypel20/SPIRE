#!/usr/bin/env python3
"""
SPIRE Process Manager
Starts all SPIRE processes from a single command.
Monitors health, handles graceful shutdown on Ctrl+C.

Starts:
  1. IMU Reader — camera (ICM-20948) → spire_imu_camera
  2. IMU Reader — platform (LSM9DS1) → spire_imu_platform
  3. Stabilization controller (reads both IMUs, drives servo)

Usage:
  python3 process_manager.py
  python3 process_manager.py --no-stabilize     # IMU only, no servo
  python3 process_manager.py --cam-sensor icm20948 --plat-sensor lsm9ds1
"""

import subprocess
import sys
import os
import signal
import time
import argparse
import logging

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.manager")


def setup_logging():
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] [manager] %(message)s", datefmt="%H:%M:%S"
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    console.setFormatter(fmt)
    log.setLevel(logging.INFO)
    log.addHandler(console)


# ---------------------------------------------------------------------------
# Process wrapper
# ---------------------------------------------------------------------------

class ManagedProcess:
    """Wrapper around subprocess with label and health check."""

    def __init__(self, name, cmd, cwd=None):
        self.name = name
        self.cmd = cmd
        self.cwd = cwd
        self.proc = None

    def start(self):
        log.info(f"Starting [{self.name}]: {' '.join(self.cmd)}")
        self.proc = subprocess.Popen(
            self.cmd,
            cwd=self.cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        log.info(f"  [{self.name}] PID={self.proc.pid}")

    def is_alive(self):
        if self.proc is None:
            return False
        return self.proc.poll() is None

    def stop(self):
        if self.proc and self.is_alive():
            log.info(f"Stopping [{self.name}] PID={self.proc.pid}")
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                log.warning(f"  [{self.name}] did not stop, killing")
                self.proc.kill()
                self.proc.wait()
            log.info(f"  [{self.name}] stopped")

    def read_output(self):
        """Read available stdout lines (non-blocking)."""
        lines = []
        if self.proc and self.proc.stdout:
            import select
            while select.select([self.proc.stdout], [], [], 0)[0]:
                line = self.proc.stdout.readline()
                if line:
                    lines.append(line.rstrip())
                else:
                    break
        return lines


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Process Manager",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Starts all SPIRE processes from a single command.

Examples:
  %(prog)s                                          # Full system
  %(prog)s --no-stabilize                           # IMU readers only
  %(prog)s --cam-rate 500 --plat-rate 200           # Custom rates
  %(prog)s --cam-sensor icm20948 --plat-sensor lsm9ds1
        """
    )

    # Camera IMU
    parser.add_argument("--cam-sensor", default="icm20948",
                        help="Camera IMU sensor (default: icm20948)")
    parser.add_argument("--cam-addr", default=None,
                        help="Camera IMU I2C address (default: auto)")
    parser.add_argument("--cam-rate", type=int, default=500,
                        help="Camera IMU sample rate Hz (default: 500)")
    parser.add_argument("--cam-cal", default="config/imu_calibration.json",
                        help="Camera IMU calibration file")

    # Platform IMU
    parser.add_argument("--plat-sensor", default="lsm9ds1",
                        help="Platform IMU sensor (default: lsm9ds1)")
    parser.add_argument("--plat-addr", default=None,
                        help="Platform IMU I2C address (default: auto)")
    parser.add_argument("--plat-rate", type=int, default=200,
                        help="Platform IMU sample rate Hz (default: 200)")
    parser.add_argument("--plat-cal", default="config/lsm9ds1_calibration.json",
                        help="Platform IMU calibration file")

    # Stabilization
    parser.add_argument("--no-stabilize", action="store_true",
                        help="Skip stabilization (IMU readers only)")
    parser.add_argument("--servo-pin", type=int, default=12,
                        help="Servo GPIO pin (default: 12)")
    parser.add_argument("--gain", type=float, default=5.0,
                        help="Stabilization gain (default: 5.0)")
    parser.add_argument("--deadband", type=float, default=1.5,
                        help="Gyro deadband deg/s (default: 1.5)")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 50)
    log.info("SPIRE Process Manager")
    log.info("=" * 50)

    src_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(src_dir)
    python = sys.executable

    # Build process commands
    processes = []

    # 1. Camera IMU reader
    cam_cmd = [
        python, os.path.join(src_dir, "imu_reader.py"),
        "--sensor", args.cam_sensor,
        "-r", str(args.cam_rate),
        "--shm-name", "spire_imu_camera",
        "--cal", args.cam_cal,
    ]
    if args.cam_addr:
        cam_cmd += ["-a", args.cam_addr]
    processes.append(ManagedProcess("imu_camera", cam_cmd, cwd=project_dir))

    # 2. Platform IMU reader
    plat_cmd = [
        python, os.path.join(src_dir, "imu_reader.py"),
        "--sensor", args.plat_sensor,
        "-r", str(args.plat_rate),
        "--shm-name", "spire_imu_platform",
        "--cal", args.plat_cal,
    ]
    if args.plat_addr:
        plat_cmd += ["-a", args.plat_addr]
    processes.append(ManagedProcess("imu_platform", plat_cmd, cwd=project_dir))

    # 3. Stabilization
    if not args.no_stabilize:
        stab_cmd = [
            python, os.path.join(src_dir, "stabilize_demo.py"),
            "--pin", str(args.servo_pin),
            "--gain", str(args.gain),
            "--deadband", str(args.deadband),
        ]
        processes.append(ManagedProcess("stabilize", stab_cmd, cwd=project_dir))

    # Start all
    for p in processes:
        p.start()
        time.sleep(1.0)  # Stagger startup

    log.info("")
    log.info(f"All {len(processes)} processes running. Ctrl+C to stop.")
    log.info("")

    # Monitor loop
    running = True

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while running:
            # Print subprocess output
            for p in processes:
                for line in p.read_output():
                    print(f"  [{p.name}] {line}")

            # Check health
            for p in processes:
                if not p.is_alive():
                    rc = p.proc.returncode
                    log.warning(f"[{p.name}] died (exit code {rc})")
                    running = False
                    break

            time.sleep(0.1)

    finally:
        log.info("")
        log.info("Shutting down all processes...")
        for p in reversed(processes):
            p.stop()
        log.info("All processes stopped.")


if __name__ == "__main__":
    main()

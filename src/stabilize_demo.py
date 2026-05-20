#!/usr/bin/env python3
"""
SPIRE Yaw Stabilization Demo — Rate Control
Servo reacts directly to gyro angular velocity, not integrated angle.
This avoids the servo fighting its own movement when IMU is on the camera.

How it works:
  - Gyro Z detects rotation → servo moves opposite direction
  - When camera stops rotating (gyro Z ≈ 0) → servo holds position
  - Servo never accumulates angle error from its own movement

Requires imu_reader.py running in a separate terminal.

Usage:
  Terminal 1: python3 src/imu_reader.py -r 500
  Terminal 2: python3 src/stabilize_demo.py
  Terminal 2: python3 src/stabilize_demo.py --gain 5.0 --deadband 2.0
"""

import time
import sys
import json
import signal
import argparse
import logging
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SERVO_PIN = 12
IMU_SHM_NAME = "spire_imu_state"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.stabilize")


def setup_logging():
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    console.setFormatter(fmt)
    log.setLevel(logging.INFO)
    log.addHandler(console)


# ---------------------------------------------------------------------------
# IMU reader
# ---------------------------------------------------------------------------

_imu_shm = None


def read_imu():
    global _imu_shm
    try:
        from multiprocessing import shared_memory, resource_tracker
        if _imu_shm is None:
            _imu_shm = shared_memory.SharedMemory(
                name=IMU_SHM_NAME, create=False
            )
            resource_tracker.unregister(
                f"/{IMU_SHM_NAME}", "shared_memory"
            )
        raw = bytes(_imu_shm.buf[:_imu_shm.size]).rstrip(b'\x00')
        if raw:
            return json.loads(raw.decode("utf-8"))
    except FileNotFoundError:
        _imu_shm = None
    except Exception:
        _imu_shm = None
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Yaw Stabilization — Rate Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tuning:
  --gain       How aggressively servo reacts to rotation.
               Higher = faster response, too high = oscillation.
  --deadband   Ignore gyro below this value (noise rejection).
               Higher = less jitter, but slower response to small movements.
  --threshold  Min servo angle change before PWM is updated.
               Higher = smoother but less precise.

Examples:
  %(prog)s                                  # Defaults
  %(prog)s --gain 5.0                       # More responsive
  %(prog)s --gain 3.0 --deadband 2.0        # Smoother
  %(prog)s --gain 8.0 --threshold 1.0       # Aggressive, precise
        """
    )

    parser.add_argument("--pin", type=int, default=SERVO_PIN,
                        help=f"GPIO pin (default: {SERVO_PIN})")
    parser.add_argument("--gain", type=float, default=5.0,
                        help="Rate gain — degrees servo per degree/s gyro "
                             "(default: 5.0)")
    parser.add_argument("--deadband", type=float, default=1.5,
                        help="Gyro deadband in deg/s (default: 1.5)")
    parser.add_argument("--threshold", type=float, default=2.0,
                        help="Min servo angle change to send PWM "
                             "(default: 2.0 deg)")
    parser.add_argument("--range", type=float, default=135.0,
                        help="Servo range +/- degrees (default: 135)")
    parser.add_argument("--invert", action="store_true",
                        help="Invert servo direction")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Yaw Stabilization — Rate Control")
    log.info("=" * 40)

    # Check IMU
    imu = read_imu()
    if imu is None:
        log.error("IMU not available. Start imu_reader.py first.")
        sys.exit(1)
    log.info("IMU connected")

    # Initialize servo
    factory = LGPIOFactory()
    servo = Servo(
        args.pin,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
    )
    servo.mid()

    log.info(f"Servo on GPIO {args.pin}")
    log.info(f"Gain: {args.gain}  Deadband: {args.deadband} deg/s  "
             f"Threshold: {args.threshold} deg")
    log.info(f"Range: +/-{args.range} deg  "
             f"Invert: {'yes' if args.invert else 'no'}")
    log.info("")
    log.info("Rotate the module — servo counteracts yaw.")
    log.info("Ctrl+C to stop.")
    log.info("")

    # State
    servo_position = 0.0  # Current servo angle in degrees
    last_sent_position = 0.0  # Last position actually sent to servo
    last_time = time.monotonic()
    last_report = time.monotonic()
    running = True
    direction = -1.0 if not args.invert else 1.0

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while running:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            # Clamp dt to avoid jumps
            if dt > 0.1:
                dt = 0.01

            # Read IMU
            imu = read_imu()
            if imu is None:
                time.sleep(0.01)
                continue

            gyro_z = imu.get("gyro_z", 0.0)

            # Deadband — ignore noise
            if abs(gyro_z) < args.deadband:
                gyro_z = 0.0

            # Rate control:
            # Gyro reads rotation → adjust servo position proportionally
            # When gyro reads 0 (camera stable) → servo holds position
            servo_position += direction * gyro_z * dt * args.gain

            # Clamp to servo range
            servo_position = max(-args.range, min(args.range, servo_position))

            # Only update servo hardware if change is significant
            if abs(servo_position - last_sent_position) > args.threshold:
                servo_value = servo_position / args.range
                servo_value = max(-1.0, min(1.0, servo_value))
                servo.value = servo_value
                last_sent_position = servo_position

            # Report
            if now - last_report >= 0.5:
                log.info(
                    f"Gyro Z: {imu.get('gyro_z', 0):+6.1f} deg/s | "
                    f"Servo: {last_sent_position:+6.1f} deg | "
                    f"Target: {servo_position:+6.1f} deg"
                )
                last_report = now

            time.sleep(0.01)

    finally:
        servo.mid()
        time.sleep(0.3)
        servo.detach()
        log.info("Done.")


if __name__ == "__main__":
    main()
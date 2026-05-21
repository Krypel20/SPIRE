#!/usr/bin/env python3
"""
SPIRE Yaw Stabilization — Dual IMU
Reads two IMUs via shared memory:
  - Platform IMU (spire_imu_platform): measures capsule/base rotation
  - Camera IMU (spire_imu_camera): measures camera orientation

Servo correction = platform rotation - camera rotation
This eliminates the servo-fighting-itself problem.

Can also run with single IMU (rate control fallback).

Requires imu_reader.py instances running (or process_manager.py).

Usage:
  python3 stabilize_demo.py
  python3 stabilize_demo.py --gain 5.0 --deadband 2.0
  python3 stabilize_demo.py --single    # Single IMU fallback
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
SHM_CAMERA = "spire_imu_camera"
SHM_PLATFORM = "spire_imu_platform"
SHM_SINGLE = "spire_imu_state"

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
# Shared memory readers
# ---------------------------------------------------------------------------

class SHMReader:
    """Read IMU state from a named shared memory block."""

    def __init__(self, name):
        self.name = name
        self._shm = None

    def read(self):
        try:
            from multiprocessing import shared_memory, resource_tracker
            if self._shm is None:
                self._shm = shared_memory.SharedMemory(
                    name=self.name, create=False
                )
                resource_tracker.unregister(
                    f"/{self.name}", "shared_memory"
                )
            raw = bytes(self._shm.buf[:self._shm.size]).rstrip(b'\x00')
            if raw:
                return json.loads(raw.decode("utf-8"))
        except FileNotFoundError:
            self._shm = None
        except Exception:
            self._shm = None
        return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Yaw Stabilization — Dual IMU",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Dual IMU mode (default):
  Platform IMU detects base rotation → servo compensates.
  Camera IMU confirms camera is stable → feedback loop.

Single IMU mode (--single):
  Rate control fallback with one IMU on camera.

Examples:
  %(prog)s                              # Dual IMU
  %(prog)s --gain 5.0 --deadband 2.0    # Tune response
  %(prog)s --single                     # Single IMU fallback
  %(prog)s --invert                     # Flip servo direction
  %(prog)s --yaw-axis gz                # Use gyro Z for yaw (default)
        """
    )

    parser.add_argument("--pin", type=int, default=SERVO_PIN,
                        help=f"Servo GPIO pin (default: {SERVO_PIN})")
    parser.add_argument("--gain", type=float, default=5.0,
                        help="Rate gain (default: 5.0)")
    parser.add_argument("--deadband", type=float, default=1.5,
                        help="Gyro deadband deg/s (default: 1.5)")
    parser.add_argument("--threshold", type=float, default=2.0,
                        help="Min servo angle change deg (default: 2.0)")
    parser.add_argument("--range", type=float, default=135.0,
                        help="Servo range +/- degrees (default: 135)")
    parser.add_argument("--invert", action="store_true",
                        help="Invert servo direction")
    parser.add_argument("--single", action="store_true",
                        help="Single IMU mode (rate control)")
    parser.add_argument("--platform-yaw", default="gz",
                        choices=["gx", "gy", "gz"],
                        help="Platform IMU yaw axis (default: gz)")
    parser.add_argument("--camera-yaw", default="gz",
                        choices=["gx", "gy", "gz"],
                        help="Camera IMU yaw axis (default: gz)")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Yaw Stabilization")
    log.info("=" * 40)

    # Axis mapping
    axis_map = {"gx": "gyro_x", "gy": "gyro_y", "gz": "gyro_z"}
    platform_yaw_key = axis_map[args.platform_yaw]
    camera_yaw_key = axis_map[args.camera_yaw]

    # Connect to shared memory
    if args.single:
        log.info("Mode: Single IMU (rate control)")
        shm_camera = SHMReader(SHM_SINGLE)
        shm_platform = None

        imu = shm_camera.read()
        if imu is None:
            log.error("IMU not available. Start imu_reader.py first.")
            sys.exit(1)
        log.info("Camera IMU connected")
    else:
        log.info("Mode: Dual IMU (platform + camera)")
        shm_platform = SHMReader(SHM_PLATFORM)
        shm_camera = SHMReader(SHM_CAMERA)

        plat = shm_platform.read()
        cam = shm_camera.read()

        if plat is None:
            log.error(f"Platform IMU not available ({SHM_PLATFORM}).")
            log.error("Start: imu_reader.py --sensor lsm9ds1 "
                      "--shm-name spire_imu_platform")
            sys.exit(1)
        if cam is None:
            log.error(f"Camera IMU not available ({SHM_CAMERA}).")
            log.error("Start: imu_reader.py --sensor icm20948 "
                      "--shm-name spire_imu_camera")
            sys.exit(1)
        log.info("Platform IMU connected")
        log.info("Camera IMU connected")

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
    log.info(f"Platform yaw axis: {args.platform_yaw}  "
             f"Camera yaw axis: {args.camera_yaw}")
    log.info(f"Invert: {'yes' if args.invert else 'no'}")
    log.info("")
    log.info("Ctrl+C to stop.")
    log.info("")

    # State
    servo_position = 0.0
    last_sent_position = 0.0
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

            if dt > 0.1:
                dt = 0.01

            if args.single:
                # Single IMU: rate control
                imu = shm_camera.read()
                if imu is None:
                    time.sleep(0.01)
                    continue

                gyro_rate = imu.get(camera_yaw_key, 0.0)

                if abs(gyro_rate) < args.deadband:
                    gyro_rate = 0.0

                servo_position += direction * gyro_rate * dt * args.gain

            else:
                # Dual IMU: platform rotation drives servo,
                # camera IMU confirms stabilization
                plat = shm_platform.read()
                cam = shm_camera.read()

                if plat is None or cam is None:
                    time.sleep(0.01)
                    continue

                platform_rate = plat.get(platform_yaw_key, 0.0)
                camera_rate = cam.get(camera_yaw_key, 0.0)

                # Platform rotation is the disturbance to counteract
                # Camera rotation is what we want to minimize
                if abs(platform_rate) < args.deadband:
                    platform_rate = 0.0
                if abs(camera_rate) < args.deadband:
                    camera_rate = 0.0

                # Use platform rate as primary input
                # Subtract camera rate as feedback correction
                effective_rate = platform_rate - camera_rate

                servo_position += direction * effective_rate * dt * args.gain

            # Clamp
            servo_position = max(-args.range, min(args.range, servo_position))

            # Update servo if change significant
            if abs(servo_position - last_sent_position) > args.threshold:
                servo_value = servo_position / args.range
                servo_value = max(-1.0, min(1.0, servo_value))
                servo.value = servo_value
                last_sent_position = servo_position

            # Report
            if now - last_report >= 0.5:
                if args.single:
                    log.info(
                        f"Gyro: {imu.get(camera_yaw_key, 0):+6.1f} deg/s | "
                        f"Servo: {last_sent_position:+6.1f} deg"
                    )
                else:
                    log.info(
                        f"Platform: {plat.get(platform_yaw_key, 0):+6.1f} deg/s | "
                        f"Camera: {cam.get(camera_yaw_key, 0):+6.1f} deg/s | "
                        f"Servo: {last_sent_position:+6.1f} deg"
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
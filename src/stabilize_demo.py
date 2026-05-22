#!/usr/bin/env python3
"""
SPIRE Yaw Stabilization — Magnetometer Heading PID
Uses LSM9DS1 magnetometer for absolute heading reference.
PID controller minimizes heading error between platform and reference.

Modes:
  --mode mag     Magnetometer heading PID (default, requires --mag on imu_reader)
  --mode rate    Rate control fallback (gyro only, no magnetometer)

Requires imu_reader.py instances running with --mag flag for platform IMU.

Usage:
  # T1: camera IMU (no mag needed)
  python3 src/imu_reader.py --sensor icm20948 -r 500 --shm-name spire_imu_camera --cal config/imu_calibration.json

  # T2: platform IMU WITH magnetometer
  python3 src/imu_reader.py --sensor lsm9ds1 -a 0x6B --mag -r 200 --shm-name spire_imu_platform --cal config/lsm9ds1_calibration.json

  # T3: stabilization
  python3 src/stabilize_demo.py --kp 2.0 --kd 0.5
"""

import time
import sys
import math
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
# Shared memory reader
# ---------------------------------------------------------------------------

class SHMReader:
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
# Heading calculation
# ---------------------------------------------------------------------------

def compute_heading(mag_x, mag_y):
    """Compute heading from magnetometer X and Y.

    For flat mounting (gravity on Z), heading is:
      heading = atan2(mag_y, mag_x)

    Returns heading in degrees, 0-360.
    """
    heading_rad = math.atan2(mag_y, mag_x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


def heading_error(current, reference):
    """Compute shortest angular difference between two headings.

    Handles wraparound (e.g. 350° → 10° = +20°, not -340°).

    Returns error in degrees, range -180 to +180.
    """
    err = current - reference
    while err > 180:
        err -= 360
    while err < -180:
        err += 360
    return err


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Yaw Stabilization — Magnetometer Heading PID",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
PID Tuning:
  --kp    Reacts to heading error (degrees off target).
          Higher = stronger correction. Start: 1.0-2.0
  --ki    Eliminates steady-state offset.
          Higher = faster correction of drift. Start: 0.0-0.1
  --kd    Damping from gyro rate. Reduces overshoot.
          Higher = smoother stops. Start: 0.3-1.0

Examples:
  %(prog)s --kp 2.0 --kd 0.5                    # PD control
  %(prog)s --kp 2.0 --ki 0.05 --kd 0.5          # Full PID
  %(prog)s --mode rate --gain 1.0 --invert       # Rate fallback
        """
    )

    # Mode
    parser.add_argument("--mode", default="mag",
                        choices=["mag", "rate"],
                        help="Control mode (default: mag)")

    # Hardware
    parser.add_argument("--pin", type=int, default=SERVO_PIN,
                        help=f"Servo GPIO pin (default: {SERVO_PIN})")
    parser.add_argument("--range", type=float, default=135.0,
                        help="Servo range +/- degrees (default: 135)")

    # PID gains (mag mode)
    parser.add_argument("--kp", type=float, default=1.5,
                        help="Proportional gain (default: 1.5)")
    parser.add_argument("--ki", type=float, default=0.0,
                        help="Integral gain (default: 0.0)")
    parser.add_argument("--kd", type=float, default=0.5,
                        help="Derivative gain (default: 0.5)")

    # Rate mode params
    parser.add_argument("--gain", type=float, default=1.0,
                        help="Rate mode gain (default: 1.0)")

    # Filtering
    parser.add_argument("--deadband", type=float, default=0.5,
                        help="Gyro deadband deg/s (default: 0.5)")
    parser.add_argument("--threshold", type=float, default=1.0,
                        help="Min servo change deg (default: 1.0)")
    parser.add_argument("--heading-deadband", type=float, default=1.0,
                        help="Heading error deadband deg (default: 1.0)")

    # Direction
    parser.add_argument("--invert", action="store_true",
                        help="Invert servo direction")

    # Yaw axis (rate mode)
    parser.add_argument("--platform-yaw", default="gz",
                        choices=["gx", "gy", "gz"],
                        help="Platform IMU yaw axis (default: gz)")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Yaw Stabilization")
    log.info("=" * 40)

    # Connect shared memory
    shm_platform = SHMReader(SHM_PLATFORM)
    shm_camera = SHMReader(SHM_CAMERA)

    plat = shm_platform.read()
    if plat is None:
        log.error("Platform IMU not available.")
        sys.exit(1)
    log.info("Platform IMU connected")

    cam = shm_camera.read()
    if cam is not None:
        log.info("Camera IMU connected")
    else:
        log.info("Camera IMU not available (monitoring disabled)")

    # Check magnetometer data
    if args.mode == "mag":
        if plat.get("mag_x", 0) == 0 and plat.get("mag_y", 0) == 0:
            log.error("No magnetometer data. Start platform imu_reader with --mag")
            log.error("Falling back to rate mode.")
            args.mode = "rate"

    # Initialize servo
    factory = LGPIOFactory()
    servo = Servo(
        args.pin,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
    )
    servo.mid()

    direction = -1.0 if not args.invert else 1.0
    axis_map = {"gx": "gyro_x", "gy": "gyro_y", "gz": "gyro_z"}
    platform_yaw_key = axis_map[args.platform_yaw]

    if args.mode == "mag":
        log.info(f"Mode: Magnetometer Heading PID")
        log.info(f"PID: Kp={args.kp}  Ki={args.ki}  Kd={args.kd}")
    else:
        log.info(f"Mode: Rate Control")
        log.info(f"Gain: {args.gain}  Yaw axis: {args.platform_yaw}")

    log.info(f"Servo GPIO {args.pin}  Range: +/-{args.range}")
    log.info(f"Deadband: {args.deadband} deg/s  Threshold: {args.threshold} deg")
    log.info(f"Invert: {'yes' if args.invert else 'no'}")
    log.info("")
    log.info("Ctrl+C to stop.")
    log.info("")

    # State
    servo_position = 0.0
    last_sent_position = 0.0
    integral = 0.0
    heading_ref = None
    last_time = time.monotonic()
    last_report = time.monotonic()
    running = True

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

            plat = shm_platform.read()
            if plat is None:
                time.sleep(0.01)
                continue

            cam = shm_camera.read() if shm_camera else None

            if args.mode == "mag":
                # --- Magnetometer Heading PID ---

                mag_x = plat.get("mag_x", 0)
                mag_y = plat.get("mag_y", 0)
                gyro_rate = plat.get("gyro_z", 0.0)

                # Compute current heading
                current_heading = compute_heading(mag_x, mag_y)

                # Set reference on first valid reading
                if heading_ref is None:
                    heading_ref = current_heading
                    log.info(f"Reference heading set: {heading_ref:.1f} deg")
                    continue

                # Heading error
                error = heading_error(current_heading, heading_ref)

                # Heading deadband
                if abs(error) < args.heading_deadband:
                    error = 0.0

                # PID
                # P: proportional to heading error
                p_out = args.kp * error

                # I: accumulated heading error
                integral += error * dt
                integral = max(-args.range, min(args.range, integral))
                i_out = args.ki * integral

                # D: damping from gyro rate (not heading derivative)
                # Gyro gives cleaner rate signal than differentiating heading
                if abs(gyro_rate) < args.deadband:
                    gyro_rate = 0.0
                d_out = args.kd * gyro_rate

                # Combine
                servo_position = direction * (p_out + i_out + d_out)
                servo_position = max(-args.range, min(args.range, servo_position))

                # Report
                if now - last_report >= 0.5:
                    cam_gz = cam.get("gyro_z", 0) if cam else 0
                    log.info(
                        f"Heading: {current_heading:5.1f} deg | "
                        f"Ref: {heading_ref:5.1f} deg | "
                        f"Error: {error:+6.1f} deg | "
                        f"Servo: {last_sent_position:+6.1f} deg | "
                        f"Camera gz: {cam_gz:+6.1f} deg/s"
                    )
                    last_report = now

            else:
                # --- Rate Control Fallback ---

                gyro_rate = plat.get(platform_yaw_key, 0.0)
                if abs(gyro_rate) < args.deadband:
                    gyro_rate = 0.0

                servo_position += direction * gyro_rate * dt * args.gain
                servo_position = max(-args.range, min(args.range, servo_position))

                if now - last_report >= 0.5:
                    cam_gz = cam.get("gyro_z", 0) if cam else 0
                    log.info(
                        f"Platform: {plat.get(platform_yaw_key, 0):+6.1f} deg/s | "
                        f"Camera: {cam_gz:+6.1f} deg/s | "
                        f"Servo: {last_sent_position:+6.1f} deg"
                    )
                    last_report = now

            # Update servo if change significant
            if abs(servo_position - last_sent_position) > args.threshold:
                servo_value = servo_position / args.range
                servo_value = max(-1.0, min(1.0, servo_value))
                servo.value = servo_value
                last_sent_position = servo_position

            time.sleep(0.01)

    finally:
        servo.detach()
        log.info("Servo detached. Done.")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
SPIRE Yaw Stabilization Demo v3
Reads IMU gyro Z from shared memory, integrates to yaw angle,
moves servo to counteract rotation using PID control.

All PID parameters tunable from command line for easy experimentation.

Requires imu_reader.py running in a separate terminal.

Usage:
  Terminal 1: python3 src/imu_reader.py -r 500
  Terminal 2: python3 src/stabilize_demo.py
  Terminal 2: python3 src/stabilize_demo.py --kp 0.8 --kd 0.1
  Terminal 2: python3 src/stabilize_demo.py --kp 0.3 --deadband 2.0 --servo-threshold 5.0
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
        description="SPIRE Yaw Stabilization Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
PID Tuning Guide:
  --kp    Proportional: reacts to current error (how far off)
          Higher = stronger response, too high = oscillation
  --ki    Integral: reacts to accumulated error (steady-state offset)
          Higher = eliminates drift, too high = overshoot
  --kd    Derivative: reacts to rate of change (damping)
          Higher = less overshoot, too high = jitter from noise

Examples:
  %(prog)s --kp 0.5                         # P-only, gentle
  %(prog)s --kp 0.8 --kd 0.1               # PD, responsive with damping
  %(prog)s --kp 0.5 --ki 0.05 --kd 0.1     # Full PID
  %(prog)s --kp 0.3 --servo-threshold 5.0   # Smooth, less updates
        """
    )

    # Hardware
    parser.add_argument("--pin", type=int, default=SERVO_PIN,
                        help=f"GPIO pin (default: {SERVO_PIN})")
    parser.add_argument("--range", type=float, default=90.0,
                        help="Servo range +/- degrees (default: 90)")

    # PID gains
    parser.add_argument("--kp", type=float, default=0.5,
                        help="Proportional gain (default: 0.5)")
    parser.add_argument("--ki", type=float, default=0.0,
                        help="Integral gain (default: 0.0)")
    parser.add_argument("--kd", type=float, default=0.0,
                        help="Derivative gain (default: 0.0)")

    # Filtering
    parser.add_argument("--deadband", type=float, default=1.5,
                        help="Gyro deadband in deg/s (default: 1.5)")
    parser.add_argument("--servo-threshold", type=float, default=2.0,
                        help="Min angle change to update servo in degrees "
                             "(default: 2.0)")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Yaw Stabilization Demo")
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

    log.info(f"PID:  Kp={args.kp}  Ki={args.ki}  Kd={args.kd}")
    log.info(f"Deadband: {args.deadband} deg/s")
    log.info(f"Servo threshold: {args.servo_threshold} deg")
    log.info(f"Servo range: +/-{args.range} deg")
    log.info("")
    log.info("Rotate the module — servo counteracts yaw.")
    log.info("Ctrl+C to stop.")
    log.info("")

    # PID state
    yaw_angle = 0.0
    integral = 0.0
    prev_error = 0.0
    last_servo_angle = 0.0
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

            # Read IMU
            imu = read_imu()
            if imu is None:
                time.sleep(0.01)
                continue

            gyro_z = imu.get("gyro_z", 0.0)

            # Deadband — ignore noise below threshold
            if abs(gyro_z) < args.deadband:
                gyro_z = 0.0

            # Integrate gyro → yaw angle
            yaw_angle += gyro_z * dt
            yaw_angle = max(-args.range, min(args.range, yaw_angle))

            # PID controller
            # Error: how far yaw has drifted from zero
            error = -yaw_angle

            # P: proportional to current error
            p_out = args.kp * error

            # I: accumulated error over time (eliminates steady-state offset)
            integral += error * dt
            integral = max(-args.range, min(args.range, integral))
            i_out = args.ki * integral

            # D: rate of change of error (damping, reduces overshoot)
            d_out = 0.0
            if dt > 0:
                d_out = args.kd * (error - prev_error) / dt
            prev_error = error

            # Combined PID output in degrees
            servo_angle = p_out + i_out + d_out
            servo_angle = max(-args.range, min(args.range, servo_angle))

            # Only update servo if change exceeds threshold
            # Prevents constant PWM restarts that cause jitter
            if abs(servo_angle - last_servo_angle) > args.servo_threshold:
                servo_value = servo_angle / args.range
                servo_value = max(-1.0, min(1.0, servo_value))
                servo.value = servo_value
                last_servo_angle = servo_angle

            # Report every 0.5s
            if now - last_report >= 0.5:
                log.info(
                    f"Yaw: {yaw_angle:+7.1f} deg | "
                    f"Gyro Z: {imu.get('gyro_z', 0):+6.1f} deg/s | "
                    f"Servo: {last_servo_angle:+6.1f} deg | "
                    f"PID: P={p_out:+5.1f} I={i_out:+5.1f} D={d_out:+5.1f}"
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
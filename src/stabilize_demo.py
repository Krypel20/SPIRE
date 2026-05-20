#!/usr/bin/env python3
"""
SPIRE Yaw Stabilization Demo
Reads IMU gyro Z from shared memory, integrates to yaw angle,
moves servo to counteract rotation.

Requires imu_reader.py running in a separate terminal.

Usage:
  Terminal 1: python3 src/imu_reader.py -r 500
  Terminal 2: python3 src/stabilize_demo.py
"""

import time
import sys
import json
import signal
import logging
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

SERVO_PIN = 12
IMU_SHM_NAME = "spire_imu_state"

# Servo range in degrees (±SERVO_RANGE from center)
SERVO_RANGE_DEG = 90.0

# PID gains — start with P only, add I and D if needed
KP = 1.0    # Proportional gain
KI = 0.0    # Integral gain (0 = disabled)
KD = 0.0    # Derivative gain (0 = disabled)

# Deadband — ignore small movements below this threshold (°/s)
GYRO_DEADBAND_DPS = 0.3

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
# IMU reader (from shared memory)
# ---------------------------------------------------------------------------

_imu_shm = None


def read_imu():
    """Read current IMU state from shared memory."""
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
# Main stabilization loop
# ---------------------------------------------------------------------------

def main():
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Yaw Stabilization Demo")
    log.info("=" * 40)

    # Check IMU connection
    imu = read_imu()
    if imu is None:
        log.error("IMU not available. Start imu_reader.py first:")
        log.error("  python3 src/imu_reader.py -r 500")
        sys.exit(1)
    log.info("IMU connected")

    # Initialize servo
    factory = LGPIOFactory()
    servo = Servo(
        SERVO_PIN,
        pin_factory=factory,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
    )
    servo.mid()
    log.info(f"Servo on GPIO {SERVO_PIN} — centered")

    # State
    yaw_angle = 0.0         # Integrated yaw in degrees
    integral = 0.0          # PID integral term
    prev_error = 0.0        # PID derivative term
    last_time = time.monotonic()

    running = True

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    log.info("")
    log.info(f"PID: Kp={KP}  Ki={KI}  Kd={KD}")
    log.info(f"Servo range: ±{SERVO_RANGE_DEG}°")
    log.info(f"Gyro deadband: {GYRO_DEADBAND_DPS} °/s")
    log.info("")
    log.info("Rotate the module — servo will counteract yaw rotation.")
    log.info("Press Ctrl+C to stop.")
    log.info("")

    report_interval = 0.5
    last_report = time.monotonic()

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

            # Get yaw rate (gyro Z axis)
            gyro_z = imu.get("gyro_z", 0.0)

            # Apply deadband
            if abs(gyro_z) < GYRO_DEADBAND_DPS:
                gyro_z = 0.0

            # Integrate gyro Z → yaw angle
            yaw_angle += gyro_z * dt

            # Clamp yaw angle to servo range
            yaw_angle = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, yaw_angle))

            # PID controller
            # Target: yaw_angle = 0 (counteract all rotation)
            error = -yaw_angle  # Negative because servo must go opposite

            # P term
            p_out = KP * error

            # I term
            integral += error * dt
            integral = max(-SERVO_RANGE_DEG, min(SERVO_RANGE_DEG, integral))
            i_out = KI * integral

            # D term
            d_out = 0.0
            if dt > 0:
                d_out = KD * (error - prev_error) / dt
            prev_error = error

            # PID output in degrees
            pid_output = p_out + i_out + d_out

            # Convert to servo value (-1.0 to 1.0)
            servo_value = pid_output / SERVO_RANGE_DEG
            servo_value = max(-1.0, min(1.0, servo_value))

            # Move servo
            servo.value = servo_value

            # Report
            if now - last_report >= report_interval:
                servo_angle = servo_value * SERVO_RANGE_DEG
                log.info(
                    f"Yaw: {yaw_angle:+7.1f}° | "
                    f"Gyro Z: {imu.get('gyro_z', 0):+6.1f} °/s | "
                    f"Servo: {servo_angle:+6.1f}° | "
                    f"PID: {pid_output:+6.1f}"
                )
                last_report = now

            # Loop rate ~100 Hz
            time.sleep(0.01)

    finally:
        servo.mid()
        time.sleep(0.3)
        servo.detach()
        log.info("Servo centered and detached.")
        log.info("Done.")


if __name__ == "__main__":
    main()
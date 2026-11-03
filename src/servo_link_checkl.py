#!/usr/bin/env python3
"""
SPIRE servo link check: live detection of a broken servo signal/GND contact.

The Pi only drives the PWM line; it cannot see whether the pulse reaches the
servo. This script checks it through the motion instead: with the platform
standing still, it swings the servo in a slow sine and compares, every
sine period, the commanded servo rate with the rate the IMUs measure between
camera and platform (camera gyro_z minus platform gyro_z). If the signal or
GND contact breaks, the servo stops following and the measured motion drops.

Use it to find the bad joint: let it run and wiggle one wire or connector
at a time. Each window prints OK or LOST; the status LED turns red while
the link is lost. The first 3 s (platform still, link good) set the
reference.

Usage:
  python3 src/servo_link_check.py                   # until Ctrl+C
  python3 src/servo_link_check.py --amp 20 --freq 0.5 --duration 120
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_capture as fc  # noqa: E402
from status_led import StatusLED  # noqa: E402

LOST_RATIO = 0.3     # window response below 30% of the reference = link lost
REF_WINDOWS = 3      # windows used to set the reference at start


def main():
    ap = argparse.ArgumentParser(description="SPIRE servo link check")
    ap.add_argument("--amp", type=float, default=20.0,
                    help="Sine amplitude in commanded deg (default 20)")
    ap.add_argument("--freq", type=float, default=0.5,
                    help="Sine frequency in Hz (default 0.5)")
    ap.add_argument("--window", type=float, default=None,
                    help="Evaluation window in s (default: one sine period)")
    ap.add_argument("-d", "--duration", type=float, default=0,
                    help="Stop after this many seconds (0 = until Ctrl+C)")
    ap.add_argument("--no-led", action="store_true")
    args = ap.parse_args()

    led = StatusLED(enabled=not args.no_led)
    led.set("INIT")
    cam = fc.IMUProcess("imu_camera", "icm20948", "spire_imu_camera",
                        "config/imu_calibration.json", rate=500)
    plat = fc.IMUProcess("imu_platform", "lsm9ds1", "spire_imu_platform",
                         "config/lsm9ds1_calibration.json", rate=200,
                         address="0x6B", mag=True)
    servo = None
    lost_windows = total_windows = 0
    try:
        cam.start()
        time.sleep(1)
        plat.start()
        time.sleep(2)
        shm_cam = fc.SHMReader("spire_imu_camera")
        shm_plat = fc.SHMReader("spire_imu_platform")
        for _ in range(20):
            if shm_cam.read() and shm_plat.read():
                break
            time.sleep(0.5)
        else:
            sys.exit("IMU data not available (check I2C)")

        servo = fc.ServoController(pin=12)
        servo.threshold_deg = 0.0          # send every update, smooth sine
        servo._drive(0.0)
        servo.last_sent = 0.0
        time.sleep(0.5)

        w = 2 * math.pi * args.freq
        window = args.window or 1.0 / args.freq
        print(f"Swinging +-{args.amp:g} deg (commanded) at {args.freq:g} Hz. "
              f"Keep the platform still. First {REF_WINDOWS} windows set the "
              f"reference, then wiggle the wires one by one. Ctrl+C to stop.",
              flush=True)
        led.set("BUSY")

        t0 = time.monotonic()
        win_start = t0
        # Least-squares slope of measured on commanded rate, with intercept,
        # so a constant gyro bias between the two IMUs does not leak in.
        n = sx = sy = sxx = sxy = 0.0
        ref = None
        ref_vals = []
        last_c = last_p = None
        while True:
            now = time.monotonic()
            t = now - t0
            servo.set_position(args.amp * math.sin(w * t))
            cmd_rate = args.amp * w * math.cos(w * t)       # deg/s commanded
            c, p = shm_cam.read(), shm_plat.read()
            if c and p and (c is not last_c or p is not last_p):
                rel = c.get("gyro_z", 0.0) - p.get("gyro_z", 0.0)
                n += 1
                sx += cmd_rate
                sy += rel
                sxx += cmd_rate * cmd_rate
                sxy += rel * cmd_rate
                last_c, last_p = c, p

            if now - win_start >= window:
                var = sxx - sx * sx / n if n else 0.0
                k = (sxy - sx * sy / n) / var if var else 0.0   # measured / commanded
                total_windows += 1
                if ref is None:
                    ref_vals.append(k)
                    print(f"{t:7.1f} s  reference window {len(ref_vals)}: "
                          f"response {k:+.2f}", flush=True)
                    if len(ref_vals) == REF_WINDOWS:
                        ref = sorted(ref_vals)[1]
                        if abs(ref) < 0.2:
                            print("Reference response is too small: the servo "
                                  "is not moving the camera now. Fix the link "
                                  "and start again.")
                            break
                        print(f"Reference {ref:+.2f}. Now wiggle the wires.",
                              flush=True)
                        led.set("HOLD_STILL")
                else:
                    ratio = k / ref
                    ok = ratio >= LOST_RATIO
                    if not ok:
                        lost_windows += 1
                    led.set("HOLD_STILL" if ok else "CYCLE_INVALID")
                    print(f"{t:7.1f} s  response {ratio:5.0%}  "
                          f"{'OK' if ok else 'LOST  <<<'}   "
                          f"(lost {lost_windows} of {total_windows - REF_WINDOWS})",
                          flush=True)
                n = sx = sy = sxx = sxy = 0.0
                win_start = now
                if args.duration and t >= args.duration:
                    break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if servo is not None:
            servo.slow_center(duration=1.0)
            servo.detach()
        cam.stop()
        plat.stop()
        led.close()
        checked = max(0, total_windows - REF_WINDOWS)
        print(f"\nLost windows: {lost_windows} of {checked}")


if __name__ == "__main__":
    main()
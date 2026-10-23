#!/usr/bin/env python3
"""
SPIRE servo hold test (hardware PWM, GPIO12 / pin 32, same driver as flight).

Moves the servo through a fixed sequence and holds each position, so the
operator can check by hand and with a multimeter whether the servo keeps
its holding torque:

  0 -> +60 (hold) -> 0 -> +134 (hold long) -> 0 -> -134 (hold long) -> 0

During each hold:
  * press the camera holder gently by hand: a powered servo resists and
    springs back; a servo in overload protection or without supply goes limp
  * measure the voltage at the servo connector (red/brown wires)

Moves are ramped at a moderate speed so nothing is slammed into an end stop.
Ctrl+C at any time centers the servo and releases it.

Endpoint mode (--endpoint) steps toward each end in small increments and
pauses at every step, to find the commanded angle where the servo reaches
its physical end stop (a jerk or buzz on arrival, no further motion).

Detach test (--detach-test N) holds +60 deg, releases the servo, waits and
attaches again, N times. With the old detach (--unsafe-detach: PWM disabled
straight away) a pulse can be cut short and the servo twitches on release
and jumps back on the next attach. Count the twitches for both variants.

Usage:
  python3 src/servo_hold_test.py
  python3 src/servo_hold_test.py --hold 20 --speed 45
  python3 src/servo_hold_test.py --endpoint
  python3 src/servo_hold_test.py --detach-test 40
  python3 src/servo_hold_test.py --detach-test 40 --unsafe-detach
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_capture as fc  # noqa: E402  (reuses the flight ServoController)


def ramp(servo, target, speed_dps):
    """Move from the last sent angle to target at speed_dps (50 Hz steps)."""
    start = servo.last_sent
    steps = max(1, int(abs(target - start) / speed_dps * 50))
    for i in range(1, steps + 1):
        servo._drive(start + (target - start) * i / steps)
        time.sleep(0.02)
    servo.last_sent = target
    servo.current_position = target


def hold(seconds, label):
    print(f"  HOLD {label}: {seconds:.0f} s  -> press the holder by hand, "
          f"measure servo voltage", flush=True)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        left = end - time.monotonic()
        print(f"\r    {left:4.1f} s left ", end="", flush=True)
        time.sleep(0.2)
    print()


def detach_test(servo, n, unsafe):
    mode = "UNSAFE (immediate disable)" if unsafe else "safe (duty 0, wait, disable)"
    print(f"Detach test, {n} cycles, {mode}. Count twitches on release and "
          f"jumps on re-attach.", flush=True)
    try:
        for i in range(1, n + 1):
            servo._drive(60.0)            # (re)attach at +60
            servo.last_sent = 60.0
            time.sleep(0.6)
            if unsafe:
                servo.pwm.stop()
                servo._attached = False
            else:
                servo.detach()
            print(f"  cycle {i:3d}/{n} released", flush=True)
            time.sleep(0.6)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        servo._drive(60.0)
        servo.last_sent = 60.0
        servo.slow_center(duration=1.5)
        servo.detach()


def main():
    ap = argparse.ArgumentParser(description="SPIRE servo hold test")
    ap.add_argument("--hold", type=float, default=15.0,
                    help="Hold time at the end positions in s (default 15)")
    ap.add_argument("--speed", type=float, default=60.0,
                    help="Ramp speed in commanded deg/s (default 60)")
    ap.add_argument("--endpoint", action="store_true",
                    help="Step 100..134 deg on each side, 3 s per step")
    ap.add_argument("--detach-test", type=int, default=0, metavar="N",
                    help="Hold +60, release, re-attach; repeat N times")
    ap.add_argument("--unsafe-detach", action="store_true",
                    help="With --detach-test: disable PWM immediately "
                         "(previous behaviour) instead of ServoController.detach()")
    args = ap.parse_args()

    servo = fc.ServoController(pin=12)
    if args.detach_test:
        detach_test(servo, args.detach_test, args.unsafe_detach)
        return
    if args.endpoint:
        steps = [100, 105, 110, 115, 120, 125, 130, 134]
        sequence = ([(a, 3.0, f"+{a}") for a in steps] + [(0.0, 3.0, "center")]
                    + [(-a, 3.0, f"-{a}") for a in steps] + [(0.0, 3.0, "center")])
    else:
        sequence = [
            (0.0, 5.0, "center"),
            (60.0, 5.0, "+60"),
            (0.0, 3.0, "center"),
            (134.0, args.hold, "+134 (flight clamp)"),
            (0.0, 3.0, "center"),
            (-134.0, args.hold, "-134 (flight clamp)"),
            (0.0, 3.0, "center"),
        ]
    try:
        servo._drive(0.0)          # attach at center
        servo.last_sent = 0.0
        for target, secs, label in sequence:
            print(f"Move to {target:+.0f} deg (commanded)", flush=True)
            ramp(servo, target, args.speed)
            hold(secs, label)
        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted, centering.")
    finally:
        servo.slow_center(duration=1.5)
        servo.detach()


if __name__ == "__main__":
    main()
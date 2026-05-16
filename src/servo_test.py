#!/usr/bin/env python3
"""
SPIRE Servo Test
Basic servo testing and manual control for TD-6622MG.
Allows testing range, speed, and precision before PID implementation.

Usage:
  python3 servo_test.py                  # Interactive mode
  python3 servo_test.py --sweep          # Full range sweep
  python3 servo_test.py --center         # Move to center and hold
  python3 servo_test.py --angle 45       # Move to specific angle
"""

import sys
import time
import argparse
import logging
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_PIN = 18

# TD-6622MG specs:
# - Pulse range: 500-2500 µs (standard servo)
# - Center: 1500 µs
# - Range: ~270° (some servos), typically 180° usable
MIN_PULSE_MS = 0.5
MAX_PULSE_MS = 2.5

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.servo_test")


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
# Servo wrapper
# ---------------------------------------------------------------------------

class ServoController:
    """Simple servo control wrapper using gpiozero."""

    def __init__(self, pin=DEFAULT_PIN):
        self.factory = LGPIOFactory()
        self.servo = Servo(
            pin,
            pin_factory=self.factory,
            min_pulse_width=MIN_PULSE_MS / 1000,
            max_pulse_width=MAX_PULSE_MS / 1000,
        )
        self.current_value = 0.0  # -1.0 to 1.0
        log.info(f"Servo initialized on GPIO {pin}")
        log.info(f"Pulse range: {MIN_PULSE_MS}-{MAX_PULSE_MS} ms")

    def set_value(self, value):
        """Set servo position.

        Args:
            value: -1.0 (min) to 1.0 (max), 0.0 = center
        """
        value = max(-1.0, min(1.0, value))
        self.servo.value = value
        self.current_value = value

    def set_angle(self, angle_deg):
        """Set servo to angle in degrees.

        Args:
            angle_deg: -90 to +90 (0 = center)
        """
        value = angle_deg / 90.0
        self.set_value(value)

    def center(self):
        """Move to center position."""
        self.set_value(0.0)

    def detach(self):
        """Stop sending PWM signal (servo goes limp)."""
        self.servo.detach()

    def close(self):
        """Clean up."""
        self.detach()
        log.info("Servo detached")


# ---------------------------------------------------------------------------
# Test modes
# ---------------------------------------------------------------------------

def test_sweep(ctrl, speed=0.5):
    """Sweep servo through full range.

    Args:
        ctrl: ServoController instance
        speed: Sweep speed (seconds per full sweep)
    """
    log.info(f"Sweep test (speed: {speed}s per direction)")
    log.info("Press Ctrl+C to stop\n")

    steps = 50
    delay = speed / steps

    try:
        while True:
            # Center to max
            for i in range(steps + 1):
                value = i / steps
                ctrl.set_value(value)
                angle = value * 90
                print(f"\r  Angle: {angle:+6.1f}°  Value: {value:+.2f}", end="")
                time.sleep(delay)

            # Max to min
            for i in range(steps * 2 + 1):
                value = 1.0 - (i / steps)
                ctrl.set_value(value)
                angle = value * 90
                print(f"\r  Angle: {angle:+6.1f}°  Value: {value:+.2f}", end="")
                time.sleep(delay)

            # Min to center
            for i in range(steps + 1):
                value = -1.0 + (i / steps)
                ctrl.set_value(value)
                angle = value * 90
                print(f"\r  Angle: {angle:+6.1f}°  Value: {value:+.2f}", end="")
                time.sleep(delay)

    except KeyboardInterrupt:
        print("\n")
        log.info("Sweep stopped")


def test_steps(ctrl):
    """Step through predefined positions."""
    positions = [
        (0, "Center (0°)"),
        (30, "Right 30°"),
        (60, "Right 60°"),
        (90, "Right 90°"),
        (0, "Center (0°)"),
        (-30, "Left 30°"),
        (-60, "Left 60°"),
        (-90, "Left 90°"),
        (0, "Center (0°)"),
    ]

    log.info("Step test — moving through predefined angles")
    log.info("Press Ctrl+C to stop\n")

    try:
        for angle, label in positions:
            ctrl.set_angle(angle)
            log.info(f"  {label}")
            time.sleep(2)
    except KeyboardInterrupt:
        print()
        log.info("Step test stopped")


def test_interactive(ctrl):
    """Interactive servo control from keyboard."""
    log.info("Interactive mode")
    log.info("Commands:")
    log.info("  number  — set angle (-90 to 90)")
    log.info("  c       — center")
    log.info("  l       — left 10°")
    log.info("  r       — right 10°")
    log.info("  s       — sweep")
    log.info("  p       — step test")
    log.info("  d       — detach (servo goes limp)")
    log.info("  q       — quit\n")

    current_angle = 0.0
    ctrl.center()

    try:
        while True:
            cmd = input(f"[{current_angle:+.1f}°] > ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'c':
                current_angle = 0
                ctrl.center()
                log.info("Center")
            elif cmd == 'l':
                current_angle = max(-90, current_angle - 10)
                ctrl.set_angle(current_angle)
                log.info(f"Angle: {current_angle:+.1f}°")
            elif cmd == 'r':
                current_angle = min(90, current_angle + 10)
                ctrl.set_angle(current_angle)
                log.info(f"Angle: {current_angle:+.1f}°")
            elif cmd == 's':
                test_sweep(ctrl)
                ctrl.center()
                current_angle = 0
            elif cmd == 'p':
                test_steps(ctrl)
                current_angle = 0
            elif cmd == 'd':
                ctrl.detach()
                log.info("Detached")
            else:
                try:
                    angle = float(cmd)
                    if -90 <= angle <= 90:
                        current_angle = angle
                        ctrl.set_angle(angle)
                        log.info(f"Angle: {current_angle:+.1f}°")
                    else:
                        log.warning("Angle must be -90 to 90")
                except ValueError:
                    log.warning(f"Unknown command: {cmd}")

    except (KeyboardInterrupt, EOFError):
        print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Servo Test — TD-6622MG",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Interactive mode
  %(prog)s --sweep            # Full range sweep
  %(prog)s --steps            # Step through angles
  %(prog)s --center           # Move to center
  %(prog)s --angle 45         # Move to 45°
        """
    )

    parser.add_argument("--pin", type=int, default=DEFAULT_PIN,
                        help=f"GPIO pin number (default: {DEFAULT_PIN})")
    parser.add_argument("--sweep", action="store_true",
                        help="Run sweep test")
    parser.add_argument("--steps", action="store_true",
                        help="Run step test")
    parser.add_argument("--center", action="store_true",
                        help="Move to center position")
    parser.add_argument("--angle", type=float, default=None,
                        help="Move to specific angle (-90 to 90)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Sweep speed in seconds (default: 1.0)")

    args = parser.parse_args()
    setup_logging()

    log.info("=" * 40)
    log.info("SPIRE Servo Test — TD-6622MG")
    log.info("=" * 40)

    ctrl = ServoController(pin=args.pin)

    try:
        if args.center:
            ctrl.center()
            log.info("Centered. Press Ctrl+C to exit.")
            while True:
                time.sleep(1)

        elif args.angle is not None:
            ctrl.set_angle(args.angle)
            log.info(f"Angle: {args.angle:+.1f}°. Press Ctrl+C to exit.")
            while True:
                time.sleep(1)

        elif args.sweep:
            ctrl.center()
            time.sleep(0.5)
            test_sweep(ctrl, speed=args.speed)

        elif args.steps:
            test_steps(ctrl)

        else:
            test_interactive(ctrl)

    except KeyboardInterrupt:
        print()

    finally:
        ctrl.close()
        log.info("Done.")


if __name__ == "__main__":
    main()
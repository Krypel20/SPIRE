#!/usr/bin/env python3
"""
SPIRE single RGB status LED.

Hardware: HW-479 / KY-016 module, common cathode, 150 ohm per channel on
board (no external resistors needed).
  R -> GPIO17 (pin 11), G -> GPIO27 (pin 13), B -> GPIO22 (pin 15)
  cathode (-) -> GND (dedicated pin, e.g. 34)

Signalling uses colour AND rhythm. Colours that mix green with another
channel are avoided: the green die dominates, so yellow reads as green in
daylight. Palette: red, green, blue, magenta (red+blue, no green).

Colour semantics (shared by bench and flight):
  GREEN    OK / hands off
  BLUE     rotation / stabilization
  MAGENTA  system busy (init, saving)
  RED      fault; flash count encodes the category

Patterns run in a background thread; set() is non-blocking and replaces the
current pattern immediately. If gpiozero or the pins are unavailable the
class degrades to a no-op, so the payload never stops because of the LED.

Run this file directly to cycle through all states (daylight legibility test).
"""

import logging
import threading

log = logging.getLogger("spire.led")

PINS = {"red": 17, "green": 27, "blue": 22}

OFF = (0, 0, 0)
RED = (1, 0, 0)
GREEN = (0, 1, 0)
BLUE = (0, 0, 1)
MAGENTA = (1, 0, 1)


def solid(color):
    return [(color, None)]


def blink(color, hz=2.0):
    half = 0.5 / hz
    return [(color, half), (OFF, half)]


def heartbeat(color, period=2.0):
    return [(color, 0.08), (OFF, period - 0.08)]


def flashes(color, n, pause=1.2):
    """n short flashes then a pause: a countable fault code."""
    seq = []
    for _ in range(n):
        seq += [(color, 0.15), (OFF, 0.25)]
    seq[-1] = (OFF, pause)
    return seq


STATES = {
    # common
    "INIT":          blink(MAGENTA, 4),  # bringing up IMU / camera
    "BUSY":          solid(MAGENTA),     # saving frames, centering servo
    "FAULT":         blink(RED, 5),      # fatal, needs attention
    "DONE":          heartbeat(GREEN),
    # bench operator cues
    "HOLD_STILL":    solid(GREEN),       # STATIC: do not touch the platform
    "ROTATE_NOW":    blink(BLUE, 4),     # countdown: start spinning
    "ROTATE_KEEP":   solid(BLUE),        # measuring + burst: keep spinning
    "CYCLE_INVALID": solid(RED),         # shown ~2 s: repeat this cycle
    # flight (proposal; used once integrated into flight_capture)
    "READY":         heartbeat(GREEN),   # all pre-launch checks passed
    "FAULT_IMU":     flashes(RED, 2),
    "FAULT_CAMERA":  flashes(RED, 3),
    "FAULT_POWER":   flashes(RED, 4),
}


class StatusLED:
    def __init__(self, pins=PINS, enabled=True):
        self._led = None
        self._pattern = solid(OFF)
        self._changed = threading.Event()
        self._stop = threading.Event()
        self.state = None
        if enabled:
            try:
                from gpiozero import RGBLED
                self._led = RGBLED(pins["red"], pins["green"], pins["blue"],
                                   active_high=True, pwm=False)
            except Exception as e:
                log.warning(f"Status LED unavailable ({e}); continuing without it")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set(self, state):
        """Switch to a named state (or a raw [(color, seconds|None), ...])."""
        if isinstance(state, str):
            self._pattern = STATES[state]
            self.state = state
        else:
            self._pattern = state
            self.state = "custom"
        self._changed.set()

    def _write(self, color):
        if self._led is None:
            return
        try:
            self._led.color = color
        except Exception as e:
            log.warning(f"Status LED write failed ({e}); disabling LED")
            self._led = None

    def _run(self):
        while not self._stop.is_set():
            self._changed.clear()
            for color, dur in self._pattern:
                self._write(color)
                if dur is None:           # solid: hold until next set()
                    self._changed.wait()
                    break
                if self._changed.wait(dur):
                    break

    def close(self):
        self._stop.set()
        self._changed.set()
        self._thread.join(timeout=1.0)
        self._write(OFF)
        if self._led is not None:
            try:
                self._led.close()
            except Exception:
                pass


if __name__ == "__main__":
    import time
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    led = StatusLED()
    try:
        for name in STATES:
            print(f"{name:14s}")
            led.set(name)
            time.sleep(4)
    finally:
        led.close()
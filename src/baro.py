#!/usr/bin/env python3
"""
SPIRE barometer: MPRLS pressure sampled in the background, so every photo can
carry the pressure at its exposure and the pressure altitude derived from it.

Sensor: Honeywell MPRLS (Adafruit 3965), 0-25 psi absolute, I2C 0x18 on bus 1.
Protocol as in mprls_test.py: write [0xAA, 0x00, 0x00], wait, read 4 bytes
(status + 24-bit count). The sensor is not register addressed, so i2c_rdwr is
used instead of write_byte.

Altitude: pressure altitude from the US Standard Atmosphere 1976 (same layers
as ICAO ISA up to 47 km). The usual 44330*(1-(p/p0)^0.19) formula only holds
in the troposphere (0-11 km): at 54.7 hPa it gives about 18.9 km instead of
20.0 km, so the layered model is used. This is pressure altitude relative to
the standard atmosphere, not geometric (GPS) altitude.

Accuracy note: the MPRLS spec is +/-1.5 %FSS (about +/-26 hPa) over its
compensated temperature range. That is small at ground level but large near
20 km (about 55 hPa), so stratospheric altitude from this sensor needs a
ground offset check and a comparison against GPS. Precision (noise) is much
better than the accuracy figure.

Usage in code:
    baro = BaroLogger()
    baro.start()
    rec = baro.at(time.monotonic_ns())   # pressure_hpa, baro_alt_m, baro_age_ms
    baro.stop()

Standalone check:  python3 baro.py   (prints a few readings and altitudes)
"""

import collections
import logging
import math
import threading
import time

log = logging.getLogger("spire.baro")

I2C_BUS = 1
MPRLS_ADDR = 0x18
STATUS_BUSY = 0x20
STATUS_INTEGRITY = 0x04
STATUS_MATH_SAT = 0x01
OUTPUT_MIN = 0x19999A          # 10 % of 2^24 counts = 0 psi
OUTPUT_MAX = 0xE66666          # 90 % of 2^24 counts = 25 psi
PSI_MAX = 25.0
PSI_TO_HPA = 68.947572932

# US Standard Atmosphere 1976 base levels: (base geopotential height m,
# base temperature K, lapse rate K/m, base pressure Pa)
_G0 = 9.80665
_R = 287.05287                 # specific gas constant of dry air, J/(kg K)
_LAYERS = (
    (0.0, 288.15, -0.0065, 101325.0),
    (11000.0, 216.65, 0.0, 22632.06),
    (20000.0, 216.65, 0.0010, 5474.889),
    (32000.0, 228.65, 0.0028, 868.0187),
    (47000.0, 270.65, 0.0, 110.9063),
)


def isa_altitude_m(pressure_hpa):
    """Pressure altitude (geopotential metres) from the standard atmosphere,
    valid 0-51 km. None for non-physical pressure."""
    if pressure_hpa is None or pressure_hpa <= 0:
        return None
    p = pressure_hpa * 100.0
    layer = _LAYERS[0]
    for lay in _LAYERS:
        if p <= lay[3]:
            layer = lay
    h_b, t_b, lapse, p_b = layer
    if lapse == 0.0:
        return h_b + (_R * t_b / _G0) * math.log(p_b / p)
    return h_b + (t_b / lapse) * ((p / p_b) ** (-lapse * _R / _G0) - 1.0)


def counts_to_hpa(raw):
    psi = (raw - OUTPUT_MIN) * PSI_MAX / (OUTPUT_MAX - OUTPUT_MIN)
    return psi * PSI_TO_HPA


def read_hpa(bus, addr=MPRLS_ADDR, timeout_s=0.1, retries=3):
    """One MPRLS conversion. Returns pressure in hPa or raises."""
    from smbus2 import i2c_msg
    for _ in range(retries):
        bus.i2c_rdwr(i2c_msg.write(addr, [0xAA, 0x00, 0x00]))
        time.sleep(0.005)
        deadline = time.monotonic() + timeout_s
        while True:
            rd = i2c_msg.read(addr, 4)
            bus.i2c_rdwr(rd)
            data = list(rd)
            st = data[0]
            if st & STATUS_INTEGRITY:
                raise RuntimeError(f"MPRLS integrity fault (status 0x{st:02X})")
            if st & STATUS_MATH_SAT:
                raise RuntimeError("MPRLS math saturation")
            if not st & STATUS_BUSY:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f"MPRLS conversion timeout (status 0x{st:02X})")
            time.sleep(0.001)
        raw = (data[1] << 16) | (data[2] << 8) | data[3]
        if raw:
            return counts_to_hpa(raw)
        time.sleep(0.005)
    raise RuntimeError("MPRLS returned zero data")


class BaroLogger(threading.Thread):
    """Samples the MPRLS every period_s and keeps (mono_ns, hPa) samples.

    Each sample is stamped with CLOCK_MONOTONIC at the middle of the
    conversion, the same clock as the IMU buffers and exp_mono_ns. I2C
    errors are counted and logged (rate limited); the thread keeps running,
    so a pressure dropout never stops image capture.
    """

    def __init__(self, bus=I2C_BUS, addr=MPRLS_ADDR, period_s=0.5, maxlen=7200):
        super().__init__(daemon=True, name="baro")
        self.bus_no, self.addr, self.period_s = bus, addr, period_s
        self.buf = collections.deque(maxlen=maxlen)
        self.errors = 0
        self.first = None              # (mono_ns, hPa) of the first good sample
        self._stop_event = threading.Event()
        self._last_warn = 0.0

    def run(self):
        try:
            from smbus2 import SMBus
            bus = SMBus(self.bus_no)
        except Exception as e:
            log.warning(f"Barometer unavailable ({e}); photos will have no pressure")
            return
        try:
            while not self._stop_event.is_set():
                t0 = time.monotonic_ns()
                try:
                    hpa = read_hpa(bus, self.addr)
                    t1 = time.monotonic_ns()
                    sample = ((t0 + t1) // 2, hpa)
                    self.buf.append(sample)
                    if self.first is None:
                        self.first = sample
                        alt = isa_altitude_m(hpa)
                        log.info(f"Barometer: {hpa:.2f} hPa, pressure altitude "
                                 f"{alt:.0f} m (ISA)")
                except (OSError, RuntimeError) as e:
                    self.errors += 1
                    now = time.monotonic()
                    if now - self._last_warn > 30.0:
                        log.warning(f"Barometer read failed ({e}); "
                                    f"{self.errors} error(s) so far")
                        self._last_warn = now
                self._stop_event.wait(self.period_s)
        finally:
            bus.close()

    def stop(self):
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=2.0)

    def _snapshot(self):
        for _ in range(3):
            try:
                return list(self.buf)
            except RuntimeError:       # deque mutated during copy
                time.sleep(0.001)
        return []

    def at(self, mono_ns, max_age_s=5.0):
        """Pressure at mono_ns: linear interpolation between the samples
        around it, else the nearest one. Returns a dict with pressure_hpa,
        baro_alt_m and baro_age_ms (distance to the nearest sample); values
        are None if there is no sample within max_age_s."""
        empty = {"pressure_hpa": None, "baro_alt_m": None, "baro_age_ms": None}
        s = self._snapshot()
        if not s or not mono_ns:
            return empty
        before = [q for q in s if q[0] <= mono_ns]
        after = [q for q in s if q[0] > mono_ns]
        a = before[-1] if before else None
        b = after[0] if after else None
        near = min((q for q in (a, b) if q), key=lambda q: abs(q[0] - mono_ns))
        age_ms = abs(near[0] - mono_ns) / 1e6
        if age_ms > max_age_s * 1000:
            return empty
        if a and b and b[0] > a[0]:
            hpa = a[1] + (b[1] - a[1]) * (mono_ns - a[0]) / (b[0] - a[0])
        else:
            hpa = near[1]
        alt = isa_altitude_m(hpa)
        return {"pressure_hpa": round(hpa, 3),
                "baro_alt_m": None if alt is None else round(alt, 1),
                "baro_age_ms": round(age_ms, 1)}


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s",
                        datefmt="%H:%M:%S")
    for p in (1013.25, 226.32, 54.749, 8.68):
        print(f"ISA check: {p:8.3f} hPa -> {isa_altitude_m(p):8.0f} m")
    b = BaroLogger(period_s=0.5)
    b.start()
    time.sleep(3.2)
    r = b.at(time.monotonic_ns())
    b.stop()
    print(f"Now: {r}  ({len(b.buf)} samples, {b.errors} errors)")


if __name__ == "__main__":
    main()
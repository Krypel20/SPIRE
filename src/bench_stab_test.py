#!/usr/bin/env python3
"""
SPIRE bench stabilization test -- core (step 1 of 3, rev. 2).

Runs the flight stabilization pipeline under controlled lab conditions over a
reference target (A4 cutting mat, 1 cm grid). The control code is IMPORTED
from flight_capture.py, not copied, so the bench tests exactly what flies.
flight_capture.py is not modified.

Test conditions (one per capture cycle):
  STATIC       platform still,   servo held rigid at center -> sharp reference
  UNCORRECTED  platform rotated, servo held rigid at center -> pure motion blur
  CORRECTED    platform rotated, flight pipeline active     -> residual blur

Every burst frame is kept. Per frame the manifest records camera and platform
yaw rate at the exposure instant, the attenuation ratio |cam|/|plat| (servo
effect, independent of the pendulum phase), and the predicted blur in pixels
under two geometries (rotation axis perpendicular to vs. along the optical
axis). Step 3 measures the real blur on the grid and decides which applies.

The status LED (status_led.py) tells the operator what to do:
  green solid   keep the platform still
  blue blinking start rotating      blue solid   keep rotating
  magenta       busy (saving / centering)
  red solid 2 s cycle invalid, repeat     red blinking   fault

Output (one session directory):
  session.json               parameters actually used
  manifest.csv               one row per frame
  c###_<MODE>_f#.jpg         all burst frames
  c###_meta.json             per-cycle record
  c###_imu.csv               raw gyro_z traces around the burst (PSF input)
"""

import os
import sys
import csv
import json
import math
import time
import logging
import threading
import collections
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_capture as fc  # noqa: E402  (flight code, imported unchanged)
from status_led import StatusLED  # noqa: E402

log = logging.getLogger("spire.bench")

MODES = ("STATIC", "UNCORRECTED", "CORRECTED")

# Flight configuration under test. Must match what the flight service runs.
FLIGHT_PARAMS = {
    "rate_gain": 1.5,        # calibrated at ~6.2 V servo supply
    "gyro_deadband": 0.5,    # deg/s; must equal the flight value
    "d_alpha": 0.3,
    "stabilize_time": 0.1,   # s, servo settle before burst
    "burst_count": 6,
    "center_time": 1.5,      # s
    "measure_time": 1.0,     # s, rotation measurement window
    "min_rotation": 5.0,     # deg/s, below this flight does not engage servo
}

# Bench-only settings
BENCH_PARAMS = {
    "exposure_us": 6667,         # locked shutter (1/150 s, the thesis budget); 0 = auto
    "static_max_blur_px": 1.0,   # STATIC cycle valid only below this (linear model)
    "ratio_min_plat_dps": 5.0,   # attenuation computed only above this platform rate
    "focal_mm": 16.0,
    "pixel_um": 1.55,
    "sensor_px": (4056, 3040),
}

TRACE_MARGIN_NS = 100_000_000  # IMU trace kept 100 ms around the burst


def setup_logging(verbose=False):
    fc.setup_logging(verbose)  # flight logger (servo/camera messages)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                            datefmt="%H:%M:%S")
    h = logging.StreamHandler(sys.stdout)
    h.setFormatter(fmt)
    for name in ("spire.bench", "spire.led"):
        lg = logging.getLogger(name)
        lg.setLevel(logging.DEBUG if verbose else logging.INFO)
        lg.addHandler(h)


def add_session_log(session_dir):
    """Mirror everything printed to the console into <session>/bench.log."""
    fh = logging.FileHandler(os.path.join(session_dir, "bench.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"))
    for name in ("spire.flight", "spire.bench", "spire.led"):
        logging.getLogger(name).addHandler(fh)


# ---------------------------------------------------------------------------
# Blur model
# ---------------------------------------------------------------------------

def blur_px(gz_dps, exposure_us, bp=BENCH_PARAMS):
    """Predicted smear in pixels for a yaw rate during one exposure.

    linear:      rotation axis perpendicular to the optical axis (oblique view);
                 uniform smear  w * t * f / p.
    rot_corner:  rotation axis along the optical axis (nadir view); smear grows
                 with radius, w * t * r, evaluated at the frame corner.
    """
    if gz_dps is None or not exposure_us:
        return None, None
    wt = math.radians(abs(gz_dps)) * exposure_us * 1e-6
    f_px = bp["focal_mm"] * 1e-3 / (bp["pixel_um"] * 1e-6)
    r_corner = math.hypot(bp["sensor_px"][0] / 2, bp["sensor_px"][1] / 2)
    return wt * f_px, wt * r_corner


# ---------------------------------------------------------------------------
# Hardware rig
# ---------------------------------------------------------------------------

class BenchRig:
    """Owns all hardware for one bench session. Same bring-up as flight."""

    def __init__(self, output_dir, cam_cal="config/imu_calibration.json",
                 plat_cal="config/lsm9ds1_calibration.json", servo_pin=12,
                 exposure_us=BENCH_PARAMS["exposure_us"]):
        self.output_dir = output_dir
        self.cam_cal = cam_cal
        self.plat_cal = plat_cal
        self.servo_pin = servo_pin
        self.exposure_us = exposure_us
        self.imu_camera = self.imu_platform = None
        self.shm_camera = self.shm_platform = None
        self.servo = self.camera = None
        self.boot_mono_offset_ns = 0

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    def start(self):
        self.imu_camera = fc.IMUProcess(
            "imu_camera", "icm20948", "spire_imu_camera", self.cam_cal, rate=500,
            log_path=os.path.join(self.output_dir, "imu_camera.log"))
        self.imu_platform = fc.IMUProcess(
            "imu_platform", "lsm9ds1", "spire_imu_platform", self.plat_cal,
            rate=200, address="0x6B", mag=True,
            log_path=os.path.join(self.output_dir, "imu_platform.log"))
        self.imu_camera.start()
        time.sleep(1)
        self.imu_platform.start()
        time.sleep(2)

        self.shm_camera = fc.SHMReader("spire_imu_camera")
        self.shm_platform = fc.SHMReader("spire_imu_platform")
        log.info("Waiting for IMU data...")
        for _ in range(20):
            if self.shm_camera.read() and self.shm_platform.read():
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("IMU data not available after 10 s "
                               "(check I2C: 0x69, 0x6b, 0x1e must be present)")
        log.info("IMU data ready")

        self.servo = fc.ServoController(pin=self.servo_pin)
        self.camera = fc.Camera(self.output_dir)
        self.camera.init()

        if self.exposure_us:
            # Fixed shutter so blur is comparable across modes and sessions;
            # gain stays automatic. Verify in manifest column exposure_time_us.
            self.camera.cam.set_controls({"ExposureTime": int(self.exposure_us)})
            time.sleep(0.5)
            md = self.camera.cam.capture_metadata()
            log.info(f"Exposure locked: requested {self.exposure_us} us, "
                     f"sensor {md.get('ExposureTime')} us, "
                     f"gain {md.get('AnalogueGain', 0):.2f}")

        # SensorTimestamp is CLOCK_BOOTTIME, IMU buffers are CLOCK_MONOTONIC.
        self.boot_mono_offset_ns = (
            time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            - time.clock_gettime_ns(time.CLOCK_MONOTONIC))

    def imu_alive(self):
        return not self.dead_imus()

    def dead_imus(self):
        """Names and exit codes of IMU readers that are not running.

        imu_reader.py catches any exception in its sampling loop, logs it and
        returns normally, so a crash after an I2C error shows up as exit code 0.
        A negative code means it was killed by that signal.
        """
        dead = []
        for p in (self.imu_camera, self.imu_platform):
            if p is None or not p.is_alive():
                rc = p.proc.returncode if p is not None and p.proc else None
                dead.append(f"{p.name if p else '?'} (exit {rc})")
        return dead

    def stop(self):
        for name, fn in (("servo", lambda: self.servo and self.servo.close()),
                         ("camera", lambda: self.camera and self.camera.close()),
                         ("imu_camera", lambda: self.imu_camera and self.imu_camera.stop()),
                         ("imu_platform", lambda: self.imu_platform and self.imu_platform.stop())):
            try:
                fn()
            except Exception as e:  # keep tearing down the rest
                log.warning(f"Shutdown of {name} failed: {e}")


# ---------------------------------------------------------------------------
# One test cycle
# ---------------------------------------------------------------------------

def hold_center(servo):
    """Drive the servo to center and keep the PWM on: camera rigidly coupled
    to the platform. This is the defined 'no correction' baseline (a detached,
    limp servo couples the camera through gear friction only, which is not
    repeatable)."""
    servo.slow_center(duration=0.3)  # leaves the pulse train running


SERVO_LIMIT_DEG = 134.0  # |command| at or above this = travel exhausted (range 135)


class ServoTrace(threading.Thread):
    """Samples the servo command at ~200 Hz into the same (mono_ns, value)
    ring-buffer format as CamGyroLogger, so it can be matched to exposures."""

    def __init__(self, servo, maxlen=4000):
        super().__init__(daemon=True)
        self.servo = servo
        self.buf = collections.deque(maxlen=maxlen)
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            self.buf.append((time.monotonic_ns(), float(self.servo.last_sent)))
            time.sleep(0.005)

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)

    def value_at(self, mono_ns):
        best, best_dt = None, None
        for ts, v in self.buf:
            dt = abs(ts - mono_ns)
            if best_dt is None or dt < best_dt:
                best_dt, best = dt, v
        return best


def _trace_window(logger, t0_ns, t1_ns):
    """Deduplicated (mono_ns, gyro_z) samples inside [t0, t1]."""
    seen, out = set(), []
    for ts, gz in list(logger.buf):
        if t0_ns <= ts <= t1_ns and ts not in seen:
            seen.add(ts)
            out.append((ts, gz))
    return out


def run_cycle(rig, mode, cycle_id, session_dir, params=FLIGHT_PARAMS,
              bench=BENCH_PARAMS, led=None):
    """Run one capture cycle in the given mode and record all burst frames.

    CORRECTED reproduces flight Phases 2-5 call for call: rotation measurement,
    integrator reset with 5 warm-up reads, integral re-zero, PIDThread,
    stabilize_time, burst, stop PID after exposure, slow center, detach.
    Below min_rotation flight leaves the servo detached; that path is recorded
    with effective_status CALM, exactly as flight would behave.
    """
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    p = params
    if led:
        led.set("HOLD_STILL" if mode == "STATIC" else "ROTATE_KEEP")

    # Loggers start before the rotation measurement: in the CALM/HELD paths
    # the burst begins immediately and frame 0 may be exposed before a
    # logger started later would hold any sample.
    cam_log = fc.CamGyroLogger(rig.shm_camera)
    plat_log = fc.CamGyroLogger(rig.shm_platform)  # same ring buffer, platform shm
    servo_log = ServoTrace(rig.servo)
    cam_log.start()
    plat_log.start()
    servo_log.start()

    rotation = fc.measure_rotation_speed(rig.shm_platform, p["measure_time"])

    pid =fc.StabilizationPID(gyro_deadband=p["gyro_deadband"],
                              d_alpha=p["d_alpha"], rate_gain=p["rate_gain"])
    pid_thread = None

    if mode == "CORRECTED" and rotation >= p["min_rotation"]:
        status = "ACTIVE"
        pid.reset()
        for _ in range(5):
            plat = rig.shm_platform.read()
            if plat:
                pid.update(plat)
            time.sleep(0.02)
        pid.servo_integral = 0.0
        pid_thread = fc.PIDThread(pid, rig.servo, rig.shm_platform)
        pid_thread.start()
        time.sleep(p["stabilize_time"])
    elif mode == "CORRECTED":
        status = "CALM"          # flight path: servo stays detached
    else:
        status = "HELD"
        hold_center(rig.servo)

    frames = rig.camera.capture_burst(cycle_id, p["burst_count"])

    if pid_thread:
        pid_thread.stop()        # flight: stop only after exposure completes
    cam_log.stop()
    plat_log.stop()
    servo_log.stop()
    if led:
        led.set("BUSY")

    # Score every frame at its exposure instant
    for i, f in enumerate(frames):
        sts = f["sensor_timestamp_ns"]
        f["index"] = i
        f["exp_mono_ns"] = sts - rig.boot_mono_offset_ns if sts else 0
        cam_gz = plat_gz = None
        gap_ms = -1.0
        if sts:
            cam_gz, gap = cam_log.gyro_at(f["exp_mono_ns"])
            plat_gz, plat_gap = plat_log.gyro_at(f["exp_mono_ns"])
            gap_ms = gap / 1e6 if gap is not None else -1.0
            # Same rule as flight: no real sample near the exposure, no score.
            if gap is None or gap_ms > fc.MAX_MATCH_GAP_MS:
                cam_gz = None
            if plat_gap is None or plat_gap / 1e6 > fc.MAX_MATCH_GAP_MS:
                plat_gz = None
        f["cam_gz"] = cam_gz
        f["plat_gz"] = plat_gz
        f["match_gap_ms"] = gap_ms
        f["servo_cmd_deg"] = servo_log.value_at(f["exp_mono_ns"]) if sts else None
        f["servo_saturated"] = (f["servo_cmd_deg"] is not None
                                and abs(f["servo_cmd_deg"]) >= SERVO_LIMIT_DEG)
        f["blur_px_linear"], f["blur_px_rot_corner"] = blur_px(
            cam_gz, f["exposure_time_us"], bench)
        f["attenuation"] = (
            abs(cam_gz) / abs(plat_gz)
            if cam_gz is not None and plat_gz is not None
            and abs(plat_gz) >= bench["ratio_min_plat_dps"] else None)

    valid = [f for f in frames if f["cam_gz"] is not None]
    winner = min(valid, key=lambda f: abs(f["cam_gz"])) if valid else None

    # Does the cycle satisfy the condition its mode is meant to represent?
    if mode == "STATIC":
        worst = max((f["blur_px_linear"] or 0.0) for f in frames) if valid else None
        cycle_valid = (len(valid) == len(frames) and worst is not None
                       and worst <= bench["static_max_blur_px"])
        reason = "" if cycle_valid else (
            f"platform moved: worst predicted blur {worst:.2f} px "
            f"> {bench['static_max_blur_px']} px" if worst is not None
            else "no exposure timestamps")
    elif mode == "UNCORRECTED":
        cycle_valid = rotation >= p["min_rotation"]
        reason = "" if cycle_valid else (
            f"rotation {rotation:.1f} < {p['min_rotation']} deg/s")
    else:
        cycle_valid = status == "ACTIVE"
        reason = "" if cycle_valid else (
            f"rotation {rotation:.1f} < {p['min_rotation']} deg/s, servo not engaged")
    if cycle_valid and len(valid) < len(frames):
        cycle_valid = False
        reason = (f"{len(frames) - len(valid)} frame(s) without a camera gyro "
                  f"sample within {fc.MAX_MATCH_GAP_MS:.0f} ms (IMU reader stalled?)")

    # Keep all frames under final names
    for f in frames:
        name = f"c{cycle_id:03d}_{mode}_f{f['index']}.jpg"
        os.replace(f["tmp_path"], os.path.join(session_dir, name))
        f["file"] = name
        f["winner"] = f is winner

    # Raw IMU traces around the burst (input for exposure-window PSF, step 3)
    exp_ts = [f["exp_mono_ns"] for f in frames if f["exp_mono_ns"]]
    if exp_ts:
        max_exp_ns = max(f["exposure_time_us"] for f in frames) * 1000
        t0 = min(exp_ts) - TRACE_MARGIN_NS
        t1 = max(exp_ts) + max_exp_ns + TRACE_MARGIN_NS
        with open(os.path.join(session_dir, f"c{cycle_id:03d}_imu.csv"),
                  "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["source", "mono_ns", "gyro_z_dps"])
            for ts, gz in _trace_window(cam_log, t0, t1):
                w.writerow(["camera", ts, f"{gz:.4f}"])
            for ts, gz in _trace_window(plat_log, t0, t1):
                w.writerow(["platform", ts, f"{gz:.4f}"])
            for ts, v in _trace_window(servo_log, t0, t1):
                w.writerow(["servo_cmd_deg", ts, f"{v:.2f}"])

    frame_keys = ("index", "file", "sensor_timestamp_ns", "exposure_time_us",
                  "exp_mono_ns", "cam_gz", "plat_gz", "match_gap_ms",
                  "blur_px_linear", "blur_px_rot_corner", "attenuation",
                  "servo_cmd_deg", "servo_saturated", "winner")
    record = {
        "cycle_id": cycle_id,
        "mode": mode,
        "effective_status": status,
        "cycle_valid": cycle_valid,
        "invalid_reason": reason,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "rotation_speed_dps": round(rotation, 2),
        "params": dict(p),
        "frames": [{k: f[k] for k in frame_keys} for f in frames],
    }
    with open(os.path.join(session_dir, f"c{cycle_id:03d}_meta.json"), "w") as fh:
        json.dump(record, fh, indent=2)

    _append_manifest(session_dir, record)

    rig.servo.slow_center(duration=p["center_time"])
    rig.servo.detach()

    def fmt(v):
        return "n/a" if v is None else f"{v:+.1f}"
    ratios = [f["attenuation"] for f in frames if f["attenuation"] is not None]
    ratio_txt = f" |cam|/|plat| median {sorted(ratios)[len(ratios)//2]:.2f}" if ratios else ""
    sat = [f["index"] for f in frames if f["servo_saturated"]]
    sat_txt = f" servo saturated in f{sat}" if sat else ""
    log.info(f"[c{cycle_id:03d} {mode}/{status}] rot={rotation:.1f} deg/s "
             f"cam_gz=[{', '.join(fmt(f['cam_gz']) for f in frames)}] "
             f"servo=[{', '.join(fmt(f['servo_cmd_deg']) for f in frames)}] "
             f"-> best {fmt(winner['cam_gz'] if winner else None)} deg/s"
             f"{ratio_txt}{sat_txt}"
             + ("" if cycle_valid else f"  INVALID: {reason}"))
    return record


MANIFEST_FIELDS = ["cycle_id", "mode", "effective_status", "cycle_valid",
                   "rotation_speed_dps", "frame_index", "file", "winner",
                   "cam_gz", "plat_gz", "attenuation", "servo_cmd_deg",
                   "servo_saturated", "blur_px_linear",
                   "blur_px_rot_corner", "match_gap_ms", "exposure_time_us",
                   "sensor_timestamp_ns", "timestamp_utc"]


def _r(v, n=2):
    return "" if v is None else round(v, n)


def _append_manifest(session_dir, record):
    path = os.path.join(session_dir, "manifest.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=MANIFEST_FIELDS)
        if new:
            w.writeheader()
        for f in record["frames"]:
            w.writerow({
                "cycle_id": record["cycle_id"], "mode": record["mode"],
                "effective_status": record["effective_status"],
                "cycle_valid": int(record["cycle_valid"]),
                "rotation_speed_dps": record["rotation_speed_dps"],
                "frame_index": f["index"], "file": f["file"],
                "winner": int(f["winner"]),
                "cam_gz": _r(f["cam_gz"]), "plat_gz": _r(f["plat_gz"]),
                "attenuation": _r(f["attenuation"], 3),
                "servo_cmd_deg": _r(f["servo_cmd_deg"], 1),
                "servo_saturated": int(f["servo_saturated"]),
                "blur_px_linear": _r(f["blur_px_linear"]),
                "blur_px_rot_corner": _r(f["blur_px_rot_corner"]),
                "match_gap_ms": _r(f["match_gap_ms"]),
                "exposure_time_us": f["exposure_time_us"],
                "sensor_timestamp_ns": f["sensor_timestamp_ns"],
                "timestamp_utc": record["timestamp_utc"],
            })
        fh.flush()
        os.fsync(fh.fileno())


def new_session_dir(base="data/bench"):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = os.path.join(base, f"session_{stamp}")
    os.makedirs(path, exist_ok=True)
    return path


# ---------------------------------------------------------------------------
# Minimal session CLI (step 2 replaces this with the interactive runner)
# ---------------------------------------------------------------------------

def main():
    import argparse
    ap = argparse.ArgumentParser(description="SPIRE bench test core")
    ap.add_argument("--mode", choices=MODES, required=True)
    ap.add_argument("-n", "--cycles", type=int, default=3)
    ap.add_argument("--countdown", type=float, default=3.0,
                    help="Seconds of operator cue before each cycle (default 3)")
    ap.add_argument("--exposure-us", type=int, default=BENCH_PARAMS["exposure_us"],
                    help="Locked shutter in us, 0 = auto exposure (default 6667)")
    ap.add_argument("--rate-gain", type=float, default=FLIGHT_PARAMS["rate_gain"])
    ap.add_argument("--gyro-deadband", type=float,
                    default=FLIGHT_PARAMS["gyro_deadband"])
    ap.add_argument("--no-led", action="store_true")
    ap.add_argument("-o", "--output", default="data/bench")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    setup_logging(args.verbose)

    params = dict(FLIGHT_PARAMS, rate_gain=args.rate_gain,
                  gyro_deadband=args.gyro_deadband)
    bench = dict(BENCH_PARAMS, exposure_us=args.exposure_us)

    session = new_session_dir(args.output)
    with open(os.path.join(session, "session.json"), "w") as fh:
        json.dump({"mode": args.mode, "flight_params": params,
                   "bench_params": bench,
                   "started_utc": datetime.now(timezone.utc).isoformat()},
                  fh, indent=2)
    add_session_log(session)
    log.info(f"Session: {session}")
    log.info(f"Params: rate_gain={params['rate_gain']} "
             f"gyro_deadband={params['gyro_deadband']} "
             f"exposure={'auto' if not args.exposure_us else str(args.exposure_us) + ' us'}")

    led = StatusLED(enabled=not args.no_led)
    led.set("INIT")
    cue = "HOLD_STILL" if args.mode == "STATIC" else "ROTATE_NOW"
    try:
        with BenchRig(session, exposure_us=args.exposure_us) as rig:
            for c in range(args.cycles):
                dead = rig.dead_imus()
                if dead:
                    log.error(f"IMU process died: {', '.join(dead)}; "
                              "stopping session")
                    led.set("FAULT")
                    time.sleep(5)
                    break
                led.set(cue)
                what = "keep still" if args.mode == "STATIC" else "rotate"
                log.info(f"Cycle {c}: {args.mode} in {args.countdown:.0f} s ({what})")
                time.sleep(args.countdown)
                rec = run_cycle(rig, args.mode, c, session, params, bench, led)
                if not rec["cycle_valid"]:
                    led.set("CYCLE_INVALID")
                    time.sleep(2)
        led.set("DONE")
        log.info("Done.")
        time.sleep(3)
    except Exception:
        led.set("FAULT")
        log.exception("Session aborted")
        time.sleep(5)
        raise
    finally:
        led.close()


if __name__ == "__main__":
    main()
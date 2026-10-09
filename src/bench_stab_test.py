#!/usr/bin/env python3
"""
SPIRE bench stabilization test -- core (step 1 of 3).

Runs the flight stabilization pipeline under controlled lab conditions over a
reference target (1 cm grid A4 mat). The control code is IMPORTED
from flight_capture.py

Test conditions (one per capture cycle):
  STATIC       platform still,   servo held rigid at center -> sharp reference
  UNCORRECTED  platform rotated, servo held rigid at center -> pure motion blur
  CORRECTED    platform rotated, flight pipeline active     -> residual blur

Every burst frame is kept: each one is a real blurred
sample with its exposure-time camera and platform yaw rates, which is the
input set for the deblurring tests.

Output (one session directory):
  manifest.csv               one row per frame
  c###_<MODE>_f#.jpg          all burst frames
  c###_meta.json             per-cycle record
  c###_imu.csv               raw gyro_z traces around the burst (PSF input)

Step 1 provides the core and a minimal smoke-test CLI.
Step 2 adds the interactive session runner, step 3 the grid-based analysis.
"""

import os
import sys
import csv
import json
import time
import logging
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_capture as fc  # noqa: E402  (flight code, imported unchanged)

log = logging.getLogger("spire.bench")

MODES = ("STATIC", "UNCORRECTED", "CORRECTED")

# Flight configuration under test. Stated explicitly so every session records
# exactly which parameters were exercised.
FLIGHT_PARAMS = {
    "rate_gain": 1.5,        # calibrated at ~6.2 V servo supply
    "gyro_deadband": 0.5,    # deg/s, final flight value
    "d_alpha": 0.3,
    "stabilize_time": 0.1,   # s, servo settle before burst
    "burst_count": 6,
    "center_time": 1.5,      # s
    "measure_time": 1.0,     # s, rotation measurement window
    "min_rotation": 5.0,     # deg/s, below this flight does not engage servo
}

TRACE_MARGIN_NS = 100_000_000  # IMU trace kept 100 ms around the burst


def setup_logging(verbose=False):
    fc.setup_logging(verbose)  # flight logger (servo/camera messages)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                            datefmt="%H:%M:%S")
    h = logging.StreamHandler(sys.stdout)
    h.setFormatter(fmt)
    log.setLevel(logging.DEBUG if verbose else logging.INFO)
    log.addHandler(h)


# ---------------------------------------------------------------------------
# Hardware rig
# ---------------------------------------------------------------------------

class BenchRig:
    """Owns all hardware for one bench session. Same bring-up as flight."""

    def __init__(self, output_dir, cam_cal="config/imu_calibration.json",
                 plat_cal="config/lsm9ds1_calibration.json", servo_pin=12):
        self.output_dir = output_dir
        self.cam_cal = cam_cal
        self.plat_cal = plat_cal
        self.servo_pin = servo_pin
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
            "imu_camera", "icm20948", "spire_imu_camera", self.cam_cal, rate=500)
        self.imu_platform = fc.IMUProcess(
            "imu_platform", "lsm9ds1", "spire_imu_platform", self.plat_cal,
            rate=200, address="0x6B", mag=True)
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

        # SensorTimestamp is CLOCK_BOOTTIME, IMU buffers are CLOCK_MONOTONIC.
        self.boot_mono_offset_ns = (
            time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            - time.clock_gettime_ns(time.CLOCK_MONOTONIC))

    def imu_alive(self):
        return bool(self.imu_camera and self.imu_camera.is_alive()
                    and self.imu_platform and self.imu_platform.is_alive())

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


def _trace_window(logger, t0_ns, t1_ns):
    """Deduplicated (mono_ns, gyro_z) samples inside [t0, t1]."""
    seen, out = set(), []
    for ts, gz in list(logger.buf):
        if t0_ns <= ts <= t1_ns and ts not in seen:
            seen.add(ts)
            out.append((ts, gz))
    return out


def run_cycle(rig, mode, cycle_id, session_dir, params=FLIGHT_PARAMS):
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

    rotation = fc.measure_rotation_speed(rig.shm_platform, p["measure_time"])

    cam_log = fc.CamGyroLogger(rig.shm_camera)
    plat_log = fc.CamGyroLogger(rig.shm_platform)  # same ring buffer, platform shm
    cam_log.start()
    plat_log.start()

    pid = fc.StabilizationPID(gyro_deadband=p["gyro_deadband"],
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

    # Score every frame at its exposure instant
    for i, f in enumerate(frames):
        sts = f["sensor_timestamp_ns"]
        f["index"] = i
        f["exp_mono_ns"] = sts - rig.boot_mono_offset_ns if sts else 0
        cam_gz = plat_gz = None
        gap_ms = -1.0
        if sts:
            cam_gz, gap = cam_log.gyro_at(f["exp_mono_ns"])
            plat_gz, _ = plat_log.gyro_at(f["exp_mono_ns"])
            gap_ms = gap / 1e6 if gap is not None else -1.0
        f["cam_gz"] = cam_gz
        f["plat_gz"] = plat_gz
        f["match_gap_ms"] = gap_ms

    valid = [f for f in frames if f["cam_gz"] is not None]
    winner = min(valid, key=lambda f: abs(f["cam_gz"])) if valid else None

    # Keep all frames under final names
    for f in frames:
        name = f"c{cycle_id:03d}_{mode}_f{f['index']}.jpg"
        final = os.path.join(session_dir, name)
        os.replace(f["tmp_path"], final)
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

    record = {
        "cycle_id": cycle_id,
        "mode": mode,
        "effective_status": status,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "rotation_speed_dps": round(rotation, 2),
        "params": dict(p),
        "frames": [{k: f[k] for k in ("index", "file", "sensor_timestamp_ns",
                                       "exposure_time_us", "exp_mono_ns",
                                       "cam_gz", "plat_gz", "match_gap_ms",
                                       "winner")} for f in frames],
    }
    with open(os.path.join(session_dir, f"c{cycle_id:03d}_meta.json"), "w") as fh:
        json.dump(record, fh, indent=2)

    _append_manifest(session_dir, record)

    rig.servo.slow_center(duration=p["center_time"])
    rig.servo.detach()

    cams = [f"{f['cam_gz']:+.1f}" if f["cam_gz"] is not None else "n/a"
            for f in frames]
    best = f"{winner['cam_gz']:+.1f}" if winner else "n/a"
    log.info(f"[c{cycle_id:03d} {mode}/{status}] rot={rotation:.1f} deg/s "
             f"cam_gz=[{', '.join(cams)}] -> best {best} deg/s")
    return record


MANIFEST_FIELDS = ["cycle_id", "mode", "effective_status", "rotation_speed_dps",
                   "frame_index", "file", "winner", "cam_gz", "plat_gz",
                   "match_gap_ms", "exposure_time_us", "sensor_timestamp_ns",
                   "timestamp_utc"]


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
                "rotation_speed_dps": record["rotation_speed_dps"],
                "frame_index": f["index"], "file": f["file"],
                "winner": int(f["winner"]),
                "cam_gz": "" if f["cam_gz"] is None else round(f["cam_gz"], 2),
                "plat_gz": "" if f["plat_gz"] is None else round(f["plat_gz"], 2),
                "match_gap_ms": round(f["match_gap_ms"], 2),
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
# Minimal smoke test (step 2 replaces this with the interactive runner)
# ---------------------------------------------------------------------------

def main():
    import argparse
    ap = argparse.ArgumentParser(description="SPIRE bench test core -- smoke test")
    ap.add_argument("--mode", choices=MODES, required=True)
    ap.add_argument("-n", "--cycles", type=int, default=3)
    ap.add_argument("--countdown", type=float, default=3.0,
                    help="Seconds before each cycle, to start rotating (default 3)")
    ap.add_argument("-o", "--output", default="data/bench")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    setup_logging(args.verbose)

    session = new_session_dir(args.output)
    with open(os.path.join(session, "session.json"), "w") as fh:
        json.dump({"params": FLIGHT_PARAMS,
                   "started_utc": datetime.now(timezone.utc).isoformat()}, fh, indent=2)
    log.info(f"Session: {session}")

    with BenchRig(session) as rig:
        for c in range(args.cycles):
            if not rig.imu_alive():
                log.error("IMU process died, stopping session")
                break
            log.info(f"Cycle {c}: {args.mode} in {args.countdown:.0f} s ...")
            time.sleep(args.countdown)
            run_cycle(rig, args.mode, c, session)
    log.info("Done.")


if __name__ == "__main__":
    main()
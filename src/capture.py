#!/usr/bin/env python3
"""
SPIRE Capture Pipeline v2.0
Feature-complete capture module for stratospheric balloon imaging.

Features:
  - Manual / auto / adaptive exposure control
  - JPEG + DNG (RAW) capture at full 12.3 MP resolution
  - Burst mode (rapid sequence without interval delay)
  - CSV metadata logging with monotonic + UTC timestamps
  - Shared memory interface for IMU synchronization (future)
  - Disk space monitoring with auto-stop
  - Graceful shutdown on SIGTERM/SIGINT
  - Session metadata logging
  - Configurable JPEG quality

Usage:
  python3 capture.py --auto -n 10 -o data/session1
  python3 capture.py -e 1000 -g 2.0 --burst 5 -o data/burst_test
  python3 capture.py --focus
  python3 capture.py --preview
"""

import time
import csv
import os
import sys
import json
import signal
import shutil
import logging
import argparse
from datetime import datetime, timezone
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Angular velocity budget: max °/s for <1px blur at given exposure
# Derived from: pixel_pitch=1.55µm, focal_length=25mm
# blur_px = (omega_rad * exposure_s * focal_length) / pixel_pitch
# For 1px: omega = pixel_pitch / (exposure_s * focal_length)
PIXEL_PITCH_UM = 1.55
FOCAL_LENGTH_MM = 25.0

# Minimum free disk space before auto-stop (MB)
MIN_DISK_FREE_MB = 500

# Estimated file sizes for disk planning (MB)
EST_JPEG_SIZE_MB = 5.0
EST_DNG_SIZE_MB = 25.0

# IMU shared memory name (convention for imu_reader process)
IMU_SHM_NAME = "spire_imu_state"

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.capture")


def setup_logging(output_dir, verbose=False):
    """Configure logging to console + file."""
    os.makedirs(output_dir, exist_ok=True)
    level = logging.DEBUG if verbose else logging.INFO

    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S"
    )

    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(fmt)

    logfile = logging.FileHandler(
        os.path.join(output_dir, "capture.log"), mode="w"
    )
    logfile.setLevel(logging.DEBUG)
    logfile.setFormatter(fmt)

    log.setLevel(logging.DEBUG)
    log.addHandler(console)
    log.addHandler(logfile)


# ---------------------------------------------------------------------------
# IMU shared memory interface (stub — filled by imu_reader)
# ---------------------------------------------------------------------------

def try_read_imu():
    """Attempt to read IMU state from shared memory.

    Returns dict with gyro/accel/timestamp if available, None otherwise.
    When imu_reader process is running, it writes current state to shared
    memory block 'spire_imu_state'. This function reads it non-blocking.

    Format (when implemented):
        64 bytes: JSON-encoded dict with keys:
        - gyro_x, gyro_y, gyro_z (°/s)
        - accel_x, accel_y, accel_z (m/s²)
        - timestamp_mono_ns (int)
    """
    try:
        from multiprocessing import shared_memory
        shm = shared_memory.SharedMemory(name=IMU_SHM_NAME, create=False)
        raw = bytes(shm.buf[:shm.size]).rstrip(b'\x00')
        shm.close()
        if raw:
            return json.loads(raw.decode("utf-8"))
    except FileNotFoundError:
        pass
    except Exception as e:
        log.debug(f"IMU read failed: {e}")
    return None


def compute_max_angular_velocity(exposure_us, focal_length_mm=FOCAL_LENGTH_MM):
    """Compute max angular velocity [°/s] for <1px blur.

    Args:
        exposure_us: Exposure time in microseconds
        focal_length_mm: Focal length in mm

    Returns:
        Maximum angular velocity in °/s
    """
    import math
    exposure_s = exposure_us / 1e6
    # omega_rad = pixel_pitch / (exposure_s * focal_length)
    omega_rad = (PIXEL_PITCH_UM * 1e-3) / (exposure_s * focal_length_mm)
    return math.degrees(omega_rad)


def compute_adaptive_exposure(angular_velocity_dps, gain,
                              focal_length_mm=FOCAL_LENGTH_MM,
                              min_exposure_us=1000,
                              max_exposure_us=6667):
    """Compute exposure time to keep blur < 1px at given angular velocity.

    Args:
        angular_velocity_dps: Current angular velocity in °/s
        gain: Current analogue gain (for minimum exposure floor)
        focal_length_mm: Focal length in mm
        min_exposure_us: Minimum exposure (1/1000s)
        max_exposure_us: Maximum exposure (1/150s)

    Returns:
        Recommended exposure time in µs
    """
    import math
    if angular_velocity_dps <= 0.01:
        return max_exposure_us

    omega_rad = math.radians(angular_velocity_dps)
    # exposure_s = pixel_pitch / (omega_rad * focal_length)
    exposure_s = (PIXEL_PITCH_UM * 1e-3) / (omega_rad * focal_length_mm)
    exposure_us = int(exposure_s * 1e6)

    return max(min_exposure_us, min(exposure_us, max_exposure_us))


# ---------------------------------------------------------------------------
# Disk monitoring
# ---------------------------------------------------------------------------

def check_disk_space(output_dir, save_raw):
    """Check if enough disk space is available.

    Returns:
        (ok: bool, free_mb: float)
    """
    stat = shutil.disk_usage(output_dir)
    free_mb = stat.free / (1024 * 1024)
    needed = EST_JPEG_SIZE_MB + (EST_DNG_SIZE_MB if save_raw else 0)
    return free_mb > max(MIN_DISK_FREE_MB, needed * 2), free_mb


# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------

def setup_camera(exposure_us, gain, auto_exposure, jpeg_quality=93):
    """Initialize camera with given parameters.

    Args:
        exposure_us: Exposure time in µs (ignored if auto_exposure)
        gain: Analogue gain (ignored if auto_exposure)
        auto_exposure: Use AE algorithm
        jpeg_quality: JPEG compression quality (1-100)

    Returns:
        Configured Picamera2 instance
    """
    cam = Picamera2()

    config = cam.create_still_configuration(
        main={"size": (4056, 3040), "format": "RGB888"},
        raw={"size": (4056, 3040), "format": "SRGGB12_CSI2P"},
        display=None
    )
    cam.configure(config)

    cam.options["quality"] = jpeg_quality

    if auto_exposure:
        cam.set_controls({"AeEnable": True})
    else:
        cam.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_us,
            "AnalogueGain": gain,
            "AwbEnable": False,
            "ColourGains": (1.5, 1.2),
        })

    return cam


# ---------------------------------------------------------------------------
# Session metadata
# ---------------------------------------------------------------------------

def save_session_info(output_dir, args, cam):
    """Save session configuration to JSON file."""
    props = cam.camera_properties
    info = {
        "session_start_utc": datetime.now(timezone.utc).isoformat(),
        "camera_model": props.get("Model", "unknown"),
        "sensor_resolution": [4056, 3040],
        "pixel_pitch_um": PIXEL_PITCH_UM,
        "focal_length_mm": FOCAL_LENGTH_MM,
        "exposure_mode": "auto" if args.auto else (
            "adaptive" if args.adaptive else "manual"
        ),
        "exposure_us": args.exposure,
        "gain": args.gain,
        "jpeg_quality": args.quality,
        "interval_s": args.interval,
        "burst": args.burst,
        "save_raw": not args.no_raw,
        "max_angular_velocity_dps": round(
            compute_max_angular_velocity(args.exposure), 3
        ),
        "output_dir": os.path.abspath(output_dir),
        "hostname": os.uname().nodename,
        "python_version": sys.version.split()[0],
    }

    path = os.path.join(output_dir, "session_info.json")
    with open(path, "w") as f:
        json.dump(info, f, indent=2)
    log.info(f"Session info: {path}")
    return info


# ---------------------------------------------------------------------------
# Focus mode
# ---------------------------------------------------------------------------

def focus_mode(cam):
    """Interactive focus helper — displays FocusFoM in real-time."""
    cam.start()
    time.sleep(1.0)

    log.info("=== FOCUS MODE ===")
    log.info("Rotate the focus ring on the lens.")
    log.info("FocusFoM: the higher the value, the sharper the image.")
    log.info("Ctrl+C to exit.\n")

    best_fom = 0
    frame = 0
    try:
        while True:
            metadata = cam.capture_metadata()
            fom = metadata.get("FocusFoM", 0)
            exp = metadata.get("ExposureTime", -1)
            lux = metadata.get("Lux", -1)

            if fom > best_fom:
                best_fom = fom
                marker = " *** BEST ***"
            else:
                marker = ""

            print(f"[{frame:04d}] FocusFoM={fom:>8}  "
                  f"exp={exp}us  lux={lux:.0f}{marker}")
            frame += 1
            time.sleep(0.3)
    except KeyboardInterrupt:
        print(f"\nBest FocusFoM: {best_fom}")
    finally:
        cam.stop()
        cam.close()


# ---------------------------------------------------------------------------
# Preview mode
# ---------------------------------------------------------------------------

def preview_mode(cam, port=8080):
    """Launch MJPEG preview server for browser viewing."""
    import io
    from http.server import HTTPServer, BaseHTTPRequestHandler
    from threading import Event
    import socket

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                html = (
                    '<!DOCTYPE html><html><head>'
                    '<title>SPIRE Preview</title></head>'
                    '<body style="margin:0;background:#111;display:flex;'
                    'align-items:center;justify-content:center;height:100vh;'
                    'flex-direction:column;">'
                    '<h2 style="color:#eee;font-family:monospace;">'
                    'SPIRE Live Preview</h2>'
                    '<img src="/stream" style="max-width:95vw;'
                    'max-height:85vh;">'
                    '<p style="color:#999;font-family:monospace;">'
                    'Ctrl+C on RPi to stop</p>'
                    '</body></html>'
                )
                self.wfile.write(html.encode())
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        buf = io.BytesIO()
                        self.server.camera.capture_file(
                            buf, format="jpeg", name="main"
                        )
                        frame = buf.getvalue()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(
                            b"Content-Type: image/jpeg\r\n"
                        )
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, fmt, *a):
            pass

    # Detect local IP
    ip = "0.0.0.0"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        pass

    # Use lower resolution for preview
    cam.stop()
    cam.close()

    cam = Picamera2()
    config = cam.create_still_configuration(
        main={"size": (1012, 760), "format": "RGB888"},
        display=None
    )
    cam.configure(config)
    cam.set_controls({"AeEnable": True})
    cam.start()
    time.sleep(1.0)

    server = HTTPServer(("0.0.0.0", port), Handler)
    server.camera = cam

    log.info(f"Preview: http://{ip}:{port}")
    log.info("Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Preview stopped.")
    finally:
        server.server_close()
        cam.stop()
        cam.close()


# ---------------------------------------------------------------------------
# Main capture loop
# ---------------------------------------------------------------------------

def capture_loop(cam, output_dir, interval, num_frames, save_raw,
                 burst_count, adaptive):
    """Main capture loop with metadata logging and disk monitoring.

    Args:
        cam: Configured Picamera2 instance
        output_dir: Directory for output files
        interval: Seconds between captures (0 for burst)
        num_frames: Total frames to capture (0 = infinite)
        save_raw: Whether to save DNG files
        burst_count: Number of rapid frames per burst trigger
        adaptive: Use adaptive exposure based on IMU data
    """
    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(output_dir, "capture_log.csv")

    with open(log_path, "w", newline="") as logfile:
        writer = csv.writer(logfile)
        writer.writerow([
            "frame_id", "timestamp_utc", "timestamp_mono_ns",
            "exposure_us", "analogue_gain", "digital_gain",
            "lux", "focus_fom", "colour_temp",
            "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
            "imu_accel_x", "imu_accel_y", "imu_accel_z",
            "imu_timestamp_ns",
            "filename_jpg", "filename_dng"
        ])

        cam.start()
        time.sleep(2.0)
        log.info(f"Saving to: {output_dir}")
        log.info(f"Interval: {interval}s | Frames: "
                 f"{num_frames if num_frames > 0 else '∞'} | "
                 f"Burst: {burst_count} | RAW: {save_raw}")

        frame_id = 0
        running = True

        # Graceful shutdown handler
        def handle_signal(signum, frame):
            nonlocal running
            log.info(f"Signal {signum} received — stopping...")
            running = False

        signal.signal(signal.SIGTERM, handle_signal)
        signal.signal(signal.SIGINT, handle_signal)

        try:
            while running and (num_frames <= 0 or frame_id < num_frames):

                # --- Disk space check (every 10 frames) ---
                if frame_id % 10 == 0:
                    ok, free_mb = check_disk_space(output_dir, save_raw)
                    if not ok:
                        log.warning(
                            f"Low disk space: {free_mb:.0f} MB. "
                            f"Stopping."
                        )
                        break
                    elif free_mb < MIN_DISK_FREE_MB * 2:
                        log.warning(f"Warning: {free_mb:.0f} MB remaining")

                # --- Read IMU state (if available) ---
                imu = try_read_imu()

                # --- Adaptive exposure ---
                if adaptive and imu:
                    gx = imu.get("gyro_x", 0)
                    gy = imu.get("gyro_y", 0)
                    gz = imu.get("gyro_z", 0)
                    omega = (gx**2 + gy**2 + gz**2) ** 0.5
                    new_exp = compute_adaptive_exposure(omega, 1.0)
                    cam.set_controls({"ExposureTime": new_exp})
                    log.debug(f"Adaptive: ω={omega:.2f}°/s → exp={new_exp}µs")

                # --- Burst or single capture ---
                frames_this_cycle = burst_count if burst_count > 1 else 1

                for b in range(frames_this_cycle):
                    if not running:
                        break

                    t_mono = time.monotonic_ns()
                    t_utc = datetime.now(timezone.utc).strftime(
                        "%Y%m%d_%H%M%S_%f"
                    )

                    fname = f"spire_{t_utc}"
                    jpg_path = os.path.join(output_dir, f"{fname}.jpg")
                    dng_path = (
                        os.path.join(output_dir, f"{fname}.dng")
                        if save_raw else None
                    )

                    metadata = cam.capture_file(
                        jpg_path, name="main", format="jpeg"
                    )

                    if save_raw:
                        cam.capture_file(dng_path, name="raw")

                    # Extract metadata
                    exp = metadata.get("ExposureTime", -1)
                    a_gain = metadata.get("AnalogueGain", -1)
                    d_gain = metadata.get("DigitalGain", -1)
                    lux = metadata.get("Lux", -1)
                    fom = metadata.get("FocusFoM", -1)
                    ctemp = metadata.get("ColourTemperature", -1)

                    # IMU columns
                    if imu:
                        imu_row = [
                            round(imu.get("gyro_x", 0), 4),
                            round(imu.get("gyro_y", 0), 4),
                            round(imu.get("gyro_z", 0), 4),
                            round(imu.get("accel_x", 0), 4),
                            round(imu.get("accel_y", 0), 4),
                            round(imu.get("accel_z", 0), 4),
                            imu.get("timestamp_mono_ns", -1),
                        ]
                    else:
                        imu_row = [""] * 7

                    writer.writerow([
                        frame_id, t_utc, t_mono,
                        exp, round(a_gain, 3), round(d_gain, 3),
                        round(lux, 2) if isinstance(lux, float) else lux,
                        fom, ctemp,
                        *imu_row,
                        os.path.basename(jpg_path),
                        os.path.basename(dng_path) if dng_path else ""
                    ])
                    logfile.flush()

                    burst_info = (
                        f" burst {b+1}/{frames_this_cycle}"
                        if frames_this_cycle > 1 else ""
                    )
                    log.info(
                        f"[{frame_id:04d}]{burst_info} {t_utc} | "
                        f"exp={exp}us gain={a_gain:.1f} "
                        f"lux={lux:.0f} fom={fom}"
                    )

                    frame_id += 1

                # --- Wait for next interval ---
                if interval > 0 and running:
                    elapsed = (time.monotonic_ns() - t_mono) / 1e9
                    wait = max(0, interval - elapsed)
                    if wait > 0:
                        time.sleep(wait)

        except Exception as e:
            log.error(f"Error: {e}", exc_info=True)

        finally:
            cam.stop()
            cam.close()
            log.info(f"Saved {frame_id} frames.")
            log.info(f"CSV: {log_path}")

            # Summary
            if frame_id > 0:
                total_size = sum(
                    os.path.getsize(os.path.join(output_dir, f))
                    for f in os.listdir(output_dir)
                    if f.endswith((".jpg", ".dng"))
                )
                log.info(
                    f"Total data size: {total_size / (1024*1024):.1f} MB"
                )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Capture Pipeline v2.0",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --auto -n 10 -o data/session1
  %(prog)s -e 1000 -g 2.0 -n 5 -o data/fast_shutter
  %(prog)s --auto --burst 3 -n 9 -o data/burst_test
  %(prog)s --adaptive -o data/adaptive_test
  %(prog)s --focus
  %(prog)s --preview
        """
    )

    # Output
    parser.add_argument("-o", "--output", default="./data/captures",
                        help="Output directory (default: ./data/captures)")

    # Capture params
    parser.add_argument("-i", "--interval", type=float, default=2.0,
                        help="Interval between frames [s] (default: 2.0)")
    parser.add_argument("-n", "--num-frames", type=int, default=0,
                        help="Number of frames, 0 = ∞ (default: 0)")
    parser.add_argument("-e", "--exposure", type=int, default=6667,
                        help="Exposure time [µs] (default: 6667=1/150s)")
    parser.add_argument("-g", "--gain", type=float, default=1.0,
                        help="Analogue gain (default: 1.0)")
    parser.add_argument("-q", "--quality", type=int, default=93,
                        help="JPEG quality 1-100 (default: 93)")

    # Modes
    parser.add_argument("--auto", action="store_true",
                        help="Auto-exposure mode")
    parser.add_argument("--adaptive", action="store_true",
                        help="Adaptive exposure based on IMU data")
    parser.add_argument("--burst", type=int, default=1,
                        help="Burst: N frames per cycle (default: 1)")
    parser.add_argument("--no-raw", action="store_true",
                        help="Do not save RAW (DNG)")

    # Tools
    parser.add_argument("--focus", action="store_true",
                        help="Focus adjustment mode")
    parser.add_argument("--preview", action="store_true",
                        help="Live preview in browser")
    parser.add_argument("--preview-port", type=int, default=8080,
                        help="Preview server port (default: 8080)")

    # Debug
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging (DEBUG level)")

    args = parser.parse_args()

    # --- Focus mode ---
    if args.focus:
        setup_logging(args.output, args.verbose)
        cam = setup_camera(args.exposure, args.gain, True, args.quality)
        focus_mode(cam)
        return

    # --- Preview mode ---
    if args.preview:
        setup_logging(args.output, args.verbose)
        cam = setup_camera(args.exposure, args.gain, True, args.quality)
        preview_mode(cam, args.preview_port)
        return

    # --- Capture mode ---
    setup_logging(args.output, args.verbose)

    is_auto = args.auto or args.adaptive
    mode = ("ADAPTIVE" if args.adaptive else
            "AUTO" if args.auto else "MANUAL")

    log.info("=" * 40)
    log.info("SPIRE Capture Pipeline v2.0")
    log.info("=" * 40)
    log.info(f"Mode: {mode}")

    if not is_auto:
        max_omega = compute_max_angular_velocity(args.exposure)
        log.info(f"Exposure: {args.exposure} µs (1/{1e6/args.exposure:.0f}s)")
        log.info(f"Gain: {args.gain}")
        log.info(f"Max ω for <1px blur: {max_omega:.2f} °/s")

    log.info(f"JPEG quality: {args.quality}")
    log.info(f"RAW: {'no' if args.no_raw else 'yes'}")
    log.info(f"Burst: {args.burst}")

    # Check disk space before starting
    ok, free_mb = check_disk_space(args.output, not args.no_raw)
    log.info(f"Free disk space: {free_mb:.0f} MB")
    if not ok:
        log.error("Insufficient disk space. Aborting.")
        sys.exit(1)

    # Check IMU availability
    imu = try_read_imu()
    if imu:
        log.info("IMU: connected (shared memory)")
    else:
        log.info("IMU: unavailable (standalone mode)")
        if args.adaptive:
            log.warning(
                "Adaptive mode without IMU — using max exposure "
                f"({args.exposure} µs)"
            )

    cam = setup_camera(args.exposure, args.gain, is_auto, args.quality)
    save_session_info(args.output, args, cam)

    capture_loop(
        cam, args.output, args.interval, args.num_frames,
        not args.no_raw, args.burst, args.adaptive
    )


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
SPIRE Capture Pipeline
Periodic stills from IMX477 with exposure control and metadata logging.
Timestamp synchronization for future IMU integration.
"""

import time
import csv
import os
import argparse
from datetime import datetime, timezone
from picamera2 import Picamera2


def setup_camera(exposure_us, gain, auto_exposure):
    """Initialize camera with given exposure and gain."""
    cam = Picamera2()

    config = cam.create_still_configuration(
        main={"size": (4056, 3040), "format": "RGB888"},
        raw={"size": (4056, 3040), "format": "SRGGB12_CSI2P"},
        display=None
    )
    cam.configure(config)

    if auto_exposure:
        cam.set_controls({
            "AeEnable": True,
        })
    else:
        cam.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_us,
            "AnalogueGain": gain,
            "AwbEnable": False,
            "ColourGains": (1.5, 1.2),
        })

    return cam


def focus_mode(cam):
    """Focus adjustment mode — fast metadata reads every 0.5s.
    Rotate the focus ring on the lens and observe the 'FocusFoM' value
    (Focus Figure of Merit). The higher the value, the sharper the image.
    """
    cam.start()
    time.sleep(1.0)

    print("=== FOCUS MODE ===")
    print("Rotate the focus ring on the lens.")
    print("Observe FocusFoM — the higher the value, the sharper the image.")
    print("Ctrl+C to exit.\n")

    frame = 0
    try:
        while True:
            metadata = cam.capture_metadata()
            fom = metadata.get("FocusFoM", "N/A")
            exp = metadata.get("ExposureTime", -1)
            lux = metadata.get("Lux", -1)
            print(f"[{frame:04d}] FocusFoM={fom}  exp={exp}us  lux={lux:.0f}")
            frame += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFocus mode stopped.")
    finally:
        cam.stop()
        cam.close()


def capture_loop(cam, output_dir, interval, num_frames, save_raw):
    """Main capture loop with metadata logging."""

    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(output_dir, "capture_log.csv")

    with open(log_path, "w", newline="") as logfile:
        writer = csv.writer(logfile)
        writer.writerow([
            "frame_id", "timestamp_utc", "timestamp_mono_ns",
            "exposure_us", "analogue_gain", "digital_gain",
            "lux", "focus_fom", "filename_jpg", "filename_dng"
        ])

        cam.start()
        time.sleep(2.0)

        frame_id = 0
        print(f"Starting capture to: {output_dir}")
        print(f"Interval: {interval}s | Frames: {num_frames if num_frames > 0 else 'infinite'}")
        print("Ctrl+C to stop\n")

        try:
            while num_frames <= 0 or frame_id < num_frames:
                t_mono = time.monotonic_ns()
                t_utc = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")

                fname = f"spire_{t_utc}"
                jpg_path = os.path.join(output_dir, f"{fname}.jpg")
                dng_path = os.path.join(output_dir, f"{fname}.dng") if save_raw else None

                metadata = cam.capture_file(
                    jpg_path,
                    name="main",
                    format="jpeg"
                )

                if save_raw:
                    cam.capture_file(dng_path, name="raw")

                exp = metadata.get("ExposureTime", -1)
                a_gain = metadata.get("AnalogueGain", -1)
                d_gain = metadata.get("DigitalGain", -1)
                lux = metadata.get("Lux", -1)
                fom = metadata.get("FocusFoM", -1)

                writer.writerow([
                    frame_id, t_utc, t_mono,
                    exp, round(a_gain, 3), round(d_gain, 3),
                    round(lux, 2) if lux != -1 else -1,
                    fom,
                    os.path.basename(jpg_path),
                    os.path.basename(dng_path) if dng_path else ""
                ])
                logfile.flush()

                print(f"[{frame_id:04d}] {t_utc} | exp={exp}us gain={a_gain:.1f} lux={lux:.0f} fom={fom} | {jpg_path}")

                frame_id += 1

                elapsed = (time.monotonic_ns() - t_mono) / 1e9
                wait = max(0, interval - elapsed)
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print(f"\nStopped. Saved {frame_id} frames.")

        finally:
            cam.stop()
            cam.close()
            print(f"Log: {log_path}")


def main():
    parser = argparse.ArgumentParser(description="SPIRE Capture Pipeline")
    parser.add_argument("-o", "--output", default="./data/captures",
                        help="Output directory (default: ./data/captures)")
    parser.add_argument("-i", "--interval", type=float, default=2.0,
                        help="Interval between frames [s] (default: 2.0)")
    parser.add_argument("-n", "--num-frames", type=int, default=0,
                        help="Number of frames, 0 = infinite (default: 0)")
    parser.add_argument("-e", "--exposure", type=int, default=6667,
                        help="Exposure time [µs] (default: 6667 = 1/150s)")
    parser.add_argument("-g", "--gain", type=float, default=1.0,
                        help="Analogue gain (default: 1.0)")
    parser.add_argument("--auto", action="store_true",
                        help="Auto-exposure mode (ignores -e and -g)")
    parser.add_argument("--no-raw", action="store_true",
                        help="Do not save RAW (DNG)")
    parser.add_argument("--focus", action="store_true",
                        help="Focus adjustment mode (no image saving)")

    args = parser.parse_args()

    if args.focus:
        print("=== SPIRE Focus Helper ===")
        cam = setup_camera(args.exposure, args.gain, auto_exposure=True)
        focus_mode(cam)
        return

    mode = "AUTO" if args.auto else "MANUAL"
    print("=== SPIRE Capture Pipeline ===")
    print(f"Mode: {mode}")
    if not args.auto:
        print(f"Exposure: {args.exposure} µs (1/{1e6/args.exposure:.0f}s)")
        print(f"Gain: {args.gain}")
    print(f"RAW: {'no' if args.no_raw else 'yes'}\n")

    cam = setup_camera(args.exposure, args.gain, auto_exposure=args.auto)
    capture_loop(cam, args.output, args.interval, args.num_frames, not args.no_raw)


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""SPIRE Capture Pipeline
Cykliczne zdjęcia z IMX477 z kontrolą ekspozycji i logowaniem metadanych.
Synchronizacja timestampów pod przyszłą integrację z IMU.
"""

import time
import csv
import os
import argparse
from datetime import datetime
from picamera2 import Picamera2


def setup_camera(exposure_us, gain):
    """Inicjalizacja kamery z zadaną ekspozycją i gain."""
    cam = Picamera2()

    # config JPEG + RAW (DNG)
    config = cam.create_still_configuration(
        main={"size": (4056, 3040), "format": "RGB888"},
        raw={"size": (4056, 3040), "format": "SRGGB12_CSI2P"},
        display=None
    )
    cam.configure(config)

    # disable auto-exposure, set manually
    cam.set_controls({
        "AeEnable": False,
        "ExposureTime": exposure_us,
        "AnalogueGain": gain,
        "AwbEnable": False,
        "ColourGains": (1.5, 1.2),  # approximate daylight WB
    })

    return cam


def capture_loop(cam, output_dir, interval, num_frames, save_raw):
    """main loop with logging metadata."""

    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(output_dir, "capture_log.csv")

    with open(log_path, "w", newline="") as logfile:
        writer = csv.writer(logfile)
        writer.writerow([
            "frame_id", "timestamp_utc", "timestamp_mono_ns",
            "exposure_us", "analogue_gain", "digital_gain",
            "lux", "filename_jpg", "filename_dng"
        ])

        cam.start()
        # wait for parameters to stabilize
        time.sleep(1.0)

        frame_id = 0
        print(f"Starting capture to: {output_dir}")
        print(f"Interval: {interval}s | Frames: {num_frames if num_frames > 0 else 'infinite'}")
        print("Ctrl+C to stop\n")

        try:
            while num_frames <= 0 or frame_id < num_frames:
                t_mono = time.monotonic_ns()
                t_utc = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")

                fname = f"spire_{t_utc}"
                jpg_path = os.path.join(output_dir, f"{fname}.jpg")
                dng_path = os.path.join(output_dir, f"{fname}.dng") if save_raw else None

                # capture image
                metadata = cam.capture_file(
                    jpg_path,
                    name="main",
                    format="jpeg"
                )

                # save RAW (DNG)
                if save_raw:
                    cam.capture_file(dng_path, name="raw")

                # read actual parameters
                exp = metadata.get("ExposureTime", -1)
                a_gain = metadata.get("AnalogueGain", -1)
                d_gain = metadata.get("DigitalGain", -1)
                lux = metadata.get("Lux", -1)

                # log to CSV
                writer.writerow([
                    frame_id, t_utc, t_mono,
                    exp, round(a_gain, 3), round(d_gain, 3),
                    round(lux, 2) if lux != -1 else -1,
                    os.path.basename(jpg_path),
                    os.path.basename(dng_path) if dng_path else ""
                ])
                logfile.flush()

                print(f"[{frame_id:04d}] {t_utc} | exp={exp}us gain={a_gain:.1f} lux={lux:.0f} | {jpg_path}")

                frame_id += 1

                # wait for next interval
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
                        help="Katalog wyjściowy (domyślnie: ./data/captures)")
    parser.add_argument("-i", "--interval", type=float, default=2.0,
                        help="Interval between frames [s] (default: 2.0)")
    parser.add_argument("-n", "--num-frames", type=int, default=0,
                        help="Number of frames, 0 = infinite (default: 0)")
    parser.add_argument("-e", "--exposure", type=int, default=6667,
                        help="Exposure time [µs] (default: 6667 = 1/150s)")
    parser.add_argument("-g", "--gain", type=float, default=1.0,
                        help="Analog gain (default: 1.0)")
    parser.add_argument("--no-raw", action="store_true",
                        help="Do not save RAW (DNG)")

    args = parser.parse_args()

    print("=== SPIRE Capture Pipeline ===")
    print(f"Exposure: {args.exposure} µs (1/{1e6/args.exposure:.0f}s)")
    print(f"Gain: {args.gain}")
    print(f"RAW: {'no' if args.no_raw else 'yes'}\n")

    cam = setup_camera(args.exposure, args.gain)
    capture_loop(cam, args.output, args.interval, args.num_frames, not args.no_raw)


if __name__ == "__main__":
    main()
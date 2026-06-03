#!/usr/bin/env python3
"""
SPIRE Flight Capture
Integrated capture + yaw stabilization cycle.
Starts IMU readers automatically, manages servo, takes photos.

Cycle:
  1. Servo detached (no movement, no payload disturbance)
  2. Before photo: set heading reference, enable PID stabilization
  3. Wait for camera to stabilize (error < threshold)
  4. Take photo
  5. Slowly center servo (recover full range)
  6. Detach servo, wait for next cycle

Usage:
  python3 src/flight_capture.py                    # 15s interval
  python3 src/flight_capture.py --interval 5       # 5s for testing
  python3 src/flight_capture.py --interval 15 -n 50  # 50 photos
"""

import time
import sys
import os
import math
import json
import signal
import subprocess
import argparse
import logging
from datetime import datetime, timezone
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import io
# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("spire.flight")


def setup_logging(verbose=False):
    level = logging.DEBUG if verbose else logging.INFO
    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
    )
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(fmt)
    log.setLevel(logging.DEBUG)
    log.addHandler(console)


# ---------------------------------------------------------------------------
# Shared memory reader
# ---------------------------------------------------------------------------

class SHMReader:
    def __init__(self, name):
        self.name = name
        self._shm = None

    def read(self):
        try:
            from multiprocessing import shared_memory, resource_tracker
            if self._shm is None:
                self._shm = shared_memory.SharedMemory(
                    name=self.name, create=False
                )
                resource_tracker.unregister(
                    f"/{self.name}", "shared_memory"
                )
            raw = bytes(self._shm.buf[:self._shm.size]).rstrip(b'\x00')
            if raw:
                return json.loads(raw.decode("utf-8"))
        except FileNotFoundError:
            self._shm = None
        except Exception:
            self._shm = None
        return None


# ---------------------------------------------------------------------------
# Heading helpers
# ---------------------------------------------------------------------------

def compute_heading(mag_x, mag_y):
    """Simple heading from magnetometer (tilt compensation disabled)."""
    heading_rad = math.atan2(mag_y, mag_x)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


def heading_error(current, reference):
    """Shortest angular difference, range -180 to +180."""
    err = current - reference
    while err > 180:
        err -= 360
    while err < -180:
        err += 360
    return err


# ---------------------------------------------------------------------------
# Subprocess management
# ---------------------------------------------------------------------------

class IMUProcess:
    """Managed IMU reader subprocess."""

    def __init__(self, name, sensor, shm_name, cal_path,
                 rate=500, address=None, mag=False):
        self.name = name
        self.proc = None
        self.sensor = sensor
        self.shm_name = shm_name
        self.cal_path = cal_path
        self.rate = rate
        self.address = address
        self.mag = mag

    def start(self):
        src_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(src_dir)
        python = sys.executable

        cmd = [
            python, os.path.join(src_dir, "imu_reader.py"),
            "--sensor", self.sensor,
            "-r", str(self.rate),
            "--shm-name", self.shm_name,
            "--cal", self.cal_path,
        ]
        if self.address:
            cmd += ["-a", self.address]
        if self.mag:
            cmd += ["--mag"]

        log.info(f"Starting [{self.name}]: {self.sensor} → {self.shm_name}")
        self.proc = subprocess.Popen(
            cmd, cwd=project_dir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def is_alive(self):
        return self.proc and self.proc.poll() is None

    def stop(self):
        if self.proc and self.is_alive():
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            log.info(f"[{self.name}] stopped")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

class Camera:
    """Simple camera wrapper using picamera2."""

    def __init__(self, output_dir):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.cam = None

    def init(self):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        config = self.cam.create_still_configuration(
            main={"size": (4056, 3040)},
        )
        self.cam.configure(config)
        self.cam.start()
        time.sleep(1)  # Warm up
        log.info("Camera initialized")

    def capture(self, frame_id, metadata=None):
        """Take a photo and save as JPEG.

        Returns:
            Path to saved image
        """
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        filename = f"spire_{timestamp}_{frame_id:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)

        self.cam.capture_file(filepath)

        # Save metadata sidecar
        if metadata:
            meta_path = filepath.replace(".jpg", "_meta.json")
            with open(meta_path, "w") as f:
                json.dump(metadata, f, indent=2)

        log.info(f"Photo {frame_id}: {filename}")
        return filepath

    def close(self):
        if self.cam:
            self.cam.stop()
            self.cam.close()
            log.info("Camera closed")

class PreviewServer:
    """Background MJPEG streaming server with photo gallery."""

    def __init__(self, camera, output_dir, port=8080):
        self.camera = camera
        self.output_dir = output_dir
        self.port = port
        self.server = None
        self.thread = None

    def start(self):
        cam = self.camera
        output_dir = self.output_dir

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(b"""
                    <html><head>
                    <title>SPIRE Preview</title>
                    <style>
                      body { margin:0; background:#111; color:#eee; font-family:monospace; }
                      .container { display:flex; height:100vh; }
                      .stream { flex:2; display:flex; align-items:center; justify-content:center; }
                      .stream img { max-width:100%; max-height:100%; }
                      .gallery { flex:1; border-left:2px solid #333; }
                      .gallery iframe { width:100%; height:100%; border:none; }
                    </style>
                    </head><body>
                    <div class="container">
                      <div class="stream"><img src="/stream"></div>
                      <div class="gallery"><iframe src="/gallery"></iframe></div>
                    </div>
                    </body></html>""")

                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            buf = io.BytesIO()
                            cam.cam.capture_file(buf, format="jpeg")
                            frame = buf.getvalue()
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(frame)}\r\n".encode())
                            self.wfile.write(b"\r\n")
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                            time.sleep(0.2)
                    except (BrokenPipeError, ConnectionResetError):
                        pass

                elif self.path == "/photos":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    photos = sorted(
                        [f for f in os.listdir(output_dir) if f.endswith(".jpg")],
                        reverse=True
                    ) if os.path.exists(output_dir) else []
                    html = ""
                    for p in photos[:20]:
                        html += f'<a href="/photo/{p}" target="_blank">'
                        html += f'<img src="/photo/{p}" title="{p}"></a>\n'
                    if not photos:
                        html = "<p>No photos yet</p>"
                    self.wfile.write(html.encode())

                elif self.path.startswith("/photo/"):
                    filename = self.path[7:]
                    filepath = os.path.join(output_dir, filename)
                    if os.path.exists(filepath) and filename.endswith(".jpg"):
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.end_headers()
                        with open(filepath, "rb") as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_error(404)
                        
                elif self.path == "/gallery":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    photos = sorted(
                        [f for f in os.listdir(output_dir) if f.endswith(".jpg")],
                        reverse=True
                    ) if os.path.exists(output_dir) else []
                    html = """<html><head>
                    <meta http-equiv="refresh" content="5">
                    <style>
                      body { margin:10px; background:#111; color:#eee; font-family:monospace; }
                      h3 { color:#0ff; margin:0 0 10px; }
                      img { width:100%; margin-bottom:8px; border:1px solid #333; }
                      .info { font-size:11px; color:#888; margin-bottom:5px; }
                    </style>
                    </head><body>
                    <h3>Photos (""" + str(len(photos)) + """)</h3>"""
                    for p in photos[:20]:
                        html += f'<div class="info">{p}</div>'
                        html += f'<a href="/photo/{p}" target="_blank">'
                        html += f'<img src="/photo/{p}"></a>\n'
                    if not photos:
                        html += "<p>No photos yet</p>"
                    html += "</body></html>"
                    self.wfile.write(html.encode())
                else:
                    self.send_error(404)

            def log_message(self, format, *args):
                pass

        self.server = HTTPServer(("0.0.0.0", self.port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        log.info(f"Preview: http://<rpi-ip>:{self.port}")

    def stop(self):
        if self.server:
            self.server.shutdown()
            
# ---------------------------------------------------------------------------
# Servo
# ---------------------------------------------------------------------------

class ServoController:
    def __init__(self, pin=12):
        from gpiozero import Servo
        from gpiozero.pins.lgpio import LGPIOFactory
        self.factory = LGPIOFactory()
        self.servo = Servo(
            pin, pin_factory=self.factory,
            min_pulse_width=0.5 / 1000,
            max_pulse_width=2.5 / 1000,
        )
        self.servo.detach()
        self.current_position = 0.0
        self.last_sent = 0.0
        self.range_deg = 135.0
        log.info(f"Servo on GPIO {pin} (detached)")

    def set_position(self, angle_deg, threshold=1.0):
        """Set servo position in degrees. Only updates if change > threshold."""
        angle_deg = max(-self.range_deg, min(self.range_deg, angle_deg))
        self.current_position = angle_deg
        if abs(angle_deg - self.last_sent) > threshold:
            value = angle_deg / self.range_deg
            value = max(-1.0, min(1.0, value))
            self.servo.value = value
            self.last_sent = angle_deg

    def slow_center(self, duration=1.5):
        """Slowly return to center to avoid disturbing payload."""
        start = self.last_sent
        steps = 30
        for i in range(steps + 1):
            t = i / steps
            angle = start * (1 - t)
            value = angle / self.range_deg
            value = max(-1.0, min(1.0, value))
            self.servo.value = value
            self.last_sent = angle
            time.sleep(duration / steps)
        self.current_position = 0.0
        self.last_sent = 0.0

    def detach(self):
        self.servo.detach()

    def close(self):
        self.servo.detach()


# ---------------------------------------------------------------------------
# Stabilization PID
# ---------------------------------------------------------------------------

class StabilizationPID:
    """Heading PID with complementary filter and gain scheduling."""

    def __init__(self, kp=0.5, kd_low=0.4, kd_high=0.8, ki=0.02,
                 heading_deadband=5.0, gyro_deadband=0.5,
                 comp_alpha=0.02):
        self.kp = kp
        self.kd_low = kd_low
        self.kd_high = kd_high
        self.ki = ki
        self.heading_deadband = heading_deadband
        self.gyro_deadband = gyro_deadband
        self.comp_alpha = comp_alpha

        # Error thresholds for gain scheduling
        self.error_low = 5.0
        self.error_high = 15.0

        # State
        self.heading_ref = None
        self.filtered_heading = None
        self.integral = 0.0
        self.last_time = None

    def reset(self, initial_heading=None):
        """Reset PID state. Optionally set new heading reference."""
        self.heading_ref = initial_heading
        self.filtered_heading = initial_heading
        self.integral = 0.0
        self.last_time = time.monotonic()

    def update(self, plat_data):
        """Compute servo position from platform IMU data.

        Returns:
            (servo_angle, error, is_stable) or None if no data
        """
        if plat_data is None:
            return None

        now = time.monotonic()
        if self.last_time is None:
            self.last_time = now
        dt = now - self.last_time
        self.last_time = now
        if dt > 0.1:
            dt = 0.01

        mag_x = plat_data.get("mag_x", 0)
        mag_y = plat_data.get("mag_y", 0)
        gyro_rate = plat_data.get("gyro_z", 0.0)

        # Raw magnetometer heading
        mag_heading = compute_heading(mag_x, mag_y)

        # First reading — set reference
        if self.heading_ref is None:
            self.heading_ref = mag_heading
            self.filtered_heading = mag_heading
            return (0.0, 0.0, True)

        # Complementary filter: 98% gyro + 2% magnetometer
        gyro_heading = self.filtered_heading + gyro_rate * dt
        mag_diff = heading_error(mag_heading, gyro_heading)
        self.filtered_heading = gyro_heading + self.comp_alpha * mag_diff
        self.filtered_heading = self.filtered_heading % 360

        # Heading error
        error = heading_error(self.filtered_heading, self.heading_ref)

        # Deadband — hold position when close to target
        if abs(error) < self.heading_deadband:
            return (None, error, True)  # None = don't move servo

        # Gain scheduling
        abs_error = abs(error)
        if abs_error <= self.error_low:
            kd_active = self.kd_low
        elif abs_error >= self.error_high:
            kd_active = self.kd_high
        else:
            t = (abs_error - self.error_low) / (self.error_high - self.error_low)
            kd_active = self.kd_low + t * (self.kd_high - self.kd_low)

        # P
        p_out = self.kp * error

        # I
        self.integral += error * dt
        self.integral = max(-135.0, min(135.0, self.integral))
        i_out = self.ki * self.integral

        # D
        if abs(gyro_rate) < self.gyro_deadband:
            gyro_rate = 0.0
        d_out = kd_active * gyro_rate

        servo_angle = -(p_out + i_out + d_out)
        servo_angle = max(-135.0, min(135.0, servo_angle))

        is_stable = abs(error) < self.heading_deadband * 2
        return (servo_angle, error, is_stable)


# ---------------------------------------------------------------------------
# Main flight loop
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Flight Capture",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --interval 5               # Test: photo every 5s
  %(prog)s --interval 15 -n 100       # Flight: 100 photos, 15s apart
  %(prog)s --interval 10 --stabilize-time 3
        """
    )

    # Capture
    parser.add_argument("--interval", type=float, default=15,
                        help="Seconds between photos (default: 15)")
    parser.add_argument("-n", "--num-photos", type=int, default=0,
                        help="Number of photos, 0=infinite (default: 0)")
    parser.add_argument("-o", "--output", default="data/flight",
                        help="Output directory (default: data/flight)")
    parser.add_argument("--preview", action="store_true",
                        help="Enable MJPEG live preview on port 8080")
    parser.add_argument("--preview-port", type=int, default=8080,
                        help="Preview server port (default: 8080)")

    # Stabilization timing
    parser.add_argument("--stabilize-time", type=float, default=2.0,
                        help="Seconds to stabilize before photo (default: 2.0)")
    parser.add_argument("--center-time", type=float, default=1.5,
                        help="Seconds for slow centering (default: 1.5)")

    # PID
    parser.add_argument("--kp", type=float, default=0.5)
    parser.add_argument("--kd-low", type=float, default=0.4)
    parser.add_argument("--kd-high", type=float, default=0.8)
    parser.add_argument("--ki", type=float, default=0.02)
    parser.add_argument("--heading-deadband", type=float, default=5.0)

    # Hardware
    parser.add_argument("--servo-pin", type=int, default=12)
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera (test stabilization only)")

    # IMU config
    parser.add_argument("--cam-cal", default="config/imu_calibration.json")
    parser.add_argument("--plat-cal", default="config/lsm9ds1_calibration.json")

    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    setup_logging(args.verbose)

    log.info("=" * 50)
    log.info("SPIRE Flight Capture")
    log.info("=" * 50)
    log.info(f"Interval: {args.interval}s | Stabilize: {args.stabilize_time}s")
    log.info(f"PID: Kp={args.kp} Kd={args.kd_low}-{args.kd_high} Ki={args.ki}")
    log.info("")

    # Validate timing
    if args.stabilize_time + args.center_time + 1 > args.interval:
        log.warning("Stabilize + center time exceeds interval!")

    # Start IMU readers
    imu_camera = IMUProcess(
        "imu_camera", "icm20948", "spire_imu_camera",
        args.cam_cal, rate=500
    )
    imu_platform = IMUProcess(
        "imu_platform", "lsm9ds1", "spire_imu_platform",
        args.plat_cal, rate=200, address="0x6B", mag=True
    )

    imu_camera.start()
    time.sleep(1)
    imu_platform.start()
    time.sleep(2)

    # Wait for shared memory
    shm_platform = SHMReader("spire_imu_platform")
    shm_camera = SHMReader("spire_imu_camera")

    log.info("Waiting for IMU data...")
    for attempt in range(20):
        plat = shm_platform.read()
        cam = shm_camera.read()
        if plat and cam:
            break
        time.sleep(0.5)
    else:
        log.error("IMU data not available after 10s")
        imu_camera.stop()
        imu_platform.stop()
        sys.exit(1)

    log.info("IMU data ready")

    # Initialize servo
    servo = ServoController(pin=args.servo_pin)

    # Initialize camera
    camera = None
    if not args.no_camera:
        camera = Camera(args.output)
        camera.init()
        
    # Preview server 
    preview = None
    if args.preview and camera:
        preview = PreviewServer(camera, args.output, port=args.preview_port)
        preview.start()

    # Initialize PID
    pid = StabilizationPID(
        kp=args.kp, kd_low=args.kd_low, kd_high=args.kd_high,
        ki=args.ki, heading_deadband=args.heading_deadband,
    )

    # Flight loop
    running = True
    frame_id = 0

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    log.info("")
    log.info("Flight capture started. Ctrl+C to stop.")
    log.info("")

    try:
        while running:
            cycle_start = time.monotonic()

            # Check IMU health
            if not imu_camera.is_alive() or not imu_platform.is_alive():
                log.error("IMU process died!")
                break

            # --- Phase 1: Wait (servo detached, no disturbance) ---
            wait_time = max(0, args.interval - args.stabilize_time
                           - args.center_time - 0.5)
            log.info(f"[Cycle {frame_id}] Waiting {wait_time:.1f}s...")

            wait_end = time.monotonic() + wait_time
            while running and time.monotonic() < wait_end:
                time.sleep(0.1)

            if not running:
                break

            # --- Phase 2: Stabilize (PID active) ---
            log.info(f"[Cycle {frame_id}] Stabilizing...")

            # Set new reference from current heading
            plat = shm_platform.read()
            if plat and plat.get("mag_x", 0) != 0:
                pid.reset()
                # Let filter initialize
                for _ in range(5):
                    plat = shm_platform.read()
                    if plat:
                        pid.update(plat)
                    time.sleep(0.02)
                # Set current filtered heading as reference
                pid.heading_ref = pid.filtered_heading
                log.info(f"  Heading ref: {pid.heading_ref:.1f} deg")

            stabilize_end = time.monotonic() + args.stabilize_time
            stable_count = 0
            last_error = 0

            while running and time.monotonic() < stabilize_end:
                plat = shm_platform.read()
                result = pid.update(plat)

                if result:
                    servo_angle, error, is_stable = result
                    last_error = error
                    if servo_angle is not None:
                        servo.set_position(servo_angle)
                    if is_stable:
                        stable_count += 1

                time.sleep(0.01)

            if not running:
                break

            # --- Phase 3: Capture ---
            log.info(f"[Cycle {frame_id}] Capturing (error: {last_error:+.1f} deg)")

            # Read current state for metadata
            plat = shm_platform.read()
            cam_imu = shm_camera.read()

            metadata = {
                "frame_id": frame_id,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "heading_ref": pid.heading_ref,
                "heading_current": pid.filtered_heading,
                "heading_error": last_error,
                "servo_position": servo.last_sent,
                "stable_count": stable_count,
                "platform_gyro_z": plat.get("gyro_z", 0) if plat else 0,
                "camera_gyro_z": cam_imu.get("gyro_z", 0) if cam_imu else 0,
            }

            if camera:
                camera.capture(frame_id, metadata)
            else:
                log.info(f"  [no-camera] Would capture frame {frame_id}")

            frame_id += 1

            # Check photo limit
            if args.num_photos > 0 and frame_id >= args.num_photos:
                log.info(f"Photo limit reached ({args.num_photos})")
                break

            # --- Phase 4: Slow center ---
            log.info(f"[Cycle {frame_id}] Centering servo...")
            servo.slow_center(duration=args.center_time)
            servo.detach()

    finally:
        log.info("")
        log.info("Shutting down...")
        servo.close()
        if preview:
            preview.stop()
        if camera:
            camera.close()
        imu_camera.stop()
        imu_platform.stop()
        log.info(f"Total photos: {frame_id}")
        log.info("Done.")


if __name__ == "__main__":
    main()
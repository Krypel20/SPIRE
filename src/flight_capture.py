#!/usr/bin/env python3
"""
SPIRE Flight Capture v2
Integrated capture + yaw stabilization with adaptive PID.
 
Improvements over v1:
  - Adaptive KP: measured rotation speed before stabilization sets KP
  - Adaptive KD: gain scheduling based on heading error (from stabilize_demo)
  - Capture gate: waits for heading error AND camera gyro below thresholds
  - IMU timestamp sync: records IMU state at moment of capture
  - Magnetometer fallback: skips capture if mag data unavailable
  - Camera IMU in gate: verifies camera is actually stable before capture
 
Cycle:
  1. Servo detached (no payload disturbance)
  2. Measure platform rotation speed (last 1s of wait phase)
  3. Set adaptive KP based on rotation speed
  4. Set heading reference, enable PID stabilization
  5. Gate: wait for error < threshold AND camera gyro_z < threshold
  6. Capture photo with IMU timestamp
  7. Slowly center servo
  8. Repeat
 
Usage:
  python3 src/flight_capture.py --interval 10 -n 10 --preview
  python3 src/flight_capture.py --kp-low 0.3 --kp-high 1.5 --kd-low 0.4 --kd-high 0.8
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
import threading
import io
import socket
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
 
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
# IMU subprocess
# ---------------------------------------------------------------------------
 
class IMUProcess:
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
        cmd = [
            sys.executable, os.path.join(src_dir, "imu_reader.py"),
            "--sensor", self.sensor,
            "-r", str(self.rate),
            "--shm-name", self.shm_name,
            "--cal", self.cal_path,
        ]
        if self.address:
            cmd += ["-a", self.address]
        if self.mag:
            cmd += ["--mag"]
        log.info(f"Starting [{self.name}]: {self.sensor} -> {self.shm_name}")
        self.proc = subprocess.Popen(
            cmd, cwd=project_dir,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
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
    def __init__(self, output_dir):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.cam = None
        self.session_photos = []
        self.last_capture_time = 0
 
    def init(self):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        config = self.cam.create_still_configuration(
            main={"size": (4056, 3040)},
            lores={"size": (800, 600)},
        )
        self.cam.configure(config)
        self.cam.start()
        time.sleep(1)
        log.info("Camera initialized")
 
    def capture(self, frame_id, metadata=None):
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        filename = f"spire_{timestamp}_{frame_id:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        self.cam.capture_file(filepath)
        self.session_photos.append(filename)
        self.last_capture_time = time.time()
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
 
# ---------------------------------------------------------------------------
# Preview server
# ---------------------------------------------------------------------------
 
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
 
 
class PreviewServer:
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
                      .stream { flex:2; position:relative; display:flex; align-items:center; justify-content:center; }
                      .stream img { max-width:100%; max-height:100%; }
                      .flash { position:absolute; top:0; left:0; right:0; bottom:0;
                               background:rgba(0,255,0,0.3); display:none;
                               align-items:center; justify-content:center;
                               font-size:48px; font-weight:bold; color:#0f0; }
                      .gallery { flex:1; border-left:2px solid #333; }
                      .gallery iframe { width:100%; height:100%; border:none; }
                    </style>
                    <script>
                      var lastCount = 0;
                      function checkCapture() {
                        var xhr = new XMLHttpRequest();
                        xhr.open('GET', '/status', true);
                        xhr.onload = function() {
                          if (xhr.status === 200) {
                            var data = JSON.parse(xhr.responseText);
                            if (data.photos > lastCount && lastCount > 0) {
                              document.getElementById('flash').style.display='flex';
                              document.getElementById('gallery').src='/gallery?' + Date.now();
                              setTimeout(function(){ document.getElementById('flash').style.display='none'; }, 1500);
                            }
                            lastCount = data.photos;
                          }
                        };
                        xhr.send();
                      }
                      setInterval(checkCapture, 500);
                    </script>
                    </head><body>
                    <div class="container">
                      <div class="stream">
                        <img src="/stream">
                        <div class="flash" id="flash">CAPTURED</div>
                      </div>
                      <div class="gallery"><iframe id="gallery" src="/gallery"></iframe></div>
                    </div>
                    </body></html>""")
 
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            frame_arr = cam.cam.capture_array("lores")
                            from PIL import Image
                            img = Image.fromarray(frame_arr)
                            buf = io.BytesIO()
                            img.save(buf, format="JPEG", quality=70)
                            frame = buf.getvalue()
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(frame)}\r\n".encode())
                            self.wfile.write(b"\r\n")
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                            time.sleep(0.033)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
 
                elif self.path.startswith("/gallery"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    photos = list(reversed(cam.session_photos)) if cam.session_photos else []
                    html = """<html><head><meta charset="utf-8">
                    <style>
                      body { margin:10px; background:#111; color:#eee; font-family:monospace; }
                      h3 { color:#0ff; margin:0 0 10px; }
                      img { width:100%; margin-bottom:8px; border:1px solid #333; }
                      .info { font-size:11px; color:#888; margin-bottom:5px; }
                    </style>
                    </head><body>"""
                    html += f"<h3>Session Photos ({len(photos)})</h3>"
                    for p in photos[:20]:
                        html += f'<div class="info">{p}</div>'
                        html += f'<a href="/photo/{p}" target="_blank">'
                        html += f'<img src="/photo/{p}"></a>\n'
                    if not photos:
                        html += "<p>No photos yet in this session</p>"
                    html += "</body></html>"
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
 
                elif self.path == "/status":
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        "photos": len(cam.session_photos),
                        "last_capture": cam.last_capture_time,
                        "capturing": (time.time() - cam.last_capture_time) < 2.0,
                    }).encode())
 
                else:
                    self.send_error(404)
 
            def log_message(self, format, *args):
                pass
 
        self.server = ThreadedHTTPServer(("0.0.0.0", self.port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = "localhost"
        log.info(f"Preview: http://{ip}:{self.port}")
 
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
        angle_deg = max(-self.range_deg, min(self.range_deg, angle_deg))
        self.current_position = angle_deg
        if abs(angle_deg - self.last_sent) > threshold:
            value = angle_deg / self.range_deg
            value = max(-1.0, min(1.0, value))
            self.servo.value = value
            self.last_sent = angle_deg
 
    def slow_center(self, duration=1.5):
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
# Stabilization PID with adaptive KP and KD
# ---------------------------------------------------------------------------
 
class StabilizationPID:
    def __init__(self, kp_low=0.3, kp_high=1.5,
                 kd_low=0.4, kd_high=0.8, ki=0.02,
                 heading_deadband=5.0, gyro_deadband=0.5,
                 comp_alpha=0.02):
        # Adaptive KP boundaries
        self.kp_low = kp_low
        self.kp_high = kp_high
        self.kp_active = kp_low  # Set per cycle
 
        # Adaptive KD boundaries
        self.kd_low = kd_low
        self.kd_high = kd_high
 
        # Fixed KI
        self.ki = ki
 
        # Deadbands
        self.heading_deadband = heading_deadband
        self.gyro_deadband = gyro_deadband
 
        # Complementary filter
        self.comp_alpha = comp_alpha
 
        # Error thresholds for KD gain scheduling
        self.kd_error_low = 5.0
        self.kd_error_high = 15.0
 
        # Rotation speed thresholds for KP scheduling
        self.kp_speed_low = 5.0    # Below: use kp_low
        self.kp_speed_high = 90.0  # Above: use kp_high
 
        # State
        self.heading_ref = None
        self.filtered_heading = None
        self.integral = 0.0
        self.last_time = None
 
    def set_kp_from_rotation_speed(self, speed_dps):
        """Set adaptive KP based on measured platform rotation speed."""
        if speed_dps <= self.kp_speed_low:
            self.kp_active = self.kp_low
        elif speed_dps >= self.kp_speed_high:
            self.kp_active = self.kp_high
        else:
            t = (speed_dps - self.kp_speed_low) / (self.kp_speed_high - self.kp_speed_low)
            self.kp_active = self.kp_low + t * (self.kp_high - self.kp_low)
        log.info(f"  Adaptive KP: {self.kp_active:.2f} (rotation: {speed_dps:.1f} deg/s)")
 
    def reset(self, initial_heading=None):
        self.heading_ref = initial_heading
        self.filtered_heading = initial_heading
        self.integral = 0.0
        self.last_time = time.monotonic()
 
    def update(self, plat_data):
        """Compute servo position from platform IMU data.
 
        Returns:
            (servo_angle, error, is_stable) or None if no data
            servo_angle=None means hold position (inside deadband)
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
 
        # Complementary filter
        gyro_heading = self.filtered_heading + gyro_rate * dt
        mag_diff = heading_error(mag_heading, gyro_heading)
        self.filtered_heading = gyro_heading + self.comp_alpha * mag_diff
        self.filtered_heading = self.filtered_heading % 360
 
        # Heading error
        error = heading_error(self.filtered_heading, self.heading_ref)
 
        # Deadband — hold position
        if abs(error) < self.heading_deadband:
            return (None, error, True)
 
        # Adaptive KD based on error magnitude
        abs_error = abs(error)
        if abs_error <= self.kd_error_low:
            kd_active = self.kd_low
        elif abs_error >= self.kd_error_high:
            kd_active = self.kd_high
        else:
            t = (abs_error - self.kd_error_low) / (self.kd_error_high - self.kd_error_low)
            kd_active = self.kd_low + t * (self.kd_high - self.kd_low)
 
        # P (adaptive)
        p_out = self.kp_active * error
 
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
# Rotation speed measurement
# ---------------------------------------------------------------------------
 
def measure_rotation_speed(shm_platform, duration=1.0):
    """Measure average absolute rotation speed over duration.
 
    Args:
        shm_platform: SHMReader for platform IMU
        duration: measurement window in seconds
 
    Returns:
        Average absolute gyro_z in deg/s
    """
    samples = []
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        plat = shm_platform.read()
        if plat:
            samples.append(abs(plat.get("gyro_z", 0.0)))
        time.sleep(0.01)
 
    if samples:
        return sum(samples) / len(samples)
    return 0.0
 
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(
        description="SPIRE Flight Capture v2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --interval 10 -n 10 --preview
  %(prog)s --kp-low 0.3 --kp-high 1.5 --kd-low 0.4 --kd-high 0.8
  %(prog)s --interval 15 --gate-timeout 3.0 --gate-error 3.0
        """
    )
 
    # Capture
    parser.add_argument("--interval", type=float, default=15,
                        help="Seconds between photos (default: 15)")
    parser.add_argument("-n", "--num-photos", type=int, default=0,
                        help="Number of photos, 0=infinite (default: 0)")
    parser.add_argument("-o", "--output", default="data/flight",
                        help="Output directory (default: data/flight)")
 
    # Timing
    parser.add_argument("--stabilize-time", type=float, default=2.0,
                        help="Min stabilization time before gate (default: 2.0)")
    parser.add_argument("--center-time", type=float, default=1.5,
                        help="Slow centering duration (default: 1.5)")
    parser.add_argument("--measure-time", type=float, default=1.0,
                        help="Rotation speed measurement window (default: 1.0)")
 
    # Capture gate
    parser.add_argument("--gate-timeout", type=float, default=1.0,
                        help="Max extra wait for stability after stabilize-time (default: 1.0)")
    parser.add_argument("--gate-error", type=float, default=5.0,
                        help="Max heading error to allow capture (default: 5.0 deg)")
    parser.add_argument("--gate-gyro", type=float, default=2.0,
                        help="Max camera gyro_z to allow capture (default: 2.0 deg/s)")
    parser.add_argument("--gate-count", type=int, default=10,
                        help="Required consecutive stable readings (default: 10)")
 
    # Adaptive PID
    parser.add_argument("--kp-low", type=float, default=0.3,
                        help="KP for slow rotation (default: 0.3)")
    parser.add_argument("--kp-high", type=float, default=1.5,
                        help="KP for fast rotation (default: 1.5)")
    parser.add_argument("--kd-low", type=float, default=0.4,
                        help="KD for small error (default: 0.4)")
    parser.add_argument("--kd-high", type=float, default=0.8,
                        help="KD for large error (default: 0.8)")
    parser.add_argument("--ki", type=float, default=0.02,
                        help="Integral gain (default: 0.02)")
    parser.add_argument("--heading-deadband", type=float, default=3.0,
                        help="Heading deadband deg (default: 3.0)")
 
    # Hardware
    parser.add_argument("--servo-pin", type=int, default=12)
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera (test stabilization only)")
    parser.add_argument("--preview", action="store_true",
                        help="Enable MJPEG preview server")
    parser.add_argument("--preview-port", type=int, default=8080)
 
    # IMU
    parser.add_argument("--cam-cal", default="config/imu_calibration.json")
    parser.add_argument("--plat-cal", default="config/lsm9ds1_calibration.json")
 
    parser.add_argument("-v", "--verbose", action="store_true")
 
    args = parser.parse_args()
    setup_logging(args.verbose)
 
    log.info("=" * 50)
    log.info("SPIRE Flight Capture v2")
    log.info("=" * 50)
    log.info(f"Interval: {args.interval}s | Stabilize: {args.stabilize_time}s")
    log.info(f"Gate: error<{args.gate_error} deg, gyro<{args.gate_gyro} deg/s, "
             f"count={args.gate_count}, timeout={args.gate_timeout}s")
    log.info(f"PID: KP={args.kp_low}-{args.kp_high} KD={args.kd_low}-{args.kd_high} "
             f"KI={args.ki}")
    log.info("")
 
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
        cam_data = shm_camera.read()
        if plat and cam_data:
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
        kp_low=args.kp_low, kp_high=args.kp_high,
        kd_low=args.kd_low, kd_high=args.kd_high,
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
            # Check IMU health
            if not imu_camera.is_alive() or not imu_platform.is_alive():
                log.error("IMU process died!")
                break
 
            # --- Phase 1: Wait (servo detached) ---
            wait_time = max(0, args.interval - args.stabilize_time
                           - args.gate_timeout - args.center_time
                           - args.measure_time - 0.5)
            log.info(f"[Cycle {frame_id}] Waiting {wait_time:.1f}s...")
 
            wait_end = time.monotonic() + wait_time
            while running and time.monotonic() < wait_end:
                time.sleep(0.1)
            if not running:
                break
 
            # --- Phase 2: Measure rotation speed ---
            log.info(f"[Cycle {frame_id}] Measuring rotation speed...")
            rotation_speed = measure_rotation_speed(
                shm_platform, duration=args.measure_time
            )
 
            # --- Phase 3: Set adaptive KP and start stabilization ---
            log.info(f"[Cycle {frame_id}] Stabilizing...")
 
            # Check magnetometer availability
            plat = shm_platform.read()
            if not plat or (plat.get("mag_x", 0) == 0 and plat.get("mag_y", 0) == 0):
                log.warning(f"[Cycle {frame_id}] No magnetometer data — skipping capture")
                frame_id += 1
                continue
 
            # Set adaptive KP
            pid.set_kp_from_rotation_speed(rotation_speed)
 
            # Reset PID with fresh heading reference
            pid.reset()
            for _ in range(5):
                plat = shm_platform.read()
                if plat:
                    pid.update(plat)
                time.sleep(0.02)
            pid.heading_ref = pid.filtered_heading
            log.info(f"  Heading ref: {pid.heading_ref:.1f} deg")
 
            # Run stabilization for minimum time
            stabilize_end = time.monotonic() + args.stabilize_time
            last_error = 0
            while running and time.monotonic() < stabilize_end:
                plat = shm_platform.read()
                result = pid.update(plat)
                if result:
                    servo_angle, error, is_stable = result
                    last_error = error
                    if servo_angle is not None:
                        servo.set_position(servo_angle)
                time.sleep(0.01)
            if not running:
                break
 
            # --- Phase 4: Capture gate ---
            log.info(f"[Cycle {frame_id}] Gate check...")
            gate_passed = False
            stable_count = 0
            gate_deadline = time.monotonic() + args.gate_timeout
 
            while running and time.monotonic() < gate_deadline:
                plat = shm_platform.read()
                cam_data = shm_camera.read()
                result = pid.update(plat)
 
                if result:
                    servo_angle, error, is_stable = result
                    last_error = error
                    if servo_angle is not None:
                        servo.set_position(servo_angle)
 
                    # Gate conditions:
                    # 1. Heading error below threshold
                    # 2. Camera gyro_z below threshold (camera actually stable)
                    cam_gyro_z = abs(cam_data.get("gyro_z", 99)) if cam_data else 99
                    error_ok = abs(error) < args.gate_error
                    gyro_ok = cam_gyro_z < args.gate_gyro
 
                    if error_ok and gyro_ok:
                        stable_count += 1
                    else:
                        stable_count = 0
 
                    if stable_count >= args.gate_count:
                        gate_passed = True
                        break
 
                time.sleep(0.01)
 
            if not running:
                break
 
            # --- Phase 5: Capture ---
            # Read IMU state at moment of capture
            plat_at_capture = shm_platform.read()
            cam_at_capture = shm_camera.read()
            capture_mono_ns = time.monotonic_ns()
 
            gate_status = "PASS" if gate_passed else "TIMEOUT"
            cam_gz = abs(cam_at_capture.get("gyro_z", 0)) if cam_at_capture else 0
            log.info(f"[Cycle {frame_id}] Capture [{gate_status}] "
                     f"error:{last_error:+.1f} deg, "
                     f"cam_gz:{cam_gz:.1f} deg/s, "
                     f"kp:{pid.kp_active:.2f}")
 
            metadata = {
                "frame_id": frame_id,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "capture_mono_ns": capture_mono_ns,
                "gate_status": gate_status,
                "gate_stable_count": stable_count,
                "heading_ref": pid.heading_ref,
                "heading_current": pid.filtered_heading,
                "heading_error": last_error,
                "servo_position": servo.last_sent,
                "rotation_speed_dps": rotation_speed,
                "kp_active": pid.kp_active,
                "platform_gyro_z": plat_at_capture.get("gyro_z", 0) if plat_at_capture else 0,
                "platform_imu_ts": plat_at_capture.get("timestamp_mono_ns", 0) if plat_at_capture else 0,
                "camera_gyro_z": cam_at_capture.get("gyro_z", 0) if cam_at_capture else 0,
                "camera_imu_ts": cam_at_capture.get("timestamp_mono_ns", 0) if cam_at_capture else 0,
            }
 
            if camera:
                camera.capture(frame_id, metadata)
            else:
                log.info(f"  [no-camera] Would capture frame {frame_id}")
 
            frame_id += 1
 
            if args.num_photos > 0 and frame_id >= args.num_photos:
                log.info(f"Photo limit reached ({args.num_photos})")
                break
 
            # --- Phase 6: Slow center ---
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
#!/usr/bin/env python3
"""
SPIRE Flight Capture v2.1
Integrated capture + yaw stabilization with adaptive PID.

Capture fires during active servo compensation (not after servo returns to 0).

Cycle:
  1. Servo detached (no payload disturbance)
  2. Measure platform rotation speed
  3. Set adaptive KP, heading reference, run PID
  4. Gate (inside PID loop): capture when error OK AND servo counter-rotating
  5. Slowly center servo
  6. Repeat

Usage:
  python3 src/flight_capture.py --interval 10 -n 10 --preview
  python3 src/flight_capture.py --gate-servo-min 4 --gate-plat-gyro 0.8
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
import collections
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

class StreamingOutput(io.BufferedIOBase):
    """MJPEG frame buffer for preview streaming."""
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
 
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
        from picamera2.encoders import MJPEGEncoder
        from picamera2.outputs import FileOutput
        self.cam = Picamera2()
        config = self.cam.create_still_configuration(
            main={"size": (4056, 3040)},
            lores={"size": (1280, 960)},
        )
        self.cam.configure(config)
        self.streaming_output = StreamingOutput()
        self.cam.start_recording(
            MJPEGEncoder(), FileOutput(self.streaming_output), name="lores"
        )
        time.sleep(1)
        log.info("Camera initialized")
 
    def capture(self, frame_id, metadata=None):
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        filename = f"spire_{timestamp}_{frame_id:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        # capture_request() returns a frame already in flight together with its
        # hardware SensorTimestamp, instead of scheduling a future exposure.
        request = self.cam.capture_request()
        try:
            request.save("main", filepath)
            cam_meta = request.get_metadata()
        finally:
            request.release()
        self.session_photos.append(filename)
        self.last_capture_time = time.time()
        if metadata:
            # SensorTimestamp: hardware exposure time (ns), CLOCK_BOOTTIME domain.
            metadata["sensor_timestamp_ns"] = cam_meta.get("SensorTimestamp", 0)
            metadata["exposure_time_us"] = cam_meta.get("ExposureTime", 0)
            meta_path = filepath.replace(".jpg", "_meta.json")
            with open(meta_path, "w") as f:
                json.dump(metadata, f, indent=2)
        log.info(f"Photo {frame_id}: {filename}")
        return filepath

    def capture_burst(self, frame_id, n):
        """Capture n consecutive full-res frames to temp files.

        Returns list of dicts: {tmp_path, sensor_timestamp_ns, exposure_time_us}.
        Each frame is saved and its request released immediately to avoid
        exhausting the camera buffer pool. Selection happens in the caller.
        """
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        frames = []
        for i in range(n):
            tmp_path = os.path.join(
                self.output_dir, f".burst_{timestamp}_{frame_id:04d}_{i}.jpg"
            )
            request = self.cam.capture_request()
            try:
                request.save("main", tmp_path)
                cam_meta = request.get_metadata()
            finally:
                request.release()
            frames.append({
                "tmp_path": tmp_path,
                "sensor_timestamp_ns": cam_meta.get("SensorTimestamp", 0),
                "exposure_time_us": cam_meta.get("ExposureTime", 0),
            })
        return frames

    def commit_burst_winner(self, frame_id, winner, losers, metadata=None):
        """Rename winning temp frame to final name, delete losers, write meta."""
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        filename = f"spire_{timestamp}_{frame_id:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        os.replace(winner["tmp_path"], filepath)
        for f in losers:
            try:
                os.remove(f["tmp_path"])
            except FileNotFoundError:
                pass
        self.session_photos.append(filename)
        self.last_capture_time = time.time()
        if metadata:
            metadata["sensor_timestamp_ns"] = winner["sensor_timestamp_ns"]
            metadata["exposure_time_us"] = winner["exposure_time_us"]
            meta_path = filepath.replace(".jpg", "_meta.json")
            with open(meta_path, "w") as f:
                json.dump(metadata, f, indent=2)
        log.info(f"Photo {frame_id}: {filename}")
        return filepath
 
    def close(self):
        if self.cam:
            self.cam.stop_recording()
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
                            with cam.streaming_output.condition:
                                cam.streaming_output.condition.wait()
                                frame = cam.streaming_output.frame
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
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
    """Hardware-PWM servo control on RPi 5 via rpi-hardware-pwm (jitter-free).

    GPIO 12 = PWM0 channel 0 on Pi 5 (physical pin 32).
    Requires: dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 in config.txt.
    On Linux kernel 6.12+ (Debian Trixie) all models use chip=0.
    """
    PERIOD_MS = 20.0          # 50 Hz servo frame
    MIN_PULSE_MS = 0.5        # maps to -range_deg
    MAX_PULSE_MS = 2.5        # maps to +range_deg

    def __init__(self, pin=12, channel=0, chip=0):
        from rpi_hardware_pwm import HardwarePWM
        self.pwm = HardwarePWM(pwm_channel=channel, hz=50, chip=chip)
        self.range_deg = 135.0
        self.current_position = 0.0
        self.last_sent = 0.0
        self._attached = False
        log.info(f"Servo on GPIO {pin} via hardware PWM ch{channel} (detached)")

    def _angle_to_duty(self, angle_deg):
        # Map [-range, +range] -> [MIN_PULSE, MAX_PULSE] ms, then to duty %
        frac = (angle_deg + self.range_deg) / (2 * self.range_deg)  # 0..1
        pulse_ms = self.MIN_PULSE_MS + frac * (self.MAX_PULSE_MS - self.MIN_PULSE_MS)
        return pulse_ms / self.PERIOD_MS * 100.0

    def _drive(self, angle_deg):
        duty = self._angle_to_duty(angle_deg)
        if not self._attached:
            self.pwm.start(duty)
            self._attached = True
        else:
            self.pwm.change_duty_cycle(duty)

    def set_position(self, angle_deg, threshold=1.0):
        angle_deg = max(-self.range_deg, min(self.range_deg, angle_deg))
        self.current_position = angle_deg
        if abs(angle_deg - self.last_sent) > threshold:
            self._drive(angle_deg)
            self.last_sent = angle_deg

    def slow_center(self, duration=1.5):
        start = self.last_sent
        steps = 30
        for i in range(steps + 1):
            t = i / steps
            angle = start * (1 - t)
            self._drive(angle)
            self.last_sent = angle
            time.sleep(duration / steps)
        self.current_position = 0.0
        self.last_sent = 0.0

    def detach(self):
        # Stop pulse train -- servo goes limp (no holding torque, no jitter)
        if self._attached:
            self.pwm.stop()
            self._attached = False

    def close(self):
        self.detach()

# ---------------------------------------------------------------------------
# Stabilization PID with adaptive KP and KD
# ---------------------------------------------------------------------------
 
class StabilizationPID:
    def __init__(self, kp_low=0.3, kp_high=1.5,
                 kd_low=0.4, kd_high=0.8, ki=0.02,
                 heading_deadband=5.0, gyro_deadband=0.5,
                 comp_alpha=0.02, d_alpha=0.3):
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

        # D-term low-pass filter (damps gyro noise at low rotation)
        self.d_alpha = d_alpha
        self.d_filtered = 0.0
 
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
        self.d_filtered = 0.0
        self.last_time = time.monotonic()
 
    def update(self, plat_data):
        """Compute servo position from platform IMU data.

        Returns:
            (servo_angle, error, is_correcting) or None if no data.
            Deadband suppresses P only; I/D stay active for ongoing rotation.
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
        plat_gyro_z = gyro_rate

        mag_heading = compute_heading(mag_x, mag_y)

        if self.heading_ref is None:
            self.heading_ref = mag_heading
            self.filtered_heading = mag_heading
            return (0.0, 0.0, False)

        gyro_heading = self.filtered_heading + gyro_rate * dt
        mag_diff = heading_error(mag_heading, gyro_heading)
        self.filtered_heading = gyro_heading + self.comp_alpha * mag_diff
        self.filtered_heading = self.filtered_heading % 360

        error = heading_error(self.filtered_heading, self.heading_ref)
        in_deadband = abs(error) < self.heading_deadband

        abs_error = abs(error)
        if abs_error <= self.kd_error_low:
            kd_active = self.kd_low
        elif abs_error >= self.kd_error_high:
            kd_active = self.kd_high
        else:
            t = (abs_error - self.kd_error_low) / (self.kd_error_high - self.kd_error_low)
            kd_active = self.kd_low + t * (self.kd_high - self.kd_low)

        # Deadband: no P chase on tiny error, but keep I/D for counter-rotation
        p_out = 0.0 if in_deadband else self.kp_active * error

        self.integral += error * dt
        self.integral = max(-135.0, min(135.0, self.integral))
        i_out = self.ki * self.integral

        d_gyro = gyro_rate
        if abs(d_gyro) < self.gyro_deadband:
            d_gyro = 0.0
        # Low-pass filter D source to suppress gyro noise at low rotation
        self.d_filtered = (1 - self.d_alpha) * self.d_filtered + self.d_alpha * d_gyro
        d_out = kd_active * self.d_filtered

        servo_angle = -(p_out + i_out + d_out)
        servo_angle = max(-135.0, min(135.0, servo_angle))

        is_correcting = (
            abs(servo_angle) >= self.heading_deadband
            or abs(plat_gyro_z) >= self.gyro_deadband
        )
        return (servo_angle, error, is_correcting)
 
# ---------------------------------------------------------------------------
# Camera gyro logger — high-rate ring buffer for exposure-time correlation
# ---------------------------------------------------------------------------

class CamGyroLogger(threading.Thread):
    """High-rate ring buffer of camera gyro_z for exposure-time correlation."""
    def __init__(self, shm_camera, maxlen=4000):
        super().__init__(daemon=True)
        self.shm = shm_camera
        self.buf = collections.deque(maxlen=maxlen)
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            c = self.shm.read()
            if c:
                self.buf.append((c.get("timestamp_mono_ns", 0), c.get("gyro_z", 0.0)))
            time.sleep(0.002)

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)

    def gyro_at(self, mono_ns):
        """Nearest-neighbour cam_gz at a given monotonic timestamp."""
        best, best_dt = None, None
        for ts, gz in self.buf:
            dt = abs(ts - mono_ns)
            if best_dt is None or dt < best_dt:
                best_dt, best = dt, gz
        return best, best_dt

# ---------------------------------------------------------------------------
# PID thread — runs servo independently of capture timing
# ---------------------------------------------------------------------------

class PIDThread(threading.Thread):
    """Continuous PID servo control, independent of capture sequence."""

    def __init__(self, pid, servo, shm_platform):
        super().__init__(daemon=True)
        self.pid = pid
        self.servo = servo
        self.shm_platform = shm_platform
        self._stop_event = threading.Event()
        self.last_error = 0.0

    def run(self):
        while not self._stop_event.is_set():
            plat = self.shm_platform.read()
            result = self.pid.update(plat)
            if result:
                servo_angle, error, _ = result
                self.last_error = error
                self.servo.set_position(servo_angle)
            time.sleep(0.003)  # ~300Hz

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)

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


def capture_gate_ok(error, plat_gyro_z, servo_position, args):
    """True when capture should proceed during active compensation."""
    error_ok = abs(error) < args.gate_error
    plat_gyro = abs(plat_gyro_z)
    servo_abs = abs(servo_position)

    active = (
        error_ok
        and servo_abs >= args.gate_servo_min
        and plat_gyro >= args.gate_plat_gyro
    )
    if active:
        return True

    if not args.gate_allow_calm:
        return False

    # Between swing peaks: payload nearly still, heading locked
    return (
        error_ok
        and plat_gyro < args.gate_plat_gyro
        and servo_abs < args.gate_servo_min
    )

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
    parser.add_argument("--stabilize-time", type=float, default=0.1,
                        help="Servo settle time before burst starts (default: 0.1)")
    parser.add_argument("--cam-gyro-threshold", type=float, default=20.0,
                        help="Max |cam gyro_z| deg/s to trigger capture (default: 20.0)")
    parser.add_argument("--cam-stable-count", type=int, default=5,
                        help="Consecutive stable samples before capture (default: 5)")
    parser.add_argument("--burst-count", type=int, default=6,
                        help="Frames per burst; keep lowest |cam_gz| (1 = no burst, default: 6)")
    parser.add_argument("--no-burst-early", dest="burst_early", action="store_false",
                        help="Disable early burst; wait for the STABLE gate before bursting")
    parser.set_defaults(burst_early=True)
    parser.add_argument("--center-time", type=float, default=1.5,
                        help="Slow centering duration (default: 1.5)")
    parser.add_argument("--measure-time", type=float, default=1.0,
                        help="Rotation speed measurement window (default: 1.0)")
    parser.add_argument("--min-rotation", type=float, default=5.0,
                    help="Min rotation speed deg/s to activate stabilization (default: 5.0)")
    parser.add_argument("--gyro-deadband", type=float, default=0.5,
                    help="Zero D-term below this |gyro_rate| deg/s (default: 0.5)")
    parser.add_argument("--d-alpha", type=float, default=0.3,
                    help="D-term low-pass factor 0-1; lower = more smoothing (default: 0.3)")
 
    # Capture gate (active compensation)
    parser.add_argument("--gate-timeout", type=float, default=2.0,
                        help="Max wait for gate after stabilize-time (default: 2.0)")
    parser.add_argument("--gate-error", type=float, default=8.0,
                        help="Max heading error during capture (default: 8.0 deg)")
    parser.add_argument("--gate-servo-min", type=float, default=4.0,
                        help="Min |servo| deg for active compensation gate (default: 4.0)")
    parser.add_argument("--gate-plat-gyro", type=float, default=0.8,
                        help="Min |platform gyro_z| deg/s during swing (default: 0.8)")
    parser.add_argument("--gate-count", type=int, default=8,
                        help="Consecutive gate-OK readings before capture (default: 8)")
    parser.add_argument("--gate-allow-calm", action="store_true",
                        help="Also capture between swings when payload is nearly still")
 
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
    parser.add_argument("--diag", action="store_true",
                        help="Enable exposure-time cam_gz correlation diagnostics")
 
    parser.add_argument("-v", "--verbose", action="store_true")
 
    args = parser.parse_args()
    setup_logging(args.verbose)
 
    log.info("=" * 50)
    log.info("SPIRE Flight Capture v2.1")
    log.info("=" * 50)
    log.info(f"Interval: {args.interval}s | Stabilize: {args.stabilize_time}s | "
            f"Cam threshold: {args.cam_gyro_threshold} deg/s x{args.cam_stable_count}"
            f" | Burst: {args.burst_count}")
    log.info(f"PID: KP={args.kp_low}-{args.kp_high} KD={args.kd_low}-{args.kd_high} "
            f"KI={args.ki} | gyro_deadband={args.gyro_deadband} d_alpha={args.d_alpha}")
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
        gyro_deadband=args.gyro_deadband, d_alpha=args.d_alpha,
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
    boottime_to_mono_offset_ns = (
        time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    )
    if args.diag:
        log.info(f"Clock offset BOOTTIME-MONOTONIC: "
                 f"{boottime_to_mono_offset_ns/1e6:.1f} ms")
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
            log.info(f"[Cycle {frame_id}] Stabilize + gate (inline capture)...")
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

            log.info(f"[Cycle {frame_id}] Stabilize + capture...")

            # Skip stabilization if rotation too slow — servo would disturb stationary payload
            if rotation_speed < args.min_rotation:
                log.info(f"[Cycle {frame_id}] Capture [CALM] rotation={rotation_speed:.1f} deg/s")
                plat = shm_platform.read()
                cam_data = shm_camera.read()
                plat_gz = plat.get("gyro_z", 0.0) if plat else 0.0
                cam_gz = cam_data.get("gyro_z", 0.0) if cam_data else 0.0
                metadata = {
                    "frame_id": frame_id,
                    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                    "capture_mono_ns": time.monotonic_ns(),
                    "capture_status": "CALM",
                    "heading_error": 0.0,
                    "servo_position": 0.0,
                    "rotation_speed_dps": rotation_speed,
                    "kp_active": pid.kp_active,
                    "platform_gyro_z": plat_gz,
                    "camera_gyro_z": cam_gz,
                }
                if camera:
                    camera.capture(frame_id, metadata)
                else:
                    log.info(f"  [no-camera] Would capture frame {frame_id}")
                frame_id += 1
                if args.num_photos > 0 and frame_id >= args.num_photos:
                    log.info(f"Photo limit reached ({args.num_photos})")
                    break
                log.info(f"[Cycle {frame_id}] Centering servo...")
                servo.slow_center(duration=args.center_time)
                servo.detach()
                continue

            # --- Phase 3: PID thread starts, runs servo independently ---
            pid_thread = PIDThread(pid, servo, shm_platform)
            pid_thread.start()

            # Logger needed for burst frame selection and for diag correlation
            need_logger = args.diag or args.burst_count > 1
            cam_logger = CamGyroLogger(shm_camera) if need_logger else None
            if cam_logger:
                cam_logger.start()

            # Minimum stabilization window before monitoring
            stab_end = time.monotonic() + args.stabilize_time
            while running and time.monotonic() < stab_end:
                time.sleep(0.01)

            if not running:
                pid_thread.stop()
                break

            # --- Phase 4: Monitor cam_gz, capture when camera is stable ---
            cam_stable_count = 0
            captured = False
            cam_data = None

            if args.burst_early and args.burst_count > 1:
                # Early burst: skip the STABLE gate, capture across the whole
                # servo correction arc right after stabilize_time. Selection by
                # cam_gz then picks the best frame from the full sweep.
                captured = True
            else:
                capture_deadline = time.monotonic() + args.gate_timeout
                while running and time.monotonic() < capture_deadline:
                    cam_data = shm_camera.read()
                    plat = shm_platform.read()
                    cam_gz_abs = abs(cam_data.get("gyro_z", 999.0)) if cam_data else 999.0
                    plat_gz_abs = abs(plat.get("gyro_z", 0.0)) if plat else 0.0

                    if cam_gz_abs < args.cam_gyro_threshold and plat_gz_abs >= args.min_rotation:
                        cam_stable_count += 1
                    else:
                        cam_stable_count = 0

                    if cam_stable_count >= args.cam_stable_count:
                        captured = True
                        break

                    time.sleep(0.01)

            if not running:
                pid_thread.stop()
                break

            # Snapshot state — servo STILL actively compensating during exposure
            plat = shm_platform.read()
            cam_data = shm_camera.read()
            plat_gz = plat.get("gyro_z", 0.0) if plat else 0.0
            cam_gz = cam_data.get("gyro_z", 0.0) if cam_data else 0.0
            if args.burst_early and args.burst_count > 1:
                status = "BURST-EARLY"
            else:
                status = "STABLE" if captured else "TIMEOUT"
            log.info(
                f"[Cycle {frame_id}] Capture [{status}] "
                f"error:{pid_thread.last_error:+.1f} deg, "
                f"servo:{servo.last_sent:+.1f} deg, "
                f"plat_gz:{plat_gz:+.1f} deg/s, "
                f"cam_gz:{cam_gz:+.1f} deg/s"
            )
            metadata = {
                "frame_id": frame_id,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "capture_mono_ns": time.monotonic_ns(),
                "capture_status": status,
                "cam_stable_count": cam_stable_count,
                "heading_ref": pid.heading_ref,
                "heading_current": pid.filtered_heading,
                "heading_error": pid_thread.last_error,
                "servo_position": servo.last_sent,
                "rotation_speed_dps": rotation_speed,
                "kp_active": pid.kp_active,
                "platform_gyro_z": plat_gz,
                "platform_imu_ts": plat.get("timestamp_mono_ns", 0) if plat else 0,
                "camera_gyro_z": cam_gz,
                "camera_imu_ts": cam_data.get("timestamp_mono_ns", 0) if cam_data else 0,
            }
            if camera:
                if args.burst_count > 1 and cam_logger:
                    # Burst: capture N frames while servo still compensates,
                    # then keep the one with the lowest |cam_gz| at its exposure.
                    frames = camera.capture_burst(frame_id, args.burst_count)
                    cam_logger.stop()
                    scored = []
                    for f in frames:
                        sts = f["sensor_timestamp_ns"]
                        if sts:
                            exp_mono = sts - boottime_to_mono_offset_ns
                            gz, gap = cam_logger.gyro_at(exp_mono)
                            f["exposure_cam_gz"] = gz if gz is not None else 999.0
                            f["match_gap_ms"] = gap / 1e6 if gap is not None else -1.0
                        else:
                            f["exposure_cam_gz"] = 999.0
                            f["match_gap_ms"] = -1.0
                        scored.append(f)
                    winner = min(scored, key=lambda f: abs(f["exposure_cam_gz"]))
                    losers = [f for f in scored if f is not winner]
                    metadata["burst_count"] = args.burst_count
                    metadata["burst_exposure_cam_gz"] = [
                        round(f["exposure_cam_gz"], 1) for f in scored
                    ]
                    metadata["winner_exposure_cam_gz"] = round(
                        winner["exposure_cam_gz"], 1
                    )
                    camera.commit_burst_winner(frame_id, winner, losers, metadata)
                    log.info(
                        f"[Cycle {frame_id}] BURST {args.burst_count} frames, "
                        f"exposure_cam_gz={metadata['burst_exposure_cam_gz']} "
                        f"-> kept {winner['exposure_cam_gz']:+.1f} deg/s"
                    )
                else:
                    camera.capture(frame_id, metadata)
                    if args.diag and cam_logger:
                        cam_logger.stop()
                        sensor_ts = metadata.get("sensor_timestamp_ns", 0)
                        if sensor_ts:
                            exp_mono = sensor_ts - boottime_to_mono_offset_ns
                            gz_exp, gap = cam_logger.gyro_at(exp_mono)
                            if gz_exp is not None:
                                log.info(
                                    f"[Cycle {frame_id}] DIAG exposure_cam_gz={gz_exp:+.1f} deg/s "
                                    f"(snapshot was {cam_gz:+.1f}, match gap={gap/1e6:.1f} ms)"
                                )
                            else:
                                log.info(f"[Cycle {frame_id}] DIAG no cam_gz sample near exposure")
                        else:
                            log.info(f"[Cycle {frame_id}] DIAG no sensor_timestamp_ns")
            else:
                log.info(f"  [no-camera] Would capture frame {frame_id}")
                if cam_logger:
                    cam_logger.stop()

            # Freeze servo only AFTER exposure completes (avoids passive-gimbal
            # judder and platform re-coupling during the capture window)
            pid_thread.stop()

            frame_id += 1

            if args.num_photos > 0 and frame_id >= args.num_photos:
                log.info(f"Photo limit reached ({args.num_photos})")
                break

            # --- Phase 5: Slow center ---
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
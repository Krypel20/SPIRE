#!/usr/bin/env python3
"""
SPIRE Live Preview
Simple MJPEG server for camera preview in browser.
Use to set focus of the lens.

Run on RPi, open in browser: http://<RPI_IP>:8080
"""

import io
import time
import argparse
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Event
from picamera2 import Picamera2


class MJPEGHandler(BaseHTTPRequestHandler):
    """Handler HTTP serving MJPEG stream."""

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>SPIRE Live Preview</title></head>
<body style="margin:0;background:#111;display:flex;
align-items:center;justify-content:center;height:100vh;
flex-direction:column;">
<h2 style="color:#eee;font-family:monospace;">SPIRE Live Preview</h2>
<img src="/stream" style="max-width:95vw;max-height:85vh;">
<p style="color:#999;font-family:monospace;margin-top:8px;">
Rotate focus ring on the lens. Ctrl+C on RPi to stop.
</p>
</body></html>"""
            self.wfile.write(html.encode())

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            cam = self.server.camera
            try:
                while not self.server.stop_event.is_set():
                    buf = io.BytesIO()
                    cam.capture_file(buf, format="jpeg", name="main")
                    frame = buf.getvalue()

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description="SPIRE Live Preview")
    parser.add_argument("-p", "--port", type=int, default=8080,
                        help="Port HTTP (default: 8080)")
    parser.add_argument("--width", type=int, default=1012,
                        help="Preview width (default: 1012)")
    parser.add_argument("--height", type=int, default=760,
                        help="Preview height (default: 760)")
    args = parser.parse_args()

    cam = Picamera2()
    config = cam.create_still_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        display=None
    )
    cam.configure(config)
    cam.set_controls({"AeEnable": True})
    cam.start()
    time.sleep(1.0)

    stop_event = Event()
    server = HTTPServer(("0.0.0.0", args.port), MJPEGHandler)
    server.camera = cam
    server.stop_event = stop_event

    ip = "<?>"
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        pass

    print(f"=== SPIRE Live Preview ===")
    print(f"Open in browser: http://{ip}:{args.port}")
    print(f"Preview resolution: {args.width}x{args.height}")
    print(f"Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        server.server_close()
        cam.stop()
        cam.close()


if __name__ == "__main__":
    main()
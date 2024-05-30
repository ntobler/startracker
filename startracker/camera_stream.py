"""Web streaming example.

Source code adapted from http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming
"""

import io
import logging
import socketserver
from http import server
from threading import Condition

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from picamera2.sensor_format import SensorFormat

CROP_FACTOR = 1
exposure_ms = 50
gain = 2
framerate = 4

PAGE = """\
<html>
<head>
<title>Raspberry Pi - Surveillance Camera</title>
</head>
<body>
<center><h1>Raspberry Pi - Surveillance Camera</h1></center>
<center><img src="stream.mjpg" width="1280" height="960" style="filter: brightness(1);"></center>
</body>
</html>
"""


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b"\xff\xd8"):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_get(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/index.html")
            self.end_headers()
        elif self.path == "/index.html":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception as e:
                logging.warning("Removed streaming client %s: %s", self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    global output
    output = StreamingOutput()

    picam2 = Picamera2()
    video_config = picam2.create_video_configuration({"size": (640 * 2, 480 * 2)})
    picam2.configure(video_config)

    raw_format = SensorFormat(picam2.sensor_format)
    raw_format.packing = None
    print(raw_format)
    print(raw_format.__dict__)

    print(picam2.camera_properties)
    print(picam2.sensor_modes)

    assert isinstance(CROP_FACTOR, int)
    w, h = picam2.camera_properties["PixelArraySize"]
    crop = (
        (w // 2) - (w // CROP_FACTOR),
        (h // 2) - (h // CROP_FACTOR),
        w // CROP_FACTOR,
        h // CROP_FACTOR,
    )
    picam2.set_controls(
        {
            "ExposureTime": int(exposure_ms * 1000),
            "AnalogueGain": gain,
            "FrameRate": framerate,
            "ScalerCrop": crop,
        }
    )

    encoder = MJPEGEncoder(10000000)
    encoder.framerate = 4
    encoder.output = FileOutput(output)
    picam2.encoders = encoder

    picam2.start_encoder()
    picam2.start()

    ret = 0
    try:
        address = ("", 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    except Exception:
        ret = -1
    finally:
        picam2.stop()
        picam2.stop_encoder()

    return ret


if __name__ == "__main__":
    SystemExit(main())

"""
Flask web application to capture images using a smartphone browser or any other browser.
"""

import queue
import threading
import logging

from flask import Flask, render_template, request, jsonify
from flask_sock import Sock
import numpy as np
import cv2

from . import camera
from . import image_processing
from . import persistent


class ImageData:
    def __init__(self):
        self._image = None
        self._data_lock = threading.Lock()
        self._data_changed = threading.Condition(lock=self._data_lock)

    def put(self, image):
        success, encoded_array = cv2.imencode(".png", image)
        if not success:
            return
        image_data = encoded_array.tobytes()

        self._image = image_data
        with self._data_lock:
            self._data_changed.notify_all()

    def get_blocking(self):
        with self._data_lock:
            self._data_changed.wait()
        assert self._image is not None
        return self._image


class IntrinsicCalibrator:
    def __init__(self):
        self._dir = persistent.Persistent.get_instance().calibration_dir
        self.index = 0
        self._logger = logging.getLogger("Camera")

    def put_image(self, image: np.ndarray):
        file = self._dir / f"image_{self.index:03d}.png"
        cv2.imwrite(str(file), image)
        self._logger.info(f"Put image {file.name}")
        self.index += 1

    def reset(self):
        self.index = 0


class Main:
    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        self.input_queue = queue.Queue()
        self.image_container = ImageData()

        self._logger = logging.getLogger("Camera")
        self._image_cache = None
        self._intrinsic_calibrator = IntrinsicCalibrator()

        threading.Thread(target=self.run, daemon=True).start()

    def singleton_interface(fun):
        def inner(*args):
            return_event = threading.Event()
            self = args[0]
            self.input_queue.put((fun, args, return_event))
            return_event.wait(10)
            return self._get_state()

        return inner

    def _get_state(self):
        state = {
            "intrinic_image_count": self._intrinsic_calibrator.index,
        }
        return state

    @singleton_interface
    def capture(
        self, exposure_ms: float, analog_gain: int, digital_gain: int, binning: int
    ):
        self._logger.info(f"Capture e={exposure_ms} g={analog_gain}")

        settings = self._cam.settings
        settings.exposure_ms = exposure_ms
        settings.analog_gain = analog_gain
        settings.digital_gain = digital_gain
        self._cam.settings = settings

        image = self._cam.capture()

        self.image_container.put(image)

        self._image_cache = image

    @singleton_interface
    def put_calibration_image(self):
        image = self._image_cache
        if image is not None:
            self._intrinsic_calibrator.put_image(image)

    @singleton_interface
    def reset_calibration(self):
        self._intrinsic_calibrator.reset()

    def run(self):
        """Thread run method."""
        print("starting Main")
        settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)
        with self._cam:
            while True:
                method, args, return_event = self.input_queue.get()
                method(*args)
                return_event.set()


main = Main.get_instance()

app = Flask(__name__, template_folder="../web")
sock = Sock(app)


@app.route("/")
def index():
    return render_template("cameraWebgui.html")


@app.post("/capture")
def capture():
    params = request.get_json()
    d = main.capture(
        float(params["exposure_ms"]),
        int(params["gain"]),
        int(params["digital_gain"]),
        int(params["binning"]),
    )
    return jsonify(d)


@app.post("/put_calibration_image")
def put_calibration_image():
    d = main.put_calibration_image()
    return jsonify(d)


@app.post("/reset_calibration")
def reset_calibration():
    d = main.reset_calibration()
    return jsonify(d)


@sock.route("/image")
def image(ws):
    while True:
        image_data = main.image_container.get_blocking()
        ws.send(image_data)


def cli():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    app.run(debug=False, host="127.0.0.1")


if __name__ == "__main__":
    cli()

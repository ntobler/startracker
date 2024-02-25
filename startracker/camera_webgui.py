"""
Flask web application to capture images using a smartphone browser or any other browser.
"""

import threading
import time
import logging
import os

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_sock import Sock, Server, ConnectionClosed
import numpy as np
import cv2

from startracker import camera
from startracker import persistent
from startracker import webutil

from typing import Literal


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


class App(webutil.QueueAbstractClass):

    terminate: bool
    """Set True to terminate the application loop."""

    def __init__(self):
        super().__init__()

        self.terminate = False

        self.image_container = webutil.ImageData()

        self._logger = logging.getLogger("Camera")
        self._image_cache = None
        self._intrinsic_calibrator = IntrinsicCalibrator()

        self._camera_job = "stop"

    def _get_state(self) -> dict:
        state = {
            "intrinic_image_count": self._intrinsic_calibrator.index,
        }
        return state

    @webutil.QueueAbstractClass.queue_abstract
    def set_settings(
        self, exposure_ms: float, analog_gain: int, digital_gain: int, binning: int
    ):
        settings = self._cam.settings
        settings.exposure_ms = exposure_ms
        settings.analog_gain = analog_gain
        settings.digital_gain = digital_gain
        settings.binning = binning
        self._cam.settings = settings

        self._logger.info(f"Setting settings {settings}")

        return self._get_state()

    @webutil.QueueAbstractClass.queue_abstract
    def capture(self, mode: Literal["single", "continuous", "stop", "darkframe"]):
        if mode not in ["single", "continuous", "stop", "darkframe"]:
            raise ValueError(f"Unknown mode: {mode}")

        self._logger.info(f"Capture {mode}")

        self._camera_job = mode

        return self._get_state()

    @webutil.QueueAbstractClass.queue_abstract
    def put_calibration_image(self):
        image = self._image_cache
        if image is not None:
            self._intrinsic_calibrator.put_image(image)
        return self._get_state()

    @webutil.QueueAbstractClass.queue_abstract
    def reset_calibration(self):
        self._intrinsic_calibrator.reset()
        return self._get_state()

    @webutil.QueueAbstractClass.queue_abstract
    def calibrate(self):
        print("calibrate")
        import time

        time.sleep(3)
        print("done")
        return self._get_state()

    def run(self):
        print("starting Main")
        settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)
        with self._cam:
            while not self.terminate:

                if self._camera_job == "darkframe":
                    self._cam.record_darkframe()
                    self._camera_job = "stop"
                elif self._camera_job in ["single", "continuous"]:
                    self._logger.info("Capture image ...")
                    image = self._cam.capture()
                    time.sleep(0.2)
                    self.image_container.put(image)
                    self._image_cache = image
                    if self._camera_job == "single":
                        self._camera_job = "stop"

                self._process_pending_calls()


class WebApp:

    flask_app = None
    app = None

    def __init__(self):

        self._app_thread = threading.Thread(target=self._run_app)

        self.flask_app = Flask(__name__, template_folder="../web")
        self.sock = Sock(self.flask_app)

        self.flask_app.route("/")(self._index)
        self.flask_app.route("/<path:filename>")(self._serve_file)
        self.flask_app.post("/set_settings")(self._set_settings)
        self.flask_app.post("/capture")(self._capture)
        self.flask_app.post("/put_calibration_image")(self._put_calibration_image)
        self.flask_app.post("/reset_calibration")(self._reset_calibration)
        self.flask_app.post("/calibrate")(self._calibrate)
        self.sock.route("/image")(self._image)

    def _run_app(self):
        self.app = App()
        self.app.run()

    def _index(self):
        return render_template("cameraWebgui.html")

    def _serve_file(self, filename: str):
        return send_from_directory("../web", filename)

    def run(self):
        try:
            self._app_thread.start()
            self.flask_app.run(debug=True, host="0.0.0.0", use_reloader=False)
        finally:
            logging.info("Terminated. Clean up app..")
            self.app.terminate = True
            self._app_thread.join()

    def _set_settings(self):
        params = request.get_json()
        d = self.app.set_settings(
            float(params["exposure_ms"]),
            int(params["gain"]),
            int(params["digital_gain"]),
            int(params["binning"]),
        )
        return jsonify(d)

    def _capture(self):
        params = request.get_json()
        d = self.app.capture(str(params["mode"]))
        return jsonify(d)

    def _put_calibration_image(self):
        d = self.app.put_calibration_image()
        return jsonify(d)

    def _reset_calibration(self):
        d = self.app.reset_calibration()
        return jsonify(d)

    def _calibrate(self):
        d = self.app.calibrate()
        return jsonify(d)

    def _image(self, ws: Server):
        try:
            while True:
                image_data = self.app.image_container.get_blocking()
                ws.send(image_data)
        except ConnectionClosed:
            pass


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    if "STARTRACKER_DEBUG" in os.environ:
        from startracker import testing_utils

        logging.warning("Using debug data")
        testing_utils.TestingMaterial(use_existing=True).patch_persistent()
        camera.RpiCamera = testing_utils.DebugCamera

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

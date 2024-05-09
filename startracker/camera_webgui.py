"""
Flask web application to capture images using a smartphone browser or any other browser.
"""

import threading
import time
import logging
import os
import tempfile
import pathlib

from flask import (
    Flask,
    render_template,
    request,
    jsonify,
    send_from_directory,
    Response,
)
from flask_sock import Sock, Server, ConnectionClosed
import numpy as np
import cv2

from startracker import camera
from startracker import persistent
from startracker import webutil
from startracker import calibration
from startracker import kalkam

from typing import Literal, Optional


class IntrinsicCalibrator:

    cal: Optional[kalkam.IntrinsicCalibrationWithData]

    def __init__(self):
        self._dir = persistent.Persistent.get_instance().calibration_dir
        self.index = 0
        self._logger = logging.getLogger("Camera")
        self.set_pattern(19, 11, 283 / 6)
        self.cal = None

    def set_pattern(self, width: int, height: int, size: float):
        self.pattern = kalkam.ChArUcoPattern(width, height, size)

    def put_image(self, image: np.ndarray):
        file = self._dir / f"image_{self.index:03d}.png"
        cv2.imwrite(str(file), image)
        self._logger.info(f"Put image {file.name}")
        self.index += 1

    def reset(self):
        self.index = 0

    def get_images(self):
        return sorted(list(self._dir.glob("*.png")))

    def calibrate(self):
        image_files = self.get_images()

        if len(image_files) < 2:
            return ValueError("Calibrate requires at least 2 pictures")

        self._logger.info(f"Calibrating using {len(image_files)} images")

        self.cal = kalkam.calibration_from_image_files(image_files, self.pattern)
        self._logger.info(f"Calibration finished rms error: {self.cal.rms_error}")

    def plot_cal_png(self, filehandler) -> bytes:
        if self.cal is None:
            return ValueError("No calibration present")

        self._logger.info("Plotting calibration")
        plot = self.cal.plot_opencv()
        self._logger.info("Done")

        success, png = cv2.imencode(".png", plot)
        if not success:
            return ValueError("Error while encoding plot PNG")

        filehandler.write(png)

    def save(self):
        width, height = self.cal.image_size
        cal = calibration.CameraCalibration.make(
            self.cal.intrinsic, width, height, self.cal.dist_coeffs, self.cal.rms_error
        )
        cal.save_json(self._dir / "calibration.json")


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
        self._cam = None

    def _get_state(self) -> dict:
        state = {
            "intrinsic_image_count": self._intrinsic_calibrator.index,
        }
        return state

    @webutil.QueueAbstractClass.queue_abstract
    def set_settings(
        self,
        exposure_ms: float,
        analog_gain: int,
        digital_gain: int,
        binning: int,
        pattern_width: int,
        pattern_height: int,
        pattern_size: float,
    ):
        if self._cam is None:
            raise ValueError("Camera is not initialized")

        settings = self._cam.settings
        settings.exposure_ms = exposure_ms
        settings.analog_gain = analog_gain
        settings.digital_gain = digital_gain
        settings.binning = binning
        self._cam.settings = settings

        self._intrinsic_calibrator.set_pattern(
            pattern_width, pattern_height, pattern_size
        )

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
        self._intrinsic_calibrator.calibrate()
        return self._get_state()

    @webutil.QueueAbstractClass.queue_abstract
    def calibration_pattern(self):
        with tempfile.TemporaryDirectory() as td:
            file = pathlib.Path(td) / "pattern.svg"
            self._intrinsic_calibrator.pattern.export_svg(file)
            with open(file, "r") as f:
                svg = f.read()
        return svg

    @webutil.QueueAbstractClass.queue_abstract
    def calibration_result(self):
        with tempfile.TemporaryFile() as f:
            self._intrinsic_calibrator.plot_cal_png(f)
            f.seek(0)
            png = f.read()
        return png

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
        self.flask_app.route("/calibration_pattern")(self._calibration_pattern)
        self.flask_app.route("/calibration_result")(self._calibration_result)
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

    def _calibration_pattern(self):
        return self.app.calibration_pattern()

    def _calibration_result(self):
        return Response(self.app.calibration_result(), mimetype="image/png")

    def _set_settings(self):
        params = request.get_json()
        d = self.app.set_settings(
            float(params["exposure_ms"]),
            int(params["gain"]),
            int(params["digital_gain"]),
            int(params["binning"]),
            int(params["pattern_width"]),
            int(params["pattern_height"]),
            float(params["pattern_size"]),
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
        camera.RpiCamera = testing_utils.RandomStarCam

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

"""Flask web application to capture images using a smartphone browser or any other browser."""

import enum
import io
import logging
import os
import pathlib
import tempfile
import threading
from typing import Any, BinaryIO, List, Optional

import cv2
import numpy as np
import scipy.spatial
from flask import (
    Flask,
    Response,
    jsonify,
    render_template,
    request,
    send_from_directory,
)
from flask_sock import ConnectionClosed, Server, Sock

from startracker import attitude_estimation, camera, image_utils, kalkam, persistent, util, webutil
from startracker.webutil import FlaskResponse


class IntrinsicCalibrator:
    cal: Optional[kalkam.IntrinsicCalibration]

    def __init__(self, dir: pathlib.Path, cal_file: pathlib.Path) -> None:
        self._dir = dir
        self._cal_file = cal_file
        self.index = 0
        self._logger = logging.getLogger("IntrinsicCalibrator")
        self.set_pattern(19, 11, 283 / 6)
        try:
            self.cal = kalkam.IntrinsicCalibration.from_json(cal_file)
        except FileNotFoundError:
            self.cal = None

    def set_pattern(self, width: int, height: int, size: float) -> None:
        """Set calibration pattern size."""
        self.pattern = kalkam.ChArUcoPattern(width, height, size)

    def put_image(self, image: np.ndarray) -> None:
        """Add an image to the calibration."""
        file = self._dir / f"image_{self.index:03d}.png"
        cv2.imwrite(str(file), image)
        self._logger.info(f"Put image {file.name}")
        self.index += 1

    def reset(self) -> None:
        """Clear all images used for the calibration."""
        self.index = 0

    def get_images(self) -> List[pathlib.Path]:
        """Get all calibration images."""
        return sorted(self._dir.glob("*.png"))

    def calibrate(self) -> None:
        """Perform calibration."""
        image_files = self.get_images()

        if len(image_files) < 2:
            raise ValueError("Calibrate requires at least 2 pictures")

        self._logger.info(f"Calibrating using {len(image_files)} images")

        self.cal = kalkam.calibration_from_image_files(image_files, self.pattern)
        self._logger.info(f"Calibration finished rms error: {self.cal.rms_error}")

        self.save()

    def plot_cal_png(self, filehandler: BinaryIO) -> None:
        """Plot calibration and return png in bytes."""
        if self.cal is None or not isinstance(self.cal, kalkam.IntrinsicCalibrationWithData):
            raise ValueError("No calibration present")

        self._logger.info("Plotting calibration")
        plot = self.cal.plot_opencv()
        self._logger.info("Done")

        success, png = cv2.imencode(".png", plot)
        if not success:
            raise ValueError("Error while encoding plot PNG")

        filehandler.write(png)

    def save(self) -> None:
        """Save calibration to the filesystem."""
        cal = self.cal
        if cal is not None:
            self._logger.info(f"Saving calibration to: {self._cal_file}")
            cal.to_json(self._cal_file)


class CaptureMode(enum.Enum):
    SINGLE = "single"
    CONTINUOUS = "continuous"
    STOP = "stop"
    DARKFRAME = "darkframe"


class App(webutil.QueueAbstractionClass):
    terminate: bool
    """Set True to terminate the application loop."""

    def __init__(self) -> None:
        super().__init__()

        self.pers = persistent.Persistent.get_instance()

        self.terminate = False

        self.image_container = webutil.ImageData()

        self._logger = logging.getLogger("App")
        self._image_cache = None
        self._intrinsic_calibrator = IntrinsicCalibrator(
            self.pers.calibration_dir, self.pers.cam_file
        )

        self._camera_job = "stop"
        self._cam = None

        self._attitude_overlay = False
        self._attitude_result = None
        self._ae = None

        self._initialize_attitude_estimator()

    def _initialize_attitude_estimator(self) -> None:
        if self._intrinsic_calibrator.cal is None:
            self._ae = None
            return
        try:
            self._ae = attitude_estimation.AttitudeEstimator(
                self._intrinsic_calibrator.cal, self.pers.star_data_dir
            )
        except FileNotFoundError:
            self._ae = None

    def _get_state(self) -> dict[str, Any]:
        quat = self._attitude_result.quat.tolist() if self._attitude_result is not None else []
        n_matches = self._attitude_result.n_matches if self._attitude_result is not None else 0

        if self._cam is not None:
            camera_settings = {
                "exposure_ms": self._cam.settings.exposure_ms,
                "analog_gain": self._cam.settings.analog_gain,
                "digital_gain": self._cam.settings.digital_gain,
                "binning": self._cam.settings.binning,
            }
        else:
            camera_settings = {}

        state = {
            "intrinsic_calibrator": {
                "index": self._intrinsic_calibrator.index,
                "pattern_width": self._intrinsic_calibrator.pattern.width,
                "pattern_height": self._intrinsic_calibrator.pattern.height,
                "pattern_size": self._intrinsic_calibrator.pattern.square_size,
            },
            "attitude": {
                "quat": quat,
                "n_matches": n_matches,
                "overlay": self._attitude_overlay,
            },
            "camera_settings": camera_settings,
        }
        return state

    @webutil.QueueAbstractionClass.queue_abstract
    def get_state(self) -> dict:
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def set_settings(
        self,
        exposure_ms: float,
        analog_gain: int,
        digital_gain: int,
        binning: int,
        pattern_width: int,
        pattern_height: int,
        pattern_size: float,
        *,
        attitude_overlay: bool = False,
    ) -> dict:
        if self._cam is None:
            raise ValueError("Camera is not initialized")

        settings = self._cam.settings
        settings.exposure_ms = exposure_ms
        settings.analog_gain = analog_gain
        settings.digital_gain = digital_gain
        settings.binning = binning
        self._logger.info(f"Setting camera settings {settings}")
        self._cam.settings = settings

        self._intrinsic_calibrator.set_pattern(pattern_width, pattern_height, pattern_size)

        self._attitude_overlay = attitude_overlay

        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def capture(self, mode: CaptureMode) -> dict:
        mode = CaptureMode(mode)
        self._logger.info(f"Capture {mode.value}")
        self._camera_job = mode.value
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def put_calibration_image(self) -> dict:
        image = self._image_cache
        if image is not None:
            self._intrinsic_calibrator.put_image(image)
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def reset_calibration(self):
        self._intrinsic_calibrator.reset()
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def calibrate(self) -> dict:
        self._intrinsic_calibrator.calibrate()
        self._initialize_attitude_estimator()
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def calibration_pattern(self) -> str:
        with tempfile.TemporaryDirectory() as td:
            file = pathlib.Path(td) / "pattern.svg"
            self._intrinsic_calibrator.pattern.export_svg(file)
            with file.open() as f:
                svg = f.read()
        return svg

    @webutil.QueueAbstractionClass.queue_abstract
    def calibration_result(self) -> bytes:
        with io.BytesIO() as buffer:
            self._intrinsic_calibrator.plot_cal_png(buffer)
            buffer.seek(0)
            png = buffer.read()
        return png

    @webutil.QueueAbstractionClass.queue_abstract
    def create_star_data(self) -> dict:
        if self._intrinsic_calibrator.cal is None:
            raise ValueError("No calibration available")
        self._logger.info("Creating star catalog")
        attitude_estimation.create_catalog(
            self._intrinsic_calibrator.cal,
            self.pers.star_data_dir,
            magnitude_threshold=5.5,
            verbose=True,
        )
        self._logger.info("Creating star catalog done")
        return self._get_state()

    def get_attitude(
        self, image: np.ndarray
    ) -> Optional[attitude_estimation.AttitudeEstimationResult]:
        if self._ae is None:
            return None
        return self._ae(image)

    def run(self) -> None:
        self._logger.info("Starting camera")
        settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)
        with self._cam:
            self._logger.info("Starting event processor")
            while not self.terminate:
                with util.max_rate(10):
                    self._tick()
        self._logger.info("Terminating event processor")

    def _tick(self) -> None:
        if self._cam is None:
            raise RuntimeError("Camera hasn't been initialized")

        if self._camera_job == "darkframe":
            self._logger.info("Record darkframe ...")
            self._cam.record_darkframe()
            self._camera_job = "stop"
        elif self._camera_job in ["single", "continuous"]:
            self._logger.info("Capture image ...")
            image = self._cam.capture()
            self._image_cache = image
            self._attitude_result = self.get_attitude(image)

            # Draw overlay if possible and enabled
            if (
                self._attitude_overlay
                and self._attitude_result is not None
                and self._intrinsic_calibrator.cal is not None
            ):
                r = scipy.spatial.transform.Rotation.from_quat(self._attitude_result.quat)
                extrinsic = np.concatenate((r.as_matrix().T, np.zeros((3, 1))), axis=-1)
                image = image_utils.draw_grid(
                    image, extrinsic, self._intrinsic_calibrator.cal, inplace=False
                )

            self.image_container.put(image)

            if self._camera_job == "single":
                self._camera_job = "stop"

        self._process_pending_calls()


class WebApp:
    flask_app: Flask
    app: Optional[App]

    def __init__(self) -> None:
        self.flask_app = Flask(__name__, template_folder="../web")
        self.sock = Sock(self.flask_app)

        self._app_loaded_event = threading.Event()

        self.flask_app.route("/")(self._index)
        self.flask_app.route("/<path:filename>")(self._serve_file)
        self.flask_app.route("/calibration_pattern")(self._calibration_pattern)
        self.flask_app.route("/calibration_result")(self._calibration_result)
        self.flask_app.post("/get_state")(self._get_state)
        self.flask_app.post("/set_settings")(self._set_settings)
        self.flask_app.post("/capture")(self._capture)
        self.flask_app.post("/put_calibration_image")(self._put_calibration_image)
        self.flask_app.post("/reset_calibration")(self._reset_calibration)
        self.flask_app.post("/calibrate")(self._calibrate)
        self.flask_app.post("/create_star_data")(self._create_star_data)
        self.sock.route("/image")(self._image)

    def _run_app(self) -> None:
        self.app = App()
        self._app_loaded_event.set()
        self.app.run()

    def _index(self) -> FlaskResponse:
        return render_template("cameraWebgui.html")

    def _serve_file(self, filename: str) -> FlaskResponse:
        return send_from_directory("../web", filename)

    def run(self) -> None:
        app_thread = threading.Thread(target=self._run_app)
        try:
            app_thread.start()
            self._app_loaded_event.wait()
            self.flask_app.run(debug=True, host="0.0.0.0", use_reloader=False)
        finally:
            logging.info("Terminated. Clean up app..")
            if self.app is not None:
                self.app.terminate = True
            app_thread.join()
            logging.info("App clean up done.")

    def _calibration_pattern(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        return self.app.calibration_pattern()

    def _calibration_result(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        return Response(self.app.calibration_result(), mimetype="image/png")

    def _get_state(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.get_state()
        return jsonify(d)

    def _set_settings(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        params = request.get_json()
        d = self.app.set_settings(
            float(params["exposure_ms"]),
            int(params["analog_gain"]),
            int(params["digital_gain"]),
            int(params["binning"]),
            int(params["pattern_width"]),
            int(params["pattern_height"]),
            float(params["pattern_size"]),
            attitude_overlay=bool(params["overlay"]),
        )
        return jsonify(d)

    def _capture(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        params = request.get_json()
        d = self.app.capture(CaptureMode(params["mode"]))
        return jsonify(d)

    def _put_calibration_image(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.put_calibration_image()
        return jsonify(d)

    def _reset_calibration(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.reset_calibration()
        return jsonify(d)

    def _calibrate(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.calibrate()
        return jsonify(d)

    def _create_star_data(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.create_star_data()
        return jsonify(d)

    def _image(self, ws: Server) -> None:
        if self.app is None:
            raise RuntimeError("App should be initialized at this point")
        try:
            while True:
                image_data = self.app.image_container.get_blocking()
                ws.send(image_data)
        except ConnectionClosed:
            pass


def main() -> None:
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
        camera.RpiCamera.simulate_exposure_time = True

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

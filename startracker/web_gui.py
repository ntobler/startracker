"""Flask web application to capture images using a smartphone browser or any other browser."""

import contextlib
import datetime
import enum
import io
import json
import logging
import os
import pathlib
import pickle
import tempfile
import threading
from collections.abc import Generator
from typing import Any, BinaryIO, Optional

import cv2
import numpy as np
import numpy.typing as npt
import scipy.spatial
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_sock import ConnectionClosed, Server, Sock

from startracker import (
    attitude_estimation,
    calibration,
    camera,
    image_utils,
    kalkam,
    persistent,
    transform,
    util,
    webutil,
)
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

    def get_images(self) -> list[pathlib.Path]:
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
        filehandler.write(png.tobytes())

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


def project_radial(xyz: np.ndarray) -> np.ndarray:
    """Return radial projection around +z of xyz vector."""
    xyz = xyz / (np.linalg.norm(xyz, axis=1, keepdims=True) + 1e-12)
    x, y, z = np.moveaxis(xyz, -1, 0)
    r = np.linalg.norm(xyz[..., :2], axis=-1)
    s = np.arctan2(r, z)
    s *= (180 / np.pi) / r
    return np.stack((x * s, y * s), axis=-1)


def to_rounded_list(x: np.ndarray, decimals: Optional[int] = None) -> list:
    """Convert numpy array in a list of rounded floats.

    JSON serialization is able to correctly truncate floats to the desired length.
    """
    x = np.array(x, dtype=np.float64, copy=False)
    if decimals is not None:
        x = x.round(decimals)
    return x.tolist()


class AxisCalibrator:
    """Class to calibrate the camera axis to the star tracker axis."""

    axis_rot: scipy.spatial.transform.Rotation
    error_rad: float

    def __init__(self, pers: persistent.Persistent) -> None:
        self._calibration_rots = []
        self.error_rad = 0

        self._file = pers.axis_rot_quat_file

        if self._file.exists():
            self.axis_rot = scipy.spatial.transform.Rotation.from_quat(np.load(self._file))
        else:
            self.axis_rot = scipy.spatial.transform.Rotation.identity()

    def put(self, quat: np.ndarray) -> None:
        """Add a quaternion to the calibration set."""
        self._calibration_rots.append(quat)

    def reset(self) -> None:
        """Clear the calibration set."""
        self._calibration_rots.clear()

    def calibrate(self) -> float:
        """Perform calibration using the current set of quaternions."""
        if len(self._calibration_rots) < 2:
            raise ValueError("Not enough data to calibrate")

        axis_vec, error_std = transform.find_common_rotation_axis(np.array(self._calibration_rots))

        # Make sure vector points in general direction of camera
        axis_vec = axis_vec if axis_vec[2] > 0 else -axis_vec
        # Create camera rotation to axis, such that camera is horizontal
        axis_rotm = kalkam.look_at_extrinsic(axis_vec, [0, 0, 0], [0, -1, 0])[:3, :3]
        self.axis_rot = scipy.spatial.transform.Rotation.from_matrix(axis_rotm)

        # Save calibrated rotation
        np.save(self._file, self.axis_rot.as_quat(canonical=False))

        return error_std

    def count(self) -> int:
        """Return the number of calibration images."""
        return len(self._calibration_rots)


class AttitudeEstimation:
    overlay: bool
    quat: np.ndarray

    def __init__(
        self,
        pers: persistent.Persistent,
        cal: kalkam.IntrinsicCalibration,
        axis_calibrator: AxisCalibrator,
    ) -> None:
        self._cal = cal
        self._axis_calibrator = axis_calibrator

        if pers.attitude_estimation_config_file.exists():
            config = attitude_estimation.AttitudeEstimatorConfig.load(
                pers.attitude_estimation_config_file
            )
        else:
            config = attitude_estimation.AttitudeEstimatorConfig()
        self._attitude_est = attitude_estimation.AttitudeEstimator(cal, config=config)

        # Get bright stars
        bright = self._attitude_est.cat_mag <= 4
        self._cat_xyz = self._attitude_est.cat_xyz[bright]
        self._cat_mags = self._attitude_est.cat_mag[bright]

        self._last_attitude_res = attitude_estimation.ERROR_ATTITUDE_RESULT
        self._calibration_rots: list[npt.NDArray[np.floating]] = []
        self._camera_frame = []

        self.overlay = False

        self.quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Database of image/object points for more accurate calibration
        self._database_folder = pers.user_data_dir / "database"
        self._database_folder.mkdir(exist_ok=True)
        self._image_database = []
        self._object_database = []

    @property
    def config(self) -> attitude_estimation.AttitudeEstimatorConfig:
        """Return the attitude estimator configuration."""
        return self._attitude_est.config

    @config.setter
    def config(self, value: attitude_estimation.AttitudeEstimatorConfig) -> None:
        self._attitude_est.config = value

    def process(self, image: npt.NDArray[np.uint8]) -> tuple[dict, npt.NDArray[np.uint8]]:
        """Process the image and return the attitude estimation result."""
        with util.TimeMeasurer() as tm1:
            kernel = np.ones((7, 7), np.uint8)
            processed_image = cv2.morphologyEx(image, cv2.MORPH_TOPHAT, kernel)
            cv2.blur(processed_image, (3, 3), dst=processed_image)

        with util.TimeMeasurer() as tm2:
            att_res = self._attitude_est(processed_image)

        obs_xy = self._attitude_est.image_xyz_to_xy(att_res.image_xyz_cam)
        cat_xy = self._attitude_est.image_xyz_to_xy(att_res.cat_xyz_cam)
        image_size = (image.shape[1], image.shape[0])

        if att_res is not attitude_estimation.ERROR_ATTITUDE_RESULT:
            self.quat = att_res.quat

            self._image_database.append(obs_xy)
            self._object_database.append(att_res.cat_xyz_cam)
            if len(self._image_database) > 100:
                self.save_database()

        with util.TimeMeasurer() as tm3:
            inverse_rotation = scipy.spatial.transform.Rotation.from_quat(self.quat).inv()

            # Rotate into camera coordinate frame
            cat_xyz = inverse_rotation.apply(self._cat_xyz)
            north_south = inverse_rotation.apply([[0, 0, 1], [0, 0, -1]])
            star_coords = att_res.image_xyz_cam

            # Merge catalog stars and detected stars (so both are displayed in the GUI)
            cat_xyz = np.concatenate((cat_xyz, att_res.cat_xyz_cam), axis=0)
            cat_mags = np.concatenate((self._cat_mags, att_res.mags), axis=0)

            # Rotate into the axis coordinate frame if the axis has been calibrated
            if self._axis_calibrator is not None:
                star_coords = self._axis_calibrator.axis_rot.apply(star_coords)
                cat_xyz = self._axis_calibrator.axis_rot.apply(cat_xyz)
                north_south = self._axis_calibrator.axis_rot.apply(north_south)

            # Calculate alignment error
            alignment_error = np.arccos(north_south[0, 2]) * (180 / np.pi)
            alignment_error = 180 - alignment_error if alignment_error > 90 else alignment_error

            # Only show hemisphere
            pos_z = cat_xyz[:, 2] > 0
            cat_xyz = cat_xyz[pos_z]
            cat_mags = cat_mags[pos_z]

            star_coords = project_radial(star_coords)
            cat_xyz = project_radial(cat_xyz)
            north_south = project_radial(north_south)

            self._update_camera_frame()

        data = {
            "star_coords": to_rounded_list(star_coords, 2),
            "alignment_error": alignment_error,
            "cat_xyz": to_rounded_list(cat_xyz, 2),
            "cat_mags": to_rounded_list(cat_mags, 2),
            "north_south": to_rounded_list(north_south, 2),
            "frame_points": self._camera_frame,
            "quat": self.quat.tolist(),
            "n_matches": att_res.n_matches,
            "obs_pix": obs_xy.tolist(),
            "cat_pix": cat_xy.tolist(),
            "image_size": image_size,
            "pre_processing_time": int(tm1.t * 1000),
            "processing_time": int(tm2.t * 1000),
            "post_processing_time": int(tm3.t * 1000),
        }

        # Draw overlay if possible and enabled
        if self.overlay:
            r = scipy.spatial.transform.Rotation.from_quat(att_res.quat)
            extrinsic = np.concatenate((r.as_matrix().T, np.zeros((3, 1))), axis=-1)
            processed_image = image_utils.draw_grid(
                processed_image, extrinsic, self._cal, inplace=False
            )
            image = processed_image

        return data, image

    def _update_camera_frame(self, segments_per_side: int = 10) -> None:
        points = calibration.get_distorted_camera_frame(self._cal, segments_per_side)
        if self._axis_calibrator is not None:
            points = self._axis_calibrator.axis_rot.apply(points)
        points = project_radial(points)
        self._camera_frame = to_rounded_list(points.T, 2)

    def save_database(self) -> None:
        """Save the fine calibration database to the filesystem."""
        if len(self._image_database) == 0:
            return
        data = {
            "image": self._image_database,
            "object": self._object_database,
        }
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        name = f"data_{now.strftime('%Y%m%d_%H%M%S')}.pkl"
        with (self._database_folder / name).open("wb") as f:
            pickle.dump(data, f)

        self._image_database.clear()
        self._object_database.clear()


class App(webutil.QueueAbstractionClass):
    terminate: bool
    """Set True to terminate the application loop."""

    def __init__(self) -> None:
        super().__init__()

        self.terminate = False

        self.stream = webutil.DataDispatcher()

        self._pers = persistent.Persistent.get_instance()

        self.image_container = webutil.DataDispatcher()

        self._logger = logging.getLogger("App")
        self._image_cache = None
        self._intrinsic_calibrator = IntrinsicCalibrator(
            self._pers.calibration_dir, self._pers.cam_file
        )

        # Load camera settings if available
        if self._pers.cam_settings_file.exists():
            settings = camera.CameraSettings.load(self._pers.cam_settings_file)
        else:
            settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)

        self._camera_job = "stop"

        self._axis_calibrator = None
        self._attitude_est = None

        self._initialize_attitude_estimator()

    def _initialize_attitude_estimator(self) -> None:
        if self._intrinsic_calibrator.cal is None:
            self._attitude_est = None
            return
        self._axis_calibrator = AxisCalibrator(self._pers)
        self._attitude_est = AttitudeEstimation(
            self._pers, self._intrinsic_calibrator.cal, self._axis_calibrator
        )

    def _get_state(self) -> dict[str, Any]:
        if self._cam is not None:
            camera_settings = {
                "exposure_ms": self._cam.settings.exposure_ms,
                "analog_gain": self._cam.settings.analog_gain,
                "digital_gain": self._cam.settings.digital_gain,
                "binning": self._cam.settings.binning,
            }
        else:
            camera_settings = {}

        if self._attitude_est is not None:
            attitude_est_config = self._attitude_est.config
            attitude = {
                "overlay": self._attitude_est.overlay,
                "min_matches": attitude_est_config.n_match,
                "pixel_tolerance": attitude_est_config.star_match_pixel_tol,
                "timeout_secs": attitude_est_config.timeout_secs,
            }
        else:
            attitude = {}

        state = {
            "intrinsic_calibrator": {
                "index": self._intrinsic_calibrator.index,
                "pattern_width": self._intrinsic_calibrator.pattern.width,
                "pattern_height": self._intrinsic_calibrator.pattern.height,
                "pattern_size": self._intrinsic_calibrator.pattern.square_size,
            },
            "attitude": attitude,
            "camera_settings": camera_settings,
        }
        return state

    @webutil.QueueAbstractionClass.queue_abstract
    def get_state(self) -> dict:
        """Geturn the application state."""
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def set_settings(self, params: dict[str, float | int | bool]) -> dict:
        """Set camera settings from a config dictionary."""
        if self._cam is None:
            raise ValueError("Camera is not initialized")

        settings = self._cam.settings
        settings.exposure_ms = float(params["exposure_ms"])
        settings.analog_gain = int(params["analog_gain"])
        settings.digital_gain = int(params["digital_gain"])
        settings.binning = int(params["binning"])
        self._logger.info(f"Setting camera settings {settings}")
        self._cam.settings = settings

        self._intrinsic_calibrator.set_pattern(
            int(params["pattern_width"]),
            int(params["pattern_height"]),
            float(params["pattern_size"]),
        )

        if self._attitude_est is not None:
            config = self._attitude_est.config
            config.star_match_pixel_tol = float(params["pixel_tolerance"])
            config.n_match = int(params["min_matches"])
            config.timeout_secs = float(params["timeout_secs"])
            self._attitude_est.config = config
            self._attitude_est.overlay = bool(params["overlay"])

        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def capture(self, mode: CaptureMode) -> dict:
        """Capture an image in the given mode."""
        mode = CaptureMode(mode)
        self._logger.info(f"Capture {mode.value}")
        self._camera_job = mode.value
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def axis_calibration(self, command: str) -> dict:
        """Execute a camera to axis alignment calibration.

        Images should be made from a different angle.

        - put: Add the current image to the set of calibration images
        - reset: Clear the calibration image set
        - calibrate: Perform calibration using the recorded set

        Args:
            command: command to execute

        Returns:
            Axis calibration state dictionary
        """
        if self._axis_calibrator is None or self._attitude_est is None:
            return {}
        if command == "put":
            self._axis_calibrator.put(self._attitude_est.quat)
        elif command == "reset":
            self._axis_calibrator.reset()
        elif command == "calibrate":
            self._axis_calibrator.calibrate()
        else:
            raise ValueError(f"unknown command '{command}'")
        return {
            "calibration_error_deg": np.degrees(self._axis_calibrator.error_rad),
            "calibration_orientations": self._axis_calibrator.count(),
        }

    @webutil.QueueAbstractionClass.queue_abstract
    def camera_calibration(self, command: str) -> dict:
        """Execute a camera calibration command.

        - put: Add the current image to the set of calibration images
        - reset: Clear the calibration image set
        - calibrate: Perform calibration using the recorded set

        Args:
            command: command to execute

        Returns:
            State dictionary
        """
        if command == "put":
            image = self._image_cache
            if image is not None:
                self._intrinsic_calibrator.put_image(image)
        elif command == "reset":
            self._intrinsic_calibrator.reset()
        elif command == "calibrate":
            self._intrinsic_calibrator.calibrate()
            self._initialize_attitude_estimator()
        else:
            raise ValueError(f"unknown command '{command}'")
        return self._get_state()

    @webutil.QueueAbstractionClass.queue_abstract
    def calibration_pattern(self) -> str:
        """Return SVG image of the calibration pattern."""
        with tempfile.TemporaryDirectory() as td:
            file = pathlib.Path(td) / "pattern.svg"
            self._intrinsic_calibrator.pattern.export_svg(file)
            with file.open() as f:
                svg = f.read()
        return svg

    @webutil.QueueAbstractionClass.queue_abstract
    def calibration_result(self) -> bytes:
        """Return PNG image of the calibration result."""
        with io.BytesIO() as buffer:
            self._intrinsic_calibrator.plot_cal_png(buffer)
            buffer.seek(0)
            png = buffer.read()
        return png

    @contextlib.contextmanager
    def _settings_saver(self) -> Generator[None, None, None]:
        yield
        self._logger.info("Safing camera settings")
        self._cam.settings.save(self._pers.cam_settings_file)
        if self._attitude_est is not None:
            self._attitude_est.config.save(self._pers.attitude_estimation_config_file)
            self._attitude_est.save_database()

    def run(self) -> None:
        """Run the backend application."""
        if self._cam is None:
            raise RuntimeError("Camera hasn't been initialized")
        self._logger.info("Starting camera")
        with self._cam, self._settings_saver():
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
            self._logger.info("Get attitude ...")

            if self._attitude_est is not None:
                data, image = self._attitude_est.process(image)
                self.stream.put(data)

            success, encoded_array = cv2.imencode(".png", image)
            if success:
                image_data = encoded_array.tobytes()
                self.image_container.put(image_data)

            self._logger.info("Capture image done")

            if self._camera_job != "continuous":
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
        self.flask_app.route("/api/calibration_pattern")(self._calibration_pattern)
        self.flask_app.route("/api/calibration_result")(self._calibration_result)
        self.flask_app.post("/api/get_state")(self._get_state)
        self.flask_app.post("/api/set_settings")(self._set_settings)
        self.flask_app.post("/api/capture")(self._capture)
        self.flask_app.post("/api/camera_calibration")(self._camera_calibration)
        self.flask_app.post("/api/axis_calibration")(self._axis_calibration)

        self.sock.route("/api/image")(self._image)
        self.sock.route("/api/stream")(self._stream)

    def _run_app(self) -> None:
        self.app = App()
        self._app_loaded_event.set()
        self.app.run()

    def _index(self) -> FlaskResponse:
        return send_from_directory("../web", "cameraWebgui.html")

    def _serve_file(self, filename: str) -> FlaskResponse:
        return send_from_directory("../web", filename)

    def run(self) -> None:
        """Start the web application."""
        app_thread = threading.Thread(target=self._run_app)
        try:
            app_thread.start()
            self._app_loaded_event.wait()
            self.flask_app.run(debug=False, host="0.0.0.0", use_reloader=False, processes=1)
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
        d = self.app.set_settings(params)
        return jsonify(d)

    def _capture(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        params = request.get_json()
        d = self.app.capture(CaptureMode(params["mode"]))
        return jsonify(d)

    def _camera_calibration(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        params = request.get_json()
        d = self.app.camera_calibration(params["command"])
        return jsonify(d)

    def _axis_calibration(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        params = request.get_json()
        d = self.app.axis_calibration(params["command"])
        return jsonify(d)

    def _stream(self, ws: Server) -> None:
        """Websocket handler for app status updates."""
        if self.app is None:
            raise RuntimeError("App should be initialized at this point")
        try:
            while True:
                d = self.app.stream.get_blocking()
                ws.send(json.dumps(d))
        except ConnectionClosed:
            pass

    def _image(self, ws: Server) -> None:
        if self.app is None:
            raise RuntimeError("App should be initialized at this point")
        try:
            while True:
                data = self.app.image_container.get_blocking()
                ws.send(data)
        except ConnectionClosed:
            pass


def main() -> None:
    """Web application main entry point."""
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
        # camera.RpiCamera = testing_utils.RandomStarCam
        camera.RpiCamera = testing_utils.AxisAlignCalibrationTestCam
        camera.RpiCamera.time_warp_factor = 1000
        camera.RpiCamera.simulate_exposure_time = True

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

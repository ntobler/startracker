"""Flask web application for the axis aligned startracker use case."""

import json
import logging
import os
import threading
from typing import List, Optional

import numpy as np
import numpy.typing as npt
import scipy.spatial.transform
from flask import Flask, jsonify, send_from_directory
from flask_sock import ConnectionClosed, Server, Sock

from startracker import (
    attitude_estimation,
    calibration,
    camera,
    kalkam,
    persistent,
    transform,
    util,
    webutil,
)
from startracker.webutil import FlaskResponse


def project_radial(xyz: np.ndarray) -> np.ndarray:
    """Return radial projection around +z of xyz vector."""
    xyz = xyz / (np.linalg.norm(xyz, axis=1, keepdims=True) + 1e-12)
    x, y, z = np.moveaxis(xyz, -1, 0)
    r = np.linalg.norm(xyz[..., :2], axis=-1)
    s = np.arctan2(r, z)
    s *= (180 / np.pi) / r
    return np.stack((x * s, y * s), axis=-1)


def to_rounded_list(x: np.ndarray, decimals: Optional[int] = None):
    """Convert numpy array in a list of rounded floats.

    JSON serialization is able to correcty truncate floats to the desired length.
    """
    x = np.array(x, dtype=np.float64, copy=False)
    if decimals is not None:
        x = x.round(decimals)
    return x.tolist()


class App(webutil.QueueAbstractionClass):
    terminate: bool
    """Set True to terminate the application loop."""

    def __init__(self) -> None:
        super().__init__()

        self.terminate = False

        self.status = webutil.DataDispatcher()

        self._pers = persistent.Persistent.get_instance()

        # Load camera settings if available
        if self._pers.cam_settings_file.exists():
            settings = camera.CameraSettings.load(self._pers.cam_settings_file)
        else:
            settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)

        # Load camera calibration and create attitude estimator
        self._cal = kalkam.IntrinsicCalibration.from_json(self._pers.cam_file)
        self._attitude_est = attitude_estimation.AttitudeEstimator(self._cal)

        # Assign default
        if self._pers.axis_rot_quat_file.exists():
            self._axis_rot = scipy.spatial.transform.Rotation.from_quat(
                np.load(self._pers.axis_rot_quat_file)
            )
        else:
            self._axis_rot = None
        self._quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Get bright stars
        bright = self._attitude_est.cat_mag <= 4
        self._cat_xyz = self._attitude_est.cat_xyz[bright]
        self._cat_mags = self._attitude_est.cat_mag[bright]

        self._last_attitude_res = attitude_estimation.ERROR_ATTITUDE_RESULT
        self._calibration_rots: List[npt.NDArray[np.floating]] = []
        self._camera_frame = []

    def _get_stars(self):
        image = self._cam.capture()

        with util.TimeMeasurer() as tm:
            att_res = self._attitude_est(image)

            self._last_attitude_res = att_res
            if self._last_attitude_res is not attitude_estimation.ERROR_ATTITUDE_RESULT:
                self._quat = att_res.quat

            inverse_rotation = scipy.spatial.transform.Rotation.from_quat(self._quat).inv()

            # Rotate into camera coordinate frame
            cat_xyz = inverse_rotation.apply(self._cat_xyz)
            north_south = inverse_rotation.apply([[0, 0, 1], [0, 0, -1]])
            star_coords = att_res.image_xyz

            # Merge catalog stars and detected stars (so both are displayed in the GUI)
            cat_xyz = np.concatenate((cat_xyz, att_res.cat_xyz), axis=0)
            cat_mags = np.concatenate((self._cat_mags, att_res.mags), axis=0)

            # Rotate into the axis coordinate frame if the axis has been calibrated
            if self._axis_rot is not None:
                star_coords = self._axis_rot.apply(star_coords)
                cat_xyz = self._axis_rot.apply(cat_xyz)
                north_south = self._axis_rot.apply(north_south)

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

        d = {
            "star_coords": to_rounded_list(star_coords, 2),
            "n_matches": att_res.n_matches,
            "alignment_error": alignment_error,
            "cat_xyz": to_rounded_list(cat_xyz, 2),
            "cat_mags": to_rounded_list(cat_mags, 2),
            "north_south": to_rounded_list(north_south, 2),
            "processing_time": int(tm.t * 1000),
            "frame_points": self._camera_frame,
        }
        return d

    def _update_camera_frame(self, segments_per_side: int = 10) -> None:
        points = calibration.get_distorted_camera_frame(self._cal, segments_per_side)
        if self._axis_rot is not None:
            points = self._axis_rot.apply(points)
        points = project_radial(points)
        self._camera_frame = to_rounded_list(points.T, 2)

    @webutil.QueueAbstractionClass.queue_abstract
    def add_to_calibration(self):
        if self._last_attitude_res is not attitude_estimation.ERROR_ATTITUDE_RESULT:
            self._calibration_rots.append(self._last_attitude_res.quat)
        return {"calibration_orientations": len(self._calibration_rots)}

    @webutil.QueueAbstractionClass.queue_abstract
    def reset_calibration(self):
        self._calibration_rots.clear()
        return {"calibration_orientations": len(self._calibration_rots)}

    @webutil.QueueAbstractionClass.queue_abstract
    def calibrate(self):
        if len(self._calibration_rots) < 2:
            raise ValueError("Not enough data to calibrate")

        axis_vec, error_std = transform.find_common_rotation_axis(np.array(self._calibration_rots))

        # Make sure vector points in general direction of camera
        axis_vec = axis_vec if axis_vec[2] > 0 else -axis_vec
        # Create camea rotation to axis, such that camera is horizontal
        axis_rotm = kalkam.look_at_extrinsic(axis_vec, [0, 0, 0], [0, -1, 0])[:3, :3]
        self._axis_rot = scipy.spatial.transform.Rotation.from_matrix(axis_rotm)

        # Save calibrated rotation
        np.save(self._pers.axis_rot_quat_file, self._axis_rot.as_quat())

        self._update_camera_frame()
        return {
            "axis_vec": axis_vec.tolist(),
            "calibration_error_deg": np.degrees(error_std),
        }

    def run(self):
        """App logic."""
        if self._cam is None:
            raise RuntimeError("Camera hasn't been initialized")
        self._update_camera_frame()
        self._logger.info("Starting camera")
        with self._cam:
            self._logger.info("Starting event processor")
            while not self.terminate:
                with util.max_rate(10):
                    self._tick()
        self._logger.info("Terminating event processor")

    def _tick(self) -> None:
        if self._cam is None:
            raise RuntimeError("Camera hasn't been initialized")
        self.status.put(self._get_stars())
        self._process_pending_calls()


class WebApp:
    flask_app: Flask
    app: Optional[App]

    def __init__(self) -> None:
        self._app_thread = threading.Thread(target=self._run_app)

        self.flask_app = Flask(__name__, template_folder="../web")
        self.sock = Sock(self.flask_app)

        self._app_loaded_event = threading.Event()

        self.flask_app.route("/")(self._index)
        self.flask_app.route("/<path:filename>")(self._serve_file)
        self.flask_app.post("/api/add_to_calibration")(self.add_to_calibration)
        self.flask_app.post("/api/reset_calibration")(self.reset_calibration)
        self.flask_app.post("/api/calibrate")(self.calibrate)

        self.sock.route("/api/state")(self._state)

    def _run_app(self) -> None:
        self.app = App()
        self._app_loaded_event.set()
        self.app.run()

    def _index(self) -> FlaskResponse:
        return send_from_directory("../web", "axisAlign.html")

    def _serve_file(self, filename: str):
        return send_from_directory("../web", filename)

    def add_to_calibration(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.add_to_calibration()
        return jsonify(d)

    def reset_calibration(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.reset_calibration()
        return jsonify(d)

    def calibrate(self) -> FlaskResponse:
        if self.app is None:
            return "Server error", 500
        d = self.app.calibrate()
        return jsonify(d)

    def _state(self, ws: Server) -> None:
        """Websocket handler for app status updates."""
        if self.app is None:
            raise RuntimeError("App should be initialized at this point")
        try:
            while True:
                d = self.app.status.get_blocking()
                ws.send(json.dumps(d))
        except ConnectionClosed:
            pass

    def run(self) -> None:
        try:
            self._app_thread.start()
            self._app_loaded_event.wait()
            self.flask_app.run(debug=False, host="0.0.0.0", use_reloader=False, processes=1)
        finally:
            logging.info("Terminated. Clean up app..")
            if self.app is not None:
                self.app.terminate = True
            self._app_thread.join()


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
        camera.RpiCamera = testing_utils.AxisAlignCalibrationTestCam
        camera.RpiCamera.time_warp_factor = 3000

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

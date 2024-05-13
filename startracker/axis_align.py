"""
Flask web application for the axis aligned startracker use case.
"""

import json
import time
import logging
import threading
import os
import queue

from flask import Flask, render_template, jsonify, send_from_directory
from flask_sock import Sock, Server, ConnectionClosed
import numpy as np
import scipy.spatial.transform

from startracker import camera
from startracker import calibration
from startracker import attitude_estimation
from startracker import persistent
from startracker import kalkam
from startracker import webutil
from startracker import transform

from typing import Optional, List


def project_radial(xyz: np.ndarray) -> np.ndarray:
    """Return radial projection around +z of xyz vector."""
    xyz = xyz / np.linalg.norm(xyz, axis=1, keepdims=True)
    x, y, z = np.moveaxis(xyz, -1, 0)
    r = np.linalg.norm(xyz[..., :2], axis=-1)
    s = np.arctan2(r, z)
    s *= (180 / np.pi) / r
    return np.stack((x * s, y * s), axis=-1)


def to_rounded_list(x: np.ndarray, decimals: Optional[int] = None):
    """
    Convert numpy array in a list of rounded floats.

    JSON serialization is able to correcty truncate floats to the desired length.
    """
    x = np.array(x, dtype=np.float64, copy=False)
    if decimals is not None:
        x = x.round(decimals)
    return x.tolist()


class TimeMeasurer:
    """Context manager to measure execution time."""

    def __init__(self):
        self.t = 0

    def __enter__(self, *args, **kwargs):
        self._t0 = time.monotonic()
        return self

    def __exit__(self, *args, **kwargs):
        self.t = time.monotonic() - self._t0


class App(webutil.QueueAbstractClass):

    terminate: bool
    """Set True to terminate the application loop."""

    def __init__(self) -> None:
        super().__init__()

        self.terminate = False

        self.status = QueueDistributingStatus()

        self._rng = np.random.default_rng(42)

        pers = persistent.Persistent.get_instance()

        self._cal = kalkam.IntrinsicCalibration.from_json(pers.cam_file)

        # TODO load real settings
        settings = camera.CameraSettings()
        self._cam = camera.RpiCamera(settings)

        self._attitude_est = attitude_estimation.AttitudeEstimator(
            self._cal, pers.star_data_dir
        )

        axis_relative_to_camera = self._rng.normal(size=(3,)) * 0.5 + [0, 0, 1]
        axis_relative_to_camera /= np.linalg.norm(axis_relative_to_camera)
        axis_rotm = kalkam.look_at_extrinsic(
            axis_relative_to_camera, [0, 0, 0], [0, -1, 0]
        )[:3, :3]
        self.axis_rot = scipy.spatial.transform.Rotation.from_matrix(axis_rotm)

        self._cat_xyz = self._attitude_est.cat_xyz.T
        self._cat_mags = self._attitude_est.cat_mag

        bright = self._cat_mags <= 4
        self._cat_xyz = self._cat_xyz[bright]
        self._cat_mags = self._cat_mags[bright]

        self._last_attitude_res = attitude_estimation.ERROR_ATTITUDE_RESULT
        self._calibration_rots: List[attitude_estimation.AttitudeEstimationResult] = []

    def _get_stars(self):
        image = self._cam.capture()

        with TimeMeasurer() as tm:
            att_res = self._attitude_est(image)

            self._last_attitude_res = att_res

            inverse_rotation = scipy.spatial.transform.Rotation.from_quat(
                att_res.quat
            ).inv()

            # Rotate into camera coordinate frame
            cat_xyz = inverse_rotation.apply(self._cat_xyz)
            north_south = inverse_rotation.apply([[0, 0, 1], [0, 0, -1]])

            # Merge catalog stars and detected stars (so both are displayed in the GUI)
            cat_xyz = np.concatenate((cat_xyz, att_res.cat_xyz), axis=0)
            cat_mags = np.concatenate((self._cat_mags, att_res.mags), axis=0)

            # Rotate into the axis coordinate frame
            star_coords = self.axis_rot.apply(att_res.image_xyz)
            cat_xyz = self.axis_rot.apply(cat_xyz)
            north_south = self.axis_rot.apply(north_south)

            # Calculate alignment error
            alignment_error = np.arccos(north_south[0, 2]) * (180 / np.pi)
            alignment_error = (
                180 - alignment_error if alignment_error > 90 else alignment_error
            )

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
        }

        return d

    def _get_camera_frame(self, segments_per_side: int = 10) -> np.ndarray:
        points = calibration.get_distorted_camera_frame(self._cal, segments_per_side)
        points = project_radial(self.axis_rot.apply(points))
        return points

    @webutil.QueueAbstractClass.queue_abstract
    def add_to_calibration(self):
        if self._last_attitude_res is not attitude_estimation.ERROR_ATTITUDE_RESULT:
            self._calibration_rots.append(self._last_attitude_res.quat)
        return {"calibration_orientations": len(self._calibration_rots)}

    def reset_calibration(self):
        self._calibration_rots.clear()
        return {"calibration_orientations": len(self._calibration_rots)}

    @webutil.QueueAbstractClass.queue_abstract
    def calibrate(self):
        if len(self._calibration_rots) < 2:
            raise ValueError("Not enough data to calibrate")

        axis_vec, error_std = transform.find_common_rotation_axis(
            np.array(self._calibration_rots)
        )

        # Make sure vector points in general direction of camera
        axis_vec = axis_vec if axis_vec[2] > 0 else -axis_vec
        # Create camea rotation to axis, such that camera is horizontal
        axis_rotm = kalkam.look_at_extrinsic(axis_vec, [0, 0, 0], [0, -1, 0])[:3, :3]
        self.axis_rot = scipy.spatial.transform.Rotation.from_matrix(axis_rotm)

        self._update_camera_frame()
        return {
            "axis_vec": axis_vec.tolist(),
            "calibration_error_deg": np.degrees(error_std),
        }

    def _update_camera_frame(self):
        self.status.update(
            {
                "frame_points": to_rounded_list(self._get_camera_frame().T, 2),
            }
        )

    def run(self):
        """App logic."""

        self._update_camera_frame()

        while not self.terminate:
            if self.status.number_of_users():
                self.status.update(self._get_stars())
            self._process_pending_calls()
            time.sleep(0.25)


class QueueDistributingStatus:
    def __init__(self) -> None:
        self._status_lock = threading.Lock()
        self._status_queues = []
        self._status = {}

    def update(self, d: dict):
        with self._status_lock:
            self._status.update(d)
            for q in self._status_queues:
                q.put(self._status)

    def register(self, q: queue.Queue):
        with self._status_lock:
            if q not in self._status_queues:
                self._status_queues.append(q)
                q.put(self._status)

    def unregister(self, q: queue.Queue):
        with self._status_lock:
            if q in self._status_queues:
                self._status_queues.remove(q)

    def number_of_users(self):
        return len(self._status_queues)


class WebApp:

    flask_app = None
    app = None

    def __init__(self):

        self._app_thread = threading.Thread(target=self._run_app)

        self.flask_app = Flask(__name__, template_folder="../web")
        self.sock = Sock(self.flask_app)

        self.flask_app.route("/")(self._index)
        self.flask_app.route("/<path:filename>")(self._serve_file)
        self.flask_app.post("/add_to_calibration")(self.add_to_calibration)
        self.flask_app.post("/calibrate")(self.calibrate)

        self.sock.route("/state")(self._state)

    def _run_app(self):
        self.app = App()
        self.app.run()

    def _index(self):
        return render_template("axisAlign.html")

    def _serve_file(self, filename: str):
        return send_from_directory("../web", filename)

    def add_to_calibration(self):
        d = self.app.add_to_calibration()
        return jsonify(d)

    def reset_calibration(self):
        d = self.app.reset_calibration()
        return jsonify(d)

    def calibrate(self):
        d = self.app.calibrate()
        return jsonify(d)

    def _state(self, ws: Server):
        """Websocket handler for app status updates."""
        q = queue.Queue()
        try:
            # Wait for app to boot
            while self.app is None:
                time.sleep(1)
            self.app.status.register(q)
            while True:
                d = q.get()
                ws.send(json.dumps(d))
        except ConnectionClosed:
            pass
        finally:
            self.app.status.unregister(q)

    def run(self):
        try:
            self._app_thread.start()
            self.flask_app.run(debug=True, host="0.0.0.0", use_reloader=False)
        finally:
            logging.info("Terminated. Clean up app..")
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

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

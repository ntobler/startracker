"""
Flask web application for the axis aligned startracker use case.
"""

import json
import time
import logging
import threading
import os
import queue

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_sock import Sock, Server, ConnectionClosed
import numpy as np
import cots_star_tracker
import scipy.spatial.transform

from . import calibration
from . import attitude_estimation
from . import persistent
from . import kalkam
from . import testing_utils


def xyz2radial(xyz: np.ndarray) -> np.ndarray:
    """Return radial projection around +z of xyz vector."""
    xyz = xyz / np.linalg.norm(xyz, axis=1, keepdims=True)
    x, y, z = np.moveaxis(xyz, -1, 0)
    r = np.linalg.norm(xyz[..., :2], axis=-1)
    s = np.arctan2(r, z)
    s *= (180 / np.pi) / r
    return np.stack((x * s, y * s), axis=-1)


class TimeMeasurer:
    """Context manager to measure execution time."""

    def __init__(self):
        self.t = 0

    def __enter__(self, *args, **kwargs):
        self._t0 = time.monotonic()
        return self

    def __exit__(self, *args, **kwargs):
        self.t = time.monotonic() - self._t0


class App:
    def __init__(self) -> None:

        self.status = QueueDistributingStatus()

        self._rng = np.random.default_rng(42)

        self._cam_cal = calibration.CameraCalibration.make_dummy()

        pers = persistent.Persistent.get_instance()

        # TODO replace with actual camera
        self._cam = testing_utils.MockStarCam()

        self._attitude_est = attitude_estimation.AttitudeEstimator(
            pers.cam_file,
            pers.star_data_dir,
        )

        axis_relative_to_camera = self._rng.normal(size=(3,)) * 0.5 + [0, 0, 1]
        axis_relative_to_camera /= np.linalg.norm(axis_relative_to_camera)
        axis_rotm = kalkam.look_at_extrinsic(
            axis_relative_to_camera, [0, 0, 0], [0, -1, 0]
        )[:3, :3]
        self.axis_rot = scipy.spatial.transform.Rotation.from_matrix(axis_rotm)

        self._cat_xyz, self._cat_mags, _ = cots_star_tracker.read_star_catalog(
            cots_star_tracker.get_star_cat_file(), 4
        )

    def _get_stars(self):
        image = self._cam()

        with TimeMeasurer() as tm:
            quat, n_matches, image_xyz, _ = self._attitude_est(image)

            inverse_rotation = scipy.spatial.transform.Rotation.from_quat(quat).inv()
            cat_xyz = inverse_rotation.apply(self._cat_xyz.T)
            north_south = inverse_rotation.apply([[0, 0, 1], [0, 0, -1]])

            star_coords = self.axis_rot.apply(image_xyz)
            cat_xyz = self.axis_rot.apply(cat_xyz)
            north_south = self.axis_rot.apply(north_south)

            alignment_error = np.arccos(north_south[0, 2]) * (180 / np.pi)
            alignment_error = (
                180 - alignment_error if alignment_error > 90 else alignment_error
            )

            pos_z = cat_xyz[:, 2] > 0
            cat_xyz = cat_xyz[pos_z]
            cat_mags = self._cat_mags[pos_z]

            star_coords = xyz2radial(star_coords)
            cat_xyz = xyz2radial(cat_xyz)
            north_south = xyz2radial(north_south)

        d = {
            "star_coords": star_coords.tolist(),
            "n_matches": n_matches,
            "alignment_error": alignment_error,
            "cat_xyz": cat_xyz.tolist(),
            "cat_mags": cat_mags.tolist(),
            "north_south": north_south.tolist(),
            "processing_time": int(tm.t * 1000),
        }

        return d

    def _get_camera_frame(self, segments_per_side: int = 10) -> np.ndarray:
        xf = np.arange(segments_per_side * 4) / segments_per_side
        points = self._cam_cal.get_frame_corners()
        points = np.array([np.interp(xf, np.arange(len(d)), d) for d in points.T]).T
        points = xyz2radial(self.axis_rot.apply(points))
        return points

    def run(self):
        """App logic."""

        fx, fy, tx, ty, width, height = self._cam_cal.intrinsic_params()
        self.status.update(
            {
                "frame_points": self._get_camera_frame().T.tolist(),
                "camera_params": {
                    "fx": fx,
                    "fy": fy,
                    "tx": tx,
                    "ty": ty,
                    "width": width,
                    "height": height,
                },
            }
        )

        while True:
            if self.status.number_of_users():
                self.status.update(self._get_stars())
            time.sleep(1)


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
        self.flask_app.post("/test")(self.test)

        self.sock.route("/state")(self._state)

    def _run_app(self):
        self.app = App()
        self.app.run()

    def _index(self):
        return render_template("axisAlign.html")

    def _serve_file(self, filename: str):
        return send_from_directory("../web", filename)

    def test(self):
        params = request.get_json()
        d = {"test": params}
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
        self._app_thread.start()
        self.flask_app.run(debug=True, host="127.0.0.1", use_reloader=False)


def main():

    if "STARTRACKER_DEBUG" in os.environ or True:
        testing_utils.TestingMaterial(use_existing=True).patch_persistent()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    webapp = WebApp()
    webapp.run()


if __name__ == "__main__":
    main()

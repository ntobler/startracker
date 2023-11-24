"""Star image based attitude estimation."""

import pathlib

import numpy as np
import scipy.spatial.transform

import cots_star_tracker

from typing import Optional, Union, Tuple


class AttitudeEstimator:
    def __init__(
        self,
        calibration_file: Union[pathlib.Path, str],
        data_dir: Union[pathlib.Path, str],
        min_star_area: int = 3,
        max_star_area: int = 100,
        n_match: int = 5,
        star_match_pixel_tol: float = 2.0,
    ):
        """
        Star based attitude estimation.

        Args:
            calibration_file: Camera calibration json file.
            data_dir: Star catalog data directory.
            min_star_area: Minimum area in pixels for a star to be detected.
            max_star_area: Maximum area in pixels for a star to be detected.
            n_match: Required minimum number of matches for a successful attitude lock.
            star_match_pixel_tol: Tolerance in pixels for a star to be recognized as match.
        """
        data_dir = pathlib.Path(data_dir)

        self._cam_file = pathlib.Path(calibration_file)
        self._min_star_area = min_star_area
        self._max_star_area = max_star_area
        self._n_match = n_match

        self._k = np.load(data_dir / "k.npy")
        self._m = np.load(data_dir / "m.npy")
        self._q = np.load(data_dir / "q.npy")
        self._x_cat = np.load(data_dir / "u.npy")
        self._indexed_star_pairs = np.load(data_dir / "indexed_star_pairs.npy")

        # Inter start angle threshold
        camera_matrix, _, _ = cots_star_tracker.read_cam_json(calibration_file)
        dx = camera_matrix[0, 0]
        self._isa_thresh = star_match_pixel_tol * (1 / dx)

    def __call__(
        self, image: np.ndarray, darkframe: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, int, np.ndarray, np.ndarray]:
        """
        Perform attitude estimation on a camera image of stars.

        Args:
            image: Grayscale camera image.
            darkframe: Grayscale darkframe image (Captured with lid on).

        Returns:
            Tuple[np.ndarray, int, np.ndarray, np.ndarray]:
                - Quaternion [x, y, z, w], transforms from the camera frame to the star frame.
                - Number of matched stars.
                - Camera frame XYZ coordinates of the image coordinates of the matches.
                - Camera frame XYZ coordinates of the catalog coordinates of the matches.
        """
        q_est, id_match, n_matches, x_obs, _ = cots_star_tracker.star_tracker(
            image,
            str(self._cam_file),
            darkframe=darkframe,
            m=self._m,
            q=self._q,
            x_cat=self._x_cat,
            k=self._k,
            indexed_star_pairs=self._indexed_star_pairs,
            graphics=False,
            min_star_area=self._min_star_area,
            max_star_area=self._max_star_area,
            isa_thresh=self._isa_thresh,
            nmatch=self._n_match,
        )

        image_xyz = x_obs.T
        cat_xyz = self._x_cat.T[id_match][:, 0]

        cat_xyz = scipy.spatial.transform.Rotation.from_quat(q_est).inv().apply(cat_xyz)

        return q_est, n_matches, image_xyz, cat_xyz


class AttitudeFilter:
    """Filter multiple attitude observations over time."""

    attitude_quat: Optional[np.ndarray]
    estimation_id: int

    def __init__(self):
        self.attitude_quat = None
        self.estimation_id = 0

    def put_quat(self, quat: np.ndarray, time: float):
        """
        Add quaternion sample to be filtered.

        Args:
            quat: Quaternion.
            time: Observation time of the quaternion in seconds
                relative to an arbitrary point in time.
        """
        # TODO implement
        # - Filter attitudes with ME-Kalman filter
        # - Or Filter vectors with Kalman filter
        # - Detect outliers/new positions and reset filter
        self.attitude_quat = quat

    def get_azimuth_elevation(self) -> Optional[Tuple[float, float]]:
        """
        Return current azimuth and elevation estimate.

        Returns:
            Optional[Tuple[float, float]]:
                Azimuth and elevation values, None if no estimate is available.
        """
        # TODO implement
        return 60.0, 40.0

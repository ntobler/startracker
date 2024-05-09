"""Star image based attitude estimation."""

import enum
import pathlib
import logging
import time
import threading
import dataclasses

import numpy as np
import scipy.spatial.transform

import cots_star_tracker

from . import camera
from . import persistent
from . import kalkam

from typing import Optional, Union, Tuple


@dataclasses.dataclass(frozen=True)
class AttitudeEstimationResult:
    quat: np.ndarray
    """Quaternion [x, y, z, w], transforms from the camera frame to the star frame."""
    n_matches: int
    """Number of matched stars that are within tolerance."""
    image_xyz: np.ndarray
    """Camera frame XYZ coordinates of the image coordinates of all matches."""
    cat_xyz: np.ndarray
    """Camera frame XYZ coordinates of the catalog coordinates of all matches."""
    star_ids: np.ndarray
    """Catalog ids of all matched stars."""
    mags: np.ndarray
    """Magnitudes of all matched stars."""


ERROR_ATTITUDE_RESULT = AttitudeEstimationResult(
    quat=np.array((1, 0, 0, 0), dtype=np.float64),
    n_matches=0,
    image_xyz=np.empty((0, 3), dtype=np.float64),
    cat_xyz=np.empty((0, 3), dtype=np.float64),
    star_ids=np.empty((0,), dtype=np.int32),
    mags=np.empty((0,), dtype=np.float64),
)


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
        self._data_dir = pathlib.Path(data_dir)
        self._cam_file = pathlib.Path(calibration_file)

        self._min_star_area = min_star_area
        self._max_star_area = max_star_area
        self._n_match = n_match

        self._k = np.load(self._data_dir / "k.npy")
        self._m = np.load(self._data_dir / "m.npy")
        self._q = np.load(self._data_dir / "q.npy")
        self.cat_xyz = np.load(self._data_dir / "u.npy")
        self._indexed_star_pairs = np.load(self._data_dir / "indexed_star_pairs.npy")
        self.cat_mag = np.load(self._data_dir / "mag.npy")

        # Inter start angle threshold
        self._intrinsic, _, self._dist_coeffs = cots_star_tracker.read_cam_json(
            calibration_file
        )
        dx = self._intrinsic[0, 0]
        self._isa_thresh = star_match_pixel_tol * (1 / dx)

    def image_xyz_to_xy(self, image_xyz):
        image_xy = (self._intrinsic @ image_xyz.T).T
        image_xy = image_xy[..., :2] / image_xy[..., 2:]
        image_xy = kalkam.PointUndistorter(
            kalkam.IntrinsicCalibration(
                self._intrinsic, self._dist_coeffs
            )  # TODO Ewwwwww fix this
        ).distort(image_xy)
        return image_xy

    def __call__(
        self, image: np.ndarray, darkframe: Optional[np.ndarray] = None
    ) -> AttitudeEstimationResult:
        """
        Perform attitude estimation on a camera image of stars.

        Args:
            image: Grayscale camera image.
            darkframe: Grayscale darkframe image (Captured with lid on).

        Returns:
            AttitudeEstimationResult: attitude estimation result object.
        """
        try:
            q_est, id_match, n_matches, x_obs, _ = cots_star_tracker.star_tracker(
                image,
                str(self._cam_file),
                darkframe=darkframe,
                m=self._m,
                q=self._q,
                x_cat=self.cat_xyz,
                k=self._k,
                indexed_star_pairs=self._indexed_star_pairs,
                graphics=False,
                min_star_area=self._min_star_area,
                max_star_area=self._max_star_area,
                isa_thresh=self._isa_thresh,
                nmatch=self._n_match,
            )
        except cots_star_tracker.StartrackerError:
            return ERROR_ATTITUDE_RESULT

        id_match = id_match[:, 0]
        image_xyz = x_obs.T
        cat_xyz = self.cat_xyz.T[id_match]
        star_mags = self.cat_mag[id_match]
        cat_xyz = scipy.spatial.transform.Rotation.from_quat(q_est).inv().apply(cat_xyz)

        return AttitudeEstimationResult(
            q_est, n_matches, image_xyz, cat_xyz, id_match, star_mags
        )


class AttitudeFilter:
    """Filter multiple attitude observations over time."""

    attitude_quat: Optional[np.ndarray]
    estimation_id: int

    def __init__(self):
        self.attitude_quat = None
        self.estimation_id = 0

    def put_quat(self, quat: np.ndarray, confidence, time: float):
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


class AttitudeEstimationModeEnum(enum.Enum):
    IDLE = 0
    RUNNING = 1
    SINGLE = 2
    RECORD_DARKFRAME = 3
    FAULT = 4


class ImageAcquisitioner:
    mode: AttitudeEstimationModeEnum
    """Current mode"""
    positions: Optional[np.ndarray]
    """xy positions of recognized stars"""
    n_matches: int
    """Number of currently matched stars."""

    def __init__(self, attitude_filter: AttitudeFilter):
        self._logger = logging.getLogger("ImageAcquisitioner")
        self.mode = AttitudeEstimationModeEnum.IDLE
        self.n_matches = 0
        self.positions = None
        self._attitude_filter = attitude_filter

        self._cam_settings = camera.CameraSettings(
            exposure_ms=250,
            analog_gain=4,
            digital_gain=4,
        )

        pers = persistent.Persistent.get_instance()

        if not pers.cam_file.exists():
            logging.critical("Missing camera calibration")
            self._attitude_estimator = None
            self.mode = AttitudeEstimationModeEnum.FAULT
        elif not pers.star_data_dir.exists():
            logging.critical("Missing star data catalog")
            self._attitude_estimator = None
            self.mode = AttitudeEstimationModeEnum.FAULT
        else:
            self._attitude_estimator = AttitudeEstimator(
                pers.cam_file,
                pers.star_data_dir,
            )

        self._dark_frame_file = pers.dark_frame_file
        try:
            self._dark_frame = np.load(self._dark_frame_file)
        except OSError:
            self._dark_frame = None

    def start_thread(self):
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def run(self):
        cam = camera.RpiCamera(self._cam_settings)
        with cam:
            while True:
                if self.mode == AttitudeEstimationModeEnum.RUNNING:
                    if self._attitude_estimator is None:
                        self.mode = AttitudeEstimationModeEnum.FAULT
                        continue
                    image = cam.capture()
                    self._logger.info(f"image mean: {image.mean()}")

                    att_res = self._attitude_estimator(image)
                    confidence = self._confidence_function(att_res.n_matches)
                    self._attitude_filter.put_quat(
                        att_res.quat, confidence, time.monotonic()
                    )
                    self.n_matches = att_res.n_matches
                    image_xyz = att_res.image_xyz

                    self.positions = (
                        self._attitude_estimator.image_xyz_to_xy(image_xyz)
                        / image.shape[1]
                    )

                elif self.mode == AttitudeEstimationModeEnum.IDLE:
                    time.sleep(0.1)
                elif self.mode == AttitudeEstimationModeEnum.RECORD_DARKFRAME:
                    cam.record_darkframe()
                    self.mode = AttitudeEstimationModeEnum.IDLE
                elif self.mode == AttitudeEstimationModeEnum.FAULT:
                    time.sleep(0.1)
                else:
                    time.sleep(0.1)

    @staticmethod
    def _confidence_function(n_matches: int) -> float:
        return float(np.clip((n_matches - 5) / 5, 0, 1))

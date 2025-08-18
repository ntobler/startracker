"""Star image based attitude estimation."""

import dataclasses
import enum
import functools
import logging
import math
import threading
import time
from typing import Callable, Literal, Optional

import numpy as np
import ruststartracker
import scipy.spatial.transform
from typing_extensions import Self

from startracker import camera, kalkam, persistent, util


@dataclasses.dataclass(frozen=True)
class AttitudeEstimationResult:
    quat: np.ndarray
    """Quaternion [x, y, z, w], transforms from the camera frame to the star frame."""
    n_matches: int
    """Number of matched stars that are within tolerance."""
    image_xyz_cam: np.ndarray
    """Camera frame XYZ coordinates of the image coordinates of all matches."""
    cat_xyz: np.ndarray
    """Camera frame XYZ coordinates of the catalog coordinates of all matches."""
    cat_xyz_cam: np.ndarray
    """Camera frame XYZ coordinates of the catalog coordinates of all matches."""
    star_ids: np.ndarray
    """Catalog ids of all matched stars."""
    obs_indices: np.ndarray
    """Indices of observations that are matched."""
    mags: np.ndarray
    """Magnitudes of all matched stars."""


ERROR_ATTITUDE_RESULT = AttitudeEstimationResult(
    quat=np.array((1, 0, 0, 0), dtype=np.float64),
    n_matches=0,
    image_xyz_cam=np.empty((0, 3), dtype=np.float64),
    cat_xyz=np.empty((0, 3), dtype=np.float64),
    cat_xyz_cam=np.empty((0, 3), dtype=np.float64),
    star_ids=np.empty((0,), dtype=np.int32),
    obs_indices=np.empty((0,), dtype=np.int32),
    mags=np.empty((0,), dtype=np.float64),
)


@dataclasses.dataclass
class AttitudeEstimatorConfig(util.PickleDataclass):
    max_candidates: int = 100
    """Maximum number of star candidates extracted from an image (sorted by brightness)."""
    min_star_area: int = 3
    """Minimum area in pixels for a star to be detected."""
    max_star_area: int = 100
    """Maximum area in pixels for a star to be detected"""
    n_match: int = 7
    """Number of required star matches for a successful attitude estimation."""
    star_match_pixel_tol: float = 2.0
    """Tolerance in pixels for a star to be recognized as match."""
    timeout_secs: float = 0.2
    """Timeout for computation in seconds."""
    epoch: Optional[float] = None
    """Astronomical epoch in years e.g. 2024.7 to calculate up-to-date star catalog.
    If None, the current date is used. Set to a fixed value to ensure reproducibility."""
    max_lookup_magnitude: float = 5.5
    """Maximum magnitude of stars used in the triangulation. Reducing this number means only
    bright stars are used for triangulation. This results in faster lookup performance."""

    def copy(self) -> Self:
        """Create a copy of this instance."""
        return dataclasses.replace(self)


FilterFunc = Callable[[np.ndarray, np.ndarray], np.ndarray]


@functools.lru_cache(1)
def get_catalog() -> ruststartracker.StarCatalog:
    """Get cached star catalog."""
    return ruststartracker.StarCatalog.from_hipparcos(max_magnitude=8.0)


class NoUpdate(enum.Enum):
    NO_UPDATE = 0


class AttitudeEstimator:
    _cat_filter: Optional[FilterFunc] = None
    """Filter function for star catalog (cat_xyz, cat_mag) -> boolean_mask."""

    def __init__(
        self,
        cal: kalkam.IntrinsicCalibration,
        config: Optional[AttitudeEstimatorConfig] = None,
    ):
        """Star based attitude estimation.

        Args:
            cal: Camera calibration instance.
            config: Attitude estimation configuration to be used
        """
        if config is None:
            config = AttitudeEstimatorConfig()

        self._cal = cal

        self._config = config

        catalog = get_catalog()
        self.all_cat_xyz = catalog.normalized_positions(epoch=config.epoch)
        self.all_cat_mag = catalog.magnitude

        self._initialize_backend()

    def _initialize_backend(self) -> None:
        camera_params = ruststartracker.CameraParameters(
            camera_matrix=self._cal.intrinsic,
            cam_resolution=self._cal.image_size,
            dist_coefs=self._cal.dist_coeffs,
        )

        # Approximate angle tolerance from pixel tolerance
        fx = float(self._cal.intrinsic[0, 0])
        isa_angle_tol = math.atan(self._config.star_match_pixel_tol / fx)

        # Apply filter to catalog if present
        if self._cat_filter is not None:
            mask = self._cat_filter(self.all_cat_xyz, self.all_cat_mag)
            self.cat_xyz = self.all_cat_xyz[mask]
            self.cat_mag = self.all_cat_mag[mask]
        else:
            self.cat_xyz = self.all_cat_xyz
            self.cat_mag = self.all_cat_mag

        self._backend = ruststartracker.StarTracker(
            self.cat_xyz,
            self.cat_mag,
            camera_params,
            max_lookup_magnitude=self._config.max_lookup_magnitude,
            inter_star_angle_tolerance=isa_angle_tol,
            n_minimum_matches=self._config.n_match,
            timeout_secs=self._config.timeout_secs,
        )

    def update(
        self,
        cal: kalkam.IntrinsicCalibration | Literal[NoUpdate.NO_UPDATE] = NoUpdate.NO_UPDATE,
        cat_filter: Optional[FilterFunc] | Literal[NoUpdate.NO_UPDATE] = NoUpdate.NO_UPDATE,
        config: AttitudeEstimatorConfig | Literal[NoUpdate.NO_UPDATE] = NoUpdate.NO_UPDATE,
    ) -> None:
        """Update parameters that require reinitialization."""
        reinitialization_needed = False
        if cal is not NoUpdate.NO_UPDATE:
            self._cal = cal
            reinitialization_needed = True
        if cat_filter is not NoUpdate.NO_UPDATE:
            self._cat_filter = cat_filter
            reinitialization_needed = True
        if config is not NoUpdate.NO_UPDATE:
            config_requireds_reinitialization = [
                "n_match",
                "timeout_secs",
                "star_match_pixel_tol",
            ]
            old_config_dict = self._config.to_dict()
            new_config_dict = config.to_dict()
            self._config = config
            if any(
                old_config_dict[k] != new_config_dict[k] for k in config_requireds_reinitialization
            ):
                reinitialization_needed = True

        if reinitialization_needed:
            self._initialize_backend()

    @property
    def cal(self) -> kalkam.IntrinsicCalibration:
        """Return the camera calibration."""
        return self._cal

    @cal.setter
    def cal(self, x: kalkam.IntrinsicCalibration):
        self._cal = x
        self._initialize_backend()

    @property
    def cat_filter(self) -> Optional[FilterFunc]:
        """Return the catalog filter used."""
        return self._cat_filter

    @cat_filter.setter
    def cat_filter(self, func: Optional[FilterFunc]):
        self._cat_filter = func
        self._initialize_backend()

    @property
    def config(self) -> AttitudeEstimatorConfig:
        """Return copy of the configuration."""
        return self._config.copy()

    @config.setter
    def config(self, config: AttitudeEstimatorConfig):
        config_requireds_reinitialization = [
            "n_match",
            "timeout_secs",
            "star_match_pixel_tol",
        ]
        old_config_dict = self._config.to_dict()
        new_config_dict = config.to_dict()
        self._config = config
        if any(old_config_dict[k] != new_config_dict[k] for k in config_requireds_reinitialization):
            self._initialize_backend()

    def image_xyz_to_xy(self, image_xyz_cam: np.ndarray) -> np.ndarray:
        """Convert camera frame XYZ coordinates to image pixel coordinates."""
        image_xy = (self._cal.intrinsic @ image_xyz_cam.T).T
        image_xy = image_xy[..., :2] / image_xy[..., 2:]
        image_xy = kalkam.PointUndistorter(self._cal).distort(image_xy)
        return image_xy

    def get_star_positions(
        self, image: np.ndarray, darkframe: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Get image positions of stars found in the given image."""
        return self._backend.get_centroids(
            image,
            darkframe=darkframe,
            n_candidates=self._config.max_candidates,
            min_star_area=self._config.min_star_area,
            max_star_area=self._config.max_star_area,
        )

    def estimate_from_image_positions(self, image_xy: np.ndarray) -> AttitudeEstimationResult:
        """Perform attitude estimation on a vector of star image positions .

        Args:
            image_xy: Star image positions, shape=[n, 2]

        Returns:
            AttitudeEstimationResult: attitude estimation result object.
        """
        try:
            result = self._backend.process_image_coordiantes(image_xy)
        except ruststartracker.StarTrackerError:
            return ERROR_ATTITUDE_RESULT
        return self._post_process_result(result)

    def __call__(
        self, image: np.ndarray, darkframe: Optional[np.ndarray] = None
    ) -> AttitudeEstimationResult:
        """Perform attitude estimation on a camera image of stars.

        Args:
            image: Grayscale camera image.
            darkframe: Grayscale darkframe image (Captured with lid on).

        Returns:
            AttitudeEstimationResult: attitude estimation result object.
        """
        try:
            result = self._backend.process_image(
                image,
                darkframe=darkframe,
                n_candidates=self._config.max_candidates,
                min_star_area=self._config.min_star_area,
                max_star_area=self._config.max_star_area,
            )
        except ruststartracker.StarTrackerError:
            return ERROR_ATTITUDE_RESULT
        return self._post_process_result(result)

    def _post_process_result(
        self, result: ruststartracker.StarTrackerResult
    ) -> AttitudeEstimationResult:
        q_est = result.quat
        id_match = result.match_ids
        n_matches = result.n_matches

        image_xyz = result.mached_obs_x
        cat_xyz = self.cat_xyz[id_match]
        star_mags = self.cat_mag[id_match]

        # Convert vectors into camera frame
        rot = scipy.spatial.transform.Rotation.from_quat(q_est).inv()
        cat_xyz_cam = rot.apply(cat_xyz)

        return AttitudeEstimationResult(
            q_est,
            n_matches,
            image_xyz,
            cat_xyz,
            cat_xyz_cam,
            id_match,
            result.obs_indices,
            star_mags,
        )

    def calculate_statistics(self, res: AttitudeEstimationResult):
        """Calculate true positive and false negative magnitudes."""
        rot = scipy.spatial.transform.Rotation.from_quat(res.quat)
        points = rot.inv().apply(self.cat_xyz)

        z = points[..., 2]

        pp = kalkam.PointProjector(self._cal, np.eye(4))
        width, height = self._cal.image_size

        x, y = pp.obj2pix(points, axis=-1).T

        condition = (z > self._cal.cos_phi(1.1)) * (x < width) * (x >= 0) * (y < height) * (y >= 0)
        in_frame_star_ids = set(np.where(condition)[0])
        detected_stars_ids = set(res.star_ids)

        false_negatives = list(in_frame_star_ids - detected_stars_ids)
        true_positives = list(detected_stars_ids & in_frame_star_ids)

        true_positive_mags = self.cat_mag[true_positives]
        false_negative_mags = self.cat_mag[false_negatives]

        if False:
            import matplotlib.pyplot as plt

            fig, axs = plt.subplots(2)
            axs[0].plot(x[true_positives], y[true_positives], "o")
            axs[0].plot(x[false_negatives], y[false_negatives], "o")

            min_mag = 3
            max_mag = 5.5
            bins = np.linspace(min_mag, max_mag, int((max_mag - min_mag) * 10) + 1)

            axs[1].hist(true_positive_mags, bins=bins, alpha=0.5, label="true positive")
            axs[1].hist(false_negative_mags, bins=bins, alpha=0.5, label="false negative")
            plt.show()

        return true_positive_mags, false_negative_mags


class AttitudeFilter:
    """Filter multiple attitude observations over time."""

    attitude_quat: Optional[np.ndarray]
    estimation_id: int

    def __init__(self):
        self.attitude_quat = None
        self.estimation_id = 0

    def put_quat(self, quat: np.ndarray, confidence: float, time: float):
        """Add quaternion sample to be filtered.

        Args:
            quat: Quaternion.
            confidence: Confidence weight
            time: Observation time of the quaternion in seconds
                relative to an arbitrary point in time.
        """
        # TODO implement
        # - Filter attitudes with ME-Kalman filter
        # - Or Filter vectors with Kalman filter
        # - Detect outliers/new positions and reset filter
        _ = confidence
        _ = time
        self.attitude_quat = quat

    def get_azimuth_elevation(self) -> Optional[tuple[float, float]]:
        """Return current azimuth and elevation estimate.

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
        self._thread: Optional[threading.Thread] = None
        self._terminate = False

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
        else:
            self._attitude_estimator = AttitudeEstimator(
                kalkam.IntrinsicCalibration.from_json(pers.cam_file),
            )

        self._dark_frame_file = pers.dark_frame_file
        try:
            self._dark_frame = np.load(self._dark_frame_file)
        except OSError:
            self._dark_frame = None

    def __enter__(self) -> Self:
        """Start the image acquisition thread."""
        if self._thread is not None:
            raise RuntimeError("Thread already started")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._thread is None:
            raise RuntimeError("Thread already stopped")
        self._terminate = True
        self._thread.join()

    def _run(self):
        cam = camera.RpiCamera(self._cam_settings)
        with cam:
            while not self._terminate:
                if self.mode == AttitudeEstimationModeEnum.RUNNING:
                    if self._attitude_estimator is None:
                        self.mode = AttitudeEstimationModeEnum.FAULT
                        continue
                    image = cam.capture()
                    self._logger.info(f"image mean: {image.mean()}")

                    att_res = self._attitude_estimator(image)
                    confidence = self._confidence_function(att_res.n_matches)
                    self._attitude_filter.put_quat(att_res.quat, confidence, time.monotonic())
                    self.n_matches = att_res.n_matches
                    image_xyz = att_res.image_xyz_cam

                    self.positions = (
                        self._attitude_estimator.image_xyz_to_xy(image_xyz) / image.shape[1]
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

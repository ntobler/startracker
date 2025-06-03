"""Create a camera calibration from stars observations only."""

import dataclasses
import time
from typing import Optional

import numpy as np
import numpy.typing as npt
import ruststartracker
import scipy.spatial

from startracker import (
    attitude_estimation,
    camera,
    const,
    kalkam,
    libstartracker,
    starcamcal,
    testing_utils,
)


class MovementRegisterer:
    """Get star movement from a sequence of star images."""

    class Candidate:
        """Data structure representing a moving star candidate."""

        positions: list[np.ndarray]
        thresholds: list[float]
        times: list[float]
        undetected_spree: int

        def __init__(self, position: np.ndarray, t: float):
            self.positions = [position]
            self.thresholds = [50]
            self.times = [t]
            self.undetected_spree = 0

    PERSIST_THRESHOLD = 10
    MAX_FLOAT = np.finfo(np.float64).max

    candidates: list[Candidate] = []

    def __init__(self):
        self.candidates = []

    def reset(self):
        """Reset tracked stars."""
        self.candidates = []

    def __call__(self, centroids: np.ndarray, t: float) -> None:
        """Add star position to be registered.

        Args:
            centroids: Image positions of stars. shape=[n, 2]
            t: time in seconds. First call has time=0
        """
        most_recent_positions = np.array(
            [c.positions[-1] for c in self.candidates], dtype=np.float64
        ).reshape((-1, 2))

        distance_thesholds = np.array([c.thresholds[-1] for c in self.candidates])

        unmatched_existing = np.ones(len(self.candidates))
        unmatched_new = np.ones(len(centroids))

        distances = np.linalg.norm(centroids[:, None] - most_recent_positions[None, :], axis=-1)
        for _ in range(np.min(distances.shape)):
            new_index, existing_index = np.unravel_index(np.argmin(distances), distances.shape)
            distance = distances[new_index, existing_index]

            threshold = distance_thesholds[existing_index]
            if distance < threshold:
                # match found

                c = self.candidates[existing_index]
                c.positions.append(centroids[new_index].copy())
                c.thresholds.append(threshold)
                c.times.append(t)

                distances[new_index, :] = self.MAX_FLOAT
                distances[:, existing_index] = self.MAX_FLOAT
                unmatched_new[new_index] = False
                unmatched_existing[existing_index] = False

            else:
                # No more matches found
                break

        for existing_index in np.where(unmatched_existing)[0]:
            c = self.candidates[int(existing_index)]
            c.undetected_spree += 1

        self.candidates = [
            c for c in self.candidates if c.undetected_spree < self.PERSIST_THRESHOLD
        ]

        for new_index in np.where(unmatched_new)[0]:
            c = MovementRegisterer.Candidate(centroids[new_index].copy(), t)
            self.candidates.append(c)

    def get_results(
        self,
        residual_threshold: float = 0.1,
        min_observations: int = 11,
        *,
        plot: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get Positions of stars and their movement direction.

        Args:
            residual_threshold: Mean squard error threshold for polinomial fit to star trails.
            min_observations: Minimum number of observations in a star trail for it to be
                considered.
            plot: Plot results with matplotlib.

        Returns:
            star image positions shape=[n, 2],
            star movement vector in pixels per second, shape=[n, 2],
            mean squared error of the star trail polinomial fit
        """
        if plot:
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots()
            ax.set_aspect(1)

        good_candidates = [c for c in self.candidates if len(c.positions) >= min_observations]

        stars_xy = []
        stars_dxy = []
        mses = []

        for c in good_candidates:
            t = np.array(c.times, np.float64)
            t -= np.mean(t)
            positions = np.array(c.positions)

            t_poly = np.linspace(t.min(), t.max(), 20)
            p, sse, _, _, _ = np.polyfit(t, positions, 2, full=True)
            mse = sse / len(t)

            # Discard fits with high error
            if np.any(mse > residual_threshold):
                continue

            xy = [np.polyval(pp, 0) for pp in p.T]
            dx_dy = [np.polyval(pp[:-1], 0) for pp in p.T]

            if plot:
                positions_poly = [np.polyval(pp, t_poly) for pp in p.T]

                x, y = positions.T
                a = ax.plot(x, y, ".-", color="red")

                x, y = positions_poly
                ax.plot(x, y, "-", color=a[0].get_color())

                x, y = xy
                dx, dy = np.array(dx_dy) * t.max()
                ax.arrow(x, y, dx, dy, color="black", lw=2)

            stars_xy.append(xy)
            stars_dxy.append(dx_dy)
            mses.append(mse)

        if plot:
            plt.show()

        stars_xy = np.array(stars_xy)
        stars_dxy = np.array(stars_dxy)
        mses = np.array(mses)

        return stars_xy, stars_dxy, mses


def star_gradient_calibration(
    stars_xy_np: np.ndarray,
    stars_dxy_np: np.ndarray,
    width: int,
    height: int,
    intrinsic0: Optional[np.ndarray] = None,
    theta0: Optional[float] = None,
    epsilon0: Optional[float] = None,
    *,
    verbose: bool = False,
) -> tuple[np.ndarray, float, float, float, float]:
    """Estimate intrinsic and partial extrinsic from star image positions and their movement.

    Args:
        stars_xy_np: Image coordinates of observed stars.
        stars_dxy_np: Temporal gradient of image coordinates of observed stars (per second).
        width: Image width in pixels.
        height: Image height in pixels.
        intrinsic0: Optional initial intrinsic matrix
        theta0: Optional declination angle in rads
        epsilon0: Optional roll angle of the camera about its axis in rads.
        verbose: Print messages
        plot: Plot using matplotlib

    Returns:
        - Intrinsic camera matrix estimate,
        - First OpenCV distortion coefficient (r^2)
        - Theta, declination angle in rads in the equatorial coordinate system
            (North pole: pi/2, South pole: -pi/2)
        - Epsilon, roll angle of the camera about it's axis
        - Estimation error
    """
    if intrinsic0 is None:
        fxy = np.maximum(width, height)
        intrinsic0s = [
            np.array(
                (
                    (f, 0, width / 2),
                    (0, f, height / 2),
                    (0, 0, 1),
                ),
                dtype=np.float32,
            )
            for f in np.logspace(np.log10(0.5), np.log10(4), 4) * fxy
        ]
    else:
        intrinsic0s = [theta0]

    intrinsic0s = np.array(intrinsic0s)

    theta0s = [np.pi / 4, -np.pi / 4] if theta0 is None else np.array([theta0])
    epsilon0s = (
        [3 * np.pi / 4, np.pi / 4, -np.pi / 4, 3 * -np.pi / 4]
        if epsilon0 is None
        else np.array([epsilon0])
    )

    # Build combinations of theta and epsilon starting conditions
    theta0s, epsilon0s, intrinsic_indices = np.meshgrid(
        theta0s, epsilon0s, np.arange(len(intrinsic0s))
    )
    theta0s = theta0s.ravel()
    epsilon0s = epsilon0s.ravel()
    intrinsic0s = intrinsic0s[intrinsic_indices.ravel()]

    if verbose:
        print(f"Finding best initial states with {len(stars_xy_np)} stars")

    prior_errors = []
    for theta0, epsilon0, intrinsic0 in zip(theta0s, epsilon0s, intrinsic0s, strict=True):
        # Build state vector
        res = libstartracker.stargradcal_objective_function(
            stars_xy_np.astype(np.float32),
            (stars_dxy_np / const.EARTH_ANGULAR_VELOCITY).astype(np.float32),
            intrinsic0.astype(np.float64),
            0.0,
            float(theta0),
            float(epsilon0),
        )
        residuals, jac = res
        error = np.square(residuals * const.EARTH_ANGULAR_VELOCITY).mean().item()

        prior_errors.append(error)
    prior_errors = np.array(prior_errors)

    best_index = np.argmin(prior_errors)

    if verbose:
        print("Starting optimization")

    theta0 = float(theta0s[best_index])
    epsilon0 = float(epsilon0s[best_index])
    intrinsic0 = intrinsic0s[best_index]

    assert intrinsic0 is not None

    res = libstartracker.stargradcal_calibrate(
        stars_xy_np.astype(np.float32),
        (stars_dxy_np / const.EARTH_ANGULAR_VELOCITY).astype(np.float32),
        intrinsic0.astype(np.float64),
        0.0,
        theta0,
        epsilon0,
    )
    intrinsic, dist_coef, theta, epsilon, residuals = res
    intrinsic = np.copy(intrinsic.T)
    residuals *= const.EARTH_ANGULAR_VELOCITY
    error = np.square(residuals).mean().item()

    epsilon = ((epsilon + np.pi) % (2 * np.pi)) - np.pi
    theta = ((theta + np.pi) % (2 * np.pi)) - np.pi

    return (intrinsic, dist_coef, theta, epsilon, error)


@dataclasses.dataclass
class StarCalibratorConfig:
    attitude_estimator_config: attitude_estimation.AttitudeEstimatorConfig
    """Initial configuration of the attitude estimator."""
    min_time_delta: float = 9
    """Minimum difference of capture times between two images."""

    movement_register_mse_threshold: float = 1.0
    """Maximum allowed mean squared error for the star movement regression."""
    movement_register_min_points: int = 20
    """Minimum amount of star movement points required to proceed with the calibration."""
    max_initial_calibration_error: float = 1.0
    """Maximum allowed error the the initial camaera calibration using star movement."""

    recalibration_percentile: float = 99.0
    """Percentile of data used for second calibration."""
    interval: int = 10
    """Skip calibrations for given number of received samples"""


class StarCalibrator:
    """Calibrate a camera from footage stream of stars.

    The camera has to be stationary and aimed at the night sky. Ideally pointed to an equatorial
    region, as the algorithm favors fast moving stars.
    """

    star_match_fraction: list[float]
    """Fraction of observations that match stars in the catalog."""
    rms_errors: list[float]
    """Root mean squared errors of the full calibration"""
    max_errors: list[float]
    """Max errors of the full calibration"""
    median_errors: list[float]
    """Max errors of the full calibration"""
    times: list[float]
    """Times of given star capture samples."""

    intrinsics: list[np.ndarray]
    dist_coeffs: list[np.ndarray]
    cal: kalkam.IntrinsicCalibration

    _stars_xy: list[np.ndarray]
    """List of star observations"""

    _first_estimate_acquired: bool

    def __init__(self, config: StarCalibratorConfig, width: int, height: int) -> None:
        self._config = config

        catalog = ruststartracker.StarCatalog(max_magnitude=5.5)
        self._cat_xyz = catalog.normalized_positions(epoch=2024.7)

        self.times = []
        self.star_match_fraction = []
        self.rms_errors = []
        self.max_errors = []
        self.median_errors = []
        self.intrinsics = []
        self.dist_coeffs = []

        self._stars_xy = []

        # These values will be recalculated if the calibration changes
        self._image_xy_list = []
        self._cat_xyz_list = []
        self._match_fractions = []
        self._quat_list = []

        # Initialize with dummy values
        cal = kalkam.IntrinsicCalibration(np.eye(3), np.zeros(5), (width, height))
        self.ae = attitude_estimation.AttitudeEstimator(
            cal, config=config.attitude_estimator_config
        )

        # Initialize movement Registerer for initial ultra coarse calibration
        self._movement_registerer = MovementRegisterer()

        self._first_estimate_acquired = False

    def put_image(self, image: npt.NDArray[np.uint8], t: float) -> None:
        """Add an image to the calibration."""
        # Only accept samples if enough time has passed between observations
        if self.times and t < self.times[-1] + self._config.min_time_delta:
            # Not enough time passed for a new sample
            return

        stars_xy = self.ae.get_star_positions(image)

        self._stars_xy.append(stars_xy)
        self.times.append(t)
        self.star_match_fraction.append(
            0 if not self.star_match_fraction else self.star_match_fraction[-1]
        )
        self.rms_errors.append(np.nan if not self.rms_errors else self.rms_errors[-1])
        self.max_errors.append(np.nan if not self.max_errors else self.max_errors[-1])
        self.median_errors.append(np.nan if not self.median_errors else self.median_errors[-1])
        self.intrinsics.append(
            np.full((3, 3), np.nan) if not self.intrinsics else self.intrinsics[-1].copy()
        )
        self.dist_coeffs.append(
            np.full((5,), np.nan) if not self.dist_coeffs else self.dist_coeffs[-1].copy()
        )

        # Track movement for star gradient calibration
        if not self._first_estimate_acquired:
            self._movement_registerer(stars_xy, t)

        # Only continue in regular intervals
        if len(self.times) % self._config.interval != 0:
            return

        if not self._first_estimate_acquired:
            # Initially, a basic calibration is needed to feed the star-based attitude estimator
            # with calibration parameters. These parameters are estimated from the movement of stars
            # across the sky.

            # Get star positions and their gradient in image coordinates.
            stars_xy, stars_dxy, mses = self._movement_registerer.get_results()

            # Check if there is enough points with good quality fits
            if (
                np.count_nonzero(mses < self._config.movement_register_mse_threshold)
                < self._config.movement_register_min_points
            ):
                return

            # Fit calibration to star movement
            height, width = image.shape
            intrinsic, dist_coef, theta, _, error = star_gradient_calibration(
                stars_xy, stars_dxy, width, height
            )

            # Store parameters
            self.intrinsics[-1][:] = intrinsic
            self.dist_coeffs[-1][0] = dist_coef
            self.dist_coeffs[-1][1:] = 0.0

            # If the calibration was bad, do it over next time
            if error > self._config.max_initial_calibration_error:
                return

            # Mark initial calibration as successful
            self._first_estimate_acquired = True

            cal = kalkam.IntrinsicCalibration(
                intrinsic=intrinsic,
                dist_coeffs=np.array([dist_coef, 0, 0, 0, 0]),
                image_size=(width, height),
            )

            # Set a filter for the attitude estimator to reduce the number of potential stars
            # Since the declination angle is known, the star catalog can be reduced to a band around
            # the night sky
            angle_margin_factor = 1.5
            half_fov_rad = angle_margin_factor * np.arctan(
                np.hypot(width, height) / intrinsic[0, 0] / 2
            )

            def filt(cat_xyz: np.ndarray, mag: np.ndarray) -> np.ndarray:
                _ = mag
                cat_theta = np.arcsin(cat_xyz[..., 2])
                mask = np.abs(cat_theta - theta) < half_fov_rad
                return mask

            # Calibration needs to be set before filter,
            # As initialization fails with a dummy camera calibration
            self.ae.cal = cal
            self.ae.cat_filter = filt

        if self._first_estimate_acquired:
            # Estimate rotations
            already_calculated = len(self._match_fractions)
            for xy, t in zip(
                self._stars_xy[already_calculated:], self.times[already_calculated:], strict=True
            ):
                result = self.ae.estimate_from_image_positions(xy)

                self._match_fractions.append(result.n_matches / len(xy))

                if result.n_matches == 0:
                    continue

                # Use image/object points pairs for calibration
                matched_img_xy = xy[result.obs_indices]
                self._image_xy_list.append(matched_img_xy)
                rads = -t * const.EARTH_ANGULAR_VELOCITY
                # Rotate the object points around the Earth's axis to compensate for the
                # time shift
                rot_correction = scipy.spatial.transform.Rotation.from_rotvec(
                    [0, 0, rads], degrees=False
                )
                r = scipy.spatial.transform.Rotation.from_quat(result.quat)
                self._quat_list.append((rot_correction * r).as_quat())
                time_corrected_cat_xyz = rot_correction.apply(result.cat_xyz)
                self._cat_xyz_list.append(time_corrected_cat_xyz)

            mean_match_fraction = np.mean(self._match_fractions).item()

            self.star_match_fraction[-1] = mean_match_fraction

            if len(self._image_xy_list) == 0:
                return

            # Filter outliers
            # Some attitude estimation results might be wrong, even though they are successful.
            # By checking the sum of rotational distances from each quaternion, it is possible
            # to track down outliers
            # TODO find a way to more efficiently discard outliers
            rots = scipy.spatial.transform.Rotation.from_quat(self._quat_list)
            mag_sums = []
            for r in rots:
                mags = (rots * r.inv()).magnitude()
                mag_sums.append(np.sum(mags))
            r = rots[np.argmin(mag_sums)]
            mags = (rots * r.inv()).magnitude()
            best_indices = np.nonzero(mags < np.radians(2))[0]

            # Concatenate all valid image observations and their catalog positions
            cat_xyz = np.concatenate(
                [self._cat_xyz_list[i] for i in best_indices], axis=0, dtype=np.float32
            )
            image_xy = np.concatenate(
                [self._image_xy_list[i] for i in best_indices], axis=0, dtype=np.float32
            )

            # Rotate catalog coordinates such they result in smaller rotation values in the
            # calibration problem
            cat_xyz = r.inv().apply(cat_xyz)

            if False:
                import matplotlib.pyplot as plt

                print("Plotting...")
                fig, axs = plt.subplots(2)
                s = np.concatenate(self._stars_xy, axis=0)
                axs[0].plot(s[..., 0], s[..., 1], ".", alpha=0.2)
                axs[0].plot(image_xy[..., 0], image_xy[..., 1], "x")
                axs[0].set_aspect("equal")
                axs[0].invert_yaxis()
                axs[0].set_title("Image points")
                axs[1].plot(cat_xyz[..., 0], cat_xyz[..., 1], "x")
                axs[1].set_aspect("equal")
                axs[1].invert_yaxis()
                axs[1].set_title("Object points (x and y axis)")
                plt.show()

            # Don't attempt calibration with very few samples
            # The optimization problem has 12 DoF, so 40 is reasonable
            if len(cat_xyz) < 40:
                return

            # Calibrate camera
            height, width = image.shape
            intrinsic, dist_coeffs, _, reprojection = starcamcal.calibrate_camera(
                cat_xyz,
                image_xy,
                (width, height),
                intrinsic_guess=self.ae.cal.intrinsic,
            )

            # Performing second calibration iteration with percentile of best samples
            if self._config.recalibration_percentile < 100:
                squared_error_distances = np.square(image_xy - reprojection).sum(axis=-1)
                threshold = np.percentile(
                    squared_error_distances, self._config.recalibration_percentile
                )
                mask = squared_error_distances < threshold
                cat_xyz = cat_xyz[mask]
                image_xy = image_xy[mask]

                # Calibrate camera with filtered set
                intrinsic, dist_coeffs, _, reprojection = starcamcal.calibrate_camera(
                    cat_xyz,
                    image_xy,
                    (width, height),
                    intrinsic_guess=intrinsic,
                    dist_coefs_guess=dist_coeffs,
                )

            # Build camera calibration parameters
            squared_error_distances: np.ndarray = np.square(image_xy - reprojection).sum(axis=-1)
            rms_error = float(np.sqrt(squared_error_distances.mean()))
            max_error = float(np.sqrt(squared_error_distances.max()))
            median_error = float(np.sqrt(np.median(squared_error_distances)))
            timestamp = int(time.time() * 1000)
            cal = kalkam.IntrinsicCalibrationWithData(
                intrinsic,
                dist_coeffs,
                self.ae.cal.image_size,
                rms_error,
                max_error,
                timestamp,
                [image_xy],
                [squared_error_distances],
            )

            # Store parameters
            self.rms_errors[-1] = float(rms_error)
            self.max_errors[-1] = float(max_error)
            self.median_errors[-1] = float(median_error)
            self.intrinsics[-1][:] = intrinsic
            self.dist_coeffs[-1][:] = dist_coeffs

            # Don't use parameters if the error is very large
            if rms_error > 2.0:
                return

            # Use new calibration if it is better than the last one
            if (
                isinstance(self.ae.cal, kalkam.IntrinsicCalibrationWithData)
                and cal.rms_error < self.ae.cal.rms_error
            ) or not isinstance(self.ae.cal, kalkam.IntrinsicCalibrationWithData):
                # Update the star tolerance with the estimated maximum error
                ae_config = self.ae.config
                ae_config.star_match_pixel_tol = rms_error * 2.0
                self.ae.config = ae_config

                # Set the updated calibration
                self.ae.cal = cal

                # Remove the filter on the catalog if present
                if self.ae.cat_filter is not None:
                    self.ae.cat_filter = None

                # Reset calculated values, so they will be recalculated
                self._image_xy_list = []
                self._cat_xyz_list = []
                self._match_fractions = []
                self._quat_list = []

    def plot(self, gt_intrinsic: Optional[np.ndarray], gt_dist_coeffs: Optional[np.ndarray] = None):
        """Create plot with matplotlib."""
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(3, sharex=True)
        axs[0].semilogy(self.times, self.rms_errors, label="rms errors")
        axs[0].semilogy(self.times, self.max_errors, label="max errors")
        axs[0].semilogy(self.times, self.median_errors, label="median errors")
        axs[0].grid()
        axs[0].legend()

        axs[1].plot(self.times, self.star_match_fraction, label="Match fraction")
        axs[1].grid()
        axs[1].set_ylim(0, 1)
        axs[1].legend()

        params = {"fx": (0, 0), "fy": (1, 1), "tx": (0, 2), "ty": (1, 2)}
        axs[2].axhline(y=1, color="black")
        for label, (i, j) in params.items():
            base = self.intrinsics[-1][i, j] if gt_intrinsic is None else gt_intrinsic[i, j]
            y = np.array([x[i, j] for x in self.intrinsics])
            y /= base
            axs[2].plot(self.times, y, label=label)
        params = {"k1": 0, "p1": 2, "p2": 3}
        for label, i in params.items():
            base = self.dist_coeffs[-1][i] if gt_dist_coeffs is None else gt_dist_coeffs[i]
            y = np.array([x[i] for x in self.dist_coeffs])
            y /= base
            axs[2].plot(self.times, y, label=label)
        axs[2].legend()

        axs[0].set_xlim(0, self.times[-1])
        axs[1].set_xlim(0, self.times[-1])
        axs[2].set_xlim(0, self.times[-1])

        plt.show()
        plt.close(fig)


def example():
    """Example script estimating camera calibration from artificial star data."""
    settings = camera.CameraSettings()
    cam = testing_utils.StarCameraCalibrationTestCam(settings)
    cam.theta = 0.6

    ae_config = attitude_estimation.AttitudeEstimatorConfig(star_match_pixel_tol=10, n_match=8)
    star_cal_config = StarCalibratorConfig(ae_config)
    sc = StarCalibrator(star_cal_config, 960, 540)

    for t in range(0, 60 * 60, 10):
        # Record images every 10 seconds for 5 minutes
        cam.t = t
        image = cam.capture()
        sc.put_image(image, t)

    sc.plot(gt_intrinsic=cam.cal.intrinsic, gt_dist_coeffs=cam.cal.dist_coeffs)


if __name__ == "__main__":
    example()

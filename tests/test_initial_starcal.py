import unittest.mock

import numpy as np
import pytest
import ruststartracker
import scipy.spatial

from startracker import (
    attitude_estimation,
    calibration,
    camera,
    const,
    initial_starcal,
    kalkam,
    testing_utils,
)


@pytest.mark.parametrize(
    ("use_distortion", "theta"),
    [(False, 0.0), (False, 1.0), (False, 1.3), (True, 0.0), (True, 1.0), (True, 1.3)],
)
def test_star_gradient_calibration(theta: float, *, use_distortion: bool, plot: bool = False):
    # Setup test data
    rng = np.random.default_rng(42)

    epsilon = rng.uniform(-np.pi / 2, np.pi / 2)  # roll component
    phi = rng.uniform(-np.pi, np.pi)  # azimuthal coordinate (this is not captured by the algorithm)

    cal = calibration.make_dummy()
    intrinsic = cal.intrinsic
    width, height = cal.image_size

    if not use_distortion:
        cal.dist_coeffs = None
    else:
        # Reduce higher order coefficients as these are not captured by the algorithm
        cal.dist_coeffs[1:] *= 0.1

    dist_coef = float(cal.dist_coeffs[0]) if cal.dist_coeffs is not None else 0.0

    # Get image projection of stars and one second later
    config = testing_utils.StarImageGeneratorConfig(exposure=200)
    sig = testing_utils.StarImageGenerator(cal, config)
    rot_matrix = scipy.spatial.transform.Rotation.from_euler(
        "zxz", (epsilon, np.pi / 2 - theta, phi), degrees=False
    ).as_matrix()
    extrinsic = np.concatenate((rot_matrix.T, np.zeros((3, 1))), axis=1)
    _, stars_xy, ids = sig.stars_from_extrinsic(extrinsic)
    delta_phi = const.EARTH_ANGULAR_VELOCITY  # One second later
    rot_matrix = scipy.spatial.transform.Rotation.from_euler(
        "zxz", (epsilon, np.pi / 2 - theta, phi + delta_phi), degrees=False
    ).as_matrix()
    extrinsic = np.concatenate((rot_matrix.T, np.zeros((3, 1))), axis=1)
    _, stars_xy_plus_delta, ids = sig.stars_from_extrinsic(extrinsic, ids=ids)
    mask = np.all(
        (stars_xy >= -0.5) * (stars_xy <= (cal.image_size[0] - 0.5, cal.image_size[1] - 0.5)),
        axis=-1,
    )
    stars_xy_plus_delta = stars_xy_plus_delta[mask]
    stars_xy = stars_xy[mask]

    # perceived star movement in pixel per second
    # First order approximation
    stars_dxy = stars_xy_plus_delta - stars_xy

    res = initial_starcal.star_gradient_calibration(
        stars_xy, stars_dxy, width, height, tol=1e-5, max_iter=50, plot=plot
    )
    intrinsic_est, dist_coef_est, theta_est, epsilon_est, error = res

    print(f"error: {error:.5f}")
    print(f"theta: gt={theta:.3f}, est={theta_est:.3f}")
    print(f"epsilon: gt={epsilon:.3f}, est={epsilon_est:.3f}")
    print(f"dist_coef_est: gt={dist_coef:.3f}, est={dist_coef_est:.3f}")
    print("intrinsic gt:")
    print(intrinsic.astype(int))
    print("intrinsic est:")
    print(intrinsic_est.astype(int))
    print()

    est_cal = kalkam.IntrinsicCalibration(
        intrinsic=intrinsic_est,
        dist_coeffs=np.array([dist_coef_est, 0, 0, 0, 0]),
        image_size=cal.image_size,
    )
    testing_utils.StarImageGenerator(est_cal, config)

    if use_distortion:
        # Optimization problem is only an approximation, so there is a residual
        assert error < 0.05
        assert testing_utils.angle_diff(theta, theta_est) < 0.05
        assert testing_utils.angle_diff(epsilon, epsilon_est) < 0.05
        np.testing.assert_allclose(intrinsic, intrinsic_est, rtol=0.19)
    else:
        # Optimization problem should be able to perfectly match the data
        assert error < 0.001
        assert testing_utils.angle_diff(theta, theta_est) < 0.005
        assert testing_utils.angle_diff(epsilon, epsilon_est) < 0.005
        np.testing.assert_allclose(intrinsic, intrinsic_est, rtol=0.01)


def test_movement_register():
    settings = camera.CameraSettings()
    config = testing_utils.StarImageGeneratorConfig(exposure=200)
    cam = testing_utils.StarCameraCalibrationTestCam(settings, config=config)
    cam.simulate_exposure_time = False
    cam.theta = 1.0

    # Setup dummy attitude estimator for star position detection
    ae_config = attitude_estimation.AttitudeEstimatorConfig()
    catalog = ruststartracker.StarCatalog.from_hipparcos(max_magnitude=5.5)
    cat_xyz = catalog.normalized_positions(epoch=2024.7)
    startracker = ruststartracker.StarTracker(
        cat_xyz,
        catalog.magnitude,
        ruststartracker.CameraParameters(
            camera_matrix=np.eye(3, dtype=np.float32),
            cam_resolution=(960, 540),
            dist_coefs=np.zeros(5, dtype=np.float32),
        ),
    )

    mr = initial_starcal.MovementRegisterer()

    # Record stars over a bunch of times
    times = np.linspace(0, 60 * 5, 25)
    mr.reset()
    for t in times:
        cam.t = float(t)
        image = cam.capture()
        centroids = startracker.get_centroids(
            image,
            None,
            min_star_area=ae_config.min_star_area,
            max_star_area=ae_config.max_star_area,
        )
        mr(centroids, t)
    stars_xy, stars_dxy, mses = mr.get_results()

    # Check if the polynomial fits converged
    np.testing.assert_equal(mses < 0.05, True)

    # Get ground truth data from the middle position (middle time)
    t_middle = times[len(times) // 2]
    cam.t = float(t_middle)
    cam.capture()
    stars_xy_gt = cam.get_last_star_image_positions()
    cam.t = float(t_middle + 1.0)
    cam.capture()
    # We have to hope that this will result in the same stars (no stars at the border of the image)
    stars_dxy_gt = cam.get_last_star_image_positions() - stars_xy_gt

    # Match the ground truth values with the values returned by the MovementRegisterer
    # (The order is most likely different)
    dists = np.linalg.norm(stars_xy_gt[np.newaxis, :, :] - stars_xy[:, np.newaxis, :], axis=-1)
    indices = dists.argmin(axis=-1)
    closest_star_dists = dists[np.arange(len(dists)), indices]
    stars_dxy_gt = stars_dxy_gt[indices]

    # These are the stars that match the ground truth ones
    matching_stars = closest_star_dists < 1.0
    # Check if the majority of stars match with a ground truth star
    assert matching_stars.mean() > 0.85

    # Check if the the detected star movement is accurate to 1%
    relative_errors = np.linalg.norm(
        stars_dxy[matching_stars] - stars_dxy_gt[matching_stars], axis=-1
    ) / np.linalg.norm(stars_dxy_gt[matching_stars], axis=-1)

    print(f"amount: {len(relative_errors)}")
    print(f"mean: {relative_errors.mean()}")
    print(f"max: {relative_errors.max()}")

    assert relative_errors.mean() < 0.005
    assert relative_errors.max() < 0.01


@pytest.mark.parametrize("theta", [-1.0, 0, 0.5])
def test_starcalibrator(theta: float):
    settings = camera.CameraSettings()
    config = testing_utils.StarImageGeneratorConfig(exposure=200)
    cam = testing_utils.StarCameraCalibrationTestCam(settings, config=config)
    cam.simulate_exposure_time = False
    cam.theta = theta

    ae_config = attitude_estimation.AttitudeEstimatorConfig(
        star_match_pixel_tol=5, n_match=13, max_candidates=100
    )
    star_cal_config = initial_starcal.StarCalibratorConfig(ae_config)
    sc = initial_starcal.StarCalibrator(star_cal_config, (960, 540))

    loop_broken = False
    for t in range(0, 60 * 20, 10):
        # Record images every 10 seconds for 5 minutes
        cam.t = t
        image = cam.capture()
        sc.put_image(image, t)
        if np.allclose(sc.intrinsics[-1], cam.cal.intrinsic, rtol=0.005):
            loop_broken = True
            break
    assert loop_broken

    with unittest.mock.patch("matplotlib.pyplot.show") as mock_show:
        sc.plot(gt_intrinsic=cam.cal.intrinsic, gt_dist_coeffs=cam.cal.dist_coeffs)
    mock_show.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__])

import unittest.mock

import numpy as np
import pytest
import ruststartracker

from startracker import attitude_estimation, camera, initial_starcal, kalkam, testing_utils


@pytest.mark.parametrize("use_distortion", [False, True])
def test_star_gradient_calibration(*, use_distortion: bool, plot: bool = False):
    # Setup test data
    rng = np.random.default_rng(42)

    theta = rng.uniform(-np.pi / 2, np.pi / 2)  # elevation coordinate
    epsilon = rng.uniform(-np.pi / 2, np.pi / 2)  # roll component
    phi = rng.uniform(-np.pi, np.pi)  # azimuthal coordinate (this is not captured by the algorithm)

    cam_file = testing_utils.TestingMaterial(use_existing=True).cam_file
    cal = kalkam.IntrinsicCalibration.from_json(cam_file)
    intrinsic = cal.intrinsic
    width, height = cal.image_size

    if not use_distortion:
        cal.dist_coeffs = None

    dist_coef = float(cal.dist_coeffs[0]) if cal.dist_coeffs is not None else 0.0

    # Get image projection of stars and one second later
    settings = camera.CameraSettings()
    cam = testing_utils.StarCameraCalibrationTestCam(settings, cal=cal)
    cam.phi = phi
    cam.epsilon = epsilon
    cam.theta = theta
    cam.t = 0.0
    cam.capture()
    stars_xy = cam.get_last_star_image_positions()
    cam.t = 1.0
    cam.capture()
    stars_xy_plus_delta = cam.get_last_star_image_positions()

    # perceived star movement in pixel per second
    # First order approximation
    stars_dxy = stars_xy_plus_delta - stars_xy

    if plot:
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(2)
        fig.suptitle("Test estimate camera test data")
        axs[0].invert_yaxis()  # we need the image coordinate system here
        axs[0].plot(stars_xy[..., 0], stars_xy[..., 1], ".")
        for (x, y), (dx, dy) in zip(stars_xy, stars_dxy * 60 * 3):
            axs[0].arrow(x, y, dx, dy)
        w, h = width, height
        axs[0].plot([0, w, w, 0, 0, 0], [0, 0, h, h, 0, 0])
        axs[0].axis("equal")
        axs[0].set_title("Star image positions with moving position (during 3 minutes)")
        plt.show()

    # Test camera estimator

    res = initial_starcal.star_gradient_calibration(stars_xy, stars_dxy, width, height)
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

    if use_distortion:
        # Optimization problem is only an approximation, so there is a residual
        assert error < 1e-5
        np.testing.assert_allclose(intrinsic, intrinsic_est, rtol=0.15)
        np.testing.assert_allclose(theta, theta_est, rtol=0.05)
        np.testing.assert_allclose(epsilon, epsilon_est, rtol=0.05)
    else:
        # Optimization problem should be able to perfectly match the data
        assert error < 1e-7
        np.testing.assert_allclose(intrinsic, intrinsic_est, rtol=0.01)
        np.testing.assert_allclose(theta, theta_est, rtol=0.001)
        np.testing.assert_allclose(epsilon, epsilon_est, rtol=0.001)


def test_movement_register():
    settings = camera.CameraSettings()
    cam = testing_utils.StarCameraCalibrationTestCam(settings)
    cam.theta = 1.0

    # Setup dummy attitude estimator for star position detection
    ae_config = attitude_estimation.AttitudeEstimatorConfig()
    catalog = ruststartracker.StarCatalog(max_magnitude=5.5)
    cat_xyz = catalog.normalized_positions(epoch=2024.7)
    startracker = ruststartracker.StarTracker(
        cat_xyz,
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

    # Check if the distance between the center star positions are closer than 1 pixel
    np.testing.assert_equal(closest_star_dists < 1.0, True)
    np.testing.assert_allclose(stars_dxy, stars_dxy_gt, rtol=0.2)


@pytest.mark.parametrize("theta", [-np.pi / 4, 0, np.pi / 4])
def test_starcalibrator(theta: float):
    settings = camera.CameraSettings()
    cam = testing_utils.StarCameraCalibrationTestCam(settings)
    cam.theta = theta

    ae_config = attitude_estimation.AttitudeEstimatorConfig(star_match_pixel_tol=10, n_match=8)
    star_cal_config = initial_starcal.StarCalibratorConfig(ae_config)
    sc = initial_starcal.StarCalibrator(star_cal_config, 960, 540)

    for t in range(0, 60 * 20, 10):
        # Record images every 10 seconds for 5 minutes
        cam.t = t
        image = cam.capture()
        sc.put_image(image, t)

    np.testing.assert_allclose(sc.intrinsics[-1], cam.cal.intrinsic, rtol=0.005)

    with unittest.mock.patch("matplotlib.pyplot.show") as mock_show:
        sc.plot(gt_intrinsic=cam.cal.intrinsic, gt_dist_coeffs=cam.cal.dist_coeffs)
    mock_show.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__])

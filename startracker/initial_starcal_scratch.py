"""Scratch space for the initial star calibration algorithm."""

import matplotlib.pyplot as plt
import numpy as np
import scipy.spatial

from startracker import (
    calibration,
    const,
    initial_starcal,
    testing_utils,
)


def debug():
    """Run the initial star calibration and display debug plots."""
    rng = np.random.default_rng(42)

    thetas = []
    epsilons = []
    errors = []
    dist_coefs = []
    intrinsics = []
    gt_thetas = []
    gt_epsilons = []

    cal = calibration.make_dummy()
    intrinsic = cal.intrinsic
    width, height = cal.image_size

    # cal.dist_coeffs = None
    cal.dist_coeffs[1:] *= 0.1
    # cal.dist_coeffs[1:] = 0

    dist_coef = float(cal.dist_coeffs[0]) if cal.dist_coeffs is not None else 1e-12

    # Get image projection of stars and one second later
    config = testing_utils.StarImageGeneratorConfig(exposure=200)
    sig = testing_utils.StarImageGenerator(cal, config)

    n_thetas = 30
    n_epsilons = 20

    noise_factor = 0.001

    for theta in np.linspace(-np.pi / 2, np.pi / 2, n_thetas):
        for epsilon in np.linspace(-np.pi, np.pi, n_epsilons):
            # theta = rng.uniform(-np.pi / 2, np.pi / 2)  # elevation coordinate
            # epsilon = rng.uniform(-np.pi / 2, np.pi / 2)  # roll component
            phi = rng.uniform(
                -np.pi, np.pi
            )  # azimuthal coordinate (this is not captured by the algorithm)

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
                (stars_xy >= -0.5)
                * (stars_xy <= (cal.image_size[0] - 0.5, cal.image_size[1] - 0.5)),
                axis=-1,
            )
            stars_xy_plus_delta = stars_xy_plus_delta[mask]
            stars_xy = stars_xy[mask]

            # perceived star movement in pixel per second
            # First order approximation
            stars_dxy = stars_xy_plus_delta - stars_xy

            if False:
                fig, axs = plt.subplots(2, sharex=True, sharey=True)
                fig.suptitle("Test estimate camera test data")
                axs.invert_yaxis()  # we need the image coordinate system here
                axs.plot(stars_xy[..., 0], stars_xy[..., 1], ".")
                for (x, y), (dx, dy) in zip(stars_xy, stars_dxy * 60 * 3):
                    axs.arrow(x, y, dx, dy)
                w, h = width, height
                axs.plot([0, w, w, 0, 0, 0], [0, 0, h, h, 0, 0])
                axs.set_aspect("equal")
                axs.set_title("Star image positions with moving position (during 3 minutes)")

                plt.show()

            median_length = np.median(np.linalg.norm(stars_dxy, axis=-1))
            n2 = rng.normal(0, median_length * noise_factor, size=stars_dxy.shape)

            try:
                res = initial_starcal.star_gradient_calibration(
                    stars_xy, stars_dxy + n2, width, height, tol=1e-5, max_iter=50
                )
                intrinsic_est, dist_coef_est, theta_est, epsilon_est, error = res
            except initial_starcal.StarGradientCalibrationError:
                intrinsic_est, dist_coef_est, theta_est, epsilon_est, error = (
                    np.nan,
                    np.nan,
                    np.nan,
                    np.nan,
                    np.nan,
                )
            thetas.append(testing_utils.angle_diff(theta_est, theta))
            epsilons.append(testing_utils.angle_diff(epsilon_est, epsilon))
            errors.append(error)
            dist_coefs.append(dist_coef_est / dist_coef)
            intrinsics.append(intrinsic_est / (intrinsic + 1e-12))
            gt_thetas.append(theta)
            gt_epsilons.append(epsilon)

    thetas = np.array(thetas)
    epsilons = np.array(epsilons)
    errors = np.array(errors)
    dist_coefs = np.array(dist_coefs)
    intrinsics = np.array(intrinsics)
    gt_thetas = np.array(gt_thetas)
    gt_epsilons = np.array(gt_epsilons)

    fig, axs = plt.subplots(1, sharex=True)
    axs.imshow(np.log10(errors).reshape((n_thetas, n_epsilons)), label="errors", vmin=-5, vmax=-3)
    axs.set_xlabel("eplison (Roll component about the camera axis)")
    axs.set_ylabel("theta (azimuth)")
    axs.set_ylabel("theta (azimuth)")

    fig, axs = plt.subplots(1, sharex=True)
    axs.imshow(np.log10(errors).reshape((n_thetas, n_epsilons)), label="errors")
    axs.set_xlabel("eplison (Roll component about the camera axis)")
    axs.set_ylabel("theta (azimuth)")
    axs.set_ylabel("theta (azimuth)")

    fig, axs = plt.subplots(3, sharex=True)
    axs[0].plot(thetas, label="thetas")
    axs[0].plot(epsilons, label="epsilons")
    axs[1].plot(dist_coefs, label="dist_coefs")
    axs[1].plot(intrinsics[..., 0, 0], label="f0")
    axs[1].plot(intrinsics[..., 1, 1], label="f1")
    axs[1].plot(intrinsics[..., 0, 2], label="t0")
    axs[1].plot(intrinsics[..., 1, 2], label="t1")
    axs[1].set_ylim(0, 2)
    axs[2].semilogy(errors, label="errors")
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()

    fig, axs = plt.subplots(3, 3, sharex=True)
    axs = axs.flatten()

    good_t1 = np.abs(intrinsics[..., 1, 2] - 1) < 0.1

    masks = [np.ones_like(good_t1, dtype=bool), good_t1]

    for m in masks:
        axs[0].semilogx(errors[m], thetas[m], ".", label="thetas")
        axs[1].semilogx(errors[m], epsilons[m], ".", label="epsilons")
        axs[2].semilogx(errors[m], dist_coefs[m], ".", label="dist_coefs")
        axs[3].semilogx(errors[m], intrinsics[m][..., 0, 0], ".", label="f0")
        axs[4].semilogx(errors[m], intrinsics[m][..., 1, 1], ".", label="f1")
        axs[5].semilogx(errors[m], intrinsics[m][..., 0, 2], ".", label="t0")
        axs[6].semilogx(errors[m], intrinsics[m][..., 1, 2], ".", label="t1")
        axs[7].semilogx(errors[m], gt_thetas[m], ".", label="theta")
        axs[8].semilogx(errors[m], gt_epsilons[m], ".", label="epsilon")
    for ax in axs:
        ax.legend()
    plt.show()


if __name__ == "__main__":
    debug()

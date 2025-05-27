"""Refine camera calibration given a history of observed stars.

This script performs a camera calibration using a history of observed star positions and their
3D coordinates taken from the star catalogue. This optimization problem is different from the
normal camera calibration workflow, usually done with a chessboard and OpenCV.

This script might not be suitable for a Raspberry Pi, as it requires a lot of memory and "jax" to
run. It is recommended to run this script on a desktop computer.
"""

import argparse
import pathlib
import pickle
import time

import jax
import jax.numpy as jnp
import jax.scipy.optimize
import matplotlib.pyplot as plt
import numpy as np

from startracker import kalkam


def _get_intrinsic(
    intrinsic_compact: jnp.ndarray,
    *,
    no_shear: bool = False,
    single_f: bool = False,
    fix_tx_ty: bool = False,
) -> jnp.ndarray:
    """Convert compact parameter form into a intrinsic matrix."""
    a, b, c, d, e = intrinsic_compact
    if no_shear:
        b = b * 0
    if single_f:
        d = a
    if fix_tx_ty:
        c = jax.lax.stop_gradient(c)
        e = jax.lax.stop_gradient(e)
    o = jnp.zeros_like(a)
    i = jnp.ones_like(a)
    intrinsic = jnp.stack([a, b, c, o, d, e, o, o, i], axis=-1).reshape((3, 3))
    return intrinsic


def _project_function(
    obj: jnp.ndarray, intrinsic: jnp.ndarray, dist_coef: jnp.ndarray
) -> jnp.ndarray:
    """Project object coordinates to image coordinates, given a camera matrix and distortion."""
    t = intrinsic[:2, 2]
    f = jnp.diagonal(intrinsic)[:2]
    k1, k2, p1, p2, k3 = dist_coef

    p_12 = jnp.stack((p1, p2), axis=-1)[jnp.newaxis]
    p_21 = p_12[:, ::-1]

    # project
    xyz = intrinsic @ obj.T
    xy = (xyz[:2] / xyz[2:]).T

    # distort
    xy = (xy - t) / f
    r2 = jnp.sum(xy**2, axis=-1, keepdims=True)
    r4 = jnp.square(r2)
    r6 = r2 * r4
    x_prod_y = (xy[:, 0] * xy[:, 1])[:, jnp.newaxis]
    xy_dist = xy * (1 + k1 * r2 + k2 * r4 + k3 * r6) + (
        2 * p_12 * x_prod_y + p_21 * (r2 + 2 * jnp.square(xy))
    )
    xy_dist = (xy_dist * f) + t

    return xy_dist


def _objective_function(
    state: jnp.ndarray, image: jnp.ndarray, obj: jnp.ndarray, level: int
) -> jnp.ndarray:
    """Objective function for calibration optimization problem.

    Available levels of fidelity (parameter space):
        0 only one focal length, locked translation, no_shear
        1 locked translation, no_shear
        2 no_shear
        3 full intrinsic
        4 full intrinsic, x**2
        5 full intrinsic, x**4
        6 full intrinsic, x**4 tangential
        7 full intrinsic, x**6
    """
    intrinsic = _get_intrinsic(
        state[:5], no_shear=level <= 2, single_f=level <= 0, fix_tx_ty=level <= 1
    )
    dist_coef = state[5:]

    level_masks = jnp.array(
        [
            [0, 0, 0, 0, 0],  # level=0 only one focal length, locked translation, no_shear
            [0, 0, 0, 0, 0],  # level=1 locked translation, no_shear
            [0, 0, 0, 0, 0],  # level=2 no_shear
            [0, 0, 0, 0, 0],  # level=3 full intrinsic
            [1, 0, 0, 0, 0],  # level=4 full intrinsic, x**2
            [1, 1, 0, 0, 0],  # level=5 full intrinsic, x**4
            [1, 1, 1, 1, 0],  # level=6 full intrinsic, x**4 tangential
            [1, 1, 1, 1, 1],  # level=7 full intrinsic, x**6
        ],
        dtype=jnp.float32,
    )[jnp.minimum(level, 6)]

    dist_coef = dist_coef * level_masks

    return image - _project_function(obj, intrinsic, dist_coef)


def lm_solv(fun, x0, args=(), max_iter=10, tol=1e-4, rtol=1e-5, lambda_init=1e-3) -> jnp.ndarray:
    """Levenberg-Marquardt solver for nonlinear least squares problems.

    Args:
        fun: Objective function to minimize, should return residuals.
        x0: Initial guess for the parameters.
        args: Additional arguments to pass to the objective function.
        max_iter: Maximum number of iterations.
        tol: Absolute tolerance for convergence.
        rtol: Relative tolerance for convergence.
        lambda_init: Initial value for the damping parameter.

    Returns:
        The optimized parameters.
    """
    x = x0
    lamb = lambda_init
    prev_cost = None
    for _ in range(max_iter):
        residuals = fun(x, *args).reshape(-1)
        cost = jnp.sum(residuals**2)
        j = jax.jacobian(lambda x_: fun(x_, *args).reshape(-1))(x)
        jtj = j.T @ j
        jtr = j.T @ residuals
        # LM step
        a = jtj + lamb * jnp.eye(jtj.shape[0], dtype=jtj.dtype)
        try:
            delta = jnp.linalg.solve(a, -jtr)
        except Exception:
            break
        x_new = x + delta
        new_residuals = fun(x_new, *args).reshape(-1)
        new_cost = jnp.sum(new_residuals**2)
        if new_cost < cost * 1.001:  # Epsilon to avoid numerical issues
            # Accept step, decrease lambda
            x = x_new
            lamb = lamb / 10

            # Check convergence absolute
            norm = jnp.linalg.norm(delta)
            if norm < tol:
                break

            # Check convergence relative
            if prev_cost is not None:
                rel_change = jnp.abs(prev_cost - new_cost) / (jnp.abs(prev_cost) + 1e-12)
                if rel_change < rtol:
                    break
            prev_cost = new_cost
        else:
            # Reject step, increase lambda
            lamb = lamb * 10

    return x


def calibrate_camera(
    object_points: np.ndarray,
    image_points: np.ndarray,
    image_size: tuple[float, float],
    *,
    intrinsic_guess: np.ndarray | None = None,
    dist_coefs_guess: np.ndarray | None = None,
    level: int = 7,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calibrate camera parameters given image points and 3d object camera frame vectors.

    Args:
        object_points: 2D array of object points in camera frame coordinates, shape=(n, 3).
        image_points: 2D array of image points in pixel coordinates, shape=(n, 2).
        image_size: Size of the camera image in pixels as (width, height).
        intrinsic_guess: Optional initial guess for the intrinsic camera parameters, shape=(3, 3).
        dist_coefs_guess: Optional initial guess for the distortion coefficients, shape=(5,).
        level: Optimization level, see _objective_function

    Returns:
        A tuple containing:
            - intrinsic: The intrinsic camera parameters as a 3x3 matrix.
            - dist_coef: The distortion coefficients as a 1D array of length 5.
            - reprojection: The reprojected image points as a 2D array of shape=(n, 2).
    """
    if intrinsic_guess is None:
        width, height = image_size
        object_points = object_points / np.linalg.norm(object_points, axis=-1, keepdims=True)
        focal_length = np.hypot(width, height) / np.tan(np.arccos(object_points[..., 2]).max()) / 2
        intrinsic_flattened = jnp.array(
            [focal_length, 0, width / 2, focal_length, height / 2], jnp.float32
        )
    else:
        intrinsic_flattened = np.concatenate(
            (intrinsic_guess[0, :], intrinsic_guess[1, 1:]), axis=0
        )

    dist_coeffs_flattened = [0, 0, 0, 0, 0] if dist_coefs_guess is None else dist_coefs_guess

    x0 = jnp.array(
        np.concatenate((intrinsic_flattened, dist_coeffs_flattened), axis=0), jnp.float32
    )

    image_xy = jnp.array(image_points, jnp.float32)
    object_xyz = jnp.array(object_points, dtype=jnp.float32)

    x0 = lm_solv(_objective_function, x0, args=(image_xy, object_xyz, level))

    intrinsic = _get_intrinsic(x0[:5])
    dist_coef = x0[5:]

    reprojection = _project_function(object_xyz, intrinsic, dist_coef)

    return (
        np.asarray(intrinsic),
        np.asarray(dist_coef),
        np.asarray(reprojection),
    )


def calibrate(
    image_points: np.ndarray,
    object_points: np.ndarray,
    image_size: tuple[int, int] = (960, 540),
    *,
    percentile: int | None = None,
    plot: bool = False,
) -> kalkam.IntrinsicCalibrationWithData:
    """Calibrate camera using a pair of image points and object points at infinite distance.

    Args:
        image_points: 2D array of image points in pixel coordinates, shape (n, 2).
        object_points: 2D array of object points in camera frame coordinates, shape (n, 3).
        image_size: Size of the camera image in pixels as (width, height).
        percentile: If not None, perform a second calibration iteration with the given percentage
            of best samples.
        plot: If True, plot the calibration process and results.

    Returns:
        The calibrated camera parameters and with calibration performance data.
    """
    if plot:
        print("Plotting...")
        fig, axs = plt.subplots(2)
        for img in image_points:
            axs[0].plot(img[..., 0], img[..., 1], "x")
            axs[0].plot(img[..., 0], img[..., 1], "x")
        for obj in object_points:
            axs[1].plot(obj[..., 0], obj[..., 1], "x")
        plt.show()
        plt.close(fig)

    print("Calibrating...")
    intrinsic, dist_coefs, reprojection = calibrate_camera(object_points, image_points, image_size)

    squared_error_distances: np.ndarray = np.square(image_points - reprojection).sum(axis=-1)
    rms_error = float(np.sqrt(squared_error_distances.mean()))
    max_error = float(np.sqrt(squared_error_distances.max()))

    print(f"RMS error: {rms_error:.2f} pixels, max error {max_error:.2f} pixels")

    if percentile is not None:
        print(f"Performing second calibration iteration with {percentile}% of best samples...")
        threshold = np.percentile(squared_error_distances, percentile)
        mask = squared_error_distances < threshold
        object_points = object_points[mask]
        image_points = image_points[mask]

        intrinsic, dist_coefs, reprojection = calibrate_camera(
            object_points,
            image_points,
            image_size,
            intrinsic_guess=intrinsic,
            dist_coefs_guess=dist_coefs,
        )

        squared_error_distances: np.ndarray = np.square(image_points - reprojection).sum(axis=-1)
        rms_error = float(np.sqrt(squared_error_distances.mean()))
        max_error = float(np.sqrt(squared_error_distances.max()))

        print(f"RMS error: {rms_error:.2f} pixels, max error {max_error:.2f} pixels")

    if plot:
        print("Plotting...")
        fig, axs = plt.subplots(2)
        axs[0].plot(image_points[..., 0], image_points[..., 1], "x")
        axs[0].plot(reprojection[..., 0], reprojection[..., 1], ".")
        axs[0].set_title("Reprojection")
        epsilon = 1e-8
        bins = np.logspace(np.log10(1e-4), np.log10(10), 41)
        bins[0] = min(squared_error_distances.min(), 1e-4) - epsilon
        bins[-1] = max(squared_error_distances.max(), 10) + epsilon
        axs[1].hist(squared_error_distances, bins=bins)
        axs[1].set_xscale("log")
        axs[1].set_xlim(1e-4, 10)
        axs[1].set_title("Squared error distance histogram")
        axs[1].set_xlabel("Squared error distance (pixels)")
        axs[1].set_ylabel("Count")
        plt.show()
        plt.close(fig)

    timestamp = int(time.time() * 1000)

    calibration = kalkam.IntrinsicCalibrationWithData(
        intrinsic,
        dist_coefs,
        image_size,
        rms_error,
        max_error,
        timestamp,
        [image_points],
        [squared_error_distances],
    )

    return calibration


def calibrate_from_database(
    directory: pathlib.Path,
    n_samples: int,
    image_size: tuple[int, int] = (960, 540),
    *,
    percentile: int | None = None,
    plot: bool = False,
) -> None:
    """Calibrate camera using history of star observations contained in directory.

    Args:
        directory: Directory containing logged star observations as .pkl files.
        n_samples: Number of samples to use for calibration.
        image_size: Size of the camera image in pixels as (width, height).
        percentile: If not None, perform a second calibration iteration with the given percentage
            of best samples.
        plot: If True, plot the calibration process and results.
    """
    files = sorted(directory.glob("data_*.pkl"))

    print(f"Found {len(files)} files")

    if not len(files):
        raise RuntimeError(f"No pickle files found in {directory}")

    images_list = []
    objects_list = []

    print("Loading files...")

    for file in files:
        with file.open("rb") as f:
            contents = pickle.load(f)

        images_list.extend(contents["image"])
        objects_list.extend(contents["object"])

    objects = np.concatenate(objects_list, axis=0)
    images = np.concatenate(images_list, axis=0)

    print(f"Found {len(objects)} logged star observations")

    rng = np.random.default_rng(42)
    indices = rng.permutation(len(images))[:n_samples]

    objects = objects[indices]
    images = images[indices]

    calibration = calibrate(
        images, objects, image_size=image_size, percentile=percentile, plot=plot
    )

    output_file = directory / "cam_file.json"

    calibration.to_json(output_file)
    print(f"Calibration saved to: {output_file}")


def cli(argv: list[str] | None = None) -> int:
    """Entrypoint for star-based camera calibraion."""
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "directory",
        help=(
            "Directory containing logged star observations as .pkl file. "
            "Usually this is the .startracker/database directory"
        ),
        type=pathlib.Path,
    )
    parser.add_argument(
        "image_size",
        help=("Comma-separated width and height of the camera image in pixels, " "e.g. 960,540"),
        type=lambda s: tuple(map(int, s.split(","))),
    )
    parser.add_argument(
        "--n_samples",
        "-n",
        help="Number of samples sampled from the observations",
        default=1000,
        type=int,
    )
    parser.add_argument(
        "--plot",
        "-p",
        help="Show plots of calibration process and result",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--percentile",
        "-c",
        help=(
            "Perform a second calibration iteration with a percentage of best-performing samples "
            "e.g. 90%% of samples"
        ),
        type=int,
        choices=range(1, 100),
        default=None,
        metavar="[1-99]",
    )

    args = parser.parse_args(argv)

    calibrate_from_database(
        args.directory, args.n_samples, args.image_size, plot=args.plot, percentile=args.percentile
    )

    return 0


if __name__ == "__main__":
    cli()

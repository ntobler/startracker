"""Calibrate camera given a history of observed stars."""

import argparse
import pathlib
import pickle
import time

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
        0 only one focal length, locked translation
        1 two focal lengths, locked translation
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
    dist_coef = state[5:] * 0.01

    if level <= 3:
        dist_coef = dist_coef * 0.0
    elif level <= 4:
        dist_coef = dist_coef * jnp.array([1, 0, 0, 0, 0], dtype=jnp.float32)
    elif level <= 5:
        dist_coef = dist_coef * jnp.array([1, 1, 0, 0, 0], dtype=jnp.float32)
    elif level <= 6:
        dist_coef = dist_coef * jnp.array([1, 1, 1, 1, 0], dtype=jnp.float32)

    return jnp.square(image - _project_function(obj, intrinsic, dist_coef)).sum()


def calibrate_camera(
    object_points: np.ndarray, image_points: np.ndarray, image_size: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calibrate camera parameters given image points and 3d object camera frame vectors."""
    width, height = image_size

    object_points = object_points / np.linalg.norm(object_points, axis=-1, keepdims=True)

    focal_length = np.hypot(width, height) / np.tan(np.arccos(object_points[..., 2]).max()) / 2

    x0 = jnp.array(
        [focal_length, 0, width / 2, focal_length, height / 2, 0, 0, 0, 0, 0], jnp.float32
    )

    image_xy = jnp.array(image_points, jnp.float32)
    object_xyz = jnp.array(object_points, dtype=jnp.float32)

    for level in [1, 2, 3, 4, 5, 6, 7]:
        print(f"Solving camera parameters level {level} of 7")
        res = jax.scipy.optimize.minimize(
            _objective_function, x0, args=(image_xy, object_xyz, level), method="BFGS"
        )
        x0 = res.x

        intrinsic = _get_intrinsic(res.x[:5])
        dist_coef = res.x[5:] * 0.01

        reprojection = _project_function(object_xyz, intrinsic, dist_coef)

    return (
        np.asarray(intrinsic),
        np.asarray(dist_coef),
        np.asarray(reprojection),
    )


def calibrate(
    directory: pathlib.Path,
    n_samples: int,
    image_size: tuple[int, int] = (960, 540),
    *,
    plot: bool = False,
) -> None:
    """Calibrate camera using history of star observations contained in directory."""
    files = sorted(directory.glob("data_*.pkl"))

    print(f"Found {len(files)} files")

    if not len(files):
        raise RuntimeError(f"No pickle files found in {directory}")

    images_list = []
    objects_list = []

    for file in files:
        with file.open("rb") as f:
            contents = pickle.load(f)

        images_list.extend(contents["image"])
        objects_list.extend(contents["object"])

    if plot:
        fig, axs = plt.subplots(2)
        for img in images_list:
            axs[0].plot(img[..., 0], img[..., 1], "x")
            axs[0].plot(img[..., 0], img[..., 1], "x")
        for obj in objects_list:
            axs[1].plot(obj[..., 0], obj[..., 1], "x")
        plt.show()
        plt.close(fig)

    objects = np.concatenate(objects_list, axis=0)
    images = np.concatenate(images_list, axis=0)

    print(f"Found {len(objects)} logged star observations")

    rng = np.random.default_rng(42)
    indices = rng.permutation(len(images))[:n_samples]

    objects = objects[indices]
    images = images[indices]

    intrinsic, dist_coefs, reprojection = calibrate_camera(objects, images, image_size)

    squared_error_distances: np.ndarray = np.square(images - reprojection).sum(axis=-1)
    rms_error = float(np.sqrt(squared_error_distances.mean()))
    max_error = float(np.sqrt(squared_error_distances.max()))

    print(f"RMS error: {rms_error:.2f}, max error {max_error:.2f}")

    if plot:
        fig, axs = plt.subplots(1)
        axs.plot(images[..., 0], images[..., 1], "x")
        axs.plot(reprojection[..., 0], reprojection[..., 1], ".")
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
        [images],
        [squared_error_distances],
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
    parser.add_argument("--n_samples", "-n", help="Number of samples", default=1000, type=int)
    parser.add_argument("--plot", "-p", help="Show plots", action="store_true", default=False)

    args = parser.parse_args(argv)

    calibrate(args.directory, args.n_samples, plot=args.plot)

    return 0


if __name__ == "__main__":
    cli()

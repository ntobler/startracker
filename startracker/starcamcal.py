"""Refine camera calibration given a history of observed stars.

This script performs a camera calibration using a history of observed star positions and their
3D coordinates taken from the star catalogue. This optimization problem is different from the
normal camera calibration workflow, usually done with a chessboard and OpenCV.
"""

import argparse
import pathlib
import pickle
import time

import matplotlib.pyplot as plt
import numpy as np
import scipy.spatial.transform

from startracker import kalkam, libstartracker


def calibrate_camera(
    object_points: np.ndarray,
    image_points: np.ndarray,
    image_size: tuple[float, float],
    *,
    intrinsic_guess: np.ndarray | None = None,
    dist_coefs_guess: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
            - extrinsic: The extrinsic camera parameters as a 3x3 rotation matrix.
            - reprojection: The reprojected image points as a 2D array of shape=(n, 2).
    """
    if len(image_points) < 13:
        msg = (
            "The optimization problem has 12 DoF. The number of passed image points only amount "
            f"to {len(image_points)*2} equations which is less than double of the DoF and will "
            f"result in a poor calibration. Please provide at least {len(image_points)} image "
            "points."
        )
        raise ValueError(msg)
    image_points = np.ascontiguousarray(image_points, dtype=np.float32)
    object_points = np.ascontiguousarray(object_points, dtype=np.float32)
    res = libstartracker.starcal_calibrate(
        image_points,
        object_points,
        image_size,
        intrinsic_guess=np.ascontiguousarray(intrinsic_guess, dtype=np.float64)
        if intrinsic_guess is not None
        else None,
        dist_coefs_guess=np.ascontiguousarray(dist_coefs_guess, dtype=np.float64)
        if dist_coefs_guess is not None
        else None,
    )
    params = np.array(res.params, dtype=np.float64)
    extrinsic = scipy.spatial.transform.Rotation.from_mrp(params[:3]).as_matrix()
    intrinsic = np.array(
        [[params[3], 0.0, params[4]], [0, params[5], params[6]], [0, 0, 1]],
        dtype=np.float64,
    )
    dist_coef = np.array(params[7:])

    error = (
        libstartracker.starcal_objective_function(
            tuple(params.tolist()), image_points, object_points
        )[0]
        .astype(np.float32)
        .reshape(-1, 2)
    )
    reprojection = error + image_points

    return (intrinsic, dist_coef, extrinsic, reprojection)


def calibrate(
    image_points: np.ndarray,
    object_points: np.ndarray,
    image_size: tuple[int, int] = (960, 540),
    *,
    percentile: int | None = None,
    plot: bool = False,
) -> tuple[kalkam.IntrinsicCalibrationWithData, np.ndarray]:
    """Calibrate camera using a pair of image points and object points at infinite distance.

    Args:
        image_points: 2D array of image points in pixel coordinates, shape (n, 2).
        object_points: 2D array of object points in camera frame coordinates, shape (n, 3).
        image_size: Size of the camera image in pixels as (width, height).
        percentile: If not None, perform a second calibration iteration with the given percentage
            of best samples.
        plot: If True, plot the calibration process and results.

    Returns:
        The calibrated camera parameters and with calibration performance data,
        and the extrinsic camera parameters as a 3x3 rotation matrix.
    """
    if plot:
        print("Plotting...")
        fig, axs = plt.subplots(2)
        axs[0].plot(image_points[..., 0], image_points[..., 1], "x")
        axs[0].set_aspect("equal")
        axs[0].invert_yaxis()
        axs[0].set_title("Image points")
        axs[1].plot(object_points[..., 0], object_points[..., 1], "x")
        axs[1].set_aspect("equal")
        axs[1].invert_yaxis()
        axs[1].set_title("Object points (x and y axis)")
        plt.show()
        plt.close(fig)

    print(f"Calibrating using {len(object_points)} samples...")
    intrinsic, dist_coefs, extrinsic, reprojection = calibrate_camera(
        object_points, image_points, image_size
    )

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

        intrinsic, dist_coefs, extrinsic, reprojection = calibrate_camera(
            object_points,
            image_points,
            image_size,
            intrinsic_guess=intrinsic,
            dist_coefs_guess=dist_coefs,
        )

        squared_error_distances = np.square(image_points - reprojection).sum(axis=-1)
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

    return calibration, extrinsic


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

    indices = libstartracker.even_spaced_indices(images.astype(np.float32), n_samples, 42)

    objects = objects[indices]
    images = images[indices]

    calibration, _ = calibrate(
        images, objects, image_size=image_size, percentile=percentile, plot=plot
    )

    if plot:
        calibration.plot(show=True)

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

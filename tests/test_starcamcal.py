import pathlib
import unittest.mock

import numpy as np
import pytest
import scipy.spatial

from startracker import calibration, kalkam, libstartracker, starcamcal


def test_objective_function():
    cal = calibration.make_dummy()

    rng = np.random.default_rng(42)

    n = 100

    image_points = rng.uniform(-0.5, np.asarray(cal.image_size) - 0.5, size=(n, 2))

    object_points = kalkam.PointProjector(cal, extrinsic=np.eye(4)).pix2obj(
        image_points, obj_z=1, axis=1
    )
    object_points /= np.linalg.norm(object_points, axis=-1, keepdims=True)

    assert cal.dist_coeffs is not None
    x0 = tuple(
        [
            float(x)
            for x in [0, 0, 0, *cal.intrinsic[0, ::2], *cal.intrinsic[1, 1:], *cal.dist_coeffs]
        ]
    )

    image_points = image_points.astype(np.float32)
    object_points = object_points.astype(np.float32)

    x0 = np.array(x0, dtype=np.float64)

    # Check if residuals are close to zero, as the image_points should match the object_points
    residuals, _ = libstartracker.starcal_objective_function(
        tuple(x0.tolist()), image_points, object_points
    )
    np.testing.assert_allclose(residuals, 0, rtol=1e-5, atol=1e-4)

    residuals, jacobian = libstartracker.starcal_objective_function(
        tuple(x0.tolist()), image_points, object_points
    )
    delta_param = x0 / 1000
    x0 += delta_param
    perturbed_residuals, _ = libstartracker.starcal_objective_function(
        tuple(x0.tolist()), image_points, object_points
    )
    # Check if residuals are not close to zero, as the image_points should not match the
    # object_points
    assert np.linalg.norm(perturbed_residuals) > 1
    expected_residuals = residuals + np.sum(delta_param[None, :] * jacobian, axis=1)
    np.testing.assert_allclose(perturbed_residuals, expected_residuals, rtol=1e-3, atol=1e-3)


def test_calibration():
    cal = calibration.make_dummy()

    rng = np.random.default_rng(42)

    n = 200

    image_points = rng.uniform(-0.5, np.asarray(cal.image_size) - 0.5, size=(n, 2))

    object_points = kalkam.PointProjector(cal, extrinsic=np.eye(4)).pix2obj(
        image_points, obj_z=1, axis=1
    )
    object_points /= np.linalg.norm(object_points, axis=-1, keepdims=True)

    axis = rng.normal(size=(3,))
    axis /= np.linalg.norm(axis)
    r = scipy.spatial.transform.Rotation.from_rotvec(axis * 10, degrees=True)
    extrinsic_gt = r.as_matrix()
    object_points = r.apply(object_points)

    # with unittest.mock.patch("matplotlib.pyplot.show") as mock_show:
    new_calibration, extrinsic = starcamcal.calibrate(
        image_points, object_points, cal.image_size, percentile=90, plot=False
    )
    # assert mock_show.call_count == 2

    np.testing.assert_allclose(cal.intrinsic, new_calibration.intrinsic, rtol=1e-2, atol=1e-2)
    np.testing.assert_allclose(extrinsic_gt, extrinsic, rtol=1e-2, atol=1e-2)
    assert new_calibration.max_error < 0.001
    assert new_calibration.rms_error < 0.001


def test_cli():
    with unittest.mock.patch("startracker.starcamcal.calibrate_from_database") as mock_calibrate:
        starcamcal.cli(["database/", "960,540", "-n200", "--plot", "-c90"])

        mock_calibrate.assert_called_once_with(
            pathlib.Path("database/"),
            200,
            (960, 540),
            percentile=90,
            plot=True,
        )


if __name__ == "__main__":
    pytest.main([__file__])

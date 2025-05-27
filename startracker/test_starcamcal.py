import pathlib
import unittest.mock

import numpy as np
import pytest

from startracker import kalkam, starcamcal, testing_utils


def test_calibration():
    tm = testing_utils.TestingMaterial(use_existing=True)
    cal = kalkam.IntrinsicCalibration.from_json(tm.cam_file)

    rng = np.random.default_rng(42)

    n = 200

    image_points = rng.uniform(-0.5, np.asarray(cal.image_size) - 0.5, size=(n, 2))

    object_points = kalkam.PointProjector(cal, extrinsic=np.eye(4)).pix2obj(
        image_points, obj_z=1, axis=1
    )
    object_points /= np.linalg.norm(object_points, axis=-1, keepdims=True)

    with unittest.mock.patch("matplotlib.pyplot.show") as mock_show:
        new_calibration = starcamcal.calibrate(
            image_points, object_points, cal.image_size, percentile=90, plot=True
        )
        assert mock_show.call_count == 2

    np.testing.assert_allclose(cal.intrinsic, new_calibration.intrinsic, rtol=1e-2, atol=1e-2)
    assert new_calibration.max_error < 0.4
    assert new_calibration.rms_error < 0.1


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

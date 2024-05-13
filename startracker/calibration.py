"""Camera Calibration."""

import collections

import numpy as np

from . import kalkam

IntrinsicParams = collections.namedtuple(
    "IntrinsicParams", ["fx", "fy", "tx", "ty", "width", "height"]
)


def make_dummy() -> kalkam.IntrinsicCalibration:
    """
    Return dummy calibration that may be used for testing.

    Returns:
        kalkam.IntrinsicCalibration: Calibration instance with dummy values.
    """
    return kalkam.IntrinsicCalibration(
        np.array(
            [
                [2126.9433827817543, 0.0, 471.5660270698475],
                [0.0, 2125.4878604007063, 310.73929485280405],
                [0.0, 0.0, 1.0],
            ]
        ),
        np.array(
            [
                -0.4412080745099301,
                -0.159542022464492,
                0.007670124986859448,
                -0.002132920872628578,
                -1.6478824652916775,
            ]
        ),
        (960, 540),
    )


def intrinsic_params(cal: kalkam.IntrinsicCalibration) -> IntrinsicParams:
    """
    Get intrinsic camera parameters.

    Returns:
        IntrinsicParams: Named tuple containing camera parameters
    """
    fx = float(cal.intrinsic[0, 0])
    fy = float(cal.intrinsic[1, 1])
    tx = float(cal.intrinsic[0, 2])
    ty = float(cal.intrinsic[1, 2])
    width, height = cal.image_size
    return IntrinsicParams(fx, fy, tx, ty, width, height)


def get_distorted_camera_frame(
    cal: kalkam.IntrinsicCalibration, segments_per_side: int = 10
) -> np.ndarray:
    """
    Get a numpy array representing a polygon of the distorted camera frame in cartesian space.

    Args:
        cal: Calibration object.
        segments_per_side: Number of segments per side of the rectangle.

    Returns:
        np.ndarray: Points xyz shape=[segments_per_side * 4, 3]
    """
    xf = np.arange(segments_per_side * 4) / segments_per_side

    p = intrinsic_params(cal)
    x0 = -0.5
    x1 = p.width - 0.5
    y0 = -0.5
    y1 = p.height - 0.5

    corners = np.array(
        ((x0, x1, x1, x0, x0), (y0, y0, y1, y1, y0), (1, 1, 1, 1, 1)),
        dtype=np.float32,
    )
    points = np.array([np.interp(xf, np.arange(len(d)), d) for d in corners]).T

    c = kalkam.PointUndistorter(cal).undisort(points[:, :2])

    points[:, :2] = (c - [p.tx, p.ty]) / [p.fx, p.fy]

    return points


def get_frame_corners(cal: kalkam.IntrinsicCalibration) -> np.ndarray:
    """
    Get a numpy array representing the corners of the camera in cartesian space.

    Args:
        cal: Calibration object.

    Returns:
        np.ndarray: Corners xyz shape=[5, 3]
    """

    p = intrinsic_params(cal)
    x0 = (-p.tx - 0.5) / p.fx
    x1 = (p.width - p.tx - 0.5) / p.fx
    y0 = (-p.ty - 0.5) / p.fy
    y1 = (p.height - p.ty - 0.5) / p.fy

    corners = np.array(
        ((x0, x1, x1, x0, x0), (y0, y0, y1, y1, y0), (1, 1, 1, 1, 1)),
        dtype=np.float32,
    ).T
    return corners

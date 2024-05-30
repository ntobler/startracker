"""Utilities for images."""

import cv2
import numpy as np

from startracker import kalkam


def draw_grid(
    image: np.ndarray,
    rot_mat: np.ndarray,
    cal: kalkam.IntrinsicCalibration,
    *,
    inplace: bool = False,
) -> np.ndarray:
    """Draw equatorial grid given image.

    Args:
        image: image to draw on.
        rot_mat: Rotation matrix defining the rotation of the star frame
        cal: Calibration object.
        inplace: Draw on the given image or return new one.

    Return:
        Image with grid drawn.
    """
    if not inplace:
        image = np.copy(image)

    pp = kalkam.PointProjector(cal, rot_mat)

    latitudes = np.radians(np.linspace(-90, 90, 18 * 2))
    longitudes = np.radians(np.arange(0, 361, 15))

    lat, lon = np.meshgrid(latitudes, longitudes)

    x = np.cos(lon) * np.cos(lat)
    y = np.sin(lon) * np.cos(lat)
    z = np.sin(lat)
    points = np.stack((x, y, z), axis=-1)

    # Get in-frame point
    target_vector = rot_mat[2, :3]
    mask = np.inner(points, target_vector) > cal.cos_phi(angle_margin_factor=1.3)
    points = pp.obj2pix(points, axis=-1)

    c = 30
    t = 1

    # To silence possible invalid number in cast following cast
    points *= mask[..., None]

    points = np.round(points).astype(np.int32)
    it = zip(
        [points, np.swapaxes(points, 1, 0)],
        [mask, np.swapaxes(mask, 1, 0)],
    )
    for points, mask in it:
        for line, mask_line in zip(points, mask):
            last_xy = None
            for xy, m in zip(line, mask_line):
                if m:
                    if last_xy is not None:
                        image = cv2.line(image, last_xy, xy, c, t)
                    last_xy = xy
                else:
                    last_xy = None

    return image

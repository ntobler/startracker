"""Coordinate transformations."""

import numpy as np
import scipy.spatial.transform

from typing import Tuple


def azel2nwu(az_el: np.ndarray, axis=-1, degrees: bool = False) -> np.ndarray:
    """Convert azimuth, elevation to a unit vector in the north west up frame."""
    if degrees:
        az_el = np.radians(az_el)
    az, el = np.moveaxis(az_el, axis, 0)
    z = np.sin(el)
    cos_el = np.cos(el)
    x = np.cos(az)
    x *= cos_el
    y = np.sin(-az)
    y *= cos_el
    nwu = np.stack((x, y, z), axis=axis)
    return nwu


def nwu2azel(nwu: np.ndarray, axis=-1, degrees: bool = False):
    """Convert north west up coordiantes to azimuth, elevation."""
    nwu = np.moveaxis(nwu, axis, 0)
    x, y, z = nwu
    az = np.arctan2(-y, x)
    el = np.arctan2(z, np.linalg.norm(nwu[:2], axis=0))
    az += (az < 0) * (np.pi * 2)
    az_el = np.stack((az, el), axis=axis)
    if degrees:
        az_el *= 180 / np.pi
    return az_el


def find_common_rotation_axis(quats: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Find the common rotation axis of a set of rotations.

    Args:
        quats: array of input rotation quaterions, shape=[n, 4]

    Returns:
        Tuple[np.ndarray, float]:
            - Common rotation axis
            - Angular standard deviation estimate in rads, can be NaN
    """
    rots = scipy.spatial.transform.Rotation.from_quat(quats)
    inv_rots = rots.inv()

    mag_matrix = [(r * r_inv).magnitude() for r_inv in inv_rots for r in rots]
    mag_matrix = np.array(mag_matrix).reshape((len(rots), len(rots)))

    dist_to_90_deg = np.abs(np.abs(mag_matrix % np.pi) - np.pi / 2)
    np.fill_diagonal(mag_matrix, 100)

    most_perpendicular_inv_rots = inv_rots[np.argmin(dist_to_90_deg, axis=-1)]

    # Get rotation difference between pairs
    axis_vecs = (most_perpendicular_inv_rots * rots).as_rotvec()
    norm = np.linalg.norm(axis_vecs, axis=-1, keepdims=True)
    axis_vecs /= norm

    # Calculate a weight factor that is 1 if the rotation difference
    # is close to 90 degrees. 0 and 180 degrees get weight = 0
    weights = np.abs(np.sin(norm))

    # Make sure all axes have a positive z compoenent
    # This will make sure the axis points in the general direciton
    # of the camera
    axis_vecs = np.where(
        np.sum(axis_vecs * axis_vecs[:1], axis=-1, keepdims=True) > 0,
        axis_vecs,
        -axis_vecs,
    )

    axis_vec = np.mean(axis_vecs * weights, axis=0)
    axis_vec /= np.linalg.norm(axis_vec, axis=-1, keepdims=True)

    # calculate estimate of certainty
    if len(axis_vecs) > 2:
        cos_error = np.clip(np.sum(axis_vecs * axis_vec[None], axis=-1), -1, 1)
        angular_errors = np.arccos(cos_error)
        std_deg = float(np.sqrt(np.mean(angular_errors**2) / len(angular_errors)))
    else:
        # Error cannot be calculated from two rotations
        std_deg = np.nan

    return axis_vec, std_deg

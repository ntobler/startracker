"""Coordinate transformations."""

import numpy as np


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

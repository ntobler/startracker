"""Utilities for image processing."""

import cv2
import numpy as np
import numpy.typing as npt


def decode_sbggr12_1x12(raw: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint16]:
    """Decode SBGGR12 format (12bit per pixel).

    3 bytes hold data for 2 pixels.
    - byte 0: pixel 0 upper 8bit
    - byte 1: pixel 1 upper 8bit
    - byte 2: pixel 0 lower 4bit | pixel 1 lower 4bit
    """
    h, w = raw.shape
    raw = raw.reshape((h, w // 3, 3))
    bayer = np.empty((h, w // 3, 2), np.uint16)

    # Use views to prevent unnecessary array allocations
    view = bayer.view(dtype=np.uint8)
    lower_bytes = view[..., ::2]
    higher_bytes = view[..., 1::2]

    # Fill higher byte with base
    higher_bytes[:] = raw[..., :2]
    # Fill lower byte with 5th raw byte and distribute data
    lower_bytes[:] = raw[..., 2:]
    lower_bytes[..., 1] <<= 4  # ggggllll -> llll0000

    # shift all bytes to the right, so lsb rubbish gets deleted
    # hhhhhhhh llllgggg -> 0000hhhh hhhhllll
    bayer >>= 4
    return bayer.reshape((h, (w * 2) // 3))


def decode_srggb10(raw: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint16]:
    """Decode SRGGB10 format (10bit per pixel).

    5 bytes hold data for 4 pixels.
    - byte 0: pixel 0 upper 8bit
    - byte 1: pixel 1 upper 8bit
    - byte 2: pixel 2 upper 8bit
    - byte 3: pixel 3 upper 8bit
    - byte 4: pixel 0 lower 4bit | pixel 1 lower 4bit | pixel 2 lower 4bit | pixel 3 lower 4bit
    """
    h, w = raw.shape
    raw = raw.reshape((h, w // 5, 5))
    bayer = np.empty((h, w // 5, 4), np.uint16)

    # Use views to prevent unnecessary array allocations
    view = bayer.view(dtype=np.uint8)
    lower_bytes = view[..., ::2]
    higher_bytes = view[..., 1::2]

    # Fill higher byte with base
    higher_bytes[:] = raw[..., :4]
    # Fill lower byte with 5th raw byte and distribute data
    lower_bytes[:] = raw[..., 4:]
    lower_bytes[..., 1] <<= 2  # ggllgggg -> llgggg00
    lower_bytes[..., 2] <<= 4  # ggggllgg -> llgg0000
    lower_bytes[..., 3] <<= 6  # ggggggll -> ll000000

    # shift all bytes to the right, so lsb rubbish gets deleted
    # hhhhhhhh llgggggg -> 000000hh hhhhhhll
    bayer >>= 6
    return bayer.reshape((h, (w * 4) // 5))


def binning(x: npt.NDArray, factor: int = 4) -> npt.NDArray:
    """Apply pixel binning to an image."""
    h, w = x.shape
    return cv2.resize(x, (w // factor, h // factor), interpolation=cv2.INTER_AREA)

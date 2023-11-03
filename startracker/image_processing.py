# TODO cleanup add docstrings
import numpy as np
import cv2


def decode_sbggr12_1x12(raw: np.ndarray):
    """
    Decode SBGGR12 format (12bit per pixel).

    3 bytes hold data for 2 pixels.
    - byte 0: pixel 0 upper 8bit
    - byte 1: pixel 1 upper 8bit
    - byte 2: pixel 0 lower 4bit | pixel 1 lower 4bit
    """
    h, w = raw.shape
    w = w // 3
    raw = raw[:, : w * 3].reshape((h, w, 3)).astype(np.uint16)
    a = raw[..., 0]
    c = raw[..., 2]
    a *= 0x10
    a += c & 0x0F
    c //= 16
    b = raw[..., 1]
    b *= 16
    b += c
    return np.stack((a, b), axis=-1).reshape((h, w * 2))


def decode_srggb10(raw: np.ndarray):
    """
    Decode SRGGB10 format (10bit per pixel).

    5 bytes hold data for 4 pixels.
    - byte 0: pixel 0 upper 8bit
    - byte 1: pixel 1 upper 8bit
    - byte 2: pixel 2 upper 8bit
    - byte 3: pixel 3 upper 8bit
    - byte 4: pixel 0 lower 4bit | pixel 1 lower 4bit | pixel 2 lower 4bit | pixel 3 lower 4bit
    """
    h, w = raw.shape
    raw = raw.reshape((h, w // 5, 5))
    bayer = np.zeros((h, w // 5, 4), np.uint16)
    bayer[..., :4] = raw[..., :4]
    bayer *= 4
    bayer[..., 0] += (raw[..., 4] & 0xC0) >> 6
    bayer[..., 1] += (raw[..., 4] & 0x30) >> 4
    bayer[..., 2] += (raw[..., 4] & 0x0C) >> 2
    bayer[..., 3] += raw[..., 4] & 0x03
    return bayer.reshape((h, (w * 4) // 5))


def extract_green(bayer: np.ndarray):
    res = bayer[::2, 1::2] + bayer[1::2, ::2]
    res //= 2
    return res


def binning(x: np.ndarray, factor: int = 4):
    h, w = x.shape
    return cv2.resize(x, (w // factor, h // factor), interpolation=cv2.INTER_AREA)

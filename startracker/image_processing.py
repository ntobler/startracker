#TODO cleanup add docstrings
import numpy as np
import cv2

def decode_sbggr12_1x12(x):
    h, w = x.shape
    w = w // 3
    x = x[:, : w * 3].reshape((h, w, 3)).astype(np.uint16)
    a = x[..., 0]
    c = x[..., 2]
    a *= 0x10
    a += c & 0x0F
    c //= 16
    b = x[..., 1]
    b *= 16
    b += c
    return np.stack((a, b), axis=-1).reshape((h, w * 2))


def extract_green(x):
    res = x[::2, 1::2] + x[1::2, ::2]
    res //= 2
    return res


def binning(x, factor=4):
    h, w = x.shape
    return cv2.resize(x, (w // factor, h // factor), interpolation=cv2.INTER_AREA)

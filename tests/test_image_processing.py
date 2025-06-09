import numpy as np
import pytest

from startracker import image_processing


def test_decode_sbggr12_1x12_random():
    rng = np.random.default_rng()
    raw = rng.integers(0, 256, (4, 15), dtype=np.uint8)
    out = image_processing.decode_sbggr12_1x12(raw)
    assert out.shape == (4, 10)
    assert out.dtype == np.uint16


def test_decode_sbggr12_1x12_numbers():
    raw = np.array(
        [
            [255, 0, 0],
            [0, 255, 0],
            [0, 0, 0xF0],
            [0, 0, 0x0F],
        ],
        dtype=np.uint8,
    )

    expected = np.array(
        [
            [255 << 4, 0],
            [0, 255 << 4],
            [15, 0],
            [0, 15],
        ],
        dtype=np.uint16,
    )
    out = image_processing.decode_sbggr12_1x12(raw)
    np.testing.assert_array_equal(out, expected)


def test_decode_srggb10_random():
    rng = np.random.default_rng()
    raw = rng.integers(0, 256, (4, 15), dtype=np.uint8)
    out = image_processing.decode_srggb10(raw)
    assert out.shape == (4, 12)
    assert out.dtype == np.uint16


def test_decode_srggb10_numbers():
    raw = np.array(
        [
            [255, 0, 0, 0, 0],
            [0, 255, 0, 0, 0],
            [0, 0, 255, 0, 0],
            [0, 0, 0, 255, 0],
            [0, 0, 0, 0, 0xC0],
            [0, 0, 0, 0, 0x30],
            [0, 0, 0, 0, 0x0C],
            [0, 0, 0, 0, 0x03],
        ],
        dtype=np.uint8,
    )

    expected = np.array(
        [
            [255 << 2, 0, 0, 0],
            [0, 255 << 2, 0, 0],
            [0, 0, 255 << 2, 0],
            [0, 0, 0, 255 << 2],
            [3, 0, 0, 0],
            [0, 3, 0, 0],
            [0, 0, 3, 0],
            [0, 0, 0, 3],
        ],
        dtype=np.uint16,
    )
    out = image_processing.decode_srggb10(raw)
    np.testing.assert_array_equal(out, expected)


def test_binning_2():
    raw = np.array(
        [
            [255, 5, 4, 5],
            [0, 255, 8, 7],
            [255, 5, 4, 5],
            [0, 255, 8, 7],
        ],
        dtype=np.uint8,
    )

    expected = np.array(
        [
            [255 + 5 + 255, 4 + 5 + 8 + 7],
            [255 + 5 + 255, 4 + 5 + 8 + 7],
        ],
        dtype=np.uint16,
    )
    expected = np.round(expected / 4).astype(np.uint8)
    out = image_processing.binning(raw, 2)
    np.testing.assert_array_equal(out, expected)


def test_binning_4():
    raw = np.array(
        [
            [255, 5, 4, 5],
            [0, 255, 8, 7],
            [255, 5, 4, 5],
            [0, 255, 8, 7],
        ],
        dtype=np.uint8,
    )

    expected = np.round(raw.mean(keepdims=True))
    out = image_processing.binning(raw, 4)
    np.testing.assert_array_equal(out, expected)


if __name__ == "__main__":
    pytest.main([__file__])

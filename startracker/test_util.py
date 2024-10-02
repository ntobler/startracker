"""Util module tests."""

import time

import pytest

from startracker import util


def test_max_rate():
    epsilon = 0.02
    t = time.monotonic()
    with util.max_rate(10):
        _ = 10
    tdiff = time.monotonic() - t
    assert tdiff >= 1 / 10 - epsilon


if __name__ == "__main__":
    pytest.main([__file__])

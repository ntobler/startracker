"""Util module tests."""

import time

from . import util


def test_max_rate():
    epsilon = 0.02
    t = time.monotonic()
    with util.max_rate(10):
        _ = 10
    tdiff = time.monotonic() - t
    assert tdiff >= 1 / 10 - epsilon

"""Protocel module tests."""

import pathlib
import tempfile

import pytest

from startracker import protocol


def test_protocol():
    with tempfile.TemporaryDirectory() as d:
        file = pathlib.Path(d) / "proctocol.c"
        protocol.generate_code(file)
        assert file.exists()


if __name__ == "__main__":
    pytest.main([__file__])

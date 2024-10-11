"""Util module tests."""

import dataclasses
import time

import pytest

from startracker import util


def test_pickle_dataclass(tmp_path):
    @dataclasses.dataclass
    class Foo(util.PickleDataclass):
        a: int = 5

    assert Foo().to_dict() == {"a": 5}
    assert Foo(a=6) == Foo.from_dict({"a": 6})

    a = Foo(a=6)
    file = tmp_path / "file.pkl"
    a.save(file)
    assert file.exists()
    assert a == Foo.load(file)


def test_max_rate():
    epsilon = 0.02
    t = time.monotonic()
    with util.max_rate(10):
        _ = 10
    tdiff = time.monotonic() - t
    assert tdiff >= 1 / 10 - epsilon


if __name__ == "__main__":
    pytest.main([__file__])

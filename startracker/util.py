"""General utility functions."""

import contextlib
import dataclasses
import pathlib
import pickle
import time

from typing_extensions import Self


@dataclasses.dataclass
class PickleDataclass:
    """Baseclass for picklable dataclasses."""

    def to_dict(self) -> dict:
        """Return dictionary of the object."""
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, dictionary: dict) -> Self:
        """Create object from a dictionary."""
        return cls(**dictionary)

    def save(self, filename: pathlib.Path) -> None:
        """Save the object to a file using pickle."""
        with filename.open("wb") as f:
            pickle.dump(self.to_dict(), f)

    @classmethod
    def load(cls, filename: pathlib.Path) -> Self:
        """Load the object from a file using pickle."""
        with filename.open("rb") as f:
            obj = cls.from_dict(pickle.load(f))
        return obj


@contextlib.contextmanager
def max_rate(max_rate_hz: float):
    """Pad a block of code with sleep, such that maximum rate is not exceeded."""
    t = time.monotonic() + 1 / max_rate_hz
    yield
    if (diff := t - time.monotonic()) > 0:
        time.sleep(diff)


class TimeMeasurer:
    """Context manager to measure execution time."""

    def __init__(self):
        self.t = 0

    def __enter__(self, *args, **kwargs):
        self._t0 = time.monotonic()
        return self

    def __exit__(self, *args, **kwargs):
        self.t = time.monotonic() - self._t0

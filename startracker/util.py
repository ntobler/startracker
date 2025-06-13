"""General utility functions."""

import contextlib
import dataclasses
import json
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
        field_names = {f.name for f in dataclasses.fields(cls)}
        filtered_dict = {k: v for k, v in dictionary.items() if k in field_names}
        return cls(**filtered_dict)

    def save(self, filename: pathlib.Path) -> None:
        """Save the object to a file using pickle."""
        with filename.open("wb") as f:
            pickle.dump(self.to_dict(), f)

    @classmethod
    def load(cls, filename: pathlib.Path, *, construct_default_if_missing: bool = False) -> Self:
        """Load the object from a file using pickle."""
        if construct_default_if_missing is not None and not filename.is_file():
            return cls()
        with filename.open("rb") as f:
            obj = cls.from_dict(pickle.load(f))
        return obj


@dataclasses.dataclass
class JsonDataclass:
    """Baseclass for picklable dataclasses."""

    def to_dict(self) -> dict:
        """Return dictionary of the object."""
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, dictionary: dict) -> Self:
        """Create object from a dictionary."""
        field_names = {f.name for f in dataclasses.fields(cls)}
        filtered_dict = {k: v for k, v in dictionary.items() if k in field_names}
        return cls(**filtered_dict)

    def save(self, filename: pathlib.Path) -> None:
        """Save the object to a file using pickle."""
        with filename.open("w") as f:
            json.dump(self.to_dict(), f)

    @classmethod
    def load(cls, filename: pathlib.Path, *, construct_default_if_missing: bool = False) -> Self:
        """Load the object from a file using pickle."""
        if construct_default_if_missing is not None and not filename.is_file():
            return cls()
        with filename.open("rb") as f:
            obj = cls.from_dict(json.load(f))
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

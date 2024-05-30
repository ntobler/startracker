"""Main entrypoint for the startracker application."""

import abc
import dataclasses
import logging
import pathlib
import pickle
from typing import Union

import numpy as np


@dataclasses.dataclass
class CameraSettings:
    exposure_ms: float = 100
    """Exposure value in miliseconds."""
    digital_gain: int = 1
    """Digital gain factor 1, 2 or 4."""
    analog_gain: int = 0
    """Analog gain value."""
    stack: int = 1
    """Number of image to accumulate"""
    binning: int = 1
    """Binning factor 1, 2, 4 or 8"""
    bias: Union[None, int, np.ndarray] = 55
    """Bias value or bias frame i.e. the value subtracted to correct black level."""
    darkframe_averaging: int = 4
    """Number of exposure to average when recording darkframe."""

    def validate(self):
        if self.digital_gain not in [1, 2, 4]:
            raise ValueError("digital_gain must be 1, 2, or 4")
        if self.analog_gain not in [1, 2, 3, 4, 5]:
            raise ValueError("analog_gain must be between 1 and 5")
        if 1 > self.stack > 16:
            raise ValueError("stack must be between 1 and 15")
        if self.binning not in [1, 2, 4, 8]:
            raise ValueError("binning must be 1, 2 or 4")
        if self.stack * self.digital_gain > 16:
            raise ValueError("stack and binning gain must be below 16 due to overflow reasons")

    def to_dict(self):
        """Return dictionary of the object."""
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, dictionary: dict):
        """Create object from a dictionary."""
        return cls(**dictionary)

    def save(self, filename: pathlib.Path):
        with open(filename, "wb") as f:
            pickle.dump(self.to_dict(), f)

    @classmethod
    def load(cls, filename: pathlib.Path):
        with open(filename, "wb") as f:
            obj = cls.from_dict(pickle.load(f))
        return obj


class Camera(abc.ABC):
    """Camera base class."""

    def __init__(self, camera_settings: CameraSettings):
        self._logger = logging.getLogger("Camera")
        self._settings = camera_settings
        self._context_manager_entered = False

    @property
    def settings(self) -> CameraSettings:
        return self._settings

    @settings.setter
    def settings(self, value: CameraSettings):
        self._check_context_manager()
        value.validate()
        self._settings = value
        self._apply_settings()

    def _apply_settings(self):
        return None

    def __enter__(self):
        self._context_manager_entered = True

    def __exit__(self, type, value, traceback):
        self._context_manager_entered = False

    def _check_context_manager(self):
        if not self._context_manager_entered:
            raise RuntimeError("Context manager not entered")

    @abc.abstractmethod
    def capture_raw(self): ...

    @abc.abstractmethod
    def capture(self) -> np.ndarray: ...

    @abc.abstractmethod
    def record_darkframe(self): ...


class MockCamera(Camera):
    """Mock Camera implementation to replace the Raspberry Pi camera if not available."""

    def capture_raw(self):
        frame = np.zeros((1080, 1920), np.uint16)
        frame[32:-32, 32:-32] = 100
        return frame

    def capture(self) -> np.ndarray:
        frame = np.zeros((540, 960), np.uint8)
        frame[32:-32, 32:-32] = 100 + np.random.default_rng().integers(-20, 20)
        return frame

    def record_darkframe(self):
        pass

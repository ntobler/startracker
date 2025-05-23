"""Main entrypoint for the startracker application."""

import abc
import dataclasses
import logging
from typing import Union

import numpy as np
import numpy.typing as npt
from typing_extensions import override

from startracker import util


@dataclasses.dataclass
class CameraSettings(util.PickleDataclass):
    exposure_ms: float = 100
    """Exposure value in milliseconds."""
    digital_gain: int = 1
    """Digital gain factor 1, 2 or 4."""
    analog_gain: int = 1
    """Analog gain value."""
    stack: int = 1
    """Number of image to accumulate"""
    binning: int = 2
    """Binning factor 1, 2, 4 or 8"""
    bias: Union[None, int, np.ndarray] = 55
    """Bias value or bias frame i.e. the value subtracted to correct black level."""
    darkframe_averaging: int = 4
    """Number of exposure to average when recording darkframe."""

    def __post_init__(self) -> None:
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


class Camera(abc.ABC):
    """Camera base class."""

    def __init__(self, camera_settings: CameraSettings) -> None:
        self._logger = logging.getLogger("Camera")
        self._settings = camera_settings
        self._context_manager_entered = False

    @property
    def settings(self) -> CameraSettings:
        """Camera settings instance."""
        return self._settings

    @settings.setter
    def settings(self, value: CameraSettings) -> None:
        self._check_context_manager()
        self._settings = value
        self._apply_settings()

    @abc.abstractmethod
    def _apply_settings(self) -> None: ...

    def __enter__(self):
        self._context_manager_entered = True

    def __exit__(self, type, value, traceback):
        self._context_manager_entered = False

    def _check_context_manager(self):
        if not self._context_manager_entered:
            raise RuntimeError("Context manager not entered")

    @abc.abstractmethod
    def capture_raw(self) -> npt.NDArray[np.uint16]:
        """Capture a raw image.

        Returns:
            np.ndarray: uint16 bayer image
        """

    @abc.abstractmethod
    def capture(self) -> npt.NDArray[np.uint8]:
        """Capture corrected, potentially stacked and binned image.

        Returns:
            np.ndarray: uint8 image
        """

    @abc.abstractmethod
    def record_darkframe(self) -> None:
        """Record a dark frame for bias correction and store it to the settings."""


class MockCamera(Camera):
    """Mock Camera implementation to replace the Raspberry Pi camera if not available."""

    @override
    def capture_raw(self) -> npt.NDArray[np.uint16]:
        frame = np.zeros((1080, 1920), np.uint16)
        frame[32:-32, 32:-32] = 100
        return frame

    @override
    def capture(self) -> npt.NDArray[np.uint8]:
        frame = np.zeros((540, 960), np.uint8)
        frame[32:-32, 32:-32] = 100 + np.random.default_rng().integers(-20, 20)
        return frame

    @override
    def record_darkframe(self) -> None:
        pass

    @override
    def _apply_settings(self) -> None:
        pass

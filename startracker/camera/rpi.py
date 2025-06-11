"""Raspberry Pi camera implementations."""

import threading
import time

import cv2
import numpy as np
import numpy.typing as npt
from picamera2 import Picamera2  # type: ignore
from picamera2.sensor_format import SensorFormat  # type: ignore
from typing_extensions import override

from startracker import image_processing
from startracker.camera import camera


class RpiCamera(camera.Camera):
    """Arducam IMX462 implementation.

    exposure_ms = 10000  # 10s (but 50s then 20s, 20s ...)
    exposure_ms = 5000  # 50s (but 25s then 10s, 10s ...)
    max exposure time: 667244877 nanoseconds?
    """

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)
        self._lock = threading.Lock()
        self._exposure_ms = -1
        self._analog_gain = -1

    @override
    def _apply_settings(self):
        if self._exposure_ms != self.settings.exposure_ms:
            self._exposure_ms = self.settings.exposure_ms
            self._logger.info("Set ExposureTime")
            with self._lock:
                self._picam2.set_controls({"ExposureTime": int(self._exposure_ms * 1000)})
        if self._analog_gain != self.settings.analog_gain:
            self._analog_gain = self.settings.analog_gain
            self._logger.info("Set AnalogueGain")
            with self._lock:
                self._picam2.set_controls({"AnalogueGain": int(self._analog_gain)})

    def __enter__(self):
        super().__enter__()
        with self._lock:
            self._logger.info("Init camera")
            self._picam2 = Picamera2()

            # Configure an unpacked raw format as these are easier to add.
            raw_format = SensorFormat(self._picam2.sensor_format)
            raw_format.packing = None
            self._logger.info(f"Raw format: {raw_format}")
            self._logger.info(f"Raw format: {raw_format.__dict__}")

            # Set buffer size to 2. One for the last frame and the new one
            config = self._picam2.create_still_configuration(buffer_count=2, queue=True)
            self._picam2.configure(config)

            self._logger.info("Start camera")
            self._picam2.start()

            self._logger.info("Set initial exposure and gain")

        self._apply_settings()

    def __exit__(self, type, value, traceback):
        super().__exit__(type, value, traceback)
        with self._lock:
            self._picam2.stop()

    @override
    def capture_raw(self, *, flush: bool = False) -> npt.NDArray[np.uint16]:
        self._check_context_manager()
        self._logger.info("Capturing image ...")
        t0 = time.monotonic()
        with self._lock:
            request = self._picam2.capture_request(flush=flush)
            try:
                timestamp = request.get_metadata()["SensorTimestamp"] / 1e9
                raw = request.make_array("raw")
            finally:
                request.release()
            self.capture_time = timestamp - (self._exposure_ms / 1000 / 2)
        self._logger.info(f"Capturing took: {time.monotonic() - t0:.2f}s")
        bayer = image_processing.decode_srggb10(raw)
        return bayer

    @override
    def capture(self, *, flush: bool = False) -> npt.NDArray[np.uint8]:
        image = None
        for _ in range(self.settings.stack):
            # Only flush the first capture
            raw = self.capture_raw(flush=flush and image is None)

            # Correct bias
            if self.settings.bias is not None:
                cv2.subtract(raw, self.settings.bias, dst=raw)  # type: ignore

            if image is None:
                image = raw
            else:
                image += raw

        if image is None:
            raise ValueError("settings.stack must be at least 1.")

        binning = self.settings.binning
        if binning in [2, 4, 8]:
            image = image_processing.binning(image, factor=binning)

        if self.settings.digital_gain in [1, 2]:
            image //= 4 // self.settings.digital_gain

        return image.astype(np.uint8)

    @override
    def record_darkframe(self) -> None:
        darkframe = None
        for _ in range(self.settings.darkframe_averaging):
            if darkframe is None:
                darkframe = self.capture_raw(flush=True)
            else:
                darkframe += self.capture_raw(flush=False)
        if darkframe is None:
            raise ValueError("settings.darkframe_averaging must be at least 1.")
        darkframe //= self.settings.darkframe_averaging
        self.settings.bias = darkframe

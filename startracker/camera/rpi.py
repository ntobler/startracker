"""Raspberry Pi camera implemenations."""

import time
import threading

import cv2
import numpy as np

from picamera2 import Picamera2
from picamera2.sensor_format import SensorFormat

from startracker import image_processing
from startracker.camera import camera


class RpiCamera(camera.Camera):
    """
    Arducam IMX462 implementation.

    exposure_ms = 10000  # 10s (but 50s then 20s, 20s ...)
    exposure_ms = 5000  # 50s (but 25s then 10s, 10s ...)
    max exposure time: 667244877 nanoseconds?
    """

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)
        self._lock = threading.Lock()
        self._exposure_ms = 0
        self._analog_gain = 0

    def _apply_settings(self):
        if self._exposure_ms != self.settings.exposure_ms:
            self._exposure_ms = self.settings.exposure_ms
            self._picam2.set_controls({"ExposureTime": int(self._exposure_ms * 1000)})
        if self._analog_gain != self.settings.analog_gain:
            self._analog_gain = self.settings.analog_gain
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

            config = self._picam2.create_still_configuration(
                buffer_count=1, queue=False
            )
            self._picam2.configure(config)

            self._logger.info("Set inital exposure and gain")
            self._picam2.set_controls(
                {
                    "ExposureTime": int(self._exposure_ms * 1000),
                    "AnalogueGain": int(self._analog_gain),
                }
            )

            self._picam2.start()
            self._logger.info("Started")

    def __exit__(self, type, value, traceback):
        super().__exit__(type, value, traceback)
        with self._lock:
            self._picam2.stop()

    def capture_raw(self):
        """
        Capture a raw image.

        Returns:
            np.ndarray: uint16 bayer image
        """
        self._check_context_manager()
        with self._lock:
            t0 = time.time()
            self._logger.info("Taking image")
            raw = self._picam2.capture_array("raw")
            self._logger.info(f"Capturing took:{time.time() - t0:.2f}s")
            bayer = image_processing.decode_srggb10(raw)
        return bayer

    def capture(self) -> np.ndarray:
        """
        Capture corrected, potentially stacked and binned image.

        Returns:
            np.ndarray: uint8 image
        """
        image = None
        for _ in range(self.settings.stack):
            raw = self.capture_raw()

            # Correct bias
            if self.settings.bias is not None:
                cv2.subtract(raw, self.settings.bias, dst=raw)

            if image is None:
                image = raw
            else:
                image += raw

        binning = self.settings.binning
        if binning in [2, 4, 8]:
            image = image_processing.binning(image, factor=binning)

        if self.settings.digital_gain in [1, 2]:
            image //= 4 // self.settings.digital_gain
        image = image.astype(np.uint8)

        return image

    def record_darkframe(self):
        """Record a darkframe for bias correciton and store it to the settings."""
        darkframe = None
        for _ in range(self.settings.darkframe_averaging):
            if darkframe is None:
                darkframe = self.capture_raw()
            else:
                darkframe += self.capture_raw()
        darkframe //= self.settings.darkframe_averaging
        self.settings.bias = darkframe

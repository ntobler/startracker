"""Main entrypoint for the startracker application."""
import numpy as np
import time
from picamera2 import Picamera2
from picamera2.sensor_format import SensorFormat
import threading
import logging

import image_processing


class Camera:
    """
    Raspberry pi camera wrapper.

    exposure_ms = 10000  # 10s (but 50s then 20s, 20s ...)
    exposure_ms = 5000  # 50s (but 25s then 10s, 10s ...)
    max exposure time: 667244877 nanoseconds?

    """

    NUM_STACK_FRAMES = 2  # must be <= 16 to prevent integer overflow

    def __init__(self, exposure_ms: int):
        self._lock = threading.Lock()
        self._logger = logging.getLogger("Camera")

        self._exposure_ms = exposure_ms
        self._gain = 0

        self._context_manager_entered = False

    def __enter__(self):
        self._context_manager_entered = True
        with self._lock:
            self._logger.info("Init camera")
            self._picam2 = Picamera2()

            # Configure an unpacked raw format as these are easier to add.
            raw_format = SensorFormat(self._picam2.sensor_format)
            raw_format.packing = None
            print(raw_format)
            print(raw_format.__dict__)

            config = self._picam2.create_still_configuration(
                buffer_count=1, queue=False
            )
            self._picam2.configure(config)

            self._logger.info("Set exposure")
            self._picam2.set_controls(
                {
                    "ExposureTime": int(self._exposure_ms * 1000),
                    "AnalogueGain": int(self._gain),
                }
            )

            self._picam2.start()
            self._logger.info("Started")

    def __exit__(self, type, value, traceback):
        with self._lock:
            self._picam2.stop()

    @property
    def exposure_ms(self):
        return self._exposure_ms

    @exposure_ms.setter
    def exposure_ms(self, value):
        assert self._context_manager_entered
        if self._exposure_ms != value:
            self._exposure_ms = value
            self._picam2.set_controls({"ExposureTime": int(self._exposure_ms * 1000)})

    @property
    def gain(self):
        return self._gain

    @gain.setter
    def gain(self, value):
        assert self._context_manager_entered
        if self._gain != value:
            self._gain = value
            self._picam2.set_controls({"AnalogueGain": int(value)})

    def capture_raw(self):
        """
        Capture a raw image.

        Returns:
            np.ndarray: uint16 bayer image
        """
        assert self._context_manager_entered
        with self._lock:
            t0 = time.time()
            self._logger.info(f"Taking image")
            raw = self._picam2.capture_array("raw")
            self._logger.info(f"Capturing took:{time.time() - t0:.2f}s")
            bayer = image_processing.decode_srggb10(raw)
        return bayer

    def capture_stacked(self) -> np.ndarray:
        """
        Capture a stacked image.

        Returns:
            np.ndarray: uint16 accumulated bayer image
        """
        assert self._context_manager_entered
        with self._lock:
            accumulated_bayer = None
            for i in range(self.NUM_STACK_FRAMES):
                t0 = time.time()
                self._logger.info(f"Taking image: {i} of {self.NUM_STACK_FRAMES}")
                raw = self._picam2.capture_array("raw")
                self._logger.info(f"Capturing took:{time.time() - t0:.2f}s")
                bayer = image_processing.decode_sbggr12_1x12(raw)
                if accumulated_bayer is None:
                    accumulated_bayer = bayer
                else:
                    accumulated_bayer += bayer
        return accumulated_bayer

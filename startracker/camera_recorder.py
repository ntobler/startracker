"""Utility tool to capture images indefinitely in a loop."""

from datetime import datetime
import logging
import time

import cv2

from . import camera
from . import persistent


class CameraRecorder:
    def __init__(self):
        self._logger = logging.getLogger("Camera")

    def capture(
        self,
        exposure_ms: float = 250,
        analog_gain: int = 4,
    ):
        self._logger.info(f"Capture e={exposure_ms} g={analog_gain}")
        self._cam.exposure_ms = exposure_ms
        self._cam.gain = analog_gain

        image = self._cam.capture_raw()
        self._logger.info(f"Raw bayer shape={image.shape} dtype={image.dtype}")

        self._logger.info(f"Output shape={image.shape} dtype={image.dtype}")

        dir = persistent.Persistent.get_instance().calibration_dir
        time_str = datetime.now().strftime("%Y%m%dT%H%M%S_%f")

        file = dir.parent / f"images/image_{time_str}.png"
        cv2.imwrite(str(file), image)

        self._image_cache = image

    def run(self):
        try:
            self._cam = camera.Camera(exposure_ms=1)
            with self._cam:
                while True:
                    time.sleep(2)
                    self.capture()
        except KeyboardInterrupt:
            pass


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    main = CameraRecorder()
    main.run()


if __name__ == "__main__":
    main()

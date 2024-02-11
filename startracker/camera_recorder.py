"""Utility tool to capture images indefinitely in a loop."""
import argparse
from datetime import datetime
import logging
import pathlib
import time

import cv2

from startracker import camera


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", "o", type=str, required=False, default=".")
    parser.add_argument("interval", "i", type=float, required=False, default=2.0)
    parser.add_argument("exposure", "e", type=float, required=False, default=30.0)
    parser.add_argument("analog_gain", "a", type=int, required=False, default=1)
    parser.add_argument("number", "n", type=int, required=False, default=1000)
    parser.add_argument("binning", "b", type=int, required=False, default=1)
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)

    settings = camera.CameraSettings(
        exposure_ms=args.exposure,
        analog_gain=args.analog_gain,
        binning=args.binning,
    )

    try:
        cam = camera.RpiCamera(settings)
        with cam:
            while True:
                time.sleep(args.interval)
                image = cam.capture()
                time_str = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
                file = output_dir / f"image_{time_str}.png"
                cv2.imwrite(str(file), image)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

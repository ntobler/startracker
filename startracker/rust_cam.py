"""Scratch script to debug rust camera access."""

import cv2

from startracker import libstartracker


def rust_cam() -> None:
    """Get a camera image with rust."""
    config = libstartracker.CameraConfig(1, 4, 100000, 2)
    cam = libstartracker.Camera(config)
    for i in range(4):
        config = libstartracker.CameraConfig(1, int(2**i), 100000, 2)
        cam.set_config(config)
        frame = cam.capture()
        cv2.imwrite(f"test{i}.png", frame)


if __name__ == "__main__":
    rust_cam()

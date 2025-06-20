"""Scratch script to debug rust camera access."""

import cv2

from startracker import libstartracker


def rust_cam() -> None:
    """Get a camera image with rust."""
    config = libstartracker.CameraConfig(1, 4, 100000, 2)
    cam = libstartracker.Camera(config)
    for b in [1, 2]:
        config = libstartracker.CameraConfig(1, 2, 100000, b)
        cam.set_config(config)
        [cam.capture() for _ in range(3)]
        frame = cam.capture()
        cv2.imwrite(f"test_binning_{b}.png", frame)

    for d in range(4):
        config = libstartracker.CameraConfig(1, int(2**d), 100000, 2)
        cam.set_config(config)
        [cam.capture() for _ in range(3)]
        frame = cam.capture()
        cv2.imwrite(f"test_digital_gain_{d}.png", frame)

    for e in [10, 30, 100, 300]:
        config = libstartracker.CameraConfig(1, 2, e * 1000, 2)
        cam.set_config(config)
        [cam.capture() for _ in range(3)]
        frame = cam.capture()
        cv2.imwrite(f"test_exposure_{e}.png", frame)

    for a in [1, 2, 3, 4]:
        config = libstartracker.CameraConfig(a, 2, 100000, 2)
        cam.set_config(config)
        [cam.capture() for _ in range(3)]
        frame = cam.capture()
        cv2.imwrite(f"test_analog_{a}.png", frame)


if __name__ == "__main__":
    rust_cam()

"""Scratch script to debug rust camera access."""

import cv2
import numpy as np

from startracker import libstartracker


def rust_cam() -> None:
    """Get a camera image with rust."""
    frame = libstartracker.get_camera_frame()
    frame = frame.reshape((1080, 1920))
    cv2.imwrite("test.png", (frame // 4).astype(np.uint8))

    cam = libstartracker.Camera()
    for i in range(4):
        frame = cam.capture()
        frame = frame.reshape((1080, 1920))
        cv2.imwrite(f"test{i}.png", (frame // 4).astype(np.uint8))


if __name__ == "__main__":
    rust_cam()

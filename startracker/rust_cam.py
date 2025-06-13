"""Scratch script to debug rust camera access."""

import cv2

from startracker import libstartracker


def rust_cam() -> None:
    """Get a camera image with rust."""
    frame = libstartracker.get_camera_frame()

    frame = frame.reshape((1080, 1920, 2))

    cv2.imwrite("test1.png", frame[..., 0])
    cv2.imwrite("test2.png", frame[..., 1])


if __name__ == "__main__":
    rust_cam()

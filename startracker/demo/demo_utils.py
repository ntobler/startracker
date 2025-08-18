"""Utility functions for the demo module."""

import pathlib

import cv2
import numpy as np
import PIL.Image


class VideoCreator:
    """Convert frames to a video."""

    def __init__(self):
        """Initialize."""
        self.frames = []

    def push(self, image: np.ndarray) -> None:
        """Push image to frame stack."""
        self.frames.append(image.copy())

    def save_webp(self, file: pathlib.Path, frame_rate: float = 5.0) -> None:
        """Save .webp video."""
        pil_frames = [PIL.Image.fromarray(x) for x in self.frames]
        # Save as animated WebP
        pil_frames[0].save(
            file,
            format="WEBP",
            save_all=True,
            append_images=pil_frames[1:],
            duration=int(1000 / frame_rate),  # milliseconds per frame
            loop=0,  # loop forever
        )

    def save_mp4(self, file: pathlib.Path, frame_rate: float = 5.0) -> None:
        """Save .mp4 video."""
        # Ensure there are frames to write
        if not self.frames:
            raise ValueError("No frames to write to video.")
        height, width = self.frames[0].shape[:2]
        # OpenCV expects (width, height) for VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # type: ignore[attr-defined]
        out = cv2.VideoWriter(str(file), fourcc, frame_rate, (width, height))
        for frame in self.frames:
            # Ensure frame is 3-channel BGR and uint8 for VideoWriter
            if frame.shape[2] == 4:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif frame.shape[2] == 1:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame
            # Ensure dtype is uint8
            if frame_bgr.dtype != np.uint8:
                frame_bgr = frame_bgr.astype(np.uint8)
            out.write(frame_bgr)
        out.release()

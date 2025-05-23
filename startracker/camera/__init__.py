"""Camera abstraction.

Includes raspberry pi camera and a mock implementation that runs in a development environment.
"""

from startracker.camera.camera import Camera, CameraSettings

try:
    from startracker.camera.rpi import RpiCamera
except ImportError:
    from startracker.camera.camera import MockCamera as RpiCamera

__all__ = [
    "Camera",
    "CameraSettings",
    "RpiCamera",
]

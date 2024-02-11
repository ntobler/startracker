"""
Camera abstraction.

Incudes raspberry pi camera and a mock implementation that runs in a development environment.
"""

from startracker.camera.camera import CameraSettings

try:
    from startracker.camera.rpi import RpiCamera
except ImportError:
    from startracker.camera.camera import MockCamera as RpiCamera

"""
Camera abstraction.

Incudes raspberry pi camera and a mock implementation that runs in a development environment.
"""

try:
    from camera.camera import Camera
except ImportError:
    from camera.mock_camera import MockCamera as Camera

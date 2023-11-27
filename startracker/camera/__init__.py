"""
Camera abstraction.

Incudes raspberry pi camera and a mock implementation that runs in a development environment.
"""

try:
    from startracker.camera.camera import Camera
except ImportError:
    from startracker.camera.mock_camera import MockCamera as Camera

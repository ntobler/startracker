"""Mock Camera implementation to replace the Raspberry Pi camera if not available"""
import numpy as np


# TODO make baseclass to derive from
class MockCamera:
    exposure_ms: float

    def __init__(self, exposure_ms: int):
        pass

    def __enter__(self):
        pass

    def __exit__(self, type, value, traceback):
        pass

    def capture_raw(self):
        return np.zeros((1080, 1920), np.uint16)

    def capture_stacked(self) -> np.ndarray:
        return np.zeros((1080, 1920), np.uint16)
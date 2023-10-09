#TODO make baseclass to derive from
import numpy as np


class MockCamera:
    exposure_ms: float

    def __init__(self, exposure_ms: int):
        pass

    def __enter__(self):
        pass

    def __exit__(self, type, value, traceback):
        pass

    def capture_raw(self):
        return np.zeros((300, 200), np.uint8)

    def capture_stacked(self) -> np.ndarray:
        return np.zeros((300, 200), np.uint8)

    def capture_auto_brightness(self):
        pass

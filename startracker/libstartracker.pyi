import numpy as np

class CalibrationResult:
    @property
    def params(self) -> list[float]: ...
    @property
    def rms_error(self) -> float: ...

def calibrate(
    image_points: np.ndarray,
    object_points: np.ndarray,
    image_size: tuple[float, float],
    *,
    intrinsic_guess: np.ndarray | None = None,
    dist_coefs_guess: np.ndarray | None = None,
) -> CalibrationResult: ...
def objective_function(
    params: tuple[float, ...],
    image_point: np.ndarray,
    object_point: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]: ...

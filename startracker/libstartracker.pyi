import numpy as np
import numpy.typing as npt

class CalibrationResult:
    @property
    def params(self) -> list[float]: ...
    @property
    def rms_error(self) -> float: ...

def starcal_calibrate(
    image_points: npt.NDArray[np.float32],
    object_points: npt.NDArray[np.float32],
    image_size: tuple[float, float],
    *,
    intrinsic_guess: npt.NDArray[np.float64] | None = None,
    dist_coefs_guess: npt.NDArray[np.float64] | None = None,
    tol: float | None = None,
    max_iter: float | None = None,
) -> CalibrationResult: ...
def starcal_objective_function(
    params: tuple[float, ...],
    image_point: npt.NDArray[np.float32],
    object_point: npt.NDArray[np.float32],
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]: ...
def stargradcal_calibrate(
    image_points: npt.NDArray[np.float32],
    image_gradients: npt.NDArray[np.float32],
    intrinsic_guess: npt.NDArray[np.float64],
    dist_coef_guess: float,
    theta_guess: float,
    epsilon_guess: float,
    tol: float,
    max_iter: int,
) -> tuple[npt.NDArray[np.float64], float, float, float, npt.NDArray[np.float64]]: ...
def stargradcal_objective_function(
    image_points: npt.NDArray[np.float32],
    image_gradients: npt.NDArray[np.float32],
    intrinsic_guess: npt.NDArray[np.float64],
    dist_coef_guess: float,
    theta_guess: float,
    epsilon_guess: float,
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]: ...
def even_spaced_indices(
    points: npt.NDArray[np.float32],
    n_samples: int,
    rng_seed: int,
) -> npt.NDArray[np.uint64]: ...
def get_camera_frame() -> npt.NDArray[np.uint8]: ...

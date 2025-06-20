use numpy::{self, PyArrayMethods, PyUntypedArrayMethods, ToPyArray};
use pyo3::Python;
use pyo3::{
    exceptions::PyRuntimeError, pyclass, pyfunction, pymethods, pymodule, types::PyModule,
    wrap_pyfunction, Bound, PyResult,
};
use std::usize;

use crate::optim::OptimizableProblem;

mod optim;
mod poisson_disk;
mod starcal;
mod stargradcal;
mod util;

#[pymodule]
fn libstartracker(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(starcal_calibrate, m)?)?;
    m.add_function(wrap_pyfunction!(starcal_objective_function, m)?)?;
    m.add_function(wrap_pyfunction!(stargradcal_calibrate, m)?)?;
    m.add_function(wrap_pyfunction!(stargradcal_objective_function, m)?)?;
    m.add_function(wrap_pyfunction!(even_spaced_indices, m)?)?;
    m.add_class::<CalibrationResult>()?;
    Ok(())
}

#[pyclass]
pub struct CalibrationResult {
    inner: starcal::CalibrationResult,
}

#[pymethods]
impl CalibrationResult {
    #[getter]
    pub fn params(&self) -> Vec<f64> {
        self.inner.params.to_vec()
    }

    #[getter]
    pub fn rms_error(&self) -> f64 {
        self.inner.rms_error
    }

    // Prevent construction from Python
    #[new]
    fn new() -> PyResult<Self> {
        Err(PyRuntimeError::new_err(
            "CalibrationResult cannot be constructed from Python",
        ))
    }
}

#[pyfunction]
fn starcal_objective_function<'py>(
    py: Python<'py>,
    params: [f64; starcal::PARAM_COUNT],
    image_points: numpy::PyReadonlyArray2<'py, f32>,
    object_points: numpy::PyReadonlyArray2<'py, f32>,
) -> (
    Bound<'py, numpy::PyArray1<f64>>,
    Bound<'py, numpy::PyArray2<f64>>,
) {
    let image_points_view = numpy_to_dynamic_slice(&image_points).unwrap();
    let object_points_view = numpy_to_dynamic_slice(&object_points).unwrap();
    let mut problem: starcal::CameraCalibrationProblem<'_> =
        starcal::CameraCalibrationProblem::new(&params, image_points_view, object_points_view);

    problem.calc_jacobian();
    problem.calc_residuals();

    let residuals = problem.get_residuals().as_slice().to_pyarray_bound(py);
    let jacobian_flat = problem.get_jacobian().as_slice().to_pyarray_bound(py);
    let jacobian = jacobian_flat
        .reshape((
            problem.get_jacobian().ncols(),
            problem.get_jacobian().nrows(),
        ))
        .expect("Shape mismatch in jacobian reshape");
    (residuals, jacobian)
}

#[pyfunction]
fn starcal_calibrate<'py>(
    image_points: numpy::PyReadonlyArray2<'py, f32>,
    object_points: numpy::PyReadonlyArray2<'py, f32>,
    image_size: [usize; 2],
    intrinsic_guess: Option<numpy::PyReadonlyArray2<'py, f64>>,
    dist_coefs_guess: Option<numpy::PyReadonlyArray1<'py, f64>>,
) -> PyResult<CalibrationResult> {
    let image_points_view = numpy_to_dynamic_slice(&image_points).unwrap();
    let object_points_view = numpy_to_dynamic_slice(&object_points).unwrap();

    let intrinsic: [[f64; 3]; 3] = if let Some(guess) = intrinsic_guess {
        if guess.shape() != [3, 3] {
            return Err(PyRuntimeError::new_err(
                "intrinsic_guess must have shape [3, 3]",
            ));
        }
        let slice = guess.as_slice()?;
        [
            [slice[0], 0.0, slice[2]],
            [0.0, slice[4], slice[5]],
            [0.0, 0.0, 1.0],
        ]
    } else {
        let width = image_size[0] as f64;
        let height = image_size[1] as f64;
        let min_z = object_points_view
            .iter()
            .map(|p| p[2])
            .fold(f32::INFINITY, f32::min) as f64;
        let focal_length =
            f64::sqrt(width * width + height * height) / f64::tan(f64::acos(min_z)) / 2.0;
        [
            [focal_length, 0.0, width / 2.0],
            [0.0, focal_length, height / 2.0],
            [0.0, 0.0, 1.0],
        ]
    };
    let dist_coefs: [f64; 5] = if let Some(guess) = dist_coefs_guess {
        let slice = guess.as_slice()?;
        if slice.len() != 5 {
            return Err(PyRuntimeError::new_err(
                "dist_coefs_guess must have length 5",
            ));
        }
        [slice[0], slice[1], slice[2], slice[3], slice[4]]
    } else {
        [0.0, 0.0, 0.0, 0.0, 0.0]
    };

    let result = starcal::calibrate(
        image_points_view,
        object_points_view,
        &intrinsic,
        &dist_coefs,
    );

    Ok(CalibrationResult { inner: result })
}

#[pyfunction]
fn stargradcal_objective_function<'py>(
    py: Python<'py>,
    image_points: numpy::PyReadonlyArray2<'py, f32>,
    image_gradients: numpy::PyReadonlyArray2<'py, f32>,
    intrinsic_guess: numpy::PyReadonlyArray2<'py, f64>,
    dist_coef_guess: f64,
    theta_guess: f64,
    epsilon_guess: f64,
) -> PyResult<(
    Bound<'py, numpy::PyArray1<f64>>,
    Bound<'py, numpy::PyArray2<f64>>,
)> {
    let image_points_view = numpy_to_dynamic_slice(&image_points).unwrap();
    let image_gradients_view = numpy_to_dynamic_slice(&image_gradients).unwrap();

    if intrinsic_guess.shape() != [3, 3] {
        return Err(PyRuntimeError::new_err(
            "intrinsic_guess must have shape [3, 3]",
        ));
    }
    let slice = intrinsic_guess.as_slice()?;
    let intrinsic = [
        [slice[0], slice[1], slice[2]],
        [slice[3], slice[4], slice[5]],
        [slice[6], slice[7], slice[8]],
    ];

    let params = [
        f64::tan(epsilon_guess * 0.25),
        f64::tan(theta_guess * 0.25),
        intrinsic[0][0],
        intrinsic[0][2],
        intrinsic[1][1],
        intrinsic[1][2],
        dist_coef_guess,
    ];

    let mut problem: stargradcal::StarGradCalibrationProblem<'_> =
        stargradcal::StarGradCalibrationProblem::new(
            &params,
            image_points_view,
            image_gradients_view,
        );

    problem.calc_jacobian();
    problem.calc_residuals();

    let residuals = problem.get_residuals().as_slice().to_pyarray_bound(py);
    let jacobian_flat = problem.get_jacobian().as_slice().to_pyarray_bound(py);
    let jacobian = jacobian_flat
        .reshape((
            problem.get_jacobian().ncols(),
            problem.get_jacobian().nrows(),
        ))
        .expect("Shape mismatch in jacobian reshape");
    Ok((residuals, jacobian))
}

#[pyfunction]
fn stargradcal_calibrate<'py>(
    py: Python<'py>,
    image_points: numpy::PyReadonlyArray2<'py, f32>,
    image_gradients: numpy::PyReadonlyArray2<'py, f32>,
    intrinsic_guess: numpy::PyReadonlyArray2<'py, f64>,
    dist_coef_guess: f64,
    theta_guess: f64,
    epsilon_guess: f64,
) -> PyResult<(
    Bound<'py, numpy::PyArray2<f64>>,
    f64,
    f64,
    f64,
    Bound<'py, numpy::PyArray1<f64>>,
)> {
    let image_points_view = numpy_to_dynamic_slice(&image_points).unwrap();
    let image_gradients_view = numpy_to_dynamic_slice(&image_gradients).unwrap();

    if intrinsic_guess.shape() != [3, 3] {
        return Err(PyRuntimeError::new_err(
            "intrinsic_guess must have shape [3, 3]",
        ));
    }
    let slice = intrinsic_guess.as_slice()?;
    let intrinsic = [
        [slice[0], 0.0, slice[2]],
        [0.0, slice[4], slice[5]],
        [0.0, 0.0, 1.0],
    ];

    let (intrinsic, dist_coef, theta, epsilon, residuals) = stargradcal::calibrate(
        image_points_view,
        image_gradients_view,
        &intrinsic,
        dist_coef_guess,
        theta_guess,
        epsilon_guess,
    );

    let intrinsic_numpy_flat = intrinsic.as_slice().to_pyarray_bound(py);
    let residuals_numpy = residuals.as_slice().to_pyarray_bound(py);
    // nalgebra is row-major, numpy expects column-major, so we need to transpose
    let intrinsic_numpy = intrinsic_numpy_flat
        .reshape((3, 3))
        .expect("Shape mismatch in intrinsic reshape");

    Ok((intrinsic_numpy, dist_coef, theta, epsilon, residuals_numpy))
}

#[pyfunction]
fn even_spaced_indices<'py>(
    py: Python<'py>,
    points: numpy::PyReadonlyArray2<'py, f32>,
    n_samples: usize,
    rng_seed: u64,
) -> Bound<'py, numpy::PyArray1<usize>> {
    let points_view = numpy_to_dynamic_slice(&points).unwrap();
    let accepted_indices = poisson_disk::even_spaced_indices(&points_view, n_samples, rng_seed);
    accepted_indices.as_slice().to_pyarray_bound(py)
}

fn numpy_to_dynamic_slice<'py, T: numpy::Element + Copy, const L: usize>(
    vectors: &'py numpy::PyReadonlyArray2<'py, T>,
) -> PyResult<&'py [[T; L]]> {
    if !vectors.is_c_contiguous() || vectors.ndim() != 2 || vectors.shape()[1] != L {
        return Err(PyRuntimeError::new_err(format!(
            "vectors must be a c_contiguous array with shape=[n, {}]",
            L
        )));
    }
    Ok(util::as_slice_of_arrays(vectors.as_slice()?).unwrap())
}

pub trait OptimizableProblem<const PARAM_COUNT: usize> {
    fn calc_residuals(&mut self);
    fn calc_jacobian(&mut self);
    fn get_params(&self) -> &nalgebra::SVector<f64, PARAM_COUNT>;
    fn set_params(&mut self, params: nalgebra::SVector<f64, PARAM_COUNT>);
    fn get_residuals(&self) -> &nalgebra::DVector<f64>;
    fn get_jacobian(&self) -> &nalgebra::DMatrix<f64>;
}

pub fn levenberg_marquardt<P: OptimizableProblem<PARAM_COUNT>, const PARAM_COUNT: usize>(
    problem: &mut P,
    max_iter: usize,
    tol: f64,
    lambda_init: f64,
) {
    // Levenberg-Marquardt solver for nonlinear least squares problems.
    let mut lamb = lambda_init;
    for _ in 0..max_iter {
        problem.calc_residuals();
        problem.calc_jacobian();

        let residuals = problem.get_residuals();
        let jacobian_transposed = problem.get_jacobian();

        let cost: f64 = residuals.as_slice().iter().map(|x| x * x).sum();
        let jtj = jacobian_transposed * jacobian_transposed.transpose();
        let jtr = jacobian_transposed * residuals;
        let n_rows_rows = jtj.nrows();
        let a = jtj + nalgebra::DMatrix::identity(n_rows_rows, n_rows_rows) * lamb;

        let lu = nalgebra::LU::new(a);
        let delta = lu.solve(&(-&jtr)).expect("Matrix may be singular");

        let old_params = problem.get_params().clone();

        problem.set_params(problem.get_params() + &delta);

        problem.calc_residuals();
        let residuals = problem.get_residuals();

        let new_cost: f64 = residuals.as_slice().iter().map(|x| x * x).sum();
        if new_cost < cost * 1.001 {
            // Epsilon to avoid numerical issues
            // Accept step, decrease lambda
            lamb /= 10.0;

            // Check convergence absolute
            if delta.norm() < tol {
                break;
            }
        } else {
            // Reject step
            problem.set_params(old_params);
            //increase lambda
            lamb *= 10.0;
        }
    }
}

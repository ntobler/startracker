use core::f32;
use nalgebra;

pub struct CalibrationResult {
    pub params: [f64; 10],
    pub rms_error: f64,
}

pub struct Problem<'a> {
    params: nalgebra::SVector<f64, 10>,
    image_points: &'a [[f32; 2]],
    object_points: &'a [[f32; 3]],
    pub residuals: nalgebra::DVector<f64>,
    pub jacobian_transposed: nalgebra::DMatrix<f64>,
}

impl<'a> Problem<'a> {
    pub fn new(
        params: &[f64; 10],
        image_points: &'a [[f32; 2]],
        object_points: &'a [[f32; 3]],
    ) -> Self {
        let num_points = image_points.len();
        let residuals = nalgebra::DVector::zeros(num_points * 2);
        let jacobian_transposed = nalgebra::DMatrix::zeros(10, num_points * 2);
        Problem {
            params: nalgebra::SVector::<f64, 10>::from_row_slice(params),
            image_points,
            object_points,
            residuals,
            jacobian_transposed,
        }
    }

    pub fn calc_residuals(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = 2;
        let res_slice = self.residuals.as_mut_slice();

        for i in 0..self.image_points.len() {
            let object_point = &self.object_points[i];
            let image_point = &self.image_points[i];

            let fx = params[0];
            let sh = params[1];
            let tx = params[2];
            let fy = params[3];
            let ty = params[4];

            let k1 = params[5];
            let k2 = params[6];
            let p1 = params[7];
            let p2 = params[8];
            let k3 = params[9];

            let obj_x = object_point[0] as f64;
            let obj_y = object_point[1] as f64;
            let obj_z = object_point[2] as f64;

            let img_x = image_point[0] as f64;
            let img_y = image_point[1] as f64;

            //// project object points
            //let x = (fx * obj_x + sh * obj_y) / (obj_z * fx);
            //let y = obj_y / obj_z;
            //// distort in the centered image frame
            //let r2 = x * x + y * y;
            //let r4 = r2 * r2;
            //let r6 = r2 * r4;
            //let d = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            //let x_dist = x * d + (2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x));
            //let y_dist = y * d + (2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y));
            //// translate to image frame and take difference to image points
            //let rx = img_x - (x_dist * fx) - tx;
            //let ry = img_y - (y_dist * fy) - ty;

            // Reusable subexpressions:
            let x0 = 1.0 / (obj_z * obj_z);
            let x1 = fx * obj_x + obj_y * sh;
            let x2 = x0 * x1;
            let x3 = 2.0 * x2;
            let x4 = obj_y * x3;
            let x5 = 1.0 / fx;
            let x6 = p1 * x5;
            let x7 = obj_y * obj_y * x0;
            let x8 = 1.0 / (fx * fx);
            let x9 = x0 * x1 * x1;
            let x10 = x8 * x9;
            let x11 = 3.0 * x10 + x7;
            let x12 = 1.0 / obj_z;
            let x13 = x1 * x12;
            let x14 = x10 + x7;
            let x15 = x14 * x14;
            let x16 = x14 * x15;
            let x17 = k1 * x14 + k2 * x15 + k3 * x16 + 1.0;
            let x18 = x17 * x5;
            let x19 = p2 * x11 + x13 * x18 + x4 * x6;
            let x20 = p2 * x5;
            let x21 = x10 + 3.0 * x7;
            let x22 = obj_y * x12;
            let x23 = x17 * x22;
            let x24 = p1 * x21 + x20 * x4 + x23;

            // value entries
            res_slice[i * stride] = -fx * x19 + img_x - tx;
            res_slice[i * stride + 1] = -fy * x24 + img_y - ty;
        }
    }

    pub fn calc_jaccobian(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = self.jacobian_transposed.nrows() * 2;
        let jac_slice = self.jacobian_transposed.as_mut_slice();

        for i in 0..self.image_points.len() {
            let object_point = &self.object_points[i];

            let fx = params[0];
            let sh = params[1];
            let fy = params[3];

            let k1 = params[5];
            let k2 = params[6];
            let p1 = params[7];
            let p2 = params[8];
            let k3 = params[9];

            let obj_x = object_point[0] as f64;
            let obj_y = object_point[1] as f64;
            let obj_z = object_point[2] as f64;

            //// project object points
            //let x = (fx * obj_x + sh * obj_y) / (obj_z * fx);
            //let y = obj_y / obj_z;
            //// distort in the centered image frame
            //let r2 = x * x + y * y;
            //let r4 = r2 * r2;
            //let r6 = r2 * r4;
            //let d = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            //let x_dist = x * d + (2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x));
            //let y_dist = y * d + (2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y));
            //// translate to image frame and take difference to image points
            //let rx = img_x - (x_dist * fx) - tx;
            //let ry = img_y - (y_dist * fy) - ty;

            // Reusable subexpressions:
            let x0 = 1.0 / (obj_z * obj_z);
            let x1 = fx * obj_x + obj_y * sh;
            let x2 = x0 * x1;
            let x3 = 2.0 * x2;
            let x4 = obj_y * x3;
            let x5 = 1.0 / fx;
            let x6 = p1 * x5;
            let x7 = obj_y * obj_y * x0;
            let x8 = 1.0 / (fx * fx);
            let x9 = x0 * x1 * x1;
            let x10 = x8 * x9;
            let x11 = 3.0 * x10 + x7;
            let x12 = 1.0 / obj_z;
            let x13 = x1 * x12;
            let x14 = x10 + x7;
            let x15 = x14 * x14;
            let x16 = x14 * x15;
            let x17 = k1 * x14 + k2 * x15 + k3 * x16 + 1.0;
            let x18 = x17 * x5;
            let x19 = p2 * x11 + x13 * x18 + x4 * x6;
            let x20 = p2 * x5;
            let x21 = x10 + 3.0 * x7;
            let x22 = obj_y * x12;
            let x23 = x17 * x22;
            let x24 = p1 * x21 + x20 * x4 + x23;
            let x25 = 2.0 * obj_x * obj_y * x0;
            let x26 = x4 * x8;
            let x27 = p1 * x26;
            let x28 = obj_x * x8;
            let x29 = x2 * x28;
            let x30 = x9 / (fx * fx * fx);
            let x31 = 6.0 * (x29 - x30);
            let x32 = x28 * x3 - 2.0 * x30;
            let x33 = k2 * x14;
            let x34 = k3 * x15;
            let x35 = k1 * x32 + x31 * x34 + x33 * (4.0 * x29 - 4.0 * x30);
            let x36 = x13 * x5;
            let x37 = 2.0 * x7;
            let x38 = obj_y * x2 * x8;
            let x39 = 6.0 * x38;
            let x40 = k1 * x26 + 4.0 * x33 * x38 + x34 * x39;
            let x41 = fy * x22;

            // jacobian entries (∂rx/∂params..., ∂ry/∂params...):
            jac_slice[i * stride + 0] = -fx
                * (obj_x * x12 * x18 + p2 * x31 - x13 * x17 * x8 + x25 * x6 - x27 + x35 * x36)
                - x19;
            jac_slice[i * stride + 1] = -fx * (p2 * x39 + x23 * x5 + x36 * x40 + x37 * x6);
            jac_slice[i * stride + 2] = -1.0;
            jac_slice[i * stride + 3] = 0.0;
            jac_slice[i * stride + 4] = 0.0;
            jac_slice[i * stride + 5] = -x13 * x14;
            jac_slice[i * stride + 6] = -x13 * x15;
            jac_slice[i * stride + 7] = -x4;
            jac_slice[i * stride + 8] = -fx * x11;
            jac_slice[i * stride + 9] = -x13 * x16;
            jac_slice[i * stride + 10] = -fy * (p1 * x32 - p2 * x26 + x20 * x25 + x22 * x35);
            jac_slice[i * stride + 11] = -fy * (x20 * x37 + x22 * x40 + x27);
            jac_slice[i * stride + 12] = 0.0;
            jac_slice[i * stride + 13] = -x24;
            jac_slice[i * stride + 14] = -1.0;
            jac_slice[i * stride + 15] = -x14 * x41;
            jac_slice[i * stride + 16] = -x15 * x41;
            jac_slice[i * stride + 17] = -fy * x21;
            jac_slice[i * stride + 18] = -fy * x4 * x5;
            jac_slice[i * stride + 19] = -x16 * x41;
        }
    }

    fn solve(&mut self, max_iter: usize, tol: f64, lambda_init: f64) -> () {
        //Levenberg-Marquardt solver for nonlinear least squares problems.
        //
        //Args:
        //    fun: Objective function to minimize, should return residuals.
        //    x0: Initial guess for the parameters.
        //    args: Additional arguments to pass to the objective function.
        //    max_iter: Maximum number of iterations.
        //    tol: Absolute tolerance for convergence.
        //    lambda_init: Initial value for the damping parameter.
        //
        //Returns:
        //    The optimized parameters.
        let mut lamb = lambda_init;
        for _ in 0..max_iter {
            self.calc_residuals();
            self.calc_jaccobian();
            let cost: f64 = self.residuals.as_slice().iter().map(|x| x * x).sum();
            let jtj = &self.jacobian_transposed * &self.jacobian_transposed.transpose();
            let jtr = &self.jacobian_transposed * &self.residuals;
            let n_rows_rows = jtj.nrows();
            let a = jtj + nalgebra::DMatrix::identity(n_rows_rows, n_rows_rows) * lamb;

            let lu = nalgebra::LU::new(a);
            let delta = lu.solve(&(-&jtr)).expect("Matrix may be singular");

            let old_params = self.params.clone();

            self.params += &delta;
            self.calc_residuals();
            let new_cost: f64 = self.residuals.as_slice().iter().map(|x| x * x).sum();
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
                self.params = old_params;
                //increase lambda
                lamb *= 10.0;
            }
        }
    }
}

pub fn calibrate(
    image_points: &[[f32; 2]],
    object_points: &[[f32; 3]],
    intrinsic: &[[f64; 3]; 3],
    dist_coefs: &[f64; 5],
) -> CalibrationResult {
    let initial_params = [
        intrinsic[0][0],
        intrinsic[0][1],
        intrinsic[0][2],
        intrinsic[1][1],
        intrinsic[1][2],
        dist_coefs[0],
        dist_coefs[1],
        dist_coefs[2],
        dist_coefs[3],
        dist_coefs[4],
    ];

    // Solve optimization problem with Levenberg-Marquardt algorithm
    let mut problem = Problem::new(&initial_params, image_points, object_points);
    problem.solve(10, 1e-4, 1e-3);

    let rms_error = problem.residuals.map(|x| x * x).mean().sqrt();
    CalibrationResult {
        params: problem
            .params
            .as_slice()
            .try_into()
            .expect("Invalid parameter length"),
        rms_error,
    }
}

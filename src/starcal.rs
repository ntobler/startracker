use core::f32;
use nalgebra;

pub const PARAM_COUNT: usize = 13;

pub struct CalibrationResult {
    pub params: [f64; PARAM_COUNT],
    pub rms_error: f64,
}

pub struct Problem<'a> {
    params: nalgebra::SVector<f64, PARAM_COUNT>,
    image_points: &'a [[f32; 2]],
    object_points: &'a [[f32; 3]],
    pub residuals: nalgebra::DVector<f64>,
    pub jacobian_transposed: nalgebra::DMatrix<f64>,
}

impl<'a> Problem<'a> {
    pub fn new(
        params: &[f64; PARAM_COUNT],
        image_points: &'a [[f32; 2]],
        object_points: &'a [[f32; 3]],
    ) -> Self {
        let num_points = image_points.len();
        let residuals = nalgebra::DVector::zeros(num_points * 2);
        let jacobian_transposed = nalgebra::DMatrix::zeros(PARAM_COUNT, num_points * 2);
        Problem {
            params: nalgebra::SVector::<f64, PARAM_COUNT>::from_row_slice(params),
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

            let mrp_0 = params[0];
            let mrp_1 = params[1];
            let mrp_2 = params[2];

            let fx = params[3];
            let sh = params[4];
            let tx = params[5];
            let fy = params[6];
            let ty = params[7];

            let k1 = params[8];
            let k2 = params[9];
            let p1 = params[10];
            let p2 = params[11];
            let k3 = params[12];

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
            let x0 = mrp_0 * mrp_0;
            let x1 = 8.0 * x0;
            let x2 = mrp_1 * mrp_1;
            let x3 = 8.0 * x2;
            let x4 = -x1 - x3;
            let x5 = mrp_2 * mrp_2;
            let x6 = x0 + x2 + x5 + 1.0;
            let x7 = 1.0 / (x6 * x6);
            let x8 = 8.0 * mrp_2;
            let x9 = mrp_0 * x8;
            let x10 = 4.0 * x0;
            let x11 = 4.0 * x2;
            let x12 = 4.0 * x5;
            let x13 = x11 + x12 - 4.0;
            let x14 = -x10 - x13;
            let x15 = mrp_1 * x14;
            let x16 = x15 + x9;
            let x17 = obj_x * x7;
            let x18 = 8.0 * mrp_1;
            let x19 = mrp_2 * x18;
            let x20 = -x19;
            let x21 = mrp_0 * x14;
            let x22 = -x20 - x21;
            let x23 = obj_y * x7;
            let x24 = obj_z * (x4 * x7 + 1.0) + x16 * x17 + x22 * x23;
            let x25 = 1.0 / x24;
            let x26 = 8.0 * x5;
            let x27 = -x26 - x3;
            let x28 = mrp_0 * x18;
            let x29 = mrp_2 * x14;
            let x30 = x28 + x29;
            let x31 = -x15 + x9;
            let x32 = obj_z * x7;
            let x33 = obj_x * (x27 * x7 + 1.0) + x23 * x30 + x31 * x32;
            let x34 = -x1 - x26;
            let x35 = x19 + x21;
            let x36 = x28 - x29;
            let x37 = obj_y * (x34 * x7 + 1.0) + x17 * x36 + x32 * x35;
            let x38 = fx * x33 + sh * x37 + tx * x24;
            let x39 = -tx + x25 * x38;
            let x40 = 1.0 / fx;
            let x41 = x39 * x40;
            let x42 = fy * x37 + ty * x24;
            let x43 = -ty + x25 * x42;
            let x44 = 1.0 / fy;
            let x45 = x43 * x44;
            let x46 = 2.0 * x45;
            let x47 = x41 * x46;
            let x48 = x44 * x44;
            let x49 = x43 * x43;
            let x50 = x48 * x49;
            let x51 = x40 * x40;
            let x52 = x39 * x39;
            let x53 = x51 * x52;
            let x54 = x50 + 3.0 * x53;
            let x55 = x50 + x53;
            let x56 = x55 * x55;
            let x57 = x55 * x55 * x55;
            let x58 = k1 * x55 + k2 * x56 + k3 * x57 + 1.0;
            let x59 = p1 * x47 + p2 * x54 + x41 * x58;
            let x60 = 3.0 * x50 + x53;
            let x61 = p1 * x60 + p2 * x47 + x45 * x58;

            // value entries
            let base = i * stride;
            res_slice[base + 0] = -fx * x59 + img_x - tx;
            res_slice[base + 1] = -fy * x61 + img_y - ty;
        }
    }

    pub fn calc_jaccobian(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = self.jacobian_transposed.nrows() * 2;
        let jac_slice = self.jacobian_transposed.as_mut_slice();

        for i in 0..self.image_points.len() {
            let object_point = &self.object_points[i];

            let mrp_0 = params[0];
            let mrp_1 = params[1];
            let mrp_2 = params[2];

            let fx = params[3];
            let sh = params[4];
            let tx = params[5];
            let fy = params[6];
            let ty = params[7];

            let k1 = params[8];
            let k2 = params[9];
            let p1 = params[10];
            let p2 = params[11];
            let k3 = params[12];

            let obj_x = object_point[0] as f64;
            let obj_y = object_point[1] as f64;
            let obj_z = object_point[2] as f64;

            // Reusable subexpressions:
            let x0 = mrp_0 * mrp_0;
            let x1 = 8.0 * x0;
            let x2 = mrp_1 * mrp_1;
            let x3 = 8.0 * x2;
            let x4 = -x1 - x3;
            let x5 = mrp_2 * mrp_2;
            let x6 = x0 + x2 + x5 + 1.0;
            let x7 = 1.0 / (x6 * x6);
            let x8 = 8.0 * mrp_2;
            let x9 = mrp_0 * x8;
            let x10 = 4.0 * x0;
            let x11 = 4.0 * x2;
            let x12 = 4.0 * x5;
            let x13 = x11 + x12 - 4.0;
            let x14 = -x10 - x13;
            let x15 = mrp_1 * x14;
            let x16 = x15 + x9;
            let x17 = obj_x * x7;
            let x18 = 8.0 * mrp_1;
            let x19 = mrp_2 * x18;
            let x20 = -x19;
            let x21 = mrp_0 * x14;
            let x22 = -x20 - x21;
            let x23 = obj_y * x7;
            let x24 = obj_z * (x4 * x7 + 1.0) + x16 * x17 + x22 * x23;
            let x25 = 1.0 / x24;
            let x26 = 8.0 * x5;
            let x27 = -x26 - x3;
            let x28 = mrp_0 * x18;
            let x29 = mrp_2 * x14;
            let x30 = x28 + x29;
            let x31 = -x15 + x9;
            let x32 = obj_z * x7;
            let x33 = obj_x * (x27 * x7 + 1.0) + x23 * x30 + x31 * x32;
            let x34 = -x1 - x26;
            let x35 = x19 + x21;
            let x36 = x28 - x29;
            let x37 = obj_y * (x34 * x7 + 1.0) + x17 * x36 + x32 * x35;
            let x38 = fx * x33 + sh * x37 + tx * x24;
            let x39 = -tx + x25 * x38;
            let x40 = 1.0 / fx;
            let x41 = x39 * x40;
            let x42 = fy * x37 + ty * x24;
            let x43 = -ty + x25 * x42;
            let x44 = 1.0 / fy;
            let x45 = x43 * x44;
            let x46 = 2.0 * x45;
            let x47 = x41 * x46;
            let x48 = x44 * x44;
            let x49 = x43 * x43;
            let x50 = x48 * x49;
            let x51 = x40 * x40;
            let x52 = x39 * x39;
            let x53 = x51 * x52;
            let x54 = x50 + 3.0 * x53;
            let x55 = x50 + x53;
            let x56 = x55 * x55;
            let x57 = x55 * x55 * x55;
            let x58 = k1 * x55 + k2 * x56 + k3 * x57 + 1.0;
            let x59 = p1 * x47 + p2 * x54 + x41 * x58;
            let x60 = 3.0 * x50 + x53;
            let x61 = p1 * x60 + p2 * x47 + x45 * x58;
            let x62 = x18 + x9;
            let x63 = 12.0 * x0 + x13;
            let x64 = 4.0 / (x6 * x6 * x6);
            let x65 = mrp_0 * x64;
            let x66 = obj_z * x65;
            let x67 = obj_x * x65;
            let x68 = 16.0 * x7;
            let x69 = mrp_0 * x68;
            let x70 = obj_y * (-x34 * x65 - x69) + x17 * x62 - x32 * x63 - x35 * x66 - x36 * x67;
            let x71 = 8.0 * mrp_2 - x28;
            let x72 = obj_y * x65;
            let x73 = obj_z * (-x4 * x65 - x69) - x16 * x67 + x17 * x71 - x22 * x72 + x23 * x63;
            let x74 = x25 * (fy * x70 + ty * x73);
            let x75 = 1.0 / (x24 * x24);
            let x76 = -x73 * x75;
            let x77 = x42 * x76;
            let x78 = x74 + x77;
            let x79 = 2.0 * x41;
            let x80 = p1 * x79;
            let x81 = x44 * x80;
            let x82 = x38 * x76;
            let x83 = 8.0 * mrp_1 - x9;
            let x84 = x28 + x8;
            let x85 = x25
                * (fx * (obj_y * x7 * x83 + obj_z * x7 * x84 - x27 * x67 - x30 * x72 - x31 * x66)
                    + sh * x70
                    + tx * x73);
            let x86 = x82 + x85;
            let x87 = p1 * x46;
            let x88 = x40 * x87;
            let x89 = x43 * x48;
            let x90 = 2.0 * x89 * (x74 + x77);
            let x91 = x39 * x51;
            let x92 = 2.0 * x91 * (x82 + x85);
            let x93 = 3.0 * x92;
            let x94 = x40 * x58;
            let x95 = k2 * x55;
            let x96 = 3.0 * x90;
            let x97 = k3 * x56;
            let x98 = k1 * (x90 + x92) + 2.0 * x95 * (x90 + x92) + x97 * (x93 + x96);
            let x99 = 8.0 * mrp_0;
            let x100 = x19 + x99;
            let x101 = mrp_1 * x64;
            let x102 = obj_y * x101;
            let x103 = obj_z * x101;
            let x104 = obj_x * x101;
            let x105 = obj_x * x100 * x7 + obj_z * x7 * x71 - x102 * x34 - x103 * x35 - x104 * x36;
            let x106 = x10 - 4.0;
            let x107 = x106 + x12 + 12.0 * x2;
            let x108 = mrp_1 * x68;
            let x109 =
                obj_z * (-x101 * x4 - x108) - x102 * x22 - x104 * x16 - x107 * x17 + x23 * x84;
            let x110 = x25 * (fy * x105 + ty * x109);
            let x111 = -x109 * x75;
            let x112 = x111 * x42;
            let x113 = x110 + x112;
            let x114 = x111 * x38;
            let x115 = x20 + x99;
            let x116 = x25
                * (fx
                    * (obj_x * (-x101 * x27 - x108) - x102 * x30 - x103 * x31
                        + x107 * x32
                        + x115 * x23)
                    + sh * x105
                    + tx * x109);
            let x117 = x114 + x116;
            let x118 = 2.0 * x89 * (x110 + x112);
            let x119 = 2.0 * x91 * (x114 + x116);
            let x120 = 3.0 * x119;
            let x121 = 3.0 * x118;
            let x122 = k1 * (x118 + x119) + 2.0 * x95 * (x118 + x119) + x97 * (x120 + x121);
            let x123 = mrp_2 * x64;
            let x124 = obj_z * x123;
            let x125 = obj_x * x123;
            let x126 = obj_y * x123;
            let x127 = -obj_x * x115 * x7 - obj_y * x62 * x7 + x124 * x4 + x125 * x16 + x126 * x22;
            let x128 = -x127;
            let x129 = x106 + x11 + 12.0 * x5;
            let x130 = mrp_2 * x68;
            let x131 =
                obj_y * (-x123 * x34 - x130) - x124 * x35 - x125 * x36 + x129 * x17 + x32 * x83;
            let x132 = x25 * (fy * x131 + ty * x128);
            let x133 = x127 * x75;
            let x134 = x133 * x42;
            let x135 = x132 + x134;
            let x136 = x133 * x38;
            let x137 = x25
                * (fx
                    * (obj_x * (-x123 * x27 - x130) + x100 * x32
                        - x124 * x31
                        - x126 * x30
                        - x129 * x23)
                    + sh * x131
                    + tx * x128);
            let x138 = x136 + x137;
            let x139 = 2.0 * x89 * (x132 + x134);
            let x140 = 2.0 * x91 * (x136 + x137);
            let x141 = 3.0 * x140;
            let x142 = 3.0 * x139;
            let x143 = k1 * (x139 + x140) + 2.0 * x95 * (x139 + x140) + x97 * (x141 + x142);
            let x144 = x25 * x33;
            let x145 = x52 / (fx * fx * fx);
            let x146 = 6.0 * x91;
            let x147 = x144 * x146 - 6.0 * x145;
            let x148 = x25 * x94;
            let x149 = x144 * x91;
            let x150 = -2.0 * x145 + 2.0 * x149;
            let x151 = k1 * x150 + x147 * x97 + x95 * (-4.0 * x145 + 4.0 * x149);
            let x152 = x25 * x37;
            let x153 = p2 * x152;
            let x154 = 2.0 * x152 * x91;
            let x155 = x39 * x55;
            let x156 = x152 * x51;
            let x157 = x39 * x56;
            let x158 = k1 * x154 + 4.0 * k2 * x155 * x156 + 6.0 * k3 * x156 * x157;
            let x159 = x49 / (fy * fy * fy);
            let x160 = x152 * x89;
            let x161 = -2.0 * x159 + 2.0 * x160;
            let x162 = -6.0 * x159 + 6.0 * x160;
            let x163 = k1 * x161 + x162 * x97 + 4.0 * x95 * (-x159 + x160);
            let x164 = p2 * x79;
            let x165 = x164 * x44;
            let x166 = p2 * x46;
            let x167 = x166 * x40;
            let x168 = x44 * x58;

            // jacobian entries
            let base = i * stride;
            jac_slice[base + 0] =
                -fx * (p2 * (x90 + x93) + x41 * x98 + x78 * x81 + x86 * x88 + x86 * x94);
            jac_slice[base + 1] =
                -fx * (p2 * (x118 + x120) + x113 * x81 + x117 * x88 + x117 * x94 + x122 * x41);
            jac_slice[base + 2] =
                -fx * (p2 * (x139 + x141) + x135 * x81 + x138 * x88 + x138 * x94 + x143 * x41);
            jac_slice[base + 3] = -fx
                * (p2 * x147 + x144 * x88 + x148 * x33 + x151 * x41 - x58 * x91 - x87 * x91)
                - x59;
            jac_slice[base + 4] = -fx * (x146 * x153 + x148 * x37 + x152 * x88 + x158 * x41);
            jac_slice[base + 5] = -1.0;
            jac_slice[base + 6] = -fx * (p2 * x161 + x152 * x81 + x163 * x41 - x80 * x89);
            jac_slice[base + 7] = 0.0;
            jac_slice[base + 8] = -x155;
            jac_slice[base + 9] = -x157;
            jac_slice[base + 10] = -x39 * x46;
            jac_slice[base + 11] = -fx * x54;
            jac_slice[base + 12] = -x39 * x57;
            jac_slice[base + 13] =
                -fy * (p1 * (x92 + x96) + x165 * x78 + x167 * x86 + x168 * x78 + x45 * x98);
            jac_slice[base + 14] =
                -fy * (p1 * (x119 + x121) + x113 * x165 + x113 * x168 + x117 * x167 + x122 * x45);
            jac_slice[base + 15] =
                -fy * (p1 * (x140 + x142) + x135 * x165 + x135 * x168 + x138 * x167 + x143 * x45);
            jac_slice[base + 16] = -fy * (p1 * x150 + x144 * x167 + x151 * x45 - x166 * x91);
            jac_slice[base + 17] = -fy * (p1 * x154 + x153 * x40 * x46 + x158 * x45);
            jac_slice[base + 18] = 0.0;
            jac_slice[base + 19] = -fy
                * (p1 * x162 + x152 * x168 + x153 * x44 * x79 + x163 * x45
                    - x164 * x89
                    - x58 * x89)
                - x61;
            jac_slice[base + 20] = -1.0;
            jac_slice[base + 21] = -x43 * x55;
            jac_slice[base + 22] = -x43 * x56;
            jac_slice[base + 23] = -fy * x60;
            jac_slice[base + 24] = -x43 * x79;
            jac_slice[base + 25] = -x43 * x57;
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
        0.0,
        0.0,
        0.0,
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

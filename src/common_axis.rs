use nalgebra;

use crate::optim::OptimizableProblem;

pub const PARAM_COUNT: usize = 5;
pub const RESIDUAL_COUNT: usize = 3;

pub struct CommonAxisProblem<'a> {
    params: nalgebra::SVector<f64, PARAM_COUNT>,
    rot_mats: &'a [[f64; 9]],
    residuals: nalgebra::DVector<f64>,
    jacobian_transposed: nalgebra::DMatrix<f64>,
}

impl<'a> CommonAxisProblem<'a> {
    pub fn new(params: &[f64; PARAM_COUNT], rot_mats: &'a [[f64; 9]]) -> Self {
        let num_points = rot_mats.len();
        let residuals = nalgebra::DVector::zeros(num_points * RESIDUAL_COUNT);
        let jacobian_transposed =
            nalgebra::DMatrix::zeros(PARAM_COUNT, num_points * RESIDUAL_COUNT);
        CommonAxisProblem {
            params: nalgebra::SVector::<f64, PARAM_COUNT>::from_row_slice(params),
            rot_mats,
            residuals,
            jacobian_transposed,
        }
    }
}

impl<'a> OptimizableProblem<PARAM_COUNT> for CommonAxisProblem<'a> {
    fn calc_residuals(&mut self) -> () {
        let params = self.params.as_slice();
        let a = params[0];
        let b = params[1];
        let mrp_0 = params[2];
        let mrp_1 = params[3];
        let mrp_2 = params[4];

        let res_slice = self.residuals.as_mut_slice();

        for i in 0..self.rot_mats.len() {
            let rot = &self.rot_mats[i];

            let r1 = rot[0];
            let r2 = rot[1];
            let r3 = rot[2];
            let r4 = rot[3];
            let r5 = rot[4];
            let r6 = rot[5];
            let r7 = rot[6];
            let r8 = rot[7];
            let r9 = rot[8];

            // Reusable subexpressions:
            let x0 = a * a + b * b;
            let x1 = 1.0 / (x0 + 1.0);
            let x2 = mrp_1 * mrp_1;
            let x3 = 8.0 * x2;
            let x4 = mrp_2 * mrp_2;
            let x5 = 8.0 * x4;
            let x6 = mrp_0 * mrp_0;
            let x7 = 1.0 / ((x2 + x4 + x6 + 1.0) * (x2 + x4 + x6 + 1.0));
            let x8 = x7 * (-1.0 * x3 - 1.0 * x5) + 1.0;
            let x9 = 8.0 * mrp_0;
            let x10 = mrp_1 * x9;
            let x11 = -4.0 * x2 - 4.0 * x4 - 4.0 * x6 + 4.0;
            let x12 = mrp_2 * x11;
            let x13 = x7 * (x10 + x12);
            let x14 = mrp_2 * x9;
            let x15 = mrp_1 * x11;
            let x16 = x7 * (x14 - 1.0 * x15);
            let x17 = 2.0 * x1;
            let x18 = a * x17;
            let x19 = b * x17;
            let x20 = 1.0 - 1.0 * x0;
            let x21 = x1 * x20;
            let x22 = 8.0 * x6;
            let x23 = x7 * (-1.0 * x22 - 1.0 * x5) + 1.0;
            let x24 = mrp_0 * x11;
            let x25 = x7 * (8.0 * mrp_1 * mrp_2 + x24);
            let x26 = x7 * (x10 - 1.0 * x12);
            let x27 = x7 * (-1.0 * x22 - 1.0 * x3) + 1.0;
            let x28 = x7 * (x14 + x15);
            let x29 = x7 * (8.0 * mrp_1 * mrp_2 - 1.0 * x24);

            // Residual:
            let base = i * RESIDUAL_COUNT;
            res_slice[base + 0] = 2.0 * a * x1
                - 1.0 * x18 * (r1 * x8 + r4 * x13 + r7 * x16)
                - 1.0 * x19 * (r2 * x8 + r5 * x13 + r8 * x16)
                - 1.0 * x21 * (r3 * x8 + r6 * x13 + r9 * x16);
            res_slice[base + 1] = 2.0 * b * x1
                - 1.0 * x18 * (r1 * x26 + r4 * x23 + r7 * x25)
                - 1.0 * x19 * (r2 * x26 + r5 * x23 + r8 * x25)
                - 1.0 * x21 * (r3 * x26 + r6 * x23 + r9 * x25);
            res_slice[base + 2] = x1 * x20
                - 1.0 * x18 * (r1 * x28 + r4 * x29 + r7 * x27)
                - 1.0 * x19 * (r2 * x28 + r5 * x29 + r8 * x27)
                - 1.0 * x21 * (r3 * x28 + r6 * x29 + r9 * x27);
        }
    }

    fn calc_jacobian(&mut self) -> () {
        let params = self.params.as_slice();
        let a = params[0];
        let b = params[1];
        let mrp_0 = params[2];
        let mrp_1 = params[3];
        let mrp_2 = params[4];

        let jac_slice = self.jacobian_transposed.as_mut_slice();

        for i in 0..self.rot_mats.len() {
            let rot = &self.rot_mats[i];

            let r1 = rot[0];
            let r2 = rot[1];
            let r3 = rot[2];
            let r4 = rot[3];
            let r5 = rot[4];
            let r6 = rot[5];
            let r7 = rot[6];
            let r8 = rot[7];
            let r9 = rot[8];

            // Reusable subexpressions:
            let x0 = a * a;
            let x1 = b * b;
            let x2 = x0 + x1;
            let x3 = x2 + 1.0;
            let x4 = 1.0 / x3;
            let x5 = 2.0 * x4;
            let x6 = 1.0 / (x3 * x3);
            let x7 = 4.0 * x6;
            let x8 = x0 * x7;
            let x9 = mrp_1 * mrp_1;
            let x10 = 8.0 * x9;
            let x11 = mrp_2 * mrp_2;
            let x12 = 8.0 * x11;
            let x13 = -1.0 * x10 - 1.0 * x12;
            let x14 = mrp_0 * mrp_0;
            let x15 = x11 + x14 + x9 + 1.0;
            let x16 = 1.0 / (x15 * x15);
            let x17 = x13 * x16 + 1.0;
            let x18 = 8.0 * mrp_1;
            let x19 = mrp_0 * x18;
            let x20 = 4.0 * x9;
            let x21 = 4.0 * x14;
            let x22 = 4.0 * x11;
            let x23 = x21 + x22 - 4.0;
            let x24 = -1.0 * x20 - 1.0 * x23;
            let x25 = mrp_2 * x24;
            let x26 = x19 + x25;
            let x27 = x16 * x26;
            let x28 = 8.0 * mrp_2;
            let x29 = mrp_0 * x28;
            let x30 = mrp_1 * x24;
            let x31 = x29 - 1.0 * x30;
            let x32 = x16 * x31;
            let x33 = r1 * x17 + r4 * x27 + r7 * x32;
            let x34 = r3 * x17 + r6 * x27 + r9 * x32;
            let x35 = a * x5;
            let x36 = r2 * x17 + r5 * x27 + r8 * x32;
            let x37 = a * b * x7;
            let x38 = 1.0 - 1.0 * x2;
            let x39 = 2.0 * x38 * x6;
            let x40 = a * x39;
            let x41 = -1.0 * x37;
            let x42 = b * x5;
            let x43 = x1 * x7;
            let x44 = b * x39;
            let x45 = 8.0 * mrp_1 - 1.0 * x29;
            let x46 = x19 + x28;
            let x47 = 4.0 * (1.0 / (x15 * x15 * x15));
            let x48 = x13 * x47;
            let x49 = mrp_0 * x48;
            let x50 = mrp_0 * x47;
            let x51 = x26 * x50;
            let x52 = x31 * x50;
            let x53 = x38 * x4;
            let x54 = 8.0 * mrp_0;
            let x55 = mrp_2 * x18;
            let x56 = -1.0 * x55;
            let x57 = x54 + x56;
            let x58 = x16 * x57;
            let x59 = x23 + 12.0 * x9;
            let x60 = x16 * x59;
            let x61 = mrp_1 * x47;
            let x62 = x26 * x61;
            let x63 = x31 * x61;
            let x64 = 16.0 * x16;
            let x65 = mrp_1 * x64;
            let x66 = -1.0 * mrp_1 * x48 - 1.0 * x65;
            let x67 = x54 + x55;
            let x68 = x16 * x67;
            let x69 = x20 - 4.0;
            let x70 = 12.0 * x11 + x21 + x69;
            let x71 = -1.0 * x16 * x70;
            let x72 = mrp_2 * x47;
            let x73 = x26 * x72;
            let x74 = x31 * x72;
            let x75 = mrp_2 * x64;
            let x76 = -1.0 * mrp_2 * x48 - 1.0 * x75;
            let x77 = 8.0 * x14;
            let x78 = -1.0 * x12 - 1.0 * x77;
            let x79 = x16 * x78 + 1.0;
            let x80 = mrp_0 * x24;
            let x81 = x55 + x80;
            let x82 = x16 * x81;
            let x83 = x19 - 1.0 * x25;
            let x84 = x16 * x83;
            let x85 = r1 * x84 + r4 * x79 + r7 * x82;
            let x86 = r3 * x84 + r6 * x79 + r9 * x82;
            let x87 = r2 * x84 + r5 * x79 + r8 * x82;
            let x88 = x18 + x29;
            let x89 = x16 * x88;
            let x90 = 12.0 * x14 + x22 + x69;
            let x91 = -1.0 * x16 * x90;
            let x92 = x50 * x81;
            let x93 = x50 * x83;
            let x94 = mrp_0 * x64;
            let x95 = x47 * x78;
            let x96 = -1.0 * mrp_0 * x95 - 1.0 * x94;
            let x97 = 8.0 * mrp_2 - 1.0 * x19;
            let x98 = mrp_1 * x95;
            let x99 = x61 * x81;
            let x100 = x61 * x83;
            let x101 = x16 * x45;
            let x102 = x16 * x70;
            let x103 = x72 * x81;
            let x104 = x72 * x83;
            let x105 = -1.0 * mrp_2 * x95 - 1.0 * x75;
            let x106 = -1.0 * x10 - 1.0 * x77;
            let x107 = x106 * x16 + 1.0;
            let x108 = x29 + x30;
            let x109 = x108 * x16;
            let x110 = -1.0 * x56 - 1.0 * x80;
            let x111 = x110 * x16;
            let x112 = r1 * x109 + r4 * x111 + r7 * x107;
            let x113 = r3 * x109 + r6 * x111 + r9 * x107;
            let x114 = r2 * x109 + r5 * x111 + r8 * x107;
            let x115 = x16 * x97;
            let x116 = x16 * x90;
            let x117 = x108 * x50;
            let x118 = x110 * x50;
            let x119 = x106 * x47;
            let x120 = -1.0 * mrp_0 * x119 - 1.0 * x94;
            let x121 = x16 * x46;
            let x122 = -1.0 * x16 * x59;
            let x123 = x108 * x61;
            let x124 = x110 * x61;
            let x125 = -1.0 * mrp_1 * x119 - 1.0 * x65;
            let x126 = mrp_2 * x119;
            let x127 = x108 * x72;
            let x128 = x110 * x72;

            // Jacobian:
            let base = i * PARAM_COUNT * RESIDUAL_COUNT;
            jac_slice[base + 0] =
                -1.0 * x33 * x5 + x33 * x8 + x34 * x35 + x34 * x40 + x36 * x37 + x5 - 1.0 * x8;
            jac_slice[base + 1] =
                x33 * x37 + x34 * x42 + x34 * x44 + x36 * x43 - 1.0 * x36 * x5 + x41;
            jac_slice[base + 2] = -1.0
                * x35
                * (-1.0 * r1 * x49 + r4 * x16 * x45 - 1.0 * r4 * x51 + r7 * x16 * x46
                    - 1.0 * r7 * x52)
                - 1.0
                    * x42
                    * (-1.0 * r2 * x49 + r5 * x16 * x45 - 1.0 * r5 * x51 + r8 * x16 * x46
                        - 1.0 * r8 * x52)
                - 1.0
                    * x53
                    * (-1.0 * r3 * x49 + r6 * x16 * x45 - 1.0 * r6 * x51 + r9 * x16 * x46
                        - 1.0 * r9 * x52);
            jac_slice[base + 3] = -1.0
                * x35
                * (r1 * x66 + r4 * x58 - 1.0 * r4 * x62 + r7 * x60 - 1.0 * r7 * x63)
                - 1.0 * x42 * (r2 * x66 + r5 * x58 - 1.0 * r5 * x62 + r8 * x60 - 1.0 * r8 * x63)
                - 1.0 * x53 * (r3 * x66 + r6 * x58 - 1.0 * r6 * x62 + r9 * x60 - 1.0 * r9 * x63);
            jac_slice[base + 4] = -1.0
                * x35
                * (r1 * x76 + r4 * x71 - 1.0 * r4 * x73 + r7 * x68 - 1.0 * r7 * x74)
                - 1.0 * x42 * (r2 * x76 + r5 * x71 - 1.0 * r5 * x73 + r8 * x68 - 1.0 * r8 * x74)
                - 1.0 * x53 * (r3 * x76 + r6 * x71 - 1.0 * r6 * x73 + r9 * x68 - 1.0 * r9 * x74);
            jac_slice[base + 5] =
                x35 * x86 + x37 * x87 + x40 * x86 + x41 - 1.0 * x5 * x85 + x8 * x85;
            jac_slice[base + 6] =
                x37 * x85 + x42 * x86 + x43 * x87 - 1.0 * x43 + x44 * x86 - 1.0 * x5 * x87 + x5;
            jac_slice[base + 7] = -1.0
                * x35
                * (r1 * x89 - 1.0 * r1 * x93 + r4 * x96 + r7 * x91 - 1.0 * r7 * x92)
                - 1.0 * x42 * (r2 * x89 - 1.0 * r2 * x93 + r5 * x96 + r8 * x91 - 1.0 * r8 * x92)
                - 1.0 * x53 * (r3 * x89 - 1.0 * r3 * x93 + r6 * x96 + r9 * x91 - 1.0 * r9 * x92);
            jac_slice[base + 8] = -1.0
                * x35
                * (-1.0 * r1 * x100 + r1 * x16 * x67 - 1.0 * r4 * x98 + r7 * x16 * x97
                    - 1.0 * r7 * x99)
                - 1.0
                    * x42
                    * (-1.0 * r2 * x100 + r2 * x16 * x67 - 1.0 * r5 * x98 + r8 * x16 * x97
                        - 1.0 * r8 * x99)
                - 1.0
                    * x53
                    * (-1.0 * r3 * x100 + r3 * x16 * x67 - 1.0 * r6 * x98 + r9 * x16 * x97
                        - 1.0 * r9 * x99);
            jac_slice[base + 9] = -1.0
                * x35
                * (r1 * x102 - 1.0 * r1 * x104 + r4 * x105 + r7 * x101 - 1.0 * r7 * x103)
                - 1.0
                    * x42
                    * (r2 * x102 - 1.0 * r2 * x104 + r5 * x105 + r8 * x101 - 1.0 * r8 * x103)
                - 1.0
                    * x53
                    * (r3 * x102 - 1.0 * r3 * x104 + r6 * x105 + r9 * x101 - 1.0 * r9 * x103);
            jac_slice[base + 10] =
                -1.0 * x112 * x5 + x112 * x8 + x113 * x35 + x113 * x40 + x114 * x37
                    - 1.0 * x35
                    - 1.0 * x40;
            jac_slice[base + 11] = x112 * x37 + x113 * x42 + x113 * x44 + x114 * x43
                - 1.0 * x114 * x5
                - 1.0 * x42
                - 1.0 * x44;
            jac_slice[base + 12] = -1.0
                * x35
                * (r1 * x115 - 1.0 * r1 * x117 + r4 * x116 - 1.0 * r4 * x118 + r7 * x120)
                - 1.0
                    * x42
                    * (r2 * x115 - 1.0 * r2 * x117 + r5 * x116 - 1.0 * r5 * x118 + r8 * x120)
                - 1.0
                    * x53
                    * (r3 * x115 - 1.0 * r3 * x117 + r6 * x116 - 1.0 * r6 * x118 + r9 * x120);
            jac_slice[base + 13] = -1.0
                * x35
                * (r1 * x122 - 1.0 * r1 * x123 + r4 * x121 - 1.0 * r4 * x124 + r7 * x125)
                - 1.0
                    * x42
                    * (r2 * x122 - 1.0 * r2 * x123 + r5 * x121 - 1.0 * r5 * x124 + r8 * x125)
                - 1.0
                    * x53
                    * (r3 * x122 - 1.0 * r3 * x123 + r6 * x121 - 1.0 * r6 * x124 + r9 * x125);
            jac_slice[base + 14] = -1.0
                * x35
                * (-1.0 * r1 * x127 + r1 * x16 * x57 - 1.0 * r4 * x128 + r4 * x16 * x88
                    - 1.0 * r7 * x126)
                - 1.0
                    * x42
                    * (-1.0 * r2 * x127 + r2 * x16 * x57 - 1.0 * r5 * x128 + r5 * x16 * x88
                        - 1.0 * r8 * x126)
                - 1.0
                    * x53
                    * (-1.0 * r3 * x127 + r3 * x16 * x57 - 1.0 * r6 * x128 + r6 * x16 * x88
                        - 1.0 * r9 * x126);
        }
    }

    fn get_params(&self) -> &nalgebra::SVector<f64, PARAM_COUNT> {
        &self.params
    }
    fn set_params(&mut self, params: nalgebra::SVector<f64, PARAM_COUNT>) {
        self.params = params;
    }
    fn get_residuals(&self) -> &nalgebra::DVector<f64> {
        &self.residuals
    }
    fn get_jacobian(&self) -> &nalgebra::DMatrix<f64> {
        &self.jacobian_transposed
    }
}

/// Return a value that represents the magnitude between two rotations.
/// Bigger value means larger rotation difference
fn rotmat_mag_proxy(rotmat1: &[f64; 9], rotmat2: &[f64; 9]) -> f64 {
    // Mononic with mag(rotmat1 * inv(rotmat2))
    let v: f64 = rotmat1.iter().zip(rotmat2.iter()).map(|(x, y)| x * y).sum();
    -v
}

fn slice_to_matrix(m: &[f64; 9]) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8])
}

fn to_mrp(rotation: &nalgebra::Rotation3<f64>) -> Option<[f64; 3]> {
    let axis = rotation.axis()?;
    let angle = rotation.angle();
    let factor = (angle / 4.0).tan();
    Some((axis.into_inner() * factor).into())
}

pub fn find_common_axis(
    rot_mats: &[[f64; 9]],
    tol: f64,
    max_iter: usize,
) -> Result<([f64; 3], [f64; 3], nalgebra::DVector<f64>, f64), &'static str> {
    if rot_mats.len() < 2 {
        return Err("At least 2 rot_mats needed.");
    }

    // Find two rotations with bigest angular distance between
    // This is equivalent to the smallest trace between matrix multiplications
    let mut best_a = 0;
    let mut best_b = 0;
    let mut best_value = -10.0; // lowest value returned by rotmat_mag_proxy is -3
    for a in 0..rot_mats.len() - 1 {
        for b in (a + 1)..rot_mats.len() {
            let value = rotmat_mag_proxy(&rot_mats[a], &rot_mats[b]);
            if value > best_value {
                best_a = a;
                best_b = b;
                best_value = value;
            }
        }
    }
    let rot_a = nalgebra::Rotation::<f64, 3>::from_matrix(&slice_to_matrix(&rot_mats[best_a]));
    let rot_b_inv =
        nalgebra::Rotation::<f64, 3>::from_matrix(&slice_to_matrix(&rot_mats[best_b])).inverse();

    // Estimate axis vector from the two rotations
    let rot_diff = rot_a * rot_b_inv;
    let mut axis_vec_0 = rot_diff.scaled_axis();
    axis_vec_0.normalize_mut();

    // Convert axis to a b representation
    let [x, y, z]: [f64; 3] = axis_vec_0.into();
    let a0 = x / (z + 1.0);
    let b0 = y / (z + 1.0);

    // Get mrp from inverse rotation B
    let mrp0: [f64; 3] = match to_mrp(&rot_b_inv) {
        Some(mrp) => mrp,
        None => return Err("Rotation to small to get axis."),
    };

    // Solve optimization problem with Levenberg-Marquardt algorithm
    let initial_params = [a0, b0, mrp0[0], mrp0[1], mrp0[2]];
    let mut problem = CommonAxisProblem::new(&initial_params, rot_mats);
    crate::optim::levenberg_marquardt(&mut problem, max_iter, tol, 1e-3);

    // Decode params
    let params = problem.params.as_slice();
    let a = params[0];
    let b = params[1];
    let mrp = [params[2], params[3], params[4]];

    // Convert a b to axis
    let s2 = a * a + b * b;
    let axis = [
        2.0 * a / (s2 + 1.0),
        2.0 * b / (s2 + 1.0),
        (1.0 - s2) / (s2 + 1.0),
    ];

    let residuals = problem.get_residuals().clone();

    // Calculate estimate of the angular standard deviation of the result
    let angular_errors: Vec<f64> = residuals
        .as_slice()
        .chunks(3)
        .map(|x| f64::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]) * std::f64::consts::PI)
        .collect();
    let estimated_std_rad = (angular_errors.iter().map(|&x| x * x).sum::<f64>()
        / angular_errors.len() as f64
        / rot_mats.len() as f64)
        .sqrt();

    Ok((axis, mrp, residuals, estimated_std_rad))
}

use core::f32;
use nalgebra;

use crate::optim::OptimizableProblem;

pub const PARAM_COUNT: usize = 12;

pub struct CalibrationResult {
    pub params: [f64; PARAM_COUNT],
    pub rms_error: f64,
}

pub struct CameraCalibrationProblem<'a> {
    params: nalgebra::SVector<f64, PARAM_COUNT>,
    image_points: &'a [[f32; 2]],
    object_points: &'a [[f32; 3]],
    residuals: nalgebra::DVector<f64>,
    jacobian_transposed: nalgebra::DMatrix<f64>,
}

impl<'a> CameraCalibrationProblem<'a> {
    pub fn new(
        params: &[f64; PARAM_COUNT],
        image_points: &'a [[f32; 2]],
        object_points: &'a [[f32; 3]],
    ) -> Self {
        let num_points = image_points.len();
        let residuals = nalgebra::DVector::zeros(num_points * 2);
        let jacobian_transposed = nalgebra::DMatrix::zeros(PARAM_COUNT, num_points * 2);
        CameraCalibrationProblem {
            params: nalgebra::SVector::<f64, PARAM_COUNT>::from_row_slice(params),
            image_points,
            object_points,
            residuals,
            jacobian_transposed,
        }
    }
}

impl<'a> OptimizableProblem<PARAM_COUNT> for CameraCalibrationProblem<'a> {
    fn calc_residuals(&mut self) -> () {
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
            let tx = params[4];
            let fy = params[5];
            let ty = params[6];

            let k1 = params[7];
            let k2 = params[8];
            let p1 = params[9];
            let p2 = params[10];
            let k3 = params[11];

            let obj_x = object_point[0] as f64;
            let obj_y = object_point[1] as f64;
            let obj_z = object_point[2] as f64;

            let img_x = image_point[0] as f64;
            let img_y = image_point[1] as f64;

            // Reusable subexpressions:
            let x0 = mrp_0 * mrp_0;
            let x1 = 8.0 * x0;
            let x2 = mrp_1 * mrp_1;
            let x3 = 8.0 * x2;
            let x4 = mrp_2 * mrp_2;
            let x5 = 1.0 / ((x0 + x2 + x4 + 1.0) * (x0 + x2 + x4 + 1.0));
            let x6 = 8.0 * mrp_2;
            let x7 = mrp_0 * x6;
            let x8 = -4.0 * x0 - 4.0 * x2 - 4.0 * x4 + 4.0;
            let x9 = mrp_1 * x8;
            let x10 = obj_x * x5;
            let x11 = mrp_0 * x8;
            let x12 = obj_y * x5;
            let x13 = obj_z * (x5 * (-1.0 * x1 - 1.0 * x3) + 1.0)
                + x10 * (x7 + x9)
                + x12 * (8.0 * mrp_1 * mrp_2 - 1.0 * x11);
            let x14 = 1.0 / x13;
            let x15 = 8.0 * x4;
            let x16 = 8.0 * mrp_0 * mrp_1;
            let x17 = mrp_2 * x8;
            let x18 = obj_z * x5;
            let x19 = -1.0 * tx
                + x14
                    * (fx
                        * (obj_x * (x5 * (-1.0 * x15 - 1.0 * x3) + 1.0)
                            + x12 * (x16 + x17)
                            + x18 * (x7 - 1.0 * x9))
                        + tx * x13);
            let x20 = (1.0 / fx) * x19;
            let x21 = -1.0 * ty
                + x14
                    * (fy
                        * (obj_y * (x5 * (-1.0 * x1 - 1.0 * x15) + 1.0)
                            + x10 * (x16 - 1.0 * x17)
                            + x18 * (mrp_1 * x6 + x11))
                        + ty * x13);
            let x22 = (1.0 / fy) * x21;
            let x23 = 2.0 * x20 * x22;
            let x24 = (1.0 / (fy * fy)) * (x21 * x21);
            let x25 = (1.0 / (fx * fx)) * (x19 * x19);
            let x26 = x24 + x25;
            let x27 = k1 * x26 + k2 * (x26 * x26) + k3 * (x26 * x26 * x26) + 1.0;

            // value entries
            let base = i * stride;
            res_slice[base + 0] =
                -1.0 * fx * (p1 * x23 + p2 * (x24 + 3.0 * x25) + x20 * x27) + img_x - 1.0 * tx;
            res_slice[base + 1] =
                -1.0 * fy * (p1 * (3.0 * x24 + x25) + p2 * x23 + x22 * x27) + img_y - 1.0 * ty;
        }
    }

    fn calc_jacobian(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = self.jacobian_transposed.nrows() * 2;
        let jac_slice = self.jacobian_transposed.as_mut_slice();

        for i in 0..self.image_points.len() {
            let object_point = &self.object_points[i];

            let mrp_0 = params[0];
            let mrp_1 = params[1];
            let mrp_2 = params[2];

            let fx = params[3];
            let tx = params[4];
            let fy = params[5];
            let ty = params[6];

            let k1 = params[7];
            let k2 = params[8];
            let p1 = params[9];
            let p2 = params[10];
            let k3 = params[11];

            let obj_x = object_point[0] as f64;
            let obj_y = object_point[1] as f64;
            let obj_z = object_point[2] as f64;

            // Reusable subexpressions:
            let x0 = mrp_0 * mrp_0;
            let x1 = 8.0 * x0;
            let x2 = mrp_1 * mrp_1;
            let x3 = 8.0 * x2;
            let x4 = -1.0 * x1 - 1.0 * x3;
            let x5 = mrp_2 * mrp_2;
            let x6 = x0 + x2 + x5 + 1.0;
            let x7 = 1.0 / (x6 * x6);
            let x8 = 8.0 * mrp_2;
            let x9 = mrp_0 * x8;
            let x10 = 4.0 * x0;
            let x11 = 4.0 * x2;
            let x12 = 4.0 * x5;
            let x13 = x11 + x12 - 4.0;
            let x14 = -1.0 * x10 - 1.0 * x13;
            let x15 = mrp_1 * x14;
            let x16 = x15 + x9;
            let x17 = obj_x * x7;
            let x18 = 8.0 * mrp_1;
            let x19 = mrp_2 * x18;
            let x20 = -1.0 * x19;
            let x21 = mrp_0 * x14;
            let x22 = -1.0 * x20 - 1.0 * x21;
            let x23 = obj_y * x7;
            let x24 = obj_z * (x4 * x7 + 1.0) + x16 * x17 + x22 * x23;
            let x25 = 1.0 / x24;
            let x26 = 8.0 * mrp_1 - 1.0 * x9;
            let x27 = mrp_0 * x18;
            let x28 = x27 + x8;
            let x29 = 8.0 * x5;
            let x30 = -1.0 * x29 - 1.0 * x3;
            let x31 = 4.0 * (1.0 / (x6 * x6 * x6));
            let x32 = mrp_0 * x31;
            let x33 = obj_x * x32;
            let x34 = mrp_2 * x14;
            let x35 = x27 + x34;
            let x36 = obj_y * x32;
            let x37 = -1.0 * x15 + x9;
            let x38 = obj_z * x32;
            let x39 = 8.0 * mrp_2 - 1.0 * x27;
            let x40 = 12.0 * x0 + x13;
            let x41 = 16.0 * x7;
            let x42 = mrp_0 * x41;
            let x43 = obj_z * (-1.0 * x32 * x4 - 1.0 * x42) - 1.0 * x16 * x33 + x17 * x39
                - 1.0 * x22 * x36
                + x23 * x40;
            let x44 = x25
                * (fx
                    * (obj_y * x26 * x7 + obj_z * x28 * x7
                        - 1.0 * x30 * x33
                        - 1.0 * x35 * x36
                        - 1.0 * x37 * x38)
                    + tx * x43);
            let x45 = obj_z * x7;
            let x46 = obj_x * (x30 * x7 + 1.0) + x23 * x35 + x37 * x45;
            let x47 = fx * x46 + tx * x24;
            let x48 = 1.0 / (x24 * x24);
            let x49 = -1.0 * x43 * x48;
            let x50 = x47 * x49;
            let x51 = x44 + x50;
            let x52 = 1.0 / fx;
            let x53 = -1.0 * x1 - 1.0 * x29;
            let x54 = x19 + x21;
            let x55 = x27 - 1.0 * x34;
            let x56 = obj_y * (x53 * x7 + 1.0) + x17 * x55 + x45 * x54;
            let x57 = fy * x56 + ty * x24;
            let x58 = -1.0 * ty + x25 * x57;
            let x59 = 1.0 / fy;
            let x60 = x58 * x59;
            let x61 = 2.0 * x60;
            let x62 = p1 * x61;
            let x63 = x52 * x62;
            let x64 = x18 + x9;
            let x65 = x25
                * (fy
                    * (obj_y * (-1.0 * x32 * x53 - 1.0 * x42) + x17 * x64
                        - 1.0 * x33 * x55
                        - 1.0 * x38 * x54
                        - 1.0 * x40 * x45)
                    + ty * x43);
            let x66 = x49 * x57;
            let x67 = x65 + x66;
            let x68 = -1.0 * tx + x25 * x47;
            let x69 = x52 * x68;
            let x70 = 2.0 * x69;
            let x71 = p1 * x70;
            let x72 = x59 * x71;
            let x73 = 1.0 / (fx * fx);
            let x74 = x68 * x73;
            let x75 = x74 * (2.0 * x44 + 2.0 * x50);
            let x76 = 3.0 * x75;
            let x77 = 1.0 / (fy * fy);
            let x78 = x58 * x77;
            let x79 = x78 * (2.0 * x65 + 2.0 * x66);
            let x80 = x68 * x68;
            let x81 = x73 * x80;
            let x82 = x58 * x58;
            let x83 = x77 * x82;
            let x84 = x81 + x83;
            let x85 = x84 * x84;
            let x86 = x84 * x84 * x84;
            let x87 = k1 * x84 + k2 * x85 + k3 * x86 + 1.0;
            let x88 = x52 * x87;
            let x89 = k2 * x84;
            let x90 = 3.0 * x79;
            let x91 = k3 * x85;
            let x92 = k1 * (x75 + x79) + x89 * (2.0 * x75 + 2.0 * x79) + x91 * (x76 + x90);
            let x93 = 8.0 * mrp_0;
            let x94 = x19 + x93;
            let x95 = mrp_1 * x31;
            let x96 = obj_y * x95;
            let x97 = obj_z * x95;
            let x98 = obj_x * x95;
            let x99 = x10 - 4.0;
            let x100 = x12 + 12.0 * x2 + x99;
            let x101 = mrp_1 * x41;
            let x102 = obj_z * (-1.0 * x101 - 1.0 * x4 * x95)
                - 1.0 * x100 * x17
                - 1.0 * x16 * x98
                - 1.0 * x22 * x96
                + x23 * x28;
            let x103 = x25
                * (fy
                    * (obj_x * x7 * x94 + obj_z * x39 * x7
                        - 1.0 * x53 * x96
                        - 1.0 * x54 * x97
                        - 1.0 * x55 * x98)
                    + ty * x102);
            let x104 = -1.0 * x102 * x48;
            let x105 = x104 * x57;
            let x106 = x103 + x105;
            let x107 = x20 + x93;
            let x108 = x25
                * (fx
                    * (obj_x * (-1.0 * x101 - 1.0 * x30 * x95) + x100 * x45 + x107 * x23
                        - 1.0 * x35 * x96
                        - 1.0 * x37 * x97)
                    + tx * x102);
            let x109 = x104 * x47;
            let x110 = x108 + x109;
            let x111 = x78 * (2.0 * x103 + 2.0 * x105);
            let x112 = x74 * (2.0 * x108 + 2.0 * x109);
            let x113 = 3.0 * x112;
            let x114 = 3.0 * x111;
            let x115 = k1 * (x111 + x112) + x89 * (2.0 * x111 + 2.0 * x112) + x91 * (x113 + x114);
            let x116 = mrp_2 * x31;
            let x117 = obj_z * x116;
            let x118 = obj_x * x116;
            let x119 = obj_y * x116;
            let x120 = -1.0 * obj_x * x107 * x7 - 1.0 * obj_y * x64 * x7
                + x117 * x4
                + x118 * x16
                + x119 * x22;
            let x121 = -1.0 * x120;
            let x122 = x11 + 12.0 * x5 + x99;
            let x123 = mrp_2 * x41;
            let x124 = x25
                * (fy
                    * (obj_y * (-1.0 * x116 * x53 - 1.0 * x123)
                        - 1.0 * x117 * x54
                        - 1.0 * x118 * x55
                        + x122 * x17
                        + x26 * x45)
                    + ty * x121);
            let x125 = x120 * x48;
            let x126 = x125 * x57;
            let x127 = x124 + x126;
            let x128 = x25
                * (fx
                    * (obj_x * (-1.0 * x116 * x30 - 1.0 * x123)
                        - 1.0 * x117 * x37
                        - 1.0 * x119 * x35
                        - 1.0 * x122 * x23
                        + x45 * x94)
                    + tx * x121);
            let x129 = x125 * x47;
            let x130 = x128 + x129;
            let x131 = x78 * (2.0 * x124 + 2.0 * x126);
            let x132 = x74 * (2.0 * x128 + 2.0 * x129);
            let x133 = 3.0 * x132;
            let x134 = 3.0 * x131;
            let x135 = k1 * (x131 + x132) + x89 * (2.0 * x131 + 2.0 * x132) + x91 * (x133 + x134);
            let x136 = 3.0 * x81 + x83;
            let x137 = x25 * x46;
            let x138 = x80 * (1.0 / (fx * fx * fx));
            let x139 = x137 * x74;
            let x140 = -6.0 * x138 + 6.0 * x139;
            let x141 = -2.0 * x138 + 2.0 * x139;
            let x142 = k1 * x141 + x140 * x91 + x89 * (-4.0 * x138 + 4.0 * x139);
            let x143 = x25 * x56;
            let x144 = x82 * (1.0 / (fy * fy * fy));
            let x145 = x143 * x78;
            let x146 = -2.0 * x144 + 2.0 * x145;
            let x147 = -6.0 * x144 + 6.0 * x145;
            let x148 = k1 * x146 + x147 * x91 + x89 * (-4.0 * x144 + 4.0 * x145);
            let x149 = p2 * x61;
            let x150 = x149 * x52;
            let x151 = p2 * x70;
            let x152 = x151 * x59;
            let x153 = x59 * x87;
            let x154 = x81 + 3.0 * x83;

            // jacobian entries
            let base = i * stride;
            jac_slice[base + 0] =
                -1.0 * fx * (p2 * (x76 + x79) + x51 * x63 + x51 * x88 + x67 * x72 + x69 * x92);
            jac_slice[base + 1] = -1.0
                * fx
                * (p2 * (x111 + x113) + x106 * x72 + x110 * x63 + x110 * x88 + x115 * x69);
            jac_slice[base + 2] = -1.0
                * fx
                * (p2 * (x131 + x133) + x127 * x72 + x130 * x63 + x130 * x88 + x135 * x69);
            jac_slice[base + 3] = -1.0
                * fx
                * (p2 * x140 + x137 * x63 + x137 * x88 + x142 * x69
                    - 1.0 * x62 * x74
                    - 1.0 * x74 * x87)
                - 1.0 * p2 * x136
                - 1.0 * x62 * x69
                - 1.0 * x69 * x87;
            jac_slice[base + 4] = -1.0;
            jac_slice[base + 5] =
                -1.0 * fx * (p2 * x146 + x143 * x72 + x148 * x69 - 1.0 * x71 * x78);
            jac_slice[base + 6] = 0.0;
            jac_slice[base + 7] = -1.0 * x68 * x84;
            jac_slice[base + 8] = -1.0 * x68 * x85;
            jac_slice[base + 9] = -1.0 * x61 * x68;
            jac_slice[base + 10] = -1.0 * fx * x136;
            jac_slice[base + 11] = -1.0 * x68 * x86;
            jac_slice[base + 12] =
                -1.0 * fy * (p1 * (x75 + x90) + x150 * x51 + x152 * x67 + x153 * x67 + x60 * x92);
            jac_slice[base + 13] = -1.0
                * fy
                * (p1 * (x112 + x114) + x106 * x152 + x106 * x153 + x110 * x150 + x115 * x60);
            jac_slice[base + 14] = -1.0
                * fy
                * (p1 * (x132 + x134) + x127 * x152 + x127 * x153 + x130 * x150 + x135 * x60);
            jac_slice[base + 15] =
                -1.0 * fy * (p1 * x141 + x137 * x150 + x142 * x60 - 1.0 * x149 * x74);
            jac_slice[base + 16] = 0.0;
            jac_slice[base + 17] = -1.0
                * fy
                * (p1 * x147 + x143 * x152 + x143 * x153 + x148 * x60
                    - 1.0 * x151 * x78
                    - 1.0 * x78 * x87)
                - 1.0 * p1 * x154
                - 1.0 * x149 * x69
                - 1.0 * x60 * x87;
            jac_slice[base + 18] = -1.0;
            jac_slice[base + 19] = -1.0 * x58 * x84;
            jac_slice[base + 20] = -1.0 * x58 * x85;
            jac_slice[base + 21] = -1.0 * fy * x154;
            jac_slice[base + 22] = -1.0 * x58 * x70;
            jac_slice[base + 23] = -1.0 * x58 * x86;
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
    let mut problem = CameraCalibrationProblem::new(&initial_params, image_points, object_points);
    crate::optim::levenberg_marquardt(&mut problem, 10, 1e-4, 1e-3);

    let rms_error = problem.get_residuals().map(|x| x * x).mean().sqrt();
    CalibrationResult {
        params: problem
            .params
            .as_slice()
            .try_into()
            .expect("Invalid parameter length"),
        rms_error,
    }
}

use core::f32;
use nalgebra;

use crate::optim::OptimizableProblem;

pub const PARAM_COUNT: usize = 7;

pub struct StarGradCalibrationProblem<'a> {
    params: nalgebra::SVector<f64, PARAM_COUNT>,
    image_points: &'a [[f32; 2]],
    image_gradients: &'a [[f32; 2]],
    residuals: nalgebra::DVector<f64>,
    jacobian_transposed: nalgebra::DMatrix<f64>,
}

impl<'a> StarGradCalibrationProblem<'a> {
    pub fn new(
        params: &[f64; PARAM_COUNT],
        image_points: &'a [[f32; 2]],
        image_gradients: &'a [[f32; 2]],
    ) -> Self {
        let num_points = image_points.len();
        let residuals = nalgebra::DVector::zeros(num_points * 2);
        let jacobian_transposed = nalgebra::DMatrix::zeros(PARAM_COUNT, num_points * 2);
        StarGradCalibrationProblem {
            params: nalgebra::SVector::<f64, PARAM_COUNT>::from_row_slice(params),
            image_points,
            image_gradients,
            residuals,
            jacobian_transposed,
        }
    }
}

impl<'a> OptimizableProblem<PARAM_COUNT> for StarGradCalibrationProblem<'a> {
    fn calc_residuals(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = 2;
        let res_slice = self.residuals.as_mut_slice();

        for i in 0..self.image_points.len() {
            let image_point = &self.image_points[i];
            let image_gradient = &self.image_gradients[i];

            let m_epsilon = params[0];
            let m_theta = params[1];

            let fx = params[2];
            let tx = params[3];
            let fy = params[4];
            let ty = params[5];

            let k1 = params[6];

            let img_x = image_point[0] as f64;
            let img_y = image_point[1] as f64;

            let img_dx = image_gradient[0] as f64;
            let img_dy = image_gradient[1] as f64;

            // Reusable subexpressions:
            let x0 = img_x - 1.0 * tx;
            let x1 = x0 * x0;
            let x2 = x1 * (1.0 / (fx * fx));
            let x3 = img_y - 1.0 * ty;
            let x4 = x3 * x3;
            let x5 = x4 * (1.0 / (fy * fy));
            let x6 = k1 * (x2 + x5) + 1.0;
            let x7 = 1.0 / (x6 * x6);
            let x8 = k1 * (x2 * x7 + x5 * x7) + 1.0;
            let x9 = 1.0 / (x8 * x8);
            let x10 = x1 * (1.0 / (fx * fx * fx));
            let x11 = 1.0 / fy;
            let x12 = x11 * x3;
            let x13 = 1.0 / (x6 * x6 * x6);
            let x14 = 4.0 * k1 * x13;
            let x15 = 2.0 * k1;
            let x16 = x4 * (1.0 / (fy * fy * fy));
            let x17 = 2.0 * x13;
            let x18 = -1.0 * x10 * x12 * x14 + x12 * x17 * (x11 * x6 - 1.0 * x15 * x16);
            let x19 = 1.0 / fx;
            let x20 = x0 * x19;
            let x21 = -1.0 * x14 * x16 * x20 + x17 * x20 * (-1.0 * x10 * x15 + x19 * x6);
            let x22 = m_epsilon * m_epsilon;
            let x23 = 1.0 / ((x22 + 1.0) * (x22 + 1.0));
            let x24 = -8.0 * x22 * x23 + 1.0;
            let x25 = 1.0 / x8;
            let x26 = ty + x25 * x3;
            let x27 = m_theta * m_theta;
            let x28 = 1.0 / ((x27 + 1.0) * (x27 + 1.0));
            let x29 = m_theta * x28 * (4.0 - 4.0 * x27);
            let x30 = x24 * x29;
            let x31 = ty * x29;
            let x32 = tx + x0 * x25;
            let x33 = m_epsilon * x23 * (4.0 - 4.0 * x22);
            let x34 = x29 * x33;
            let x35 = tx * x29;
            let x36 = 8.0 * x27 * x28 - 1.0;
            let x37 = -1.0 * x11 * x24 * x31 + x11 * x26 * x30 + x19 * x32 * x34
                - 1.0 * x19 * x33 * x35
                + x36;
            let x38 = x19 * x24;
            let x39 = x11 * x33;
            let x40 = -1.0 * tx * x38 + ty * x39 - 1.0 * x26 * x39 + x32 * x38;
            let x41 = -1.0 * x36;
            let x42 = x24 * x41;
            let x43 = x11 * x42;
            let x44 = x33 * x41;
            let x45 = x19 * x44;
            let x46 = -1.0 * tx * x45 - 1.0 * ty * x43 + x26 * x43 + x29 + x32 * x45;
            let x47 = 1.0 / f64::sqrt(x37 * x37 + x40 * x40 + x46 * x46);
            let x48 = x37 * x47;
            let x49 = x46 * x47;
            let x50 = x29 * x49 + x36 * x48;
            let x51 = 1.0 / x50;
            let x52 = fx * x34 + tx * x36;
            let x53 = x40 * x47;
            let x54 = x53 * (1.0 / (x50 * x50));
            let x55 = fy * x33;
            let x56 = fy * x30 + ty * x36;

            // value entries
            let base = i * stride;
            res_slice[base + 0] = -1.0 * fx * img_dx * x9 * (-1.0 * k1 * x20 * x21 + x19 * x8)
                + fx * x24 * x37 * x47 * x51
                + img_dy * k1 * x0 * x18 * x9
                - 1.0
                    * x54
                    * (-1.0 * x36 * (fx * x24 * x53 + x48 * x52 + x49 * (fx * x44 + x35))
                        + x50 * x52);
            res_slice[base + 1] = -1.0 * fy * img_dy * x9 * (-1.0 * k1 * x12 * x18 + x11 * x8)
                + img_dx * k1 * x21 * x3 * x9
                - 1.0 * x48 * x51 * x55
                - 1.0
                    * x54
                    * (-1.0 * x36 * (x48 * x56 + x49 * (fy * x42 + x31) - 1.0 * x53 * x55)
                        + x50 * x56);
        }
    }

    fn calc_jacobian(&mut self) -> () {
        let params = self.params.as_slice();

        let stride = self.jacobian_transposed.nrows() * 2;
        let jac_slice = self.jacobian_transposed.as_mut_slice();

        for i in 0..self.image_points.len() {
            let image_point = &self.image_points[i];
            let image_gradient = &self.image_gradients[i];

            let m_epsilon = params[0];
            let m_theta = params[1];

            let fx = params[2];
            let tx = params[3];
            let fy = params[4];
            let ty = params[5];

            let k1 = params[6];

            let img_x = image_point[0] as f64;
            let img_y = image_point[1] as f64;

            let img_dx = image_gradient[0] as f64;
            let img_dy = image_gradient[1] as f64;

            // Reusable subexpressions:
            let x0 = m_epsilon * m_epsilon;
            let x1 = x0 + 1.0;
            let x2 = 1.0 / (x1 * x1);
            let x3 = m_epsilon * x2;
            let x4 = 16.0 * x3;
            let x5 = 1.0 / (x1 * x1 * x1);
            let x6 = 32.0 * x5 * (m_epsilon * m_epsilon * m_epsilon) - 1.0 * x4;
            let x7 = 1.0 / fx;
            let x8 = x0 * x2;
            let x9 = 8.0 * x8;
            let x10 = 1.0 - 1.0 * x9;
            let x11 = x10 * x7;
            let x12 = 1.0 / fy;
            let x13 = ty * x12;
            let x14 = 4.0 * x0;
            let x15 = 4.0 - 1.0 * x14;
            let x16 = x15 * x3;
            let x17 = img_x - 1.0 * tx;
            let x18 = 1.0 / (fx * fx);
            let x19 = x17 * x17;
            let x20 = x18 * x19;
            let x21 = 1.0 / (fy * fy);
            let x22 = img_y - 1.0 * ty;
            let x23 = x22 * x22;
            let x24 = x21 * x23;
            let x25 = x20 + x24;
            let x26 = k1 * x25 + 1.0;
            let x27 = 1.0 / (x26 * x26);
            let x28 = x20 * x27;
            let x29 = x24 * x27;
            let x30 = x28 + x29;
            let x31 = k1 * x30 + 1.0;
            let x32 = 1.0 / x31;
            let x33 = tx + x17 * x32;
            let x34 = ty + x22 * x32;
            let x35 = x12 * x34;
            let x36 = -1.0 * tx * x11 + x11 * x33 + x13 * x16 - 1.0 * x16 * x35;
            let x37 = m_theta * m_theta;
            let x38 = 4.0 * x37;
            let x39 = 4.0 - 1.0 * x38;
            let x40 = x37 + 1.0;
            let x41 = 1.0 / (x40 * x40);
            let x42 = x39 * x41;
            let x43 = m_theta * x42;
            let x44 = x37 * x41;
            let x45 = 8.0 * x44;
            let x46 = x45 - 1.0;
            let x47 = -1.0 * x46;
            let x48 = x10 * x47;
            let x49 = x12 * x48;
            let x50 = x47 * x7;
            let x51 = x16 * x50;
            let x52 = -1.0 * tx * x51 - 1.0 * ty * x49 + x33 * x51 + x34 * x49 + x43;
            let x53 = x10 * x43;
            let x54 = x33 * x43;
            let x55 = x16 * x7;
            let x56 = tx * x43;
            let x57 = -1.0 * x13 * x53 + x35 * x53 + x46 + x54 * x55 - 1.0 * x55 * x56;
            let x58 = x36 * x36 + x52 * x52 + x57 * x57;
            let x59 = 1.0 / f64::sqrt(x58);
            let x60 = x57 * x59;
            let x61 = x52 * x59;
            let x62 = x43 * x61 + x46 * x60;
            let x63 = 1.0 / x62;
            let x64 = x63;
            let x65 = fx * x64;
            let x66 = x15 * x2;
            let x67 = x66 * x7;
            let x68 = x43 * x6;
            let x69 = x15 * x5;
            let x70 = x14 * x69;
            let x71 = x59
                * (4.0 * m_theta * tx * x0 * x15 * x39 * x41 * x5 * x7
                    + 8.0 * m_theta * tx * x0 * x2 * x39 * x41 * x7
                    + m_theta * x12 * x34 * x39 * x41 * x6
                    + m_theta * x15 * x2 * x33 * x39 * x41 * x7
                    - 1.0 * x13 * x68
                    - 1.0 * x54 * x7 * x70
                    - 1.0 * x54 * x7 * x9
                    - 1.0 * x56 * x67);
            let x72 = x10 * x65;
            let x73 = f64::powf(x58, -3.0 / 2.0);
            let x74 = 16.0 * x8;
            let x75 = 8.0 * x0 * x69;
            let x76 = 2.0 * tx;
            let x77 = x6 * x7;
            let x78 = x35 * x66;
            let x79 = x36 / 2.0;
            let x80 = x50 * x66;
            let x81 = x47 * x6;
            let x82 = 2.0 * ty;
            let x83 = x12 * x82;
            let x84 = x52 / 2.0;
            let x85 = x57 / 2.0;
            let x86 = x73
                * (-1.0
                    * x79
                    * (2.0 * ty * x12 * x15 * x2
                        + 8.0 * x0 * x12 * x15 * x34 * x5
                        + 16.0 * x0 * x12 * x2 * x34
                        - 1.0 * x13 * x74
                        - 1.0 * x13 * x75
                        + 2.0 * x33 * x6 * x7
                        - 1.0 * x76 * x77
                        - 2.0 * x78)
                    - 1.0
                        * x84
                        * (8.0 * tx * x0 * x15 * x47 * x5 * x7
                            + 16.0 * tx * x0 * x2 * x47 * x7
                            + 2.0 * x12 * x34 * x47 * x6
                            + 2.0 * x15 * x2 * x33 * x47 * x7
                            - 1.0 * x33 * x50 * x74
                            - 1.0 * x33 * x50 * x75
                            - 1.0 * x76 * x80
                            - 1.0 * x81 * x83)
                    - 1.0
                        * x85
                        * (8.0 * m_theta * tx * x0 * x15 * x39 * x41 * x5 * x7
                            + 16.0 * m_theta * tx * x0 * x2 * x39 * x41 * x7
                            + 2.0 * m_theta * x12 * x34 * x39 * x41 * x6
                            + 2.0 * m_theta * x15 * x2 * x33 * x39 * x41 * x7
                            - 1.0 * x43 * x67 * x76
                            - 1.0 * x54 * x7 * x74
                            - 1.0 * x54 * x7 * x75
                            - 1.0 * x68 * x83));
            let x87 = x57 * x86;
            let x88 = -1.0 * tx * x77
                + ty * x12 * x15 * x2
                + 4.0 * x0 * x12 * x15 * x34 * x5
                + 8.0 * x0 * x12 * x2 * x34
                - 1.0 * x13 * x70
                - 1.0 * x13 * x9
                + x33 * x6 * x7
                - 1.0 * x78;
            let x89 = x59 * x88;
            let x90 = fx * x43;
            let x91 = tx * x46 + x16 * x90;
            let x92 = x10 * x59;
            let x93 = x36 * x92;
            let x94 = fx * x47;
            let x95 = x16 * x94 + x56;
            let x96 = x59 * x95;
            let x97 = fx * x93 + x52 * x96 + x60 * x91;
            let x98 = -1.0 * x46 * x97 + x62 * x91;
            let x99 = 1.0 / (x62 * x62);
            let x100 = x99;
            let x101 = x100 * x98;
            let x102 = x101 * x36;
            let x103 = 4.0 * tx * x0 * x15 * x47 * x5 * x7 + 8.0 * tx * x0 * x2 * x47 * x7
                - 1.0 * tx * x80
                + x12 * x34 * x47 * x6
                - 1.0 * x13 * x81
                + x15 * x2 * x33 * x47 * x7
                - 1.0 * x33 * x50 * x70
                - 1.0 * x33 * x50 * x9;
            let x104 = x43 * x59;
            let x105 = x103 * x104;
            let x106 = x46 * x71;
            let x107 = x46 * x87;
            let x108 = x43 * x52;
            let x109 = x108 * x86;
            let x110 = x105 + x106 + x107 + x109;
            let x111 = -1.0 * x100 * x110;
            let x112 = x10 * x60;
            let x113 = fx * x112;
            let x114 = -2.0 * x105 - 2.0 * x106 - 2.0 * x107 - 2.0 * x109;
            let x115 = x36 * x59;
            let x116 = x115 * (1.0 / (x62 * x62 * x62));
            let x117 = x116 * x98;
            let x118 = fx * m_theta * x15 * x2 * x39 * x41 - 1.0 * x70 * x90 - 1.0 * x9 * x90;
            let x119 = fx * x36;
            let x120 = fx * x92;
            let x121 = x10 * x119;
            let x122 = x52 * x95;
            let x123 = x100 * x115;
            let x124 = ty * x42;
            let x125 = x10 * x12;
            let x126 = 1.0 / (x40 * x40 * x40);
            let x127 = x126 * x39;
            let x128 = x127 * x38;
            let x129 = x10 * x35;
            let x130 = tx * x42;
            let x131 = x33 * x7;
            let x132 = x16 * x45;
            let x133 = x131 * x16;
            let x134 = x126 * (m_theta * m_theta * m_theta);
            let x135 = -16.0 * m_theta * x41 + 32.0 * x134;
            let x136 = x59
                * (4.0 * m_epsilon * tx * x126 * x15 * x2 * x37 * x39 * x7
                    + 8.0 * m_epsilon * tx * x15 * x2 * x37 * x41 * x7
                    + m_epsilon * x15 * x2 * x33 * x39 * x41 * x7
                    + 4.0 * ty * x10 * x12 * x126 * x37 * x39
                    + 8.0 * ty * x10 * x12 * x37 * x41
                    + x10 * x12 * x34 * x39 * x41
                    - 1.0 * x10 * x35 * x45
                    - 1.0 * x124 * x125
                    - 1.0 * x128 * x129
                    - 1.0 * x128 * x133
                    - 1.0 * x130 * x55
                    - 1.0 * x131 * x132
                    - 1.0 * x135);
            let x137 = 16.0 * x44;
            let x138 = 8.0 * x127 * x37;
            let x139 = x10 * x135;
            let x140 = x135 * x16;
            let x141 = x140 * x7;
            let x142 = x73
                * (-1.0
                    * x84
                    * (2.0 * m_epsilon * x135 * x15 * x2 * x33 * x7
                        + 2.0 * x10 * x12 * x135 * x34
                        - 1.0 * x137
                        - 1.0 * x138
                        - 1.0 * x139 * x83
                        - 1.0 * x141 * x76
                        + 2.0 * x39 * x41)
                    - 1.0
                        * x85
                        * (8.0 * m_epsilon * tx * x126 * x15 * x2 * x37 * x39 * x7
                            + 16.0 * m_epsilon * tx * x15 * x2 * x37 * x41 * x7
                            + 2.0 * m_epsilon * x15 * x2 * x33 * x39 * x41 * x7
                            + 32.0 * m_theta * x41
                            + 8.0 * ty * x10 * x12 * x126 * x37 * x39
                            + 16.0 * ty * x10 * x12 * x37 * x41
                            + 2.0 * x10 * x12 * x34 * x39 * x41
                            - 1.0 * x10 * x42 * x83
                            - 1.0 * x129 * x137
                            - 1.0 * x129 * x138
                            - 1.0 * x131 * x15 * x4 * x44
                            - 1.0 * x133 * x138
                            - 64.0 * x134
                            - 1.0 * x16 * x42 * x7 * x76));
            let x143 = x142 * x57;
            let x144 = -1.0 * x135;
            let x145 = x144 * x60;
            let x146 = m_epsilon * x135 * x15 * x2 * x33 * x7 - 1.0 * tx * x141
                + x10 * x12 * x135 * x34
                - 1.0 * x128
                - 1.0 * x13 * x139
                + x39 * x41
                - 1.0 * x45;
            let x147 = x104 * x146;
            let x148 = x136 * x46;
            let x149 = x143 * x46;
            let x150 = x108 * x142;
            let x151 =
                -1.0 * x128 * x61 + x145 + x147 + x148 + x149 + x150 + x42 * x61 - 1.0 * x45 * x61;
            let x152 = -1.0 * x151;
            let x153 = 8.0 * x126 * x37 * x39 * x52 * x59
                - 2.0 * x145
                - 2.0 * x147
                - 2.0 * x148
                - 2.0 * x149
                - 2.0 * x150
                + 16.0 * x37 * x41 * x52 * x59
                - 2.0 * x42 * x61;
            let x154 =
                fx * m_epsilon * x15 * x2 * x39 * x41 - 1.0 * fx * x128 * x16 - 1.0 * fx * x132
                    + tx * x144;
            let x155 = k1 * x7;
            let x156 = x155 * x17;
            let x157 = 1.0 / (fy * fy * fy);
            let x158 = x157 * x23;
            let x159 = 1.0 / (x26 * x26 * x26);
            let x160 = 4.0 * x159;
            let x161 = x158 * x160;
            let x162 = 1.0 / (fx * fx * fx);
            let x163 = 2.0 * x19;
            let x164 = x162 * x163;
            let x165 = -1.0 * k1 * x164 + x26 * x7;
            let x166 = 2.0 * x159;
            let x167 = x165 * x166;
            let x168 = x167 * x7;
            let x169 = -1.0 * x156 * x161 + x168 * x17;
            let x170 = x155 * x169;
            let x171 = -1.0 * x17 * x170 + x31 * x7;
            let x172 = 1.0 / (x31 * x31);
            let x173 = img_dx * x172;
            let x174 = 1.0 / (fx * fx * fx * fx);
            let x175 = k1 * x12;
            let x176 = x175 * x22;
            let x177 = 12.0 * x159;
            let x178 = x21 * x22;
            let x179 = 4.0 * k1 * x159;
            let x180 = x162 * x19;
            let x181 = x179 * x180;
            let x182 = x178 * x181;
            let x183 = x17 * x17 * x17 * x17;
            let x184 = x12 * x22;
            let x185 = 1.0 / (x26 * x26 * x26 * x26);
            let x186 = k1 * k1;
            let x187 = 24.0 * x185 * x186;
            let x188 = 2.0 * x158;
            let x189 = -1.0 * k1 * x188 + x12 * x26;
            let x190 = x185 * x189;
            let x191 = 12.0 * x190;
            let x192 = x176 * x180;
            let x193 = x174 * x176 * x177 * x19
                - 1.0 * x182
                - 1.0 * x183 * x184 * x187 * 1.0 / (fx * fx * fx * fx * fx * fx)
                + x191 * x192;
            let x194 = 1.0 / (fx * fx * fx * fx * fx);
            let x195 = -1.0 * x164 * x27 + x179 * x183 * x194 + x181 * x24;
            let x196 = 1.0 / (x31 * x31 * x31);
            let x197 = 2.0 * x196;
            let x198 = x186 * x195 * x197;
            let x199 = x160 * x180;
            let x200 = x166 * x189;
            let x201 = x12 * x200;
            let x202 = -1.0 * x176 * x199 + x201 * x22;
            let x203 = img_dy * x202;
            let x204 = x17 * x203;
            let x205 = k1 * x161 * x17 * x18;
            let x206 = x17 * x17 * x17;
            let x207 = x166 * x17;
            let x208 = x207 * x7;
            let x209 = k1 * x174;
            let x210 = x165 * x185;
            let x211 = 12.0 * x210;
            let x212 = -1.0 * x158 * x174 * x187 * x206 - 1.0 * x167 * x17 * x18
                + x205
                + x206 * x209 * x211
                + x208 * (4.0 * k1 * x174 * x19 - 1.0 * x18 * x26);
            let x213 = fx * x173;
            let x214 = x16 * x18 * x54;
            let x215 = x175 * x195;
            let x216 = x172 * x22;
            let x217 = x216 * x53;
            let x218 = x215 * x217;
            let x219 = x155 * x195;
            let x220 = x16 * x43;
            let x221 = x17 * x172;
            let x222 = x220 * x221;
            let x223 = x219 * x222;
            let x224 = m_epsilon * m_theta * tx * x15 * x18 * x2 * x39 * x41
                - 1.0 * x214
                - 1.0 * x218
                - 1.0 * x223;
            let x225 = x10 * x18;
            let x226 = x225 * x33;
            let x227 = 2.0 * x11;
            let x228 = x17 * x227;
            let x229 = k1 * x172;
            let x230 = x195 * x229;
            let x231 = x16 * x216;
            let x232 = x215 * x231;
            let x233 = x16 * x47;
            let x234 = x18 * x233 * x33;
            let x235 = 2.0 * x49;
            let x236 = x22 * x235;
            let x237 = x221 * x233;
            let x238 = x219 * x237;
            let x239 = -1.0 * x79 * (x225 * x76 - 2.0 * x226 - 1.0 * x228 * x230 + 2.0 * x232)
                - 1.0
                    * x84
                    * (2.0 * m_epsilon * tx * x15 * x18 * x2 * x47
                        - 1.0 * x230 * x236
                        - 2.0 * x234
                        - 2.0 * x238)
                - 1.0
                    * x85
                    * (2.0 * m_epsilon * m_theta * tx * x15 * x18 * x2 * x39 * x41
                        - 2.0 * x214
                        - 2.0 * x218
                        - 2.0 * x223);
            let x240 = x11 * x17;
            let x241 = x229 * x240;
            let x242 = tx * x225 - 1.0 * x195 * x241 - 1.0 * x226 + x232;
            let x243 = x101 * x59;
            let x244 = x239 * x73;
            let x245 = x22 * x49;
            let x246 =
                m_epsilon * tx * x15 * x18 * x2 * x47 - 1.0 * x230 * x245 - 1.0 * x234 - 1.0 * x238;
            let x247 = x104 * x246;
            let x248 = x224 * x59;
            let x249 = x248 * x46;
            let x250 = x244 * x57;
            let x251 = x250 * x46;
            let x252 = x108 * x244;
            let x253 = x247 + x249 + x251 + x252;
            let x254 = -1.0 * x253;
            let x255 = -2.0 * x247 - 2.0 * x249 - 2.0 * x251 - 2.0 * x252;
            let x256 = img_dy * x172;
            let x257 = -2.0 * img_x + 2.0 * tx;
            let x258 = x162 * x257;
            let x259 = x18 * x257;
            let x260 = 6.0 * x190;
            let x261 = 2.0 * k1 * x159 * x18 * x21 * x22 * x257
                + 12.0 * x12 * x185 * x186 * x19 * x194 * x22 * x257
                - 1.0 * x160 * x176 * x258
                - 1.0 * x176 * x259 * x260;
            let x262 = x209 * x257;
            let x263 = 2.0 * x24;
            let x264 = k1 * x159;
            let x265 = -1.0 * x159 * x163 * x262 + x18 * x257 * x27 - 1.0 * x259 * x263 * x264;
            let x266 = x17 * x265;
            let x267 = x186 * x197;
            let x268 = 6.0 * x210;
            let x269 = 4.0 * k1 * x157 * x159 * x23 * x7 - 1.0 * k1 * x17 * x258 * x268
                + 12.0 * x157 * x162 * x17 * x185 * x186 * x23 * x257
                - 1.0 * x168
                - 1.0 * x207 * x262;
            let x270 = x43 * x55;
            let x271 = x175 * x265;
            let x272 = x217 * x271;
            let x273 = x32 - 1.0;
            let x274 = -1.0 * x229 * x266 - 1.0 * x273;
            let x275 =
                m_epsilon * m_theta * x15 * x2 * x274 * x39 * x41 * x7 - 1.0 * x270 - 1.0 * x272;
            let x276 = x231 * x271;
            let x277 = x11 * x274;
            let x278 = 2.0 * x51;
            let x279 = -1.0 * x79 * (-1.0 * x227 + 2.0 * x276 + 2.0 * x277)
                - 1.0
                    * x84
                    * (2.0 * m_epsilon * x15 * x2 * x274 * x47 * x7
                        - 1.0 * x229 * x236 * x265
                        - 1.0 * x278)
                - 1.0
                    * x85
                    * (2.0 * m_epsilon * m_theta * x15 * x2 * x274 * x39 * x41 * x7
                        - 2.0 * x270
                        - 2.0 * x272);
            let x280 = -1.0 * x11 + x276 + x277;
            let x281 = x279 * x73;
            let x282 = x229 * x245;
            let x283 = m_epsilon * x15 * x2 * x274 * x47 * x7 - 1.0 * x265 * x282 - 1.0 * x51;
            let x284 = x104 * x283;
            let x285 = x275 * x59;
            let x286 = x285 * x46;
            let x287 = x281 * x57;
            let x288 = x287 * x46;
            let x289 = x108 * x281;
            let x290 = x284 + x286 + x288 + x289;
            let x291 = -1.0 * x290;
            let x292 = -2.0 * x284 - 2.0 * x286 - 2.0 * x288 - 2.0 * x289;
            let x293 = x46 * x62;
            let x294 = 1.0 / (fy * fy * fy * fy * fy);
            let x295 = x22 * x22 * x22 * x22;
            let x296 = x158 * x179 * x20 + x179 * x294 * x295 - 1.0 * x188 * x27;
            let x297 = x204 * x267;
            let x298 = 1.0 / (fy * fy * fy * fy);
            let x299 = x22 * x22 * x22;
            let x300 = x166 * x22;
            let x301 = x12 * x300;
            let x302 = k1 * x298;
            let x303 = -1.0 * x178 * x200 - 1.0 * x180 * x187 * x298 * x299
                + x182
                + x191 * x299 * x302
                + x301 * (4.0 * k1 * x23 * x298 - 1.0 * x21 * x26);
            let x304 = x23 * x298;
            let x305 = x17 * x7;
            let x306 = x156 * x158;
            let x307 = x156 * x177 * x304
                - 1.0 * x187 * x295 * x305 * 1.0 / (fy * fy * fy * fy * fy * fy)
                - 1.0 * x205
                + x211 * x306;
            let x308 = x21 * x34 * x53;
            let x309 = x175 * x296;
            let x310 = x217 * x309;
            let x311 = x155 * x296;
            let x312 = x222 * x311;
            let x313 = m_theta * ty * x10 * x21 * x39 * x41 - 1.0 * x308 - 1.0 * x310 - 1.0 * x312;
            let x314 = x16 * x21;
            let x315 = x314 * x34;
            let x316 = x229 * x296;
            let x317 = x231 * x309;
            let x318 = x21 * x34 * x48;
            let x319 = x237 * x311;
            let x320 = -1.0
                * x79
                * (-1.0 * x228 * x316 - 1.0 * x314 * x82 + 2.0 * x315 + 2.0 * x317)
                - 1.0
                    * x84
                    * (2.0 * ty * x10 * x21 * x47 - 1.0 * x236 * x316 - 2.0 * x318 - 2.0 * x319)
                - 1.0
                    * x85
                    * (2.0 * m_theta * ty * x10 * x21 * x39 * x41
                        - 2.0 * x308
                        - 2.0 * x310
                        - 2.0 * x312);
            let x321 = -1.0 * ty * x314 - 1.0 * x241 * x296 + x315 + x317;
            let x322 = x320 * x73;
            let x323 = ty * x10 * x21 * x47 - 1.0 * x282 * x296 - 1.0 * x318 - 1.0 * x319;
            let x324 = x104 * x323;
            let x325 = x313 * x59;
            let x326 = x325 * x46;
            let x327 = x322 * x57;
            let x328 = x327 * x46;
            let x329 = x108 * x322;
            let x330 = x324 + x326 + x328 + x329;
            let x331 = -1.0 * x330;
            let x332 = -2.0 * x324 - 2.0 * x326 - 2.0 * x328 - 2.0 * x329;
            let x333 = -2.0 * img_y + 2.0 * ty;
            let x334 = x21 * x333;
            let x335 = 2.0 * x20;
            let x336 = -1.0 * k1 * x166 * x304 * x333 + x21 * x27 * x333 - 1.0 * x264 * x334 * x335;
            let x337 = x157 * x333;
            let x338 = 4.0 * k1 * x12 * x159 * x162 * x19 - 1.0 * k1 * x22 * x260 * x337
                + 12.0 * x157 * x162 * x185 * x186 * x19 * x22 * x333
                - 1.0 * x201
                - 1.0 * x300 * x302 * x333;
            let x339 = 2.0 * k1 * x159 * x17 * x18 * x21 * x333
                - 1.0 * x156 * x160 * x337
                - 1.0 * x156 * x268 * x334
                + 12.0 * x17 * x185 * x186 * x23 * x294 * x333 * x7;
            let x340 = x125 * x43;
            let x341 = x155 * x336;
            let x342 = x222 * x341;
            let x343 = x229 * x336;
            let x344 = -1.0 * x22 * x343 - 1.0 * x273;
            let x345 = m_theta * x10 * x12 * x344 * x39 * x41 - 1.0 * x340 - 1.0 * x342;
            let x346 = x12 * x16;
            let x347 = x344 * x346;
            let x348 = x237 * x341;
            let x349 = 2.0 * x340;
            let x350 =
                -1.0 * x79 * (2.0 * m_epsilon * x12 * x15 * x2 - 1.0 * x228 * x343 - 2.0 * x347)
                    - 1.0 * x84 * (2.0 * x10 * x12 * x344 * x47 - 1.0 * x235 - 2.0 * x348)
                    - 1.0
                        * x85
                        * (2.0 * m_theta * x10 * x12 * x344 * x39 * x41 - 2.0 * x342 - 1.0 * x349);
            let x351 = m_epsilon * x12 * x15 * x2 - 1.0 * x240 * x343 - 1.0 * x347;
            let x352 = x350 * x73;
            let x353 = x10 * x12 * x344 * x47 - 1.0 * x348 - 1.0 * x49;
            let x354 = x104 * x353;
            let x355 = x345 * x59;
            let x356 = x355 * x46;
            let x357 = x352 * x57;
            let x358 = x357 * x46;
            let x359 = x108 * x352;
            let x360 = x354 + x356 + x358 + x359;
            let x361 = -1.0 * x360;
            let x362 = -2.0 * x354 - 2.0 * x356 - 2.0 * x358 - 2.0 * x359;
            let x363 = -3.0 * x20 - 3.0 * x24;
            let x364 = 4.0 * x185 * x363;
            let x365 = 2.0 * x363;
            let x366 = x184 * x190 * x365 - 1.0 * x184 * x199 - 1.0 * x192 * x364
                + x301 * (x12 * x25 - 1.0 * x188);
            let x367 = x159 * (-1.0 * x263 - 1.0 * x335);
            let x368 = k1 * (x20 * x367 + x24 * x367);
            let x369 = -2.0 * x28 - 2.0 * x29 - 2.0 * x368;
            let x370 = x196 * x369;
            let x371 = x30 + x368;
            let x372 = -1.0 * x161 * x305 + x208 * (-1.0 * x164 + x25 * x7) + x210 * x305 * x365
                - 1.0 * x306 * x364;
            let x373 = -1.0 * x371;
            let x374 = x172 * x373;
            let x375 = x22 * x374;
            let x376 = x220 * x305 * x374;
            let x377 = x340 * x375 + x376;
            let x378 = x17 * x374;
            let x379 = -1.0 * x79 * (2.0 * x10 * x17 * x172 * x373 * x7 - 2.0 * x346 * x375)
                - 1.0 * x84 * (x235 * x375 + x278 * x378)
                - 1.0 * x85 * (x349 * x375 + 2.0 * x376);
            let x380 = x10 * x17 * x172 * x373 * x7 - 1.0 * x346 * x375;
            let x381 = x379 * x73;
            let x382 = x245 * x374 + x378 * x51;
            let x383 = x104 * x382;
            let x384 = x377 * x59;
            let x385 = x384 * x46;
            let x386 = x381 * x57;
            let x387 = x386 * x46;
            let x388 = x108 * x381;
            let x389 = x383 + x385 + x387 + x388;
            let x390 = -1.0 * x389;
            let x391 = -2.0 * x383 - 2.0 * x385 - 2.0 * x387 - 2.0 * x388;
            let x392 = fy * x16;
            let x393 = x392 * x64;
            let x394 = fy * x10;
            let x395 = ty * x46 + x394 * x43;
            let x396 = fy * x115;
            let x397 = fy * x48 + ty * x43;
            let x398 = -1.0 * x16 * x396 + x395 * x60 + x397 * x61;
            let x399 = x395 * x62 - 1.0 * x398 * x46;
            let x400 = x100 * x399;
            let x401 = x36 * x400;
            let x402 = x392 * x60;
            let x403 = x116 * x399;
            let x404 = fy * x68;
            let x405 = x397 * x59;
            let x406 = x36 * x392;
            let x407 = x397 * x52;
            let x408 = x100 * x402;
            let x409 = fy * x10 * x39 * x41 + ty * x144 - 1.0 * x128 * x394 - 1.0 * x394 * x45;
            let x410 = img_dx * x169 * x22;
            let x411 = x175 * x202;
            let x412 = x12 * x31 - 1.0 * x22 * x411;
            let x413 = fy * x256;
            let x414 = x400 * x59;
            let x415 = x392 * x59;
            let x416 = x267 * x410;

            // Jacobian:
            let base = i * stride;
            jac_slice[base + 0] = -1.0 * x101 * x89 - 1.0 * x102 * x86 + x111 * x113
                - 1.0 * x114 * x117
                - 1.0
                    * x123
                    * (x110 * x91 + x118 * x62
                        - 1.0
                            * x46
                            * (x103 * x96
                                + x118 * x60
                                + x119 * x59 * x6
                                + x120 * x88
                                + x121 * x86
                                + x122 * x86
                                + x61 * (fx * x15 * x2 * x47 - 1.0 * x70 * x94 - 1.0 * x9 * x94)
                                + x71 * x91
                                + x87 * x91))
                + x6 * x60 * x65
                + x71 * x72
                + x72 * x87;
            jac_slice[base + 1] = x100 * x113 * x152
                - 1.0 * x102 * x142
                - 1.0 * x117 * x153
                - 1.0
                    * x123
                    * (-1.0 * x144 * x97 + x151 * x91 + x154 * x62
                        - 1.0
                            * x46
                            * (x121 * x142
                                + x122 * x142
                                + x136 * x91
                                + x143 * x91
                                + x146 * x96
                                + x154 * x60
                                + x61 * (fx * x140 - 1.0 * tx * x128 - 1.0 * tx * x45 + x130)))
                + x136 * x72
                + x143 * x72;
            jac_slice[base + 2] = 2.0 * fx * img_dx * k1 * x171 * x195 * x196
                + fx * x10 * x224 * x59 * x63
                + fx * x10 * x239 * x57 * x63 * x73
                + fx * x10 * x254 * x57 * x59 * x99
                + img_dy * k1 * x17 * x172 * x193
                + x10 * x57 * x59 * x63
                - 1.0 * x102 * x244
                - 1.0 * x117 * x255
                - 1.0
                    * x123
                    * (x220 * x62 + x253 * x91
                        - 1.0
                            * x46
                            * (x120 * x242
                                + x121 * x244
                                + x122 * x244
                                + x220 * x60
                                + x233 * x61
                                + x246 * x96
                                + x248 * x91
                                + x250 * x91
                                + x93))
                - 1.0 * x171 * x173
                - 1.0 * x198 * x204
                - 1.0
                    * x213
                    * (k1 * x169 * x17 * x18 + k1 * x195 * x7
                        - 1.0 * x156 * x212
                        - 1.0 * x18 * x31)
                - 1.0 * x242 * x243;
            jac_slice[base + 3] = 2.0 * fx * img_dx * k1 * x171 * x196 * x265
                + fx * x10 * x275 * x59 * x63
                + fx * x10 * x279 * x57 * x63 * x73
                + fx * x10 * x291 * x57 * x59 * x99
                + img_dy * k1 * x17 * x172 * x261
                - 1.0 * k1 * x202 * x256
                - 1.0 * x102 * x281
                - 1.0 * x117 * x292
                - 1.0
                    * x123
                    * (x290 * x91 + x293
                        - 1.0
                            * x46
                            * (x120 * x280
                                + x121 * x281
                                + x122 * x281
                                + x283 * x96
                                + x285 * x91
                                + x287 * x91
                                + x62))
                - 1.0 * x203 * x266 * x267
                - 1.0 * x213 * (x155 * x265 - 1.0 * x156 * x269 + x170)
                - 1.0 * x243 * x280;
            jac_slice[base + 4] = 2.0 * fx * img_dx * k1 * x171 * x196 * x296
                + fx * x10 * x313 * x59 * x63
                + fx * x10 * x320 * x57 * x63 * x73
                + fx * x10 * x331 * x57 * x59 * x99
                + img_dy * k1 * x17 * x172 * x303
                - 1.0 * x102 * x322
                - 1.0 * x117 * x332
                - 1.0
                    * x123
                    * (x330 * x91
                        - 1.0
                            * x46
                            * (x120 * x321
                                + x121 * x322
                                + x122 * x322
                                + x323 * x96
                                + x325 * x91
                                + x327 * x91))
                - 1.0 * x213 * (k1 * x296 * x7 - 1.0 * x156 * x307)
                - 1.0 * x243 * x321
                - 1.0 * x296 * x297;
            jac_slice[base + 5] = 2.0 * fx * img_dx * k1 * x171 * x196 * x336
                + fx * x10 * x345 * x59 * x63
                + fx * x10 * x350 * x57 * x63 * x73
                + fx * x10 * x361 * x57 * x59 * x99
                + img_dy * k1 * x17 * x172 * x338
                - 1.0 * x102 * x352
                - 1.0 * x117 * x362
                - 1.0
                    * x123
                    * (x360 * x91
                        - 1.0
                            * x46
                            * (x120 * x351
                                + x121 * x352
                                + x122 * x352
                                + x353 * x96
                                + x355 * x91
                                + x357 * x91))
                - 1.0 * x213 * (k1 * x336 * x7 - 1.0 * x156 * x339)
                - 1.0 * x243 * x351
                - 1.0 * x297 * x336;
            jac_slice[base + 6] = -1.0 * fx * img_dx * x171 * x370
                + fx * x10 * x377 * x59 * x63
                + fx * x10 * x379 * x57 * x63 * x73
                + fx * x10 * x390 * x57 * x59 * x99
                + img_dy * k1 * x17 * x172 * x366
                + img_dy * k1 * x17 * x196 * x202 * x369
                + img_dy * x17 * x172 * x202
                - 1.0 * x102 * x381
                - 1.0 * x117 * x391
                - 1.0
                    * x123
                    * (x389 * x91
                        - 1.0
                            * x46
                            * (x120 * x380
                                + x121 * x381
                                + x122 * x381
                                + x382 * x96
                                + x384 * x91
                                + x386 * x91))
                - 1.0 * x213 * (-1.0 * x156 * x372 - 1.0 * x169 * x17 * x7 + x371 * x7)
                - 1.0 * x243 * x380;
            jac_slice[base + 7] = 4.0 * fy * x0 * x15 * x5 * x57 * x59 * x63
                + 8.0 * fy * x0 * x2 * x57 * x59 * x63
                - 1.0 * fy * x60 * x64 * x66
                - 1.0 * x111 * x402
                - 1.0 * x114 * x403
                - 1.0
                    * x123
                    * (x110 * x395 + x404 * x62
                        - 1.0
                            * x46
                            * (fy * x61 * x81 + x103 * x405 - 1.0 * x392 * x89
                                + x395 * x71
                                + x395 * x87
                                - 1.0 * x396 * x66
                                + x396 * x70
                                + x396 * x9
                                + x404 * x60
                                - 1.0 * x406 * x86
                                + x407 * x86))
                - 1.0 * x393 * x71
                - 1.0 * x393 * x87
                - 1.0 * x400 * x89
                - 1.0 * x401 * x86;
            jac_slice[base + 8] = -1.0
                * x123
                * (-1.0 * x144 * x398 + x151 * x395 + x409 * x62
                    - 1.0
                        * x46
                        * (x136 * x395 - 1.0 * x142 * x406
                            + x142 * x407
                            + x143 * x395
                            + x146 * x405
                            + x409 * x60
                            + x61 * (fy * x139 - 1.0 * ty * x128 - 1.0 * ty * x45 + x124)))
                - 1.0 * x136 * x393
                - 1.0 * x142 * x401
                - 1.0 * x143 * x393
                - 1.0 * x152 * x408
                - 1.0 * x153 * x403;
            jac_slice[base + 9] = 2.0 * fy * img_dy * k1 * x195 * x196 * x412
                + img_dx * k1 * x172 * x212 * x22
                - 1.0
                    * x123
                    * (x253 * x395
                        - 1.0
                            * x46
                            * (-1.0 * x242 * x415 - 1.0 * x244 * x406
                                + x244 * x407
                                + x246 * x405
                                + x248 * x395
                                + x250 * x395))
                - 1.0 * x198 * x410
                - 1.0 * x242 * x414
                - 1.0 * x244 * x401
                - 1.0 * x248 * x393
                - 1.0 * x250 * x393
                - 1.0 * x254 * x408
                - 1.0 * x255 * x403
                - 1.0 * x413 * (k1 * x12 * x195 - 1.0 * x176 * x193);
            jac_slice[base + 10] = 2.0 * fy * img_dy * k1 * x196 * x265 * x412
                + img_dx * k1 * x172 * x22 * x269
                - 1.0
                    * x123
                    * (x290 * x395
                        - 1.0
                            * x46
                            * (-1.0 * x280 * x415 - 1.0 * x281 * x406
                                + x281 * x407
                                + x283 * x405
                                + x285 * x395
                                + x287 * x395))
                - 1.0 * x265 * x416
                - 1.0 * x280 * x414
                - 1.0 * x281 * x401
                - 1.0 * x285 * x393
                - 1.0 * x287 * x393
                - 1.0 * x291 * x408
                - 1.0 * x292 * x403
                - 1.0 * x413 * (k1 * x12 * x265 - 1.0 * x176 * x261);
            jac_slice[base + 11] = 2.0 * fy * img_dy * k1 * x196 * x296 * x412
                + img_dx * k1 * x172 * x22 * x307
                - 1.0
                    * x123
                    * (x330 * x395
                        - 1.0
                            * x46
                            * (x112 * x43
                                - 1.0 * x115 * x16
                                - 1.0 * x321 * x415
                                - 1.0 * x322 * x406
                                + x322 * x407
                                + x323 * x405
                                + x325 * x395
                                + x327 * x395
                                + x48 * x61)
                        + x53 * x62)
                - 1.0 * x16 * x60 * x64
                - 1.0 * x256 * x412
                - 1.0 * x296 * x416
                - 1.0 * x321 * x414
                - 1.0 * x322 * x401
                - 1.0 * x325 * x393
                - 1.0 * x327 * x393
                - 1.0 * x331 * x408
                - 1.0 * x332 * x403
                - 1.0
                    * x413
                    * (k1 * x12 * x296 + k1 * x202 * x21 * x22
                        - 1.0 * x176 * x303
                        - 1.0 * x21 * x31);
            jac_slice[base + 12] = 2.0 * fy * img_dy * k1 * x196 * x336 * x412
                + img_dx * k1 * x172 * x22 * x339
                - 1.0 * k1 * x169 * x173
                - 1.0
                    * x123
                    * (x293 + x360 * x395
                        - 1.0
                            * x46
                            * (-1.0 * x351 * x415 - 1.0 * x352 * x406
                                + x352 * x407
                                + x353 * x405
                                + x355 * x395
                                + x357 * x395
                                + x62))
                - 1.0 * x336 * x416
                - 1.0 * x351 * x414
                - 1.0 * x352 * x401
                - 1.0 * x355 * x393
                - 1.0 * x357 * x393
                - 1.0 * x361 * x408
                - 1.0 * x362 * x403
                - 1.0 * x413 * (x175 * x336 - 1.0 * x176 * x338 + x411);
            jac_slice[base + 13] = -1.0 * fy * img_dy * x370 * x412
                + img_dx * k1 * x169 * x196 * x22 * x369
                + img_dx * k1 * x172 * x22 * x372
                + img_dx * x169 * x172 * x22
                - 1.0
                    * x123
                    * (x389 * x395
                        - 1.0
                            * x46
                            * (-1.0 * x380 * x415 - 1.0 * x381 * x406
                                + x381 * x407
                                + x382 * x405
                                + x384 * x395
                                + x386 * x395))
                - 1.0 * x380 * x414
                - 1.0 * x381 * x401
                - 1.0 * x384 * x393
                - 1.0 * x386 * x393
                - 1.0 * x390 * x408
                - 1.0 * x391 * x403
                - 1.0 * x413 * (-1.0 * x12 * x202 * x22 + x12 * x371 - 1.0 * x176 * x366);
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
    image_gradients: &[[f32; 2]],
    intrinsic0: &[[f64; 3]; 3],
    dist_coef0: f64,
    theta0: f64,
    epsilon0: f64,
) -> (
    nalgebra::Matrix3<f64>,
    f64,
    f64,
    f64,
    nalgebra::DVector<f64>,
) {
    let initial_params = [
        f64::tan(epsilon0 * 0.25),
        f64::tan(theta0 * 0.25),
        intrinsic0[0][0],
        intrinsic0[0][2],
        intrinsic0[1][1],
        intrinsic0[1][2],
        dist_coef0,
    ];

    // Solve optimization problem with Levenberg-Marquardt algorithm
    let mut problem =
        StarGradCalibrationProblem::new(&initial_params, image_points, image_gradients);
    crate::optim::levenberg_marquardt(&mut problem, 10, 1e-4, 1e-3);

    let params = problem.params.as_slice();

    let epsilon = 4.0 * f64::atan(params[0]);
    let theta = 4.0 * f64::atan(params[1]);
    let intrinsic = nalgebra::Matrix3::new(
        params[2], 0.0, params[3], 0.0, params[4], params[5], 0.0, 0.0, 1.0,
    );
    let dist_coef = params[6];
    let residuals = problem.get_residuals().clone();

    (intrinsic, dist_coef, theta, epsilon, residuals)
}

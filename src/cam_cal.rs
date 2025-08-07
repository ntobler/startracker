use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct CameraParameters {
    pub fx: f64,
    pub fy: f64,
    pub tx: f64,
    pub ty: f64,
    pub cam_resolution: (usize, usize),
    pub dist_coefs: Option<[f64; 5]>,
}

impl CameraParameters {
    pub fn new(
        intrinsic_matrix: Matrix3<f64>,
        cam_resolution: (usize, usize),
        dist_coefs: Option<[f64; 5]>,
    ) -> Self {
        Self {
            fx: intrinsic_matrix[(0, 0)],
            fy: intrinsic_matrix[(1, 1)],
            tx: intrinsic_matrix[(0, 2)],
            ty: intrinsic_matrix[(1, 2)],
            cam_resolution,
            dist_coefs,
        }
    }

    pub fn width(&self) -> usize {
        self.cam_resolution.0
    }
    pub fn height(&self) -> usize {
        self.cam_resolution.1
    }

    pub fn intrinsic_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(self.fx, 0.0, self.tx, 0.0, self.fy, self.ty, 0.0, 0.0, 1.0)
    }

    pub fn undistort(&self, xy_dist: &[f64; 2]) -> [f64; 2] {
        match self.dist_coefs {
            Some(dist_coefs) => {
                let [k1, k2, p1, p2, k3] = dist_coefs;

                let x_0 = (xy_dist[0] - self.tx) / self.fx;
                let y_0 = (xy_dist[1] - self.ty) / self.fy;

                let mut x = x_0;
                let mut y = y_0;

                // only possible with iterative algorithm
                for _ in 0..5 {
                    let r2 = x * x + y * y;
                    let d_inv = 1.0 / (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3)));

                    let delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
                    let delta_y = 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);

                    x = (x_0 - delta_x) * d_inv;
                    y = (y_0 - delta_y) * d_inv;
                }

                [(x * self.fx) + self.tx, (y * self.fy) + self.ty]
            }
            None => [xy_dist[0], xy_dist[1]],
        }
    }

    pub fn distort(&self, xy: &[f64; 2]) -> [f64; 2] {
        match self.dist_coefs {
            Some(dist_coefs) => {
                let [k1, k2, p1, p2, k3] = dist_coefs;

                let x = (xy[0] - self.tx) / self.fx;
                let y = (xy[1] - self.ty) / self.fy;

                let r2 = x * x + y * y;
                let d = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));

                let x_dist = x * d + (2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x));
                let y_dist = y * d + (2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y));

                [(x_dist * self.fx) + self.tx, (y_dist * self.fy) + self.ty]
            }
            None => [xy[0], xy[1]],
        }
    }

    pub fn camera_to_pixels(&self, xyz: &[f64; 3]) -> [f64; 2] {
        let x = (xyz[0] * self.fx) / xyz[2] + self.tx;
        let y = (xyz[1] * self.fy) / xyz[2] + self.ty;
        self.distort(&[x, y])
    }

    pub fn image_coord_to_normed_vector(&self, image_coord: &[f64; 2]) -> [f64; 3] {
        let [mut x, mut y] = self.undistort(image_coord);
        x = (x - self.tx) / self.fx;
        y = (y - self.ty) / self.fy;

        let norm = f64::sqrt(x * x + y * y + 1.0);
        [x / norm, y / norm, 1.0 / norm]
    }

    /// Conservative higher estimate of the angle containing the frame
    pub fn diagonal_angle(&self) -> f64 {
        let diagonal = f64::sqrt(
            (self.cam_resolution.0 * self.cam_resolution.0
                + self.cam_resolution.1 * self.cam_resolution.1) as f64,
        );
        2.0 * f64::atan(diagonal / f64::min(self.fx, self.fy) / 2.0)
    }

    /// Conservative higher estimate of angle per pixels
    pub fn max_angle_per_pixel(&self) -> f64 {
        f64::atan(1.0 / f64::min(self.fx, self.fy))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_testing_utils() {
        let instance = CameraParameters::new(
            nalgebra::Matrix3::new(
                2126.9433827817543,
                0.0,
                471.5660270698475,
                0.0,
                2125.4878604007063,
                310.73929485280405,
                0.0,
                0.0,
                1.0,
            ),
            (960, 540),
            Some([
                -0.4412080745099301,
                -0.159542022464492,
                0.007670124986859448,
                -0.002132920872628578,
                -1.6478824652916775,
            ]),
        );

        let pix = [20.0, 30.0];

        let distorted = instance.distort(&pix);
        let should = [33.14199007, 39.36597996];
        assert!((distorted[0] - should[0]).abs() < 1e-6);
        assert!((distorted[1] - should[1]).abs() < 1e-6);

        let undistorted = instance.undistort(&distorted);
        assert!((undistorted[0] - pix[0]).abs() < 1e-4);
        assert!((undistorted[1] - pix[1]).abs() < 1e-4);

        let res = instance.image_coord_to_normed_vector(&pix);
        let should = [-0.21217088, -0.13259812, 0.96819484];
        assert!((res[0] - should[0]).abs() < 1e-4);
        assert!((res[1] - should[1]).abs() < 1e-4);
        assert!((res[2] - should[2]).abs() < 1e-4);

        let pix2 = instance.camera_to_pixels(&should);
        assert!((pix[0] - pix2[0]).abs() < 1e-4);
        assert!((pix[1] - pix2[1]).abs() < 1e-4);
    }
}

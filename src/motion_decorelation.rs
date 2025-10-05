use crate::cam;
use crate::cam_cal;

pub struct DeviceCameraTransform {
    quat: nalgebra::UnitQuaternion<f64>,
}

impl DeviceCameraTransform {
    pub fn new() -> Self {
        DeviceCameraTransform {
            quat: nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                0.0, 0.0, 0.0, 1.0,
            )),
        }
    }

    pub fn quat_camera_to_device(
        &self,
        quat: &nalgebra::UnitQuaternion<f64>,
    ) -> nalgebra::UnitQuaternion<f64> {
        self.quat.conjugate() * quat * self.quat
    }

    pub fn quat_device_to_camera(
        &self,
        quat: &nalgebra::UnitQuaternion<f64>,
    ) -> nalgebra::UnitQuaternion<f64> {
        self.quat * quat * self.quat.conjugate()
    }
}

pub struct MotionDecorelator {
    cal: cam_cal::CameraParameters,
    relevant_stars_xyz: Vec<nalgebra::Vector3<f64>>,
}

impl MotionDecorelator {
    pub fn new(cal: cam_cal::CameraParameters, relevant_stars_xyz: &[[f64; 3]]) -> Self {
        let relevant_stars_xyz = relevant_stars_xyz
            .iter()
            .map(|v| nalgebra::Vector3::new(v[0] as f64, v[1] as f64, v[2] as f64))
            .collect();
        MotionDecorelator {
            cal,
            relevant_stars_xyz,
        }
    }

    pub fn draw_motion(&self, quats: &[nalgebra::UnitQuaternion<f64>]) -> cam::Frame<u8> {
        let w = self.cal.width();
        let h = self.cal.height();

        if quats.len() == 0 {
            let frame = cam::Frame::<u8>::new(vec![0; w * h], w, h, 0).unwrap();
            return frame;
        }

        let f_dot = 255.0 / quats.len() as f32;

        let max_cos_phi = self.cal.max_cos_phi(1.2);

        let mut canvas = vec![0.0; w * h];

        for star_xyz in &self.relevant_stars_xyz {
            for &quat in quats {
                let q = quat.inverse();
                let xyz = q.transform_vector(star_xyz);

                if xyz.z < max_cos_phi {
                    continue;
                }

                let [x, y] = self
                    .cal
                    .camera_to_pixels(xyz.as_slice().try_into().unwrap());

                let x = x as f32;
                let y = y as f32;

                if (x < 0.0)
                    || (y < 0.0)
                    || (x >= (self.cal.width() - 1) as f32)
                    || (y >= (self.cal.height() - 1) as f32)
                {
                    continue;
                }

                let x0 = x.floor();
                let y0 = y.floor();
                let dx = x - x0;
                let dy = y - y0;

                let dx_n = 1.0 - dx;
                let dy_n = 1.0 - dy;

                let x0 = x0 as usize;
                let y0 = y0 as usize;

                canvas[y0 * w + x0] += f_dot * dy_n * dx_n;
                canvas[y0 * w + x0 + 1] += f_dot * dy_n * dx;
                canvas[(y0 + 1) * w + x0] += f_dot * dy * dx_n;
                canvas[(y0 + 1) * w + x0 + 1] += f_dot * dy * dx;
            }
        }

        cam::Frame::<u8>::new(canvas.iter().map(|x| *x as u8).collect(), w, h, 0).unwrap()
    }
}

use crate::cam_cal::CameraParameters;

pub fn motion_to_pixels(
    quats: &[[f64; 4]],
    camera_params: CameraParameters,
    camera_device_quat: nalgebra::UnitQuaternion<f64>,
) -> Vec<[f32; 2]> {
    if quats.len() == 0 {
        return Vec::new();
    }

    let q = &quats[0];
    let fist_quat = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q[0], q[1], q[2], q[3],
    ));

    let corr_quat = (camera_device_quat * fist_quat).inverse() * camera_device_quat;

    let mut pixels = Vec::with_capacity(quats.len());
    for &q in quats {
        let quat = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q[0], q[1], q[2], q[3],
        ));

        let z = nalgebra::Vector3::new(0.0, 0.0, 1.0);

        let vec = (corr_quat * quat).transform_vector(&z);
        let xyz: [f64; 3] = [vec[0], vec[1], vec[2]];
        let [u, v] = camera_params.camera_to_pixels(&xyz);
        pixels.push([u as f32, v as f32]);
    }
    pixels
}

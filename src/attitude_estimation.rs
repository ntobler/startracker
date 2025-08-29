use serde::{Deserialize, Serialize};

use crate::cam;
use crate::cam_cal;
use crate::common_axis;
use crate::utils;
use ruststartracker::star::{MatchResult, StarMatcher};
use ruststartracker::starcat::StarCatalog;
use ruststartracker::starextraction::extract_observations;

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct AttitudeEstimationConfig {
    min_matches: u32,
    pixel_tolerance: f32,
    timeout_secs: f32,
    threshold_value: u8,
    min_area: usize,
    max_area: usize,
    max_magnitude: f32,
    max_lookup_magnitude: f32,
}

impl Default for AttitudeEstimationConfig {
    fn default() -> Self {
        AttitudeEstimationConfig {
            min_matches: 7,
            pixel_tolerance: 2.0,
            timeout_secs: 0.5,
            threshold_value: 2,
            min_area: 3,
            max_area: 100,
            max_magnitude: 7.5,
            max_lookup_magnitude: 5.5,
        }
    }
}

impl AttitudeEstimationConfig {
    pub fn validate(self) -> Result<Self, String> {
        if self.min_matches < 3 {
            return Err("min_matches must be larger than 3".to_string());
        }
        if self.pixel_tolerance < 0.0 {
            return Err("pixel_tolerance must be larger than 0.0".to_string());
        }
        if self.timeout_secs < 0.001 {
            return Err("timeout_secs must be larger than 1ms".to_string());
        }
        Ok(self)
    }
}

pub struct AttitudeEstimation {
    config: AttitudeEstimationConfig,
    cal: cam_cal::CameraParameters,
    catalog: StarCatalog,
    star_matcher: StarMatcher,
    stars_mag: Vec<f32>,
}

pub struct AttitudeEstimationResult {
    pub quat: [f64; 4],
    pub n_matches: u32,
    pub obs_xy: Vec<[f32; 2]>,
    pub obs_xyz: Vec<[f64; 3]>,
    pub obs_matched_mask: Vec<bool>,
    pub matched_cat_xy: Vec<[f32; 2]>,
    pub matched_cat_xyz: Vec<[f64; 3]>,
    pub matched_cat_mags: Vec<f32>,

    pub processing_time: f32,
    pub post_processing_time: f32,

    pub image_size: [usize; 2],
    pub extrinsic: nalgebra::Matrix3<f64>,
    pub intrinsic: nalgebra::Matrix3<f64>,
    pub dist_coeffs: Vec<f64>,
    pub error_string: Option<String>,
}

impl AttitudeEstimation {
    pub fn new(
        config: AttitudeEstimationConfig,
        cal: cam_cal::CameraParameters,
    ) -> Result<Self, String> {
        let config = config.validate()?;

        let catalog: StarCatalog =
            StarCatalog::new_from_hipparcos(Some(config.max_magnitude as f64))?;

        let stars_xyz: Vec<[f32; 3]> = catalog.normalized_positions(None, None);
        let stars_mag: Vec<f32> = catalog.magnitudes();

        let max_inter_star_angle = cal.diagonal_angle() as f32;
        let inter_star_angle_tolerance = config.pixel_tolerance * cal.max_angle_per_pixel() as f32;

        let star_matcher = StarMatcher::new(
            stars_xyz,
            &stars_mag,
            config.max_lookup_magnitude,
            max_inter_star_angle,
            inter_star_angle_tolerance,
            config.min_matches as usize,
            config.timeout_secs,
        )?;

        Ok(AttitudeEstimation {
            config,
            cal,
            catalog,
            star_matcher,
            stars_mag,
        })
    }

    pub fn extract_cat_stars(&self, max_magnitude: f32) -> (Vec<[f64; 3]>, Vec<f32>) {
        let xyz = self
            .star_matcher
            .stars_xyz()
            .iter()
            .zip(self.stars_mag.iter())
            .filter(|(_, &mag)| mag <= max_magnitude)
            .map(|(&xyz, _)| [xyz[0] as f64, xyz[1] as f64, xyz[2] as f64])
            .collect();
        let mags = self
            .stars_mag
            .iter()
            .filter(|&mag| *mag <= max_magnitude)
            .map(|&mag| mag)
            .collect();
        (xyz, mags)
    }

    pub fn estimate_attitude(
        &self,
        obs_xy: Vec<[f64; 2]>,
    ) -> Result<AttitudeEstimationResult, String> {
        let start_instant = std::time::Instant::now();

        let obs_xy_f32: Vec<[f32; 2]> = obs_xy
            .iter()
            .map(|xy| [xy[0] as f32, xy[1] as f32])
            .collect();

        // Convert 2D observations to 3D by adding a z-coordinate
        let obs_xyz_camera: Vec<[f64; 3]> = obs_xy
            .iter()
            .map(|xy| self.cal.image_coord_to_normed_vector(xy))
            .collect();

        let obs_xyz_camera_f32: Vec<[f32; 3]> = obs_xyz_camera
            .iter()
            .map(|xyz| [xyz[0] as f32, xyz[1] as f32, xyz[2] as f32])
            .collect();

        let (res, error_string) = match self.star_matcher.find(&obs_xyz_camera_f32) {
            Ok(res) => (res, None),
            Err(e) => (
                MatchResult {
                    quat: [1.0, 0.0, 0.0, 0.0],
                    match_ids: vec![],
                    n_matches: 0,
                    obs_matched: vec![],
                    obs_indices: vec![],
                },
                Some(e.to_string()),
            ),
        };

        let processing_time = start_instant.elapsed().as_secs_f32();

        let quat_vec = [
            res.quat[0] as f64,
            res.quat[1] as f64,
            res.quat[2] as f64,
            res.quat[3] as f64,
        ];
        let quat: nalgebra::Unit<nalgebra::Quaternion<f64>> =
            nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                quat_vec[3],
                quat_vec[0],
                quat_vec[1],
                quat_vec[2],
            ));

        let extrinsic: nalgebra::Matrix3<f64> = quat.to_rotation_matrix().inverse().into_inner();

        let obs_xyz = dmatrix_to_array(extrinsic.transpose() * array_to_dmatrix(obs_xyz_camera));

        // Calculate catalog star coordinates
        let all_cat_xyz = self.star_matcher.stars_xyz();
        let matched_cat_xyz: Vec<[f32; 3]> = res
            .match_ids
            .iter()
            .map(|&i| all_cat_xyz[i as usize])
            .collect();
        let matched_cat_xyz_f64: Vec<[f64; 3]> = matched_cat_xyz
            .iter()
            .map(|&i| [i[0] as f64, i[1] as f64, i[2] as f64])
            .collect();
        let matched_cat_xyz_camera = extrinsic * slice_to_matrix(&matched_cat_xyz_f64);
        let matched_cat_xy: Vec<[f32; 2]> = matched_cat_xyz_camera
            .column_iter()
            .map(|xyz| {
                let array_ref: &[f64; 3] = xyz.as_slice().try_into().unwrap();
                let xy = self.cal.camera_to_pixels(array_ref);
                [xy[0] as f32, xy[1] as f32]
            })
            .collect();
        let matched_cat_mags = res
            .match_ids
            .iter()
            .map(|&i| self.catalog.stars[i as usize].magnitude as f32)
            .collect();

        let intrinsic = self.cal.intrinsic_matrix();
        let dist_coeffs = match self.cal.dist_coefs {
            Some(coefs) => coefs.to_vec(),
            None => vec![],
        };

        let post_processing_time = start_instant.elapsed().as_secs_f32() - processing_time;

        let mut obs_matched_mask = vec![false; obs_xy.len()];
        for i in res.obs_indices {
            obs_matched_mask[i as usize] = true;
        }

        Ok(AttitudeEstimationResult {
            quat: quat_vec,
            n_matches: res.n_matches as u32,
            obs_xy: obs_xy_f32,
            obs_xyz: obs_xyz,
            obs_matched_mask: obs_matched_mask,
            matched_cat_xy,
            matched_cat_xyz: matched_cat_xyz_f64,
            matched_cat_mags,
            processing_time: processing_time * 1000.0,
            post_processing_time: post_processing_time * 1000.0,
            image_size: [self.cal.width(), self.cal.height()],
            extrinsic: extrinsic,
            intrinsic: intrinsic,
            dist_coeffs: dist_coeffs,
            error_string: error_string,
        })
    }

    pub fn estimate_attitude_from_image(
        &self,
        image: &cam::Frame<u8>,
    ) -> Result<AttitudeEstimationResult, String> {
        // Extract observations from the image
        let (obs_xy, _) = extract_observations(
            image.data_row_major.as_slice(),
            (image.width, image.height),
            self.config.threshold_value,
            self.config.min_area,
            self.config.max_area,
        )?;

        self.estimate_attitude(obs_xy)
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct AttitudeEstimationPayload {
    pub quat: [f64; 4],
    pub n_matches: u32,
    pub alignment_error: f64,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub matched_obs_radial: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub north_south: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub obs_xy: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    pub obs_matched_mask: Vec<bool>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub cat_xy: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub cat_radial: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    pub cat_mags: Vec<f32>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub frame_points_radial: Vec<[f32; 2]>,

    pub processing_time: f32,
    pub post_processing_time: f32,

    pub image_size: [usize; 2],
    pub extrinsic: Vec<f64>,
    pub intrinsic: Vec<f64>,
    pub dist_coeffs: Vec<f64>,
    pub error_string: Option<String>,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct AxisCalibrationState {
    error_deg: Option<f64>,
    orientations: u32,
    error_string: Option<String>,
}

pub struct AxisCalibration {
    rotations: Vec<nalgebra::Rotation<f64, 3>>,
    calibrated_rotation: nalgebra::Rotation<f64, 3>,
    error_deg: Option<f64>,
    error_string: Option<String>,
}

impl AxisCalibration {
    pub fn new() -> Self {
        AxisCalibration {
            rotations: Vec::new(),
            calibrated_rotation: nalgebra::Rotation::identity(),
            error_deg: None,
            error_string: None,
        }
    }

    pub fn reset(&mut self) {
        self.rotations.clear();
    }

    pub fn put(&mut self, rot: &nalgebra::Rotation<f64, 3>) {
        self.rotations.push(rot.clone());
    }

    pub fn calibrate(&mut self) {
        // Convert rotations to matrices
        let rot_mats: Vec<[f64; 9]> = self
            .rotations
            .iter()
            .map(|r| r.matrix().transpose().as_slice().try_into().unwrap())
            .collect();

        // Find common axis
        match common_axis::find_common_axis(&rot_mats, 1e-6, 20) {
            Ok((axis, _, _, estimated_std_rad)) => {
                let target = nalgebra::Vector3::new(axis[0], axis[1], axis[2]); // what it looks at
                let up = nalgebra::Vector3::new(0.0, -1.0, 0.0);
                self.calibrated_rotation = nalgebra::Rotation3::look_at_lh(&target, &up)
                    * nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::<f64>::new(
                        -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
                    ));

                self.error_deg = Some(estimated_std_rad * (180.0 / std::f64::consts::PI));
                self.error_string = None;
            }
            Err(e) => {
                self.error_string = Some(e.to_string());
            }
        }
    }

    pub fn get_state(&self) -> AxisCalibrationState {
        AxisCalibrationState {
            error_deg: self.error_deg,
            orientations: self.rotations.len() as u32,
            error_string: self.error_string.clone(),
        }
    }

    /// Transform points from celestial to corrected camera frame
    pub fn transform_to_corrected(
        &self,
        xyz: &[[f64; 3]],
        extrinsic: &nalgebra::Matrix3<f64>,
    ) -> Vec<[f64; 3]> {
        let xyz_mat = slice_to_matrix(&xyz);
        let rot_mat = self.calibrated_rotation.matrix().transpose() * extrinsic;
        dmatrix_to_array(rot_mat * xyz_mat)
    }

    pub fn correct_attitude_estimation_result(
        &self,
        att: &AttitudeEstimation,
        res: AttitudeEstimationResult,
    ) -> AttitudeEstimationPayload {
        //Find alignment error
        let alignment_error = nalgebra::Rotation3::from_matrix_unchecked(
            res.extrinsic * self.calibrated_rotation.matrix().transpose(),
        )
        .angle()
            * 180.0
            / std::f64::consts::PI;

        // TODO populate
        let frame_points_xyz = att.cal.get_distorted_camera_frame(10);
        let frame_points_aligned =
            self.transform_to_corrected(&frame_points_xyz, &nalgebra::Matrix3::identity());

        let matched_obs_xyz = res
            .obs_matched_mask
            .iter()
            .enumerate()
            .filter_map(|(i, &m)| if m { Some(res.obs_xyz[i]) } else { None })
            .collect::<Vec<_>>();

        let matched_obs_xyz_aligned = self.transform_to_corrected(&matched_obs_xyz, &res.extrinsic);
        let matched_cat_xyz_aligned =
            self.transform_to_corrected(&res.matched_cat_xyz, &res.extrinsic);

        let (bright_cat_xyz, bright_cat_mags) = att.extract_cat_stars(4.0);
        let bright_cat_xyz_aligned = self.transform_to_corrected(&bright_cat_xyz, &res.extrinsic);

        let cat_xyz_aligned = [matched_cat_xyz_aligned, bright_cat_xyz_aligned].concat();
        let cat_mags = [res.matched_cat_mags, bright_cat_mags].concat();

        let north_south_aligned =
            self.transform_to_corrected(&vec![[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]], &res.extrinsic);

        let matched_obs_radial = to_radial_vec(&matched_obs_xyz_aligned);
        let cat_radial = to_radial_vec(&cat_xyz_aligned);
        let north_south_radial = to_radial_vec(&north_south_aligned);
        let frame_points_radial = to_radial_vec(&frame_points_aligned);

        AttitudeEstimationPayload {
            quat: res.quat,
            n_matches: res.n_matches,
            alignment_error,
            matched_obs_radial: matched_obs_radial,
            north_south: north_south_radial,
            obs_xy: res.obs_xy,
            obs_matched_mask: res.obs_matched_mask,
            cat_xy: res.matched_cat_xy,
            cat_radial: cat_radial,
            cat_mags: cat_mags,
            frame_points_radial: frame_points_radial,
            processing_time: res.processing_time,
            post_processing_time: res.post_processing_time,
            image_size: res.image_size,
            extrinsic: res.extrinsic.transpose().as_slice().to_vec(),
            intrinsic: res.intrinsic.transpose().as_slice().to_vec(),
            dist_coeffs: res.dist_coeffs,
            error_string: res.error_string,
        }
    }
}

fn to_radial_vec(xyz: &[[f64; 3]]) -> Vec<[f32; 2]> {
    xyz.iter()
        .map(|xyz| {
            let r = (xyz[0] * xyz[0] + xyz[1] * xyz[1]).sqrt();
            let s = f64::atan2(r, xyz[2]) * 180.0 / std::f64::consts::PI / r;
            [(xyz[0] * s) as f32, (xyz[1] * s) as f32]
        })
        .collect()
}

fn flatten_vec_reinterpret<T, const N: usize>(mut v: Vec<[T; N]>) -> Vec<T> {
    let len = v.len() * N;
    let cap = v.capacity() * N;
    let ptr = v.as_mut_ptr() as *mut T;
    // Prevent Vec<[T; N]> from dropping its elements
    std::mem::forget(v);
    unsafe { Vec::from_raw_parts(ptr, len, cap) }
}

pub fn array_to_dmatrix<'a, const N: usize, T>(v: Vec<[T; N]>) -> nalgebra::DMatrix<T>
where
    T: nalgebra::Scalar,
{
    let len = v.len();
    let flat = flatten_vec_reinterpret(v);
    let storage = nalgebra::VecStorage::new(
        nalgebra::Dim::from_usize(N),
        nalgebra::Dim::from_usize(len),
        flat,
    );
    nalgebra::DMatrix::from_vec_storage(storage)
}

pub fn dmatrix_to_array<'a, const N: usize, T>(
    m: nalgebra::Matrix<
        T,
        nalgebra::Const<N>,
        nalgebra::Dyn,
        nalgebra::VecStorage<T, nalgebra::Const<N>, nalgebra::Dyn>,
    >,
) -> Vec<[T; N]>
where
    T: nalgebra::Scalar,
{
    let v: Vec<T> = m.data.as_slice().to_vec();
    let len = v.len() / N;
    let cap = v.capacity() / N;
    let ptr = v.as_ptr() as *mut [T; N];
    // Prevent Vec<T> from dropping its elements
    std::mem::forget(v);
    unsafe { Vec::from_raw_parts(ptr, len, cap) }
}

pub fn slice_to_matrix<'a, const N: usize, T>(
    v: &'a [[T; N]],
) -> nalgebra::MatrixView<'a, T, nalgebra::Const<N>, nalgebra::Dyn>
where
    T: nalgebra::Scalar,
{
    let rows = v.len();
    let total_len = rows * N;

    // safe because &[ [T; N] ] is contiguous
    let flat: &'a [T] = unsafe { std::slice::from_raw_parts(v.as_ptr() as *const T, total_len) };

    nalgebra::MatrixView::<T, nalgebra::Const<N>, nalgebra::Dyn>::from_slice(flat, rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Const, Dyn, MatrixView};

    #[test]
    fn test_slice_to_matrix() {
        let data = [[1, 2, 3], [4, 5, 6]];

        let mat: MatrixView<'_, _, Const<3>, Dyn> = slice_to_matrix(&data);

        // Check dimensions
        assert_eq!(mat.nrows(), 3);
        assert_eq!(mat.ncols(), 2);

        // Check values
        assert_eq!(mat[(0, 0)], 1);
        assert_eq!(mat[(1, 0)], 2);
        assert_eq!(mat[(2, 0)], 3);
        assert_eq!(mat[(0, 1)], 4);
        assert_eq!(mat[(1, 1)], 5);
        assert_eq!(mat[(2, 1)], 6);
    }
}

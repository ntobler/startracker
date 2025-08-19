use serde::{Deserialize, Serialize};

use crate::cam;
use crate::cam_cal;
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
    frame_points: Vec<[f32; 2]>,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct AttitudeEstimationResult {
    pub quat: [f64; 4],
    pub n_matches: u32,
    // pub star_coords: to_rounded_list(star_coords, 2),
    pub alignment_error: f64,
    // pub north_south: to_rounded_list(north_south, 2),
    // pub frame_points: self._camera_frame,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub obs_xy: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub obs_xyz: Vec<[f32; 3]>,
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    pub obs_matched_mask: Vec<bool>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub cat_xy: Vec<[f32; 2]>,
    #[serde(serialize_with = "utils::contiguous_serialize_2d")]
    pub cat_xyz: Vec<[f32; 3]>,
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    pub cat_mags: Vec<f32>,
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    pub frame_points: Vec<[f32; 2]>,

    pub processing_time: f32,
    pub post_processing_time: f32,

    pub image_size: [usize; 2],
    pub extrinsic: Vec<f64>,
    pub intrinsic: Vec<f64>,
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

        // TODO populate
        let frame_points = vec![];

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
            frame_points,
        })
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

        let extrinsic: nalgebra::Rotation<f64, 3> = quat.to_rotation_matrix();

        let obs_xyz = (extrinsic.transpose() * view_as_dmatrix(&obs_xyz_camera))
            .column_iter()
            .map(|xyz| {
                let array_ref: &[f64; 3] = xyz.as_slice().try_into().unwrap();
                [
                    array_ref[0] as f32,
                    array_ref[1] as f32,
                    array_ref[2] as f32,
                ]
            })
            .collect();

        // Calculate catalog star coordinates
        let all_cat_xyz = self.star_matcher.stars_xyz();
        let cat_xyz: Vec<[f32; 3]> = res
            .match_ids
            .iter()
            .map(|&i| all_cat_xyz[i as usize])
            .collect();
        let cat_xyz_f64: Vec<[f64; 3]> = cat_xyz
            .iter()
            .map(|&i| [i[0] as f64, i[1] as f64, i[2] as f64])
            .collect();
        let cat_camera = extrinsic.transpose() * view_as_dmatrix(&cat_xyz_f64);
        let cat_xy: Vec<[f32; 2]> = cat_camera
            .column_iter()
            .map(|xyz| {
                let array_ref: &[f64; 3] = xyz.as_slice().try_into().unwrap();
                let xy = self.cal.camera_to_pixels(array_ref);
                [xy[0] as f32, xy[1] as f32]
            })
            .collect();
        let cat_mags = res
            .match_ids
            .iter()
            .map(|&i| self.catalog.stars[i as usize].magnitude as f32)
            .collect();

        let alignment_error = 0.0; // Placeholder for alignment error calculation

        let intrinsic = self.cal.intrinsic_matrix().as_slice().to_vec();
        let dist_coeffs = match self.cal.dist_coefs {
            Some(coefs) => coefs.to_vec(),
            None => vec![],
        };
        let extrinsic = extrinsic.transpose().matrix().as_slice().to_vec();

        let post_processing_time = start_instant.elapsed().as_secs_f32() - processing_time;

        let mut obs_matched_mask = vec![false; obs_xy.len()];
        for i in res.obs_indices {
            obs_matched_mask[i as usize] = true;
        }

        Ok(AttitudeEstimationResult {
            quat: quat_vec,
            n_matches: res.n_matches as u32,
            alignment_error: alignment_error,
            obs_xy: obs_xy_f32,
            obs_xyz: obs_xyz,
            obs_matched_mask: obs_matched_mask,
            cat_xy: cat_xy,
            cat_xyz: cat_xyz,
            cat_mags: cat_mags,
            frame_points: self.frame_points.clone(),
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

pub fn view_as_dmatrix<'a, const N: usize, T>(points: &'a [[T; N]]) -> nalgebra::DMatrix<T>
where
    T: nalgebra::Scalar,
{
    let len = points.len(); // n
    let ptr = points.as_ptr() as *const T;
    let total_len = len * N;

    let flat: &'a [T] = unsafe { std::slice::from_raw_parts(ptr, total_len) };

    nalgebra::DMatrix::from_column_slice(N, len, flat)
}

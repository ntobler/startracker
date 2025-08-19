use arc_swap::ArcSwap;
use opencv;
use opencv::prelude::*;
use rmp_serde;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::attitude_estimation;
use crate::cam;
use crate::cam_cal;
use crate::opencvutils;
use crate::utils;
use crate::webutils;

#[derive(Serialize, Deserialize, Copy, Clone)]
pub enum CameraModeRequest {
    Single,
    Continuous,
    ToggleContinuous,
    Stop,
    Darkframe,
}

#[derive(Serialize, Deserialize, Copy, Clone, PartialEq)]
enum CameraMode {
    Single,
    Continuous,
    Stop,
    Darkframe,
}

enum AcqusitionRequest {
    None,
    Normal,
    Darkframe,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
enum ImageType {
    Raw,
    Processed,
    Crop2x,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub enum AutoCalibrationRequest {
    Restart,
    Discard,
    Accept,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub enum AxisCalibrationRequest {
    Put,
    Reset,
    Calibrate,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct ViewSettings {
    /// Overlay star coordinate frame.
    coordinate_frame: bool,
    /// Brightness for the camera display
    brightness: u32,
    /// Image type to show. One of raw, processed, or crop2x
    image_type: ImageType,
    /// Target quality for the image in kilobytes.
    target_quality_kb: u32,
}

impl Default for ViewSettings {
    fn default() -> Self {
        ViewSettings {
            coordinate_frame: false,
            brightness: 1,
            image_type: ImageType::Raw,
            target_quality_kb: 50,
        }
    }
}

impl ViewSettings {
    fn validate(self) -> Result<Self, String> {
        Ok(self)
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct Settings {
    camera_config: Option<cam::CameraConfig>,
    view_settings: Option<ViewSettings>,
    attitude_est_config: Option<attitude_estimation::AttitudeEstimationConfig>,
    send_image: Option<bool>,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct Persistent {
    camera_config: cam::CameraConfig,
    view_settings: ViewSettings,
    attitude_est_config: attitude_estimation::AttitudeEstimationConfig,
    cal: Option<cam_cal::CameraParameters>,
    send_image: bool,
}

impl Persistent {
    fn validate(self) -> Result<Self, String> {
        self.camera_config.validate()?;
        self.view_settings.validate()?;
        self.attitude_est_config.validate()?;
        Ok(self)
    }
}

impl Default for Persistent {
    fn default() -> Self {
        Persistent {
            camera_config: cam::CameraConfig {
                analogue_gain: 5,
                digital_gain: 2,
                exposure_us: 500_000,
                binning: 2,
            },
            view_settings: ViewSettings::default(),
            attitude_est_config: attitude_estimation::AttitudeEstimationConfig::default(),
            cal: None,
            send_image: false,
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct AxisCalibrationState {
    error_deg: Option<f64>,
    orientations: u32,
}

struct AxisCalibration {
    rotations: Vec<nalgebra::Rotation<f64, 3>>,
    calibrated_rotation: nalgebra::Rotation<f64, 3>,
    error_deg: Option<f64>,
}

impl AxisCalibration {
    pub fn new() -> Self {
        AxisCalibration {
            rotations: Vec::new(),
            calibrated_rotation: nalgebra::Rotation::identity(),
            error_deg: None,
        }
    }

    pub fn reset(&mut self) {
        self.rotations.clear();
    }

    pub fn put(&mut self, rot: &nalgebra::Rotation<f64, 3>) {
        self.rotations.push(rot.clone());
    }

    pub fn calibrate(&mut self) {}

    pub fn get_state(&self) -> AxisCalibrationState {
        AxisCalibrationState {
            error_deg: self.error_deg,
            orientations: self.rotations.len() as u32,
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct AutoCalibrationState {
    active: bool,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct AutoCalibration {
    active: bool,
}

impl AutoCalibration {
    fn new() -> Self {
        AutoCalibration { active: false }
    }

    pub fn get_state(&self) -> AutoCalibrationState {
        AutoCalibrationState { active: false }
    }
}

struct State {
    persistent: Persistent,
    axis_calibration: AxisCalibration,
    auto_calibration: AutoCalibration,
    camera_mode: CameraMode,
    attitude: Option<nalgebra::Rotation<f64, 3>>,
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct PublicState {
    persistent: Persistent,
    camera_mode: CameraMode,
    axis_calibration: AxisCalibrationState,
}

impl State {
    fn set_persistent(&mut self, settings: Settings) -> Result<(), String> {
        self.persistent = Persistent {
            camera_config: settings
                .camera_config
                .unwrap_or_else(|| self.persistent.camera_config),
            view_settings: settings
                .view_settings
                .unwrap_or_else(|| self.persistent.view_settings),
            attitude_est_config: settings
                .attitude_est_config
                .unwrap_or_else(|| self.persistent.attitude_est_config),
            cal: self.persistent.cal,
            send_image: settings
                .send_image
                .unwrap_or_else(|| self.persistent.send_image),
        }
        .validate()?;
        Ok(())
    }
}

pub struct App {
    config_path: PathBuf,
    state: Mutex<State>,
    pub stream_dispatcher: webutils::DataDispatcher<Vec<u8>>,
    pub running: AtomicBool,
    attitude_estimation: ArcSwap<Option<attitude_estimation::AttitudeEstimation>>,
}

impl App {
    pub fn load_or_default<P: AsRef<Path>>(filename: P) -> Self {
        let persistent = match std::fs::read_to_string(&filename) {
            Ok(data) => match serde_json::from_str(&data) {
                Ok(data) => data,
                Err(_) => {
                    eprintln!("Failed to load persistent data, using defaults.");
                    Persistent::default()
                }
            },
            Err(e) => {
                eprintln!(
                    "Failed to read persistent data file: {}. Using defaults.",
                    e
                );
                Persistent::default()
            }
        };

        let state = State {
            persistent,
            axis_calibration: AxisCalibration::new(),
            auto_calibration: AutoCalibration::new(),
            camera_mode: CameraMode::Stop,
            attitude: None,
        };

        App {
            config_path: filename.as_ref().to_path_buf(),
            state: Mutex::new(state),
            stream_dispatcher: webutils::DataDispatcher::<Vec<u8>>::new(),
            running: AtomicBool::new(true),
            attitude_estimation: ArcSwap::new(Arc::new(None)),
        }
    }

    pub fn get_state(&self) -> Result<PublicState, String> {
        let state_ref = self.state.lock().map_err(|e| e.to_string())?;

        Ok(PublicState {
            persistent: state_ref.persistent.clone(),
            camera_mode: state_ref.camera_mode,
            axis_calibration: state_ref.axis_calibration.get_state(),
        })
    }

    pub fn set_settings(&self, settings: Settings) -> Result<PublicState, String> {
        {
            let mut state_ref = self.state.lock().map_err(|e| e.to_string())?;
            state_ref.set_persistent(settings)?;
        }
        self.get_state()
    }

    pub fn set_camera_mode(&self, mode: CameraModeRequest) -> Result<PublicState, String> {
        {
            let mut state_ref = self.state.lock().map_err(|e| e.to_string())?;
            state_ref.camera_mode = match mode {
                CameraModeRequest::Single => CameraMode::Single,
                CameraModeRequest::Continuous => CameraMode::Continuous,
                CameraModeRequest::ToggleContinuous => {
                    if state_ref.camera_mode == CameraMode::Continuous {
                        CameraMode::Stop
                    } else {
                        CameraMode::Continuous
                    }
                }
                CameraModeRequest::Stop => CameraMode::Stop,
                CameraModeRequest::Darkframe => CameraMode::Darkframe,
            };
        }
        self.get_state()
    }

    pub fn axis_calibration(&self, request: AxisCalibrationRequest) -> Result<PublicState, String> {
        {
            let mut state_ref = self.state.lock().map_err(|e| e.to_string())?;
            match request {
                AxisCalibrationRequest::Put => match state_ref.attitude {
                    Some(attitude) => state_ref.axis_calibration.put(&attitude),
                    None => return Err("No attitude available for calibration.".to_string()),
                },
                AxisCalibrationRequest::Reset => {
                    state_ref.axis_calibration.reset();
                }
                AxisCalibrationRequest::Calibrate => {
                    state_ref.axis_calibration.calibrate();
                }
            }
        }
        self.get_state()
    }

    pub fn auto_calibration(&self, request: AutoCalibrationRequest) -> Result<PublicState, String> {
        match request {
            AutoCalibrationRequest::Restart => {
                // TODO implement
            }
            AutoCalibrationRequest::Discard => {
                // Discard current calibration
                // TODO implement
            }
            AutoCalibrationRequest::Accept => {
                // Accept current calibration
                // TODO Example calibration parameters, replace with actual calibration logic
                let instance1 = cam_cal::CameraParameters::new(
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

                {
                    let mut state_ref = self.state.lock().map_err(|e| e.to_string())?;
                    state_ref.persistent.cal = Some(instance1);
                }

                self.init_attitude_estimation()?;
            }
        }
        self.get_state()
    }

    fn init_attitude_estimation(&self) -> Result<(), String> {
        let (att_config, cal) = {
            let state_ref = self.state.lock().map_err(|e| e.to_string())?;
            (
                state_ref.persistent.attitude_est_config.clone(),
                state_ref.persistent.cal.clone(),
            )
        };

        self.attitude_estimation.store(Arc::new(match cal {
            Some(cal) => Some(
                attitude_estimation::AttitudeEstimation::new(att_config, cal)
                    .map_err(|e| format!("Error initializing attitude estimation: {}.", e))?,
            ),
            None => None,
        }));
        Ok(())
    }
}

impl Drop for App {
    fn drop(&mut self) {
        let persistent = { self.state.lock().unwrap().persistent.clone() };

        if let Err(e) = std::fs::write(
            &self.config_path,
            serde_json::to_string(&persistent).unwrap(),
        ) {
            eprintln!("Failed to save persistent data: {}", e);
        }
    }
}

#[derive(Serialize)]
struct StreamObject {
    #[serde(serialize_with = "utils::contiguous_serialize_1d")]
    encoded_frame: Vec<u8>,
    image_size: (usize, usize),
    image_quality: String,
    auto_calibrator: AutoCalibrationState,
    #[serde(serialize_with = "attitude_serialize")]
    attitude_estimation: Result<attitude_estimation::AttitudeEstimationResult, String>,
    pre_processing_time: f32,
}

impl StreamObject {
    fn new(
        encoded_frame: Vec<u8>,
        image_size: (usize, usize),
        image_quality: String,
        attitude_estimation: Result<attitude_estimation::AttitudeEstimationResult, String>,
        pre_processing_time: f32,
    ) -> Self {
        let auto_calibrator = AutoCalibrationState { active: false };
        StreamObject {
            encoded_frame: encoded_frame,
            image_size: image_size,
            image_quality: image_quality,
            auto_calibrator: auto_calibrator,
            attitude_estimation: attitude_estimation,
            pre_processing_time: pre_processing_time,
        }
    }
}

fn attitude_serialize<S>(
    val: &Result<attitude_estimation::AttitudeEstimationResult, String>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    match val {
        Ok(v) => v.serialize(serializer),
        Err(e) => {
            // serialize as { "error": e }
            let mut map: std::collections::BTreeMap<&'static str, String> =
                std::collections::BTreeMap::new();
            map.insert("error", e.clone());
            map.serialize(serializer)
        }
    }
}

pub fn tick(app: &App) -> Result<(), String> {
    let stream_dispatcher = &app.stream_dispatcher;
    let running = &app.running;

    println!("Setup Camera");

    let cam_config = {
        app.state
            .lock()
            .map_err(|e| e.to_string())?
            .persistent
            .camera_config
            .clone()
    };

    let mut camera =
        cam::Camera::new(&cam_config).map_err(|e| format!("Error initializing camera: {}.", e))?;

    app.init_attitude_estimation()?;

    let mut ie = opencvutils::ImageEncoder::new(Some(30.0));

    while running.load(Ordering::Acquire) {
        //Update config
        let (acquisition_request, image_type, send_image) = {
            let mut state_ref = app.state.lock().map_err(|e| e.to_string())?;

            match state_ref.persistent.view_settings.target_quality_kb {
                0u32 => ie.set_max_kb(None),
                x => ie.set_max_kb(Some(x as f32)),
            }
            camera.set_config(&state_ref.persistent.camera_config);
            let request_mode = match state_ref.camera_mode {
                CameraMode::Single => AcqusitionRequest::Normal,
                CameraMode::Continuous => AcqusitionRequest::Normal,
                CameraMode::Stop => AcqusitionRequest::None,
                CameraMode::Darkframe => AcqusitionRequest::Darkframe,
            };

            state_ref.camera_mode = match state_ref.camera_mode {
                CameraMode::Single => CameraMode::Stop,
                CameraMode::Continuous => CameraMode::Continuous,
                CameraMode::Stop => CameraMode::Stop,
                CameraMode::Darkframe => CameraMode::Stop,
            };

            (
                request_mode,
                state_ref.persistent.view_settings.image_type,
                state_ref.persistent.send_image,
            )
        };

        // Acquire image from camera
        let raw: cam::Frame<u8> = match acquisition_request {
            AcqusitionRequest::None => {
                // No acquisition, just sleep
                std::thread::sleep(std::time::Duration::from_millis(10));
                continue;
            }
            AcqusitionRequest::Normal => {
                // Normal acquisition, get fresh image
                match camera.capture() {
                    Ok(v) => v,
                    Err(e) => {
                        eprintln!("Error getting camera image: {}.", e);
                        std::thread::sleep(std::time::Duration::from_millis(10));
                        continue;
                    }
                }
            }
            AcqusitionRequest::Darkframe => {
                if let Err(e) = camera.record_darkframe() {
                    eprintln!("Error getting camera image: {}.", e);
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
                continue; // Skip further processing for darkframe
            }
        };

        // Process image
        let start_instant = std::time::Instant::now();
        let preprocessed = preprocess(&raw)?;
        let pre_processing_time = start_instant.elapsed().as_secs_f32() * 1000.0;

        // Find attitude estimation result
        let att_result = match app.attitude_estimation.load_full().as_ref() {
            Some(att_est) => att_est.estimate_attitude_from_image(&preprocessed),
            None => Err("attitude estimation not available".to_string()),
        };

        let encoded_frame = if send_image {
            // Chose image to send
            let send_image = match image_type {
                ImageType::Raw => &raw,
                ImageType::Processed => &preprocessed,
                ImageType::Crop2x => {
                    // TODO crop to 2x
                    // let crop = opencvutils::crop_to_2x(&raw);
                    &raw
                }
            };

            // Encode image
            match ie.encode(send_image) {
                Some(v) => v,
                None => vec![],
            }
        } else {
            vec![]
        };

        // Send stream
        let stream_object = StreamObject::new(
            encoded_frame,
            (raw.width, raw.height),
            ie.quality_str(),
            att_result,
            pre_processing_time,
        );
        let encoded = rmp_serde::to_vec_named(&stream_object)
            .expect("Failed to serialize stream in MessagePack format");
        stream_dispatcher.put(encoded);
    }

    println!("Camera thread stopped gracefully");
    Ok(())
}

fn preprocess(raw: &cam::Frame<u8>) -> Result<cam::Frame<u8>, String> {
    // Create OpenCV Mat from raw data
    let raw_mat = opencv::core::Mat::new_rows_cols_with_data(
        raw.height as i32,
        raw.width as i32,
        &raw.data_row_major,
    )
    .map_err(|e| format!("Failed to create Mat: {}", e))?;

    // 7x7 top hat filter
    let kernel = opencv::core::Mat::ones(7, 7, opencv::core::CV_8U).unwrap();
    let mut morth_result = opencv::core::Mat::default();
    opencv::imgproc::morphology_ex(
        &raw_mat,
        &mut morth_result,
        opencv::imgproc::MORPH_TOPHAT,
        &kernel,
        opencv::core::Point::new(-1, -1),
        1,
        opencv::core::BORDER_DEFAULT,
        opencv::core::Scalar::default(),
    )
    .map_err(|e| format!("Failed to apply morphology: {}", e))?;

    // 3x3 blur
    let mut blur_result = opencv::core::Mat::default();
    opencv::imgproc::blur(
        &morth_result,
        &mut blur_result,
        opencv::core::Size::new(3, 3),
        opencv::core::Point::new(-1, -1),
        opencv::core::BORDER_DEFAULT,
    )
    .map_err(|e| format!("Failed to apply blur: {}", e))?;

    Ok(cam::Frame {
        data_row_major: blur_result
            .data_typed::<u8>()
            .map_err(|e| format!("Failed to get data from Mat: {}", e))?
            .to_vec(),
        width: raw.width,
        height: raw.height,
        timestamp_ns: raw.timestamp_ns,
    })
}

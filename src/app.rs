use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use crate::cam;
use crate::opencvutils;
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
struct AttitudeEstimationConfig {
    min_matches: u32,
    pixel_tolerance: f32,
    timeout_secs: f32,
}

impl Default for AttitudeEstimationConfig {
    fn default() -> Self {
        AttitudeEstimationConfig {
            min_matches: 10,
            pixel_tolerance: 0.5,
            timeout_secs: 0.2,
        }
    }
}

impl AttitudeEstimationConfig {
    fn validate(self) -> Result<Self, String> {
        if self.timeout_secs < 0.001 {
            return Err("timeout_secs mut be larger than 1ms".to_string());
        }
        Ok(self)
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct Persistent {
    camera_config: cam::CameraConfig,
    view_settings: ViewSettings,
    attitude_est_config: AttitudeEstimationConfig,
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
            attitude_est_config: AttitudeEstimationConfig::default(),
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct AxisCalibration {
    error_deg: f64,
    orientations: u32,
}

impl AxisCalibration {
    pub fn new() -> Self {
        AxisCalibration {
            error_deg: 0.0,
            orientations: 0,
        }
    }

    pub fn reset(&mut self) {
        self.error_deg = 0.0;
        self.orientations = 0;
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
struct AutoCalibration {
    active: bool,
}

impl AutoCalibration {
    fn new() -> Self {
        AutoCalibration { active: false }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct State {
    pub persistent: Persistent,
    pub axis_calibration: AxisCalibration,
    pub auto_calibration: AutoCalibration,
    pub camera_mode: CameraMode,
}

impl State {
    fn set_persistent(&mut self, persistent: Persistent) -> Result<(), String> {
        self.persistent = persistent.validate()?;
        Ok(())
    }
}

pub struct App {
    config_path: PathBuf,
    state: Mutex<State>,
    pub image_dispatcher: webutils::DataDispatcher<std::sync::Arc<Vec<u8>>>,
    pub stream_dispatcher: webutils::DataDispatcher<String>,
    pub running: AtomicBool,
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
        };

        App {
            config_path: filename.as_ref().to_path_buf(),
            state: Mutex::new(state),
            image_dispatcher: webutils::DataDispatcher::<std::sync::Arc<Vec<u8>>>::new(),
            stream_dispatcher: webutils::DataDispatcher::<String>::new(),
            running: AtomicBool::new(true),
        }
    }

    pub fn get_state(&self) -> Result<State, String> {
        let state_ref = self.state.lock().map_err(|e| e.to_string())?;
        Ok(state_ref.clone())
    }

    pub fn set_settings(&self, settings: Persistent) -> Result<State, String> {
        let mut state_ref = self.state.lock().map_err(|e| e.to_string())?;
        state_ref.set_persistent(settings)?;
        Ok(state_ref.clone())
    }

    pub fn set_camera_mode(&self, mode: CameraModeRequest) -> Result<State, String> {
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
        Ok(state_ref.clone())
    }
}

impl Drop for App {
    fn drop(&mut self) {
        let persistent = self.state.lock().unwrap().persistent.clone();

        if let Err(e) = std::fs::write(
            &self.config_path,
            serde_json::to_string(&persistent).unwrap(),
        ) {
            eprintln!("Failed to save persistent data: {}", e);
        }
    }
}

pub fn tick(app: &App) {
    let image_dispatcher = &app.image_dispatcher;
    let stream_dispatcher = &app.stream_dispatcher;
    let running = &app.running;

    println!("Setup Camera");

    let config = cam::CameraConfig {
        analogue_gain: 1,
        digital_gain: 4,
        exposure_us: 500000,
        binning: 2,
    };
    let mut camera = match cam::Camera::new(&config) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("Error initializing camera: {}.", e);
            return;
        }
    };

    camera.set_config(&config);

    let mut ie = opencvutils::ImageEncoder::new(Some(30.0));

    while running.load(Ordering::Acquire) {
        //Update config
        let acquisition_request = match app.state.lock() {
            Ok(mut state_ref) => {
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

                request_mode
            }
            Err(_) => {
                break;
            }
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

        // Encode image
        let encoded = match ie.encode(&raw) {
            Some(v) => v,
            None => {
                eprintln!("Error encoding image");
                continue;
            }
        };

        image_dispatcher.put(std::sync::Arc::new(encoded));

        let data = serde_json::json!({
            "image_size": (raw.width, raw.height),
            "image_quality": ie.quality_str(),
            "auto_calibrator": serde_json::json!({
                "active": false,
            }),

        });

        stream_dispatcher.put(data.to_string());
    }

    println!("Camera thread stopped gracefully");
}

use std::sync::{Arc, Mutex};

use crate::attitude_history;
use crate::commands;
use crate::quat_optim;
use crate::utils;

pub struct GyroParameters {
    scale: [f64; 3],
    bias: [f64; 3],
}

impl GyroParameters {
    pub fn zero() -> Self {
        GyroParameters {
            scale: [0.0; 3],
            bias: [0.0; 3],
        }
    }

    pub fn correct_raw(&self, raw: &[f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| raw[i] * (self.scale[i] + 1.0) + self.bias[i])
    }
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct GyroCalibratorStatus {
    #[serde(serialize_with = "utils::quaternion_serialize_f32")]
    pub quat: nalgebra::UnitQuaternion<f64>,
    pub scale: [f64; 3],
    pub bias: [f64; 3],
    pub id: u64,
}

impl GyroCalibratorStatus {
    pub fn to_payload(&self) -> commands::StarQuat {
        commands::StarQuat::new(
            [
                self.quat.w as f32,
                self.quat.i as f32,
                self.quat.j as f32,
                self.quat.k as f32,
            ],
            std::array::from_fn(|i| self.scale[i] as f32),
            std::array::from_fn(|i| self.bias[i] as f32),
            (self.id & 0xFFFF) as u16,
        )
    }
}

pub struct GyroCalibrator {
    id_quat: Option<(u64, nalgebra::UnitQuaternion<f64>)>,
}

impl GyroCalibrator {
    pub fn new() -> Self {
        GyroCalibrator { id_quat: None }
    }

    pub fn calibrate(
        &mut self,
        quat: nalgebra::UnitQuaternion<f64>,
        id: u64,
        attitude_history: &Arc<Mutex<attitude_history::AttitudeHistory>>,
    ) -> Result<GyroCalibratorStatus, String> {
        // TODO expose parameters
        let delta_t = attitude_history::GYRO_SAMPLE_PERIOD_NOMINAL;
        let lamb = 1e-3;
        let alpha = 0.2;
        let trust_region = 1e-3;

        let max_omega = 6.0;
        let max_delta_id = 300;

        // if there is no previous quaternion, we can't see a difference
        let (last_id, last_quat) = match self.id_quat {
            Some(id_quat) => {
                self.id_quat = Some((id, quat));
                id_quat
            }
            None => {
                self.id_quat = Some((id, quat));
                return Err("No previous quat available".to_string());
            }
        };

        // Don't use data to calibrate if the time between reference quaterions is too large
        let delta_id = id - last_id;
        if delta_id > max_delta_id {
            return Err(format!("Delta id {delta_id} is larger than {max_delta_id}"));
        }

        // Get data from attitude history. Only lock mutex to extract data
        let (scale, bias, omega_t) = {
            let attitude_history = attitude_history.lock().unwrap();

            let scale = attitude_history.gyro_parameters.scale;
            let bias = attitude_history.gyro_parameters.bias;

            let (omega_t, actual_start, actual_end) = attitude_history
                .get_raw_gyro_between(last_id, id)
                .ok_or("Could not get raw gyro".to_string())?;

            if (actual_start != last_id) || (actual_end != id) {
                return Err(format!(
                    "Id slice ({last_id}, {id}) not found. Only got ({actual_start}, {actual_end})"
                ));
            }

            (scale, bias, omega_t)
        };

        // Check if there are gyro values close to be clipped
        if omega_t
            .iter()
            .any(|omega| omega.iter().any(|&v| v.abs() > max_omega))
        {
            return Err(format!("Gyro reading exceeds max_omega ({max_omega})"));
        }

        // Convert quat forms
        let q_0 = [last_quat.w, last_quat.i, last_quat.j, last_quat.k];
        let q_1 = [quat.w, quat.i, quat.j, quat.k];

        // Solve for new scale and bias parameters
        let (scale, bias) = quat_optim::filter_step(
            &q_0,
            &q_1,
            &omega_t,
            scale,
            bias,
            delta_t,
            lamb,
            alpha,
            trust_region,
        );
        let parameters = GyroParameters { scale, bias };

        // Correct current attitude
        let (latest_quat, latest_id) = {
            let mut attitude_history = attitude_history.lock().unwrap();
            attitude_history.correct(quat, id, parameters);
            attitude_history
                .get_latest()
                .ok_or("Could not get latest quaternion".to_string())?
        };

        Ok(GyroCalibratorStatus {
            quat: latest_quat,
            scale,
            bias,
            id: latest_id,
        })
    }
}

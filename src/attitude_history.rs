use crate::gyro_calibrator;
use std::collections::VecDeque;

const GYRO_DATA_READY_TO_UART_START_NS: u64 = 140_000;
const LINUX_UART_RX_DELAY_NS: u64 = 2_000_000; //estimate

pub const GYRO_SAMPLE_PERIOD_NOMINAL: f64 = 1.0 / 200.0;

struct Attitude {
    raw_gyro: [f64; 3],
    quat: nalgebra::UnitQuaternion<f64>,
}

pub struct AttitudeHistory {
    buffer: CircularBuffer<Attitude>,
    timing: Option<Timing>,
    pub gyro_parameters: gyro_calibrator::GyroParameters,
}

fn from_angular_velocities(gyro: &[f64; 3], delta_t: f64) -> nalgebra::UnitQuaternion<f64> {
    let norm = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
    let angle = norm * (0.5 * delta_t);
    let c = angle.cos();
    let s = angle.sin();
    let s_div_norm = s / norm;
    nalgebra::UnitQuaternion::<f64>::from_quaternion(nalgebra::Quaternion::new(
        c,
        gyro[0] * s_div_norm,
        gyro[1] * s_div_norm,
        gyro[2] * s_div_norm,
    ))
}

fn integrate_quat(
    old_quat: &nalgebra::UnitQuaternion<f64>,
    omega: [f64; 3],
) -> nalgebra::UnitQuaternion<f64> {
    let delta_q = from_angular_velocities(&omega, GYRO_SAMPLE_PERIOD_NOMINAL);
    old_quat * delta_q
}

impl AttitudeHistory {
    pub fn new(gyro_parameters: gyro_calibrator::GyroParameters) -> Self {
        Self {
            buffer: CircularBuffer::new(1000),
            timing: None,
            gyro_parameters: gyro_parameters,
        }
    }

    pub fn push_raw(&mut self, raw_gyro: [f64; 3], id_raw: u16, rx_time_ns: u64) {
        // Unwrap id to u64
        let id = if let Some((_, last_id)) = self.buffer.latest() {
            if (id_raw as u64) < (last_id & 0xffff) {
                // Overflow happened
                (last_id & !0xffff) + 0x10000 + id_raw as u64
            } else {
                (last_id & !0xffff) + id_raw as u64
            }
        } else {
            id_raw as u64
        };

        // Fill in missing attitudes
        if let Some((last_attitude, last_id)) = self.buffer.latest() {
            let delta_id = id - last_id;
            if delta_id > 1 {
                // Copy necessary to satisfy borrow checker
                let last_raw_gyro = last_attitude.raw_gyro;
                let mut last_quat = last_attitude.quat;

                // Fill in missing attitudes with last known attitude
                for i in 1..delta_id {
                    // Linear interpolation
                    let f = (i as f64) / (delta_id + 1) as f64;
                    let fill_raw_gyro =
                        std::array::from_fn(|i| last_raw_gyro[i] * (1.0 - f) + raw_gyro[i] * f);

                    let omega = self.gyro_parameters.correct_raw(&fill_raw_gyro);
                    let quat = integrate_quat(&last_quat, omega);

                    let fill_attitude = Attitude {
                        raw_gyro: fill_raw_gyro,
                        quat: quat,
                    };
                    self.buffer.push(fill_attitude);

                    last_quat = quat;
                }
            }
        }

        // Find time difference to reference point in time
        let gyro_time_estimate =
            rx_time_ns - GYRO_DATA_READY_TO_UART_START_NS - LINUX_UART_RX_DELAY_NS;
        if let Some(timing) = &mut self.timing {
            timing.add_sample(id, gyro_time_estimate);
        } else {
            self.timing = Some(Timing::new(gyro_time_estimate));
        };

        let last_quat = match self.buffer.latest() {
            Some((a, _)) => a.quat,
            None => nalgebra::UnitQuaternion::<f64>::identity(),
        };

        let omega = self.gyro_parameters.correct_raw(&raw_gyro);
        let quat = integrate_quat(&last_quat, omega);

        let attitude = Attitude { raw_gyro, quat };

        self.buffer.push(attitude);
    }

    pub fn get_id_floor(&self, time_ns: u64) -> Option<u64> {
        Some(self.timing.as_ref()?.get_id_floor(time_ns))
    }

    pub fn get_id_ceil(&self, time_ns: u64) -> Option<u64> {
        Some(self.timing.as_ref()?.get_id_ceil(time_ns))
    }

    pub fn get_quats_between(
        &self,
        start_ns: u64,
        end_ns: u64,
    ) -> Option<(Vec<nalgebra::UnitQuaternion<f64>>, u64)> {
        let timing = self.timing.as_ref()?;
        let start_id = timing.get_id_floor(start_ns);
        let end_id = timing.get_id_ceil(end_ns);

        let quats = self
            .buffer
            .iter_between(start_id, end_id)
            .map(|(_, attitude)| attitude.quat)
            .collect();

        Some((quats, start_id))
    }

    pub fn get_raw_gyro_between(
        &self,
        start_id: u64,
        end_id: u64,
    ) -> Option<(Vec<[f64; 3]>, u64, u64)> {
        let raw_gyros: Vec<[f64; 3]> = self
            .buffer
            .iter_between(start_id, end_id)
            .map(|(_, attitude)| attitude.raw_gyro)
            .collect();

        if raw_gyros.is_empty() {
            return None;
        }
        Some((raw_gyros, start_id, end_id))
    }

    pub fn correct(
        &mut self,
        mut quat: nalgebra::UnitQuaternion<f64>,
        id: u64,
        gyro_parameters: gyro_calibrator::GyroParameters,
    ) {
        // Before we recalculate the history, update the gyro parameters
        self.gyro_parameters = gyro_parameters;

        // Iterate from id to end
        for (_, next) in self.buffer.iter_mut_from(id) {
            // Update quaterion
            next.quat = quat;

            // calculate next quaternion
            let omega = self.gyro_parameters.correct_raw(&next.raw_gyro);
            quat = integrate_quat(&next.quat, omega);
        }
    }

    pub fn get_latest(&self) -> Option<(nalgebra::UnitQuaternion<f64>, u64)> {
        let (attitude, id) = self.buffer.latest()?;
        Some((attitude.quat, id))
    }

    pub fn fs(&self) -> Option<f64> {
        match &self.timing {
            Some(timing) => Some(1.0 / timing.sample_period_s),
            None => None,
        }
    }
}

pub struct CircularBuffer<T> {
    buf: VecDeque<T>,
    capacity: usize,
    next_id: u64,  // monotonically increasing ID
    first_id: u64, // ID of the oldest element currently in buffer
}

impl<T> CircularBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            buf: VecDeque::with_capacity(capacity),
            capacity,
            next_id: 0,
            first_id: 0,
        }
    }

    /// Push a new item, returns its ID
    pub fn push(&mut self, item: T) -> u64 {
        if self.buf.len() == self.capacity {
            self.buf.pop_front();
            self.first_id += 1;
        }
        let id = self.next_id;
        self.next_id += 1;
        self.buf.push_back(item);
        id
    }

    /// Get last pushed item (if any)
    pub fn latest(&self) -> Option<(&T, u64)> {
        if self.buf.is_empty() {
            None
        } else {
            let id = self.next_id - 1;
            self.buf.back().map(|v| (v, id))
        }
    }

    /// Iterate from given ID (inclusive) up to newest
    pub fn iter_mut_from(&mut self, id: u64) -> impl Iterator<Item = (u64, &mut T)> {
        let start = if id < self.first_id {
            0
        } else {
            (id - self.first_id) as usize
        };
        let offset = self.first_id + start as u64;
        self.buf
            .iter_mut()
            .skip(start)
            .enumerate()
            .map(move |(i, item)| (offset + i as u64, item))
    }

    /// Iterate between two IDs [start_id, end_id)
    pub fn iter_between(&self, start_id: u64, end_id: u64) -> impl Iterator<Item = (u64, &T)> {
        // Clamp range to what’s available in the buffer
        let start = start_id.max(self.first_id);
        let end = end_id.min(self.next_id);

        let start_idx = (start - self.first_id) as usize;
        let end_idx = (end - self.first_id) as usize;
        let count = end_idx.saturating_sub(start_idx);

        self.buf
            .iter()
            .skip(start_idx)
            .take(count)
            .enumerate()
            .map(move |(i, item)| (self.first_id + start_idx as u64 + i as u64, item))
    }
}

fn ns_to_s(ns: u64) -> f64 {
    (ns as f64) * 1e-9
}

struct Timing {
    timing_optimizer: TimingOptimizer,
    sample_period_s: f64,
    delay: f64,
    reference_instant_ns: u64,
}
impl Timing {
    fn new(reference_instant_ns: u64) -> Self {
        Timing {
            timing_optimizer: TimingOptimizer::new(),
            sample_period_s: 1.0,
            delay: 0.0,
            reference_instant_ns,
        }
    }

    fn add_sample(&mut self, id: u64, time_ns: u64) {
        let delta_t = ns_to_s(time_ns - self.reference_instant_ns);
        if let Some((delay, period)) = self.timing_optimizer.add_sample(id as f64, delta_t) {
            self.delay = delay;
            self.sample_period_s = period;
        }
    }

    fn get_id_floor(&self, time_ns: u64) -> u64 {
        let id_f =
            (ns_to_s(time_ns - self.reference_instant_ns) - self.delay) / self.sample_period_s;
        id_f.floor() as u64
    }

    fn get_id_ceil(&self, time_ns: u64) -> u64 {
        let id_f =
            (ns_to_s(time_ns - self.reference_instant_ns) - self.delay) / self.sample_period_s;
        id_f.ceil() as u64
    }
}

#[derive(Debug, Clone)]
struct Sample {
    y: f64,
    x: f64,
    slope: f64, // slope to the next sample
}

impl Sample {
    fn new(x: f64, y: f64) -> Self {
        Self { y, x, slope: 0.0 }
    }
}

#[derive(Debug)]
struct TimingOptimizer {
    samples: Vec<Sample>,
    best_index: usize,
    first_b: f64,
}

impl TimingOptimizer {
    fn new() -> Self {
        Self {
            samples: Vec::new(),
            best_index: 0,
            first_b: 0.0,
        }
    }

    fn add_sample(&mut self, x: f64, y: f64) -> Option<(f64, f64)> {
        // one sample, solution is not defined
        if self.samples.is_empty() {
            self.samples.push(Sample::new(x, y));
            self.first_b = x;
            return None;
        }

        let mut last = self.samples.last().unwrap();

        if last.x >= x {
            panic!("b must be strictly increasing");
        }

        let mut slope = (y - last.y) / (x - last.x);

        // two samples, always accept
        if self.samples.len() == 1 {
            if let Some(last_mut) = self.samples.last_mut() {
                last_mut.slope = slope;
            }
            self.samples.push(Sample::new(x, y));
            self.best_index = 0;
            return Some(self.calc_result());
        }

        while self.samples.len() > 1 {
            let last_slope = self.samples[self.samples.len() - 2].slope;
            if slope > last_slope {
                break;
            }

            // previous point is inside the convex hull, remove it and continue
            self.samples.pop();
            last = self.samples.last().unwrap();
            slope = (y - last.y) / (x - last.x);
        }

        // found a new convex segment
        if let Some(last_mut) = self.samples.last_mut() {
            last_mut.slope = slope;
        }
        self.samples.push(Sample::new(x, y));

        // New point has replaced the previous best segment
        if self.best_index >= self.samples.len() - 1 {
            self.best_index = self.samples.len() - 2;
            return Some(self.calc_result());
        }

        // Check if the next segment yields a better fit to the data
        while self.best_index < self.samples.len() - 2 {
            let best = &self.samples[self.best_index];
            let next = &self.samples[self.best_index + 1];

            let best_left_box = best.slope * (next.x - self.first_b);
            let best_right_box = best.slope * (x - next.x);
            let next_left_box = next.slope * (next.x - self.first_b);
            let next_right_box = next.slope * (x - next.x);

            if best_left_box - next_left_box <= best_right_box - next_right_box {
                break;
            }

            // next segment is better, continue
            self.best_index += 1;
        }

        Some(self.calc_result())
    }

    fn calc_result(&self) -> (f64, f64) {
        let best = &self.samples[self.best_index];
        let offset = best.y - best.x * best.slope;
        (offset, best.slope)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let mut cb: CircularBuffer<i32> = CircularBuffer::new(3);

        assert!(cb.latest().is_none());
        assert!(
            cb.iter_mut_from(0)
                .map(|(_, v)| *v)
                .collect::<Vec<i32>>()
                .len()
                == 0
        );

        for i in 0..5 {
            cb.push(i);
        }
        assert_eq!(cb.latest().unwrap().0, &4);

        assert_eq!(
            cb.iter_mut_from(3).map(|(_, v)| *v).collect::<Vec<i32>>(),
            vec![3, 4]
        );
        assert_eq!(
            cb.iter_mut_from(6).map(|(_, v)| *v).collect::<Vec<i32>>(),
            Vec::<i32>::new()
        );
        assert_eq!(
            cb.iter_mut_from(0).map(|(_, v)| *v).collect::<Vec<i32>>(),
            vec![2, 3, 4]
        );
    }
}

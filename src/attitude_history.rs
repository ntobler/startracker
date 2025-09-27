use std::collections::VecDeque;
use std::time::Instant;
struct Attitude {
    q: [f64; 4],
    id: u64,
}

pub struct AttitudeHistory {
    timeing_optimizer: TimingOptimizer,
    buffer: VecDeque<Attitude>,
    sample_period_s: f64,
    delay: f64,
    reference_instant: Option<Instant>,
}

impl AttitudeHistory {
    pub fn new() -> Self {
        Self {
            timeing_optimizer: TimingOptimizer::new(),
            buffer: VecDeque::with_capacity(1000),
            sample_period_s: 1.0,
            delay: 0.0,
            reference_instant: None,
        }
    }

    pub fn add(&mut self, quat: &[f64; 4], id_raw: u16, rx_time: Instant) {
        // Unwrap id to u64
        let id = if let Some(last_attitude) = self.buffer.back() {
            if (id_raw as u64) < (last_attitude.id & 0xffff) {
                // Overflow happened
                (last_attitude.id & !0xffff) + 0x10000 + id_raw as u64
            } else {
                (last_attitude.id & !0xffff) + id_raw as u64
            }
        } else {
            id_raw as u64
        };

        // Fill in missing attitudes
        if let Some(last_attitude) = self.buffer.back() {
            let delta_id = id - last_attitude.id;
            if delta_id > 1 {
                // Copy necessary to satisfy borrow checker
                let last_id = last_attitude.id;
                let last_q = last_attitude.q;

                // Fill in missing attitudes with last known attitude
                for i in 1..delta_id {
                    // TODO lerp instead
                    let q = last_q;

                    let fill_id = last_id + i;
                    let fill_attitude = Attitude { q: q, id: fill_id };
                    self.put(fill_attitude);
                }
            }
        }

        let delta_t = if let Some(r) = self.reference_instant {
            rx_time.duration_since(r).as_secs_f64()
        } else {
            self.reference_instant = Some(rx_time);
            0.0
        };

        if let Some((delay, period)) = self.timeing_optimizer.add_sample(id as f64, delta_t) {
            self.delay = delay;
            self.sample_period_s = period;
        }

        // println!(
        //     "Attitude id: {}, delta_t: {:.6}, delay: {:.6}, period: {:.6}",
        //     id, delta_t, self.delay, self.sample_period_s
        // );

        let attitude = Attitude { q: *quat, id };

        self.put(attitude);
    }

    fn put(&mut self, attitude: Attitude) {
        // Store in circular buffer
        if self.buffer.len() == self.buffer.capacity() {
            self.buffer.pop_front();
        }
        self.buffer.push_back(attitude);
    }

    pub fn get_between(&self, start: Instant, end: Instant) -> Vec<[f64; 4]> {
        if let Some(r) = self.reference_instant {
            let start_id_f =
                (start.duration_since(r).as_secs_f64() - self.delay) / self.sample_period_s;
            let end_id_f =
                (end.duration_since(r).as_secs_f64() - self.delay) / self.sample_period_s;

            let start_id = start_id_f.floor() as u64;
            let end_id = end_id_f.ceil() as u64;

            self.buffer
                .iter()
                .filter(|attitude| attitude.id >= start_id && attitude.id <= end_id)
                .map(|attitude| attitude.q)
                .collect::<Vec<[f64; 4]>>()
        } else {
            Vec::new()
        }
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

        let mut best_index = self.best_index;
        while best_index < self.samples.len() - 2 {
            let best = &self.samples[best_index];
            let next = &self.samples[best_index + 1];

            let best_left_box = best.slope * (next.x - self.first_b);
            let best_right_box = best.slope * (x - next.x);
            let next_left_box = next.slope * (next.x - self.first_b);
            let next_right_box = next.slope * (x - next.x);

            if best_left_box - next_left_box <= best_right_box - next_right_box {
                break;
            }

            // next segment is better, continue
            best_index += 1;
        }

        self.best_index = best_index;
        Some(self.calc_result())
    }

    fn calc_result(&self) -> (f64, f64) {
        let best = &self.samples[self.best_index];
        let offset = best.y - best.x * best.slope;
        (offset, best.slope)
    }
}

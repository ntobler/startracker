use nix::time::{clock_gettime, ClockId};
use serde::ser::{SerializeSeq, Serializer};
use std::fs;
use std::time::Duration;

pub fn contiguous_serialize_2d<S, const N: usize, T>(
    val: &Vec<[T; N]>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    T: serde::Serialize + bytemuck::NoUninit,
{
    let len = val.len();
    let ptr = val.as_ptr() as *const T;
    let total_len = len * N;
    let contiguous_slice = unsafe { std::slice::from_raw_parts(ptr, total_len) };
    let bytes: &[u8] = bytemuck::cast_slice(contiguous_slice);
    serializer.serialize_bytes(&bytes)
}

pub fn contiguous_serialize_1d<S, T>(val: &Vec<T>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    T: serde::Serialize + bytemuck::NoUninit,
{
    let bytes: &[u8] = bytemuck::cast_slice(&val);
    serializer.serialize_bytes(&bytes)
}

pub fn quaternion_serialize_f32<S>(
    q: &nalgebra::UnitQuaternion<f64>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let a = [q.w as f32, q.i as f32, q.j as f32, q.k as f32];
    let mut seq = serializer.serialize_seq(Some(a.len()))?;
    for element in &a {
        seq.serialize_element(element)?;
    }
    seq.end()
}

pub fn monotonic_time_ns() -> u64 {
    let ts = clock_gettime(ClockId::CLOCK_MONOTONIC).unwrap();
    ts.tv_sec() as u64 * 1_000_000_000 + ts.tv_nsec() as u64
}

pub fn duration_since_process_start() -> Duration {
    let stat = fs::read_to_string("/proc/self/stat").unwrap();
    let fields: Vec<&str> = stat.split_whitespace().collect();
    let start_ticks: u64 = fields[21].parse().unwrap();
    let ticks_per_sec = unsafe { libc::sysconf(libc::_SC_CLK_TCK) as u64 };

    let uptime_secs: f64 = fs::read_to_string("/proc/uptime")
        .unwrap()
        .split_whitespace()
        .next()
        .unwrap()
        .parse()
        .unwrap();

    let uptime_ticks = uptime_secs * ticks_per_sec as f64;
    let delta_ticks = uptime_ticks - start_ticks as f64;
    Duration::from_secs_f64(delta_ticks / ticks_per_sec as f64)
}

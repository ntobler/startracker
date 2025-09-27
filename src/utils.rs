use nix::time::{clock_gettime, ClockId};

pub fn contiguous_serialize_2d<S, const N: usize, T>(
    val: &Vec<[T; N]>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
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
    S: serde::Serializer,
    T: serde::Serialize + bytemuck::NoUninit,
{
    let bytes: &[u8] = bytemuck::cast_slice(&val);
    serializer.serialize_bytes(&bytes)
}

pub fn monotonic_time_ns() -> u64 {
    let ts = clock_gettime(ClockId::CLOCK_MONOTONIC).unwrap();
    ts.tv_sec() as u64 * 1_000_000_000 + ts.tv_nsec() as u64
}

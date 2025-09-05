use serde::{Deserialize, Serialize};
use std::thread;

#[cfg(feature = "cam")]
#[path = "cam_thread.rs"]
mod cam_thread;

#[cfg(not(feature = "cam"))]
#[path = "cam_thread_mock.rs"]
mod cam_thread;

use num_traits::{PrimInt, Unsigned, WrappingAdd, Zero};
use std::ops::Shr;

pub trait Cast<T> {
    fn cast(self) -> T;
}

macro_rules! impl_cast {
    ($from:ty => $to:ty) => {
        impl Cast<$to> for $from {
            #[inline(always)]
            fn cast(self) -> $to {
                self as $to
            }
        }
    };
}
macro_rules! impl_clip_cast {
    ($from:ty => $to:ty) => {
        impl Cast<$to> for $from {
            #[inline(always)]
            fn cast(self) -> $to {
                self.min(<$to>::MAX as $from) as $to
            }
        }
    };
}
impl_cast!(u8 => u8);
impl_cast!(u8 => u16);
impl_clip_cast!(u16 => u8);
impl_cast!(u16 => u16);
impl_cast!(i32 => u16);
impl_cast!(i32 => u8);
impl_cast!(i32 => usize);

pub trait PixelType: PrimInt + Unsigned + Zero + Copy + Shr<Self> + WrappingAdd {}
impl<T: PrimInt + Unsigned + Zero + Copy + Shr + WrappingAdd> PixelType for T {}

pub struct Frame<T: PixelType> {
    pub data_row_major: Vec<T>,
    pub width: usize,
    pub height: usize,
    pub timestamp_ns: u64,
}

impl<T: PixelType> Frame<T> {
    pub fn new(
        data_row_major: Vec<T>,
        width: usize,
        height: usize,
        timestamp_ns: u64,
    ) -> Result<Self, String> {
        if data_row_major.len() != width * height {
            return Err("Frame dimensions don't match data".to_string());
        }
        Ok(Frame {
            data_row_major,
            width,
            height,
            timestamp_ns,
        })
    }

    pub fn crop(&self, x: usize, y: usize, w: usize, h: usize) -> Result<Frame<T>, String> {
        if (x + w > self.width) || (y + h > self.height) {
            return Err("Crop dimensions out of bounds".to_string());
        }
        let mut out = Vec::<T>::with_capacity(w * h);
        for row in 0..h {
            let in_base = (y + row) * self.width + x;
            out.extend_from_slice(&self.data_row_major[in_base..in_base + w]);
        }
        Frame::new(out, w, h, self.timestamp_ns)
    }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct CameraConfig {
    pub analogue_gain: u32,
    pub digital_gain: u32,
    pub exposure_us: u32,
    pub binning: u32,
}

impl CameraConfig {
    pub fn validate(self) -> Result<Self, String> {
        // Check if exponential of 2
        if (self.binning & (self.binning - 1)) != 0 {
            return Err("binning must be integer exponential of 2".to_string());
        }
        if (self.binning < 1) || (self.binning > 8) {
            return Err("binning must be between 1 and 8".to_string());
        }
        Ok(self)
    }
}

pub struct Camera {
    thread_handle: Option<thread::JoinHandle<Result<(), String>>>,
    thread_error: Option<String>,
    trigger_tx: Option<crossbeam_channel::Sender<(u32, u32)>>,
    frame_rx: crossbeam_channel::Receiver<Frame<u16>>,
    config: CameraConfig,
    darkframe: Option<Frame<u8>>,
}

impl Camera {
    pub fn new(config: &CameraConfig) -> Result<Self, String> {
        let (trigger_tx, trigger_rx) = crossbeam_channel::bounded::<(u32, u32)>(0); // unbuffered: strictly 1:1 signal
        let (frame_tx, frame_rx) = crossbeam_channel::bounded::<Frame<u16>>(1); // frame response

        let config_clone = config.clone();

        let thread_handle = thread::spawn(move || {
            cam_thread::camera_thread(
                trigger_rx,
                frame_tx,
                config_clone.exposure_us,
                config_clone.analogue_gain,
            )
        });

        Ok(Camera {
            thread_handle: Some(thread_handle),
            thread_error: None,
            trigger_tx: Some(trigger_tx),
            frame_rx,
            config: config_clone,
            darkframe: None,
        })
    }

    fn get_thread_error(&mut self) -> String {
        if let Some(err) = &self.thread_error {
            return err.clone();
        }

        let err = match self.thread_handle.take() {
            Some(handle) => match handle.join() {
                Ok(Ok(_)) => "Thread ended without errors.".to_string(),
                Ok(Err(e)) => e,
                Err(panic) => format!("Thread panicked: {:?}", panic),
            },
            None => "Thread already joined.".to_string(),
        };

        self.thread_error = Some(err.clone());
        err
    }

    pub fn set_config(&mut self, config: &CameraConfig) {
        self.config = config.clone();
    }

    pub fn capture_internal(&mut self) -> Result<Frame<u8>, String> {
        if let Some(e) = &self.thread_error {
            return Err(e.clone());
        }
        match self
            .trigger_tx
            .as_ref()
            .ok_or("trigger has been dropped".to_string())?
            .send((self.config.exposure_us, self.config.analogue_gain))
        {
            Ok(_) => {}
            Err(_) => {
                return Err(self.get_thread_error());
            }
        };
        let frame = match self.frame_rx.recv() {
            Ok(f) => Ok(f),
            Err(_) => Err(self.get_thread_error()),
        }?;

        let log2_f = match self.config.digital_gain {
            1 => Ok(-2),
            2 => Ok(-1),
            4 => Ok(0),
            8 => Ok(1),
            x => Err(format!("Digital gain {:?} not available", x)),
        }?;

        // Apply binning and digital gain
        match self.config.binning {
            1 => Ok(amplify(&frame, log2_f)),
            2 => Ok(amplify(&binning::<1, u16>(&frame)?, log2_f - 2)),
            4 => Ok(amplify(&binning::<2, u16>(&frame)?, log2_f - 4)),
            x => Err(format!("Binning {:?} not available", x)),
        }
    }

    pub fn capture(&mut self) -> Result<Frame<u8>, String> {
        let mut frame = self.capture_internal()?;

        // Compute darkframe subtraction if available
        match &mut self.darkframe {
            Some(dark) => {
                if (dark.width == frame.width) && (dark.height == frame.height) {
                    frame
                        .data_row_major
                        .iter_mut()
                        .zip(dark.data_row_major.iter())
                        .for_each(|(a, b)| *a = a.saturating_sub(*b));
                }
            }
            None => {}
        }

        Ok(frame)
    }

    pub fn record_darkframe(&mut self, average: usize) -> Result<(), String> {
        let frame = self.capture_internal()?;
        let mut accumulator: Vec<u16> = frame.data_row_major.iter().map(|&v| v as u16).collect();

        if average < 1 {
            return Err("Average must be at least 1".to_string());
        }

        for _ in 0..(average - 1) {
            let f = self.capture_internal()?;
            for (a, b) in accumulator.iter_mut().zip(f.data_row_major.iter()) {
                *a += *b as u16;
            }
        }

        self.darkframe = Some(Frame::new(
            accumulator
                .iter()
                .map(|&v| (v / (average as u16)) as u8)
                .collect(),
            frame.width,
            frame.height,
            frame.timestamp_ns,
        )?);
        Ok(())
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        // Drop trigger_tx to close channel (happens automatically here)
        // But explicitly drop to show intent:
        drop(self.trigger_tx.take());

        if let Some(handle) = self.thread_handle.take() {
            println!("Dropping Camera: joining camera thread...");
            match handle.join() {
                Ok(Ok(())) => println!("Camera thread terminated gracefully."),
                Ok(Err(e)) => eprintln!("Camera thread returned error: {}", e),
                Err(e) => eprintln!("Camera thread panicked: {:?}", e),
            }
        }
    }
}

pub fn amplify<T1, T2>(frame: &Frame<T1>, log2_f: i32) -> Frame<T2>
where
    T1: PixelType + Cast<T2>,
    T2: PixelType,
    <T1 as Shr<T1>>::Output: Cast<T2>,
    i32: Cast<T1>,
{
    let new_frame = match log2_f {
        x if x < 0 => frame
            .data_row_major
            .iter()
            .map(|&v| (v >> Cast::<T1>::cast(-x)).cast())
            .collect(),
        0 => frame.data_row_major.iter().map(|&v| v.cast()).collect(),
        x => frame
            .data_row_major
            .iter()
            .map(|&v| (v << x as usize).cast())
            .collect(),
    };

    Frame {
        data_row_major: new_frame,
        width: frame.width,
        height: frame.height,
        timestamp_ns: frame.timestamp_ns,
    }
}

pub fn binning<const LOG2_F: usize, T: PixelType>(frame: &Frame<T>) -> Result<Frame<T>, String> {
    let f: usize = 1 << LOG2_F;
    let out_width: usize = frame.width >> LOG2_F;
    let out_height: usize = frame.height >> LOG2_F;

    if out_height * f != frame.height {
        return Err("Can't bin height".to_string());
    }
    if out_width * f != frame.width {
        return Err("Can't bin width".to_string());
    }

    let mut out = Vec::<T>::with_capacity(frame.data_row_major.len() >> (LOG2_F + LOG2_F));

    for h in 0..out_height {
        let out_base = h * out_width;

        //first line append to out vector
        let in_base = (h << LOG2_F) * frame.width;
        for w in 0..out_width {
            let in_base2 = in_base + (w << LOG2_F);
            let mut out_value = frame.data_row_major[in_base2];
            for sub_w in 1..f {
                out_value = out_value.wrapping_add(&frame.data_row_major[in_base2 + sub_w]);
            }
            out.push(out_value);
        }

        // All other lines add to it
        for sub_h in 1..f {
            let in_base = ((h << LOG2_F) + sub_h) * frame.width;
            for w in 0..out_width {
                let in_base2 = in_base + (w << LOG2_F);
                let mut out_value = frame.data_row_major[in_base2];
                for sub_w in 1..f {
                    out_value = out_value.wrapping_add(&frame.data_row_major[in_base2 + sub_w]);
                }
                out[out_base + w] = out[out_base + w].wrapping_add(&out_value);
            }
        }
    }
    Frame::new(out, out_width, out_height, frame.timestamp_ns)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera() {
        let config = CameraConfig {
            analogue_gain: 1,
            digital_gain: 1,
            exposure_us: 10000,
            binning: 1,
        };
        let mut cam = Camera::new(&config).unwrap();
        let _: Frame<u8> = cam.capture().unwrap();
    }

    #[test]
    fn test_binning() {
        #[rustfmt::skip]
        let data = Frame::new(vec![
                5, 5, 0, 5, 5, 5,
                5, 5, 1, 5, 5, 5,
                0, 1, 8, 1, 0, 0,
                4, 5, 1, 5, 7, 7,
            ], 6, 4, 0).unwrap();
        #[rustfmt::skip]
        let expected = Frame::new(vec![
            20, 11, 20,
            10, 15, 14,
        ], 3, 2, 0).unwrap();

        let out = binning::<1, u16>(&data).unwrap();

        assert_eq!(out.data_row_major, expected.data_row_major);
        assert_eq!(out.width, expected.width);
        assert_eq!(out.height, expected.height);

        // Test with 1x1 binning (no change)
        let frame = Frame::new(vec![1, 2, 3, 4], 2, 2, 0).unwrap();
        let expected = Frame::new(vec![1, 2, 3, 4], 2, 2, 0).unwrap();
        let out = binning::<0, u16>(&frame).unwrap();
        assert_eq!(out.data_row_major, expected.data_row_major);
        assert_eq!(out.width, expected.width);
        assert_eq!(out.height, expected.height);

        // Test with 2x2 binning
        #[rustfmt::skip]
        let data = Frame::new(vec![
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16,
        ], 4, 4, 0).unwrap();
        #[rustfmt::skip]
        let expected = Frame::new(vec![
            1+2+5+6, 3+4+7+8,
            9+10+13+14, 11+12+15+16,
        ], 2, 2, 0).unwrap();
        let out = binning::<1, u16>(&data).unwrap();
        assert_eq!(out.data_row_major, expected.data_row_major);
        assert_eq!(out.width, expected.width);
        assert_eq!(out.height, expected.height);

        // Test 4x4 binning
        #[rustfmt::skip]
        let expected = Frame::new(vec![
            1+2+5+6+ 3+4+7+8 + 9+10+13+14 + 11+12+15+16,
        ], 1, 1, 0).unwrap();
        let out = binning::<2, u16>(&data).unwrap();
        assert_eq!(out.data_row_major, expected.data_row_major);
        assert_eq!(out.width, expected.width);
        assert_eq!(out.height, expected.height);

        // Test with non-divisible dimensions (should return None)
        let expected = Frame::new(vec![1, 2, 3, 4, 5, 6], 3, 2, 0).unwrap();
        assert!(binning::<1, u8>(&expected).is_err());

        // Test with empty input
        let expected = Frame::new(vec![], 0, 0, 0).unwrap();
        assert!(binning::<1, u8>(&expected).is_ok());
    }
}

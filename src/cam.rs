use std::thread;

#[cfg(feature = "cam")]
#[path = "cam_thread.rs"]
mod cam_thread;

#[cfg(not(feature = "cam"))]
#[path = "cam_thread_mock.rs"]
mod cam_thread;

pub struct Camera {
    thread_handle: Option<thread::JoinHandle<Result<(), String>>>,
    thread_error: Option<String>,
    trigger_tx: Option<crossbeam_channel::Sender<()>>,
    frame_rx: crossbeam_channel::Receiver<Option<Vec<u16>>>,
}

impl Camera {
    pub fn new() -> Result<Self, String> {
        let (trigger_tx, trigger_rx) = crossbeam_channel::bounded::<()>(0); // unbuffered: strictly 1:1 signal
        let (frame_tx, frame_rx) = crossbeam_channel::bounded::<Option<Vec<u16>>>(1); // frame response

        let thread_handle = thread::spawn(move || {
            cam_thread::camera_thread(trigger_rx, frame_tx, |x| binning::<1>(x, 1920, 1080))
        });

        Ok(Camera {
            thread_handle: Some(thread_handle),
            thread_error: None,
            trigger_tx: Some(trigger_tx),
            frame_rx,
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

    pub fn capture(&mut self) -> Result<Vec<u16>, String> {
        if let Some(e) = &self.thread_error {
            return Err(e.clone());
        }
        println!("Waiting for camera request execution");
        match self
            .trigger_tx
            .as_ref()
            .ok_or("trigger has been dropped".to_string())?
            .send(())
        {
            Ok(_) => {}
            Err(_) => {
                return Err(self.get_thread_error());
            }
        };
        let frame = match self.frame_rx.recv() {
            Ok(f) => f,
            Err(_) => {
                return Err(self.get_thread_error());
            }
        };
        println!("Capture received frame");
        match frame {
            Some(x) => Ok(x),
            None => Err("Frame extraction function failed".to_string()),
        }
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

fn binning<const LOG2_F: usize>(
    buffer_row_major: &[u16],
    width: usize,
    height: usize,
) -> Option<Vec<u16>> {
    let f: usize = 1 << LOG2_F;
    let out_width: usize = width >> LOG2_F;
    let out_height: usize = height >> LOG2_F;

    if out_height * f != height
        || out_width * f != width
        || buffer_row_major.len() != width * height
    {
        return None;
    }

    let mut out = Vec::<u16>::with_capacity(buffer_row_major.len() >> (LOG2_F + LOG2_F));

    for h in 0..out_height {
        let out_base = h * out_width;

        //first line append to out vector
        let in_base = (h << LOG2_F) * width;
        for w in 0..out_width {
            let in_base2 = in_base + (w << LOG2_F);
            let mut out_value: u16 = 0;
            for sub_w in 0..f {
                out_value = out_value.wrapping_add(buffer_row_major[in_base2 + sub_w]);
            }
            out.push(out_value);
        }

        // All other lines add to it
        for sub_h in 1..f {
            let in_base = ((h << LOG2_F) + sub_h) * width;
            for w in 0..out_width {
                let in_base2 = in_base + (w << LOG2_F);
                let out_value = &mut out[out_base + w];
                for sub_w in 0..f {
                    *out_value = out_value.wrapping_add(buffer_row_major[in_base2 + sub_w]);
                }
            }
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera() {
        let mut cam = Camera::new().unwrap();
        let frame = cam.capture().unwrap();
        assert_eq!(frame.len(), 1920 * 1080 / 4);
    }

    #[test]
    fn test_binning() {
        #[rustfmt::skip]
        let data = [
            5, 5, 0, 5, 5, 5,
            5, 5, 1, 5, 5, 5,
            0, 1, 8, 1, 0, 0,
            4, 5, 1, 5, 7, 7,
        ];
        #[rustfmt::skip]
        let expected = [
            20, 11, 20,
            10, 15, 14,
        ];

        let out = binning::<1>(&data, 6, 4).unwrap();

        assert_eq!(out, expected);

        // Test with 1x1 binning (no change)
        #[rustfmt::skip]
        let data = [
            1, 2,
            3, 4,
        ];
        #[rustfmt::skip]
        let expected = [
            1, 2,
            3, 4,
        ];
        let out = binning::<0>(&data, 2, 2).unwrap();
        assert_eq!(out, expected);

        // Test with 2x2 binning
        #[rustfmt::skip]
        let data = [
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16,
        ];
        #[rustfmt::skip]
        let expected = [
            1+2+5+6, 3+4+7+8,
            9+10+13+14, 11+12+15+16,
        ];
        let out = binning::<1>(&data, 4, 4).unwrap();
        assert_eq!(out, expected);

        // Test 4x4 binning
        #[rustfmt::skip]
        let expected = [
            1+2+5+6+ 3+4+7+8 + 9+10+13+14 + 11+12+15+16,
        ];
        let out = binning::<2>(&data, 4, 4).unwrap();
        assert_eq!(out, expected);

        // Test with non-divisible dimensions (should return None)
        let data = [1, 2, 3, 4, 5, 6];
        assert!(binning::<1>(&data, 3, 2).is_none());

        // Test with divisible dimensions but wrong shape (should return None)
        let data = [1, 2, 3, 4];
        assert!(binning::<1>(&data, 2, 4).is_none());

        // Test with empty input
        let data: [u16; 0] = [];
        assert!(binning::<1>(&data, 0, 0).is_some());
    }
}

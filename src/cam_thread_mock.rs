use crossbeam_channel;
use std::time::Duration;

use crate::cam;
use crate::testingutils;
use crate::testingutils::ImageSource;

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<(u32, u32)>,
    frame_tx: crossbeam_channel::Sender<cam::Frame<u16>>,
    mut exposure_us: u32,
    mut analogue_gain: u32,
) -> Result<(), String> {
    println!("Camera thread: starting");

    let home = dirs::home_dir().expect("Could not find home directory");
    let app_path = home.join(".startracker");
    let image_path = app_path.join("testing_material/axis_align");

    println!("image_path: {:?}", image_path);

    let mut random_image_generator = testingutils::FileImageSource::new(image_path)?;

    let mut timestamp_ns = 0;

    let interval = Duration::from_micros(exposure_us as u64);
    _ = analogue_gain; // Unused in mock, but kept for interface compatibility

    loop {
        std::thread::sleep(interval);
        timestamp_ns += interval.as_nanos() as u64;

        match trigger_rx.try_recv() {
            Ok((new_exposure_ns, new_analogue_gain)) => {
                // Trigger signal present -> get buffer and read data

                // Get new parameters from trigger channel
                exposure_us = new_exposure_ns;
                analogue_gain = new_analogue_gain;

                let mut frame =
                    random_image_generator.get(exposure_us, analogue_gain, timestamp_ns);

                // Add offset
                frame.data_row_major.iter_mut().for_each(|p| {
                    *p = p.saturating_add(32);
                });

                frame_tx.send(frame).ok();
            }
            Err(crossbeam_channel::TryRecvError::Empty) => {
                // nothing available now, proceed
            }
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                // Channel has been closed from the other side. We can shut down the thread
                println!("Camera thread: terminated gracefully.");
                return Ok(());
            }
        }
    }
}

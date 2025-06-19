use crossbeam_channel;
use rand;
use rand::RngCore;
use rand::SeedableRng;
use std::time::Duration;

use crate::cam;

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<(u32, u32)>,
    frame_tx: crossbeam_channel::Sender<cam::Frame<u16>>,
    mut exposure_us: u32,
    mut analogue_gain: u32,
) -> Result<(), String> {
    let _ = analogue_gain;
    println!("Camera thread: starting");
    let seed: [u8; 32] = [42; 32];
    let mut rng = rand::rngs::StdRng::from_seed(seed);
    println!("Camera thread: started");

    let width = 1920;
    let height = 1080;

    let mut timestamp_ns = 0;

    let interval = Duration::from_micros(exposure_us as u64);

    loop {
        std::thread::sleep(interval);
        timestamp_ns += interval.as_nanos() as u64;

        match trigger_rx.try_recv() {
            Ok((new_exposure_ns, new_analogue_gain)) => {
                // Get new parameters from trigger channel
                exposure_us = new_exposure_ns;
                analogue_gain = new_analogue_gain;
                // Trigger signal present -> get buffer and read data
                println!(
                    "Camera thread: trigger ok received {:?}, {:?}",
                    exposure_us, analogue_gain
                );
                let mut frame_data = vec![0; width * height];
                rng.fill_bytes(bytemuck::cast_slice_mut(&mut frame_data));
                frame_data.iter_mut().for_each(|x| *x &= 0xf);

                let frame = cam::Frame {
                    data_row_major: frame_data,
                    width: width,
                    height: height,
                    timestamp_ns,
                };
                frame_tx.send(frame).ok();
            }
            Err(crossbeam_channel::TryRecvError::Empty) => {
                // nothing available now, proceed
            }
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                // Channel has been closed from the other side. We can shut down the thread
                println!("Camera thread: trigger error received");
                println!("Camera thread: terminating gracefully");
                return Ok(());
            }
        }
    }
}

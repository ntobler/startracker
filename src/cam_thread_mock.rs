use crossbeam_channel;
use rand;
use rand::RngCore;
use rand::SeedableRng;
use std::time::Duration;

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<()>,
    frame_tx: crossbeam_channel::Sender<(Vec<u16>, u32, u32)>,
    exposure_us: u32,
    analogue_gain: u32,
) -> Result<(), String> {
    let _ = analogue_gain;
    println!("Camera thread: starting");
    let seed: [u8; 32] = [42; 32];
    let mut rng = rand::rngs::StdRng::from_seed(seed);
    println!("Camera thread: started");

    let width = 1920;
    let height = 1080;

    loop {
        std::thread::sleep(Duration::from_micros(exposure_us as u64));

        match trigger_rx.try_recv() {
            Ok(_) => {
                // Trigger signal present -> get buffer and read data
                println!("Camera thread: trigger ok received");
                let mut frame = vec![0; width * height];
                rng.fill_bytes(bytemuck::cast_slice_mut(&mut frame));
                frame.iter_mut().for_each(|x| *x &= 0xf);
                frame_tx.send((frame, width as u32, height as u32)).ok();
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

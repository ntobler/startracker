use crossbeam_channel;
use rand;
use rand::RngCore;
use rand::SeedableRng;
use std::time::Duration;

pub fn camera_thread<T>(
    trigger_rx: crossbeam_channel::Receiver<()>,
    frame_tx: crossbeam_channel::Sender<T>,
    extraction_func: impl Fn(&[u16]) -> T,
) -> Result<(), String> {
    println!("Camera thread: starting");
    let seed: [u8; 32] = [42; 32];
    let mut rng = rand::rngs::StdRng::from_seed(seed);
    println!("Camera thread: started");
    loop {
        std::thread::sleep(Duration::from_secs(1));

        match trigger_rx.try_recv() {
            Ok(_) => {
                // Trigger signal present -> get buffer and read data
                println!("Camera thread: trigger ok received");
                let gray_value = rng.next_u32() as u16;
                let raw = vec![gray_value; 1920 * 1080];
                let frame = extraction_func(&raw);
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

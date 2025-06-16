use crossbeam_channel;
use rand;
use rand::RngCore;
use rand::SeedableRng;
use std::time::Duration;

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<()>,
    frame_tx: crossbeam_channel::Sender<Vec<u16>>,
) -> Result<(), String> {
    let seed: [u8; 32] = [42; 32];
    let mut rng = rand::rngs::StdRng::from_seed(seed);

    loop {
        let trigger: Result<(), crossbeam_channel::RecvError> = trigger_rx.recv();

        std::thread::sleep(Duration::from_secs(1));

        if trigger.is_ok() {
            println!("Ok Received");
            let gray_value = rng.next_u32() as u16;
            let frame = vec![gray_value; 1920 * 1080];
            frame_tx.send(frame).ok();
        }
        if trigger.is_err() {
            println!("Error Received");
            break;
        }
    }
    Ok(())
}

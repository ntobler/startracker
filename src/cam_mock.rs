#[cfg(not(feature = "cam"))]
use rand::rngs::StdRng;
use rand::RngCore;
use rand::SeedableRng;

pub struct Camera {
    rng: StdRng,
}

impl Camera {
    pub fn new() -> Self {
        let seed: [u8; 32] = [42; 32];
        let rng = rand::rngs::StdRng::from_seed(seed);
        Camera { rng: rng }
    }

    pub fn capture(&mut self) -> Result<Vec<u16>, String> {
        // Create a random image
        let gray_value = self.rng.next_u32() as u16;
        Ok(vec![gray_value; 1920 * 1080])
    }
}

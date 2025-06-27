use crate::cam;
use opencv::{
    core::{Mat, Vector},
    imgcodecs::{imencode, IMWRITE_JPEG_QUALITY},
    prelude::*,
};

pub fn encode_image(frame: &cam::Frame<u8>, ext: &'static str, quality: i32) -> Option<Vec<u8>> {
    // Encode to JPEG
    let img = Mat::new_rows_cols_with_data(
        frame.height as i32,
        frame.width as i32,
        &frame.data_row_major,
    )
    .ok()?;
    let mut buf = Vector::<u8>::new();
    imencode(
        ext,
        &img,
        &mut buf,
        &Vector::<i32>::from_iter([IMWRITE_JPEG_QUALITY, quality]),
    )
    .ok()?;
    Some(buf.to_vec())
}

enum FileType {
    Jpeg,
    Png,
}

pub struct ImageEncoder {
    max_kb: Option<f32>,
    file_type: FileType,
    quality: i32,
}

impl ImageEncoder {
    pub fn new(max_kb: Option<f32>) -> Self {
        ImageEncoder {
            max_kb,
            file_type: FileType::Jpeg,
            quality: 80,
        }
    }

    pub fn quality_str(&self) -> String {
        match self.file_type {
            FileType::Png => "PNG".to_string(),
            FileType::Jpeg => format!("JPG q={}", self.quality),
        }
    }

    pub fn set_max_kb(&mut self, max_kb: Option<f32>) {
        self.max_kb = max_kb;
    }

    pub fn encode(&mut self, frame: &cam::Frame<u8>) -> Option<Vec<u8>> {
        let encoded_bytes = encode_image(
            frame,
            match self.file_type {
                FileType::Jpeg => ".jpg",
                FileType::Png => ".png",
            },
            self.quality,
        )?;

        if let Some(max_kb) = self.max_kb {
            let size_fraction = encoded_bytes.len() as f32 / 1024.0 / max_kb as f32;

            match self.file_type {
                FileType::Png if size_fraction > 1.0 => {
                    self.file_type = FileType::Jpeg;
                    self.quality = 100;
                }
                FileType::Jpeg => {
                    if self.quality >= 100 && size_fraction < 0.7 {
                        self.file_type = FileType::Png;
                    } else {
                        let gain = 20.0 - 19.0 * (self.quality as f32 - 50.0) / 50.0;
                        let quality_estimate = self.quality as f32 - (size_fraction - 1.0) * gain;
                        self.quality = quality_estimate.round().clamp(50.0, 100.0) as i32;
                    }
                }
                _ => {}
            }
        } else {
            self.file_type = FileType::Png;
        }
        Some(encoded_bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv;
    use opencv::core::MatTraitConst;
    use opencv::prelude::MatTraitConstManual;
    use rand;
    use rand::RngCore;
    use rand::SeedableRng;

    #[test]
    fn test_opencv_array() {
        let scaled: Vec<u8> = vec![1, 2, 3, 4, 5, 6];
        let mat = opencv::core::Mat::new_rows_cols_with_data(2, 3, &scaled).unwrap();
        let mut out_mat = opencv::core::Mat::new_rows_cols_with_default(
            mat.rows(),
            mat.cols(),
            opencv::core::CV_8UC1,
            opencv::core::Scalar::new(0.0, 0.0, 0.0, 0.0),
        )
        .unwrap();
        opencv::imgproc::threshold(
            &mat,
            &mut out_mat,
            2 as f64,
            1 as f64,
            opencv::imgproc::THRESH_BINARY,
        )
        .unwrap();
        let out_array = out_mat.data_bytes().unwrap();
        assert!(out_array[0] == 0);
        assert!(out_array[1] == 0);
        assert!(out_array[2] == 1);
        assert!(out_array[3] == 1);
        assert!(out_array[4] == 1);
        assert!(out_array[5] == 1);
    }

    #[test]
    fn test_image_encoder() {
        let seed: [u8; 32] = [42; 32];
        let mut rng = rand::rngs::StdRng::from_seed(seed);

        let width = 960;
        let height = 540;

        let mut frame_data: Vec<u8> = vec![0; width * height];
        rng.fill_bytes(&mut frame_data);
        frame_data.iter_mut().for_each(|x| {
            *x = (*x).min(20u8);
        });
        let frame = cam::Frame {
            data_row_major: frame_data,
            width: width,
            height: height,
            timestamp_ns: 0,
        };

        let mut ie = ImageEncoder::new(Some(1.0));
        let sizes = [40.0, 200.0, f32::NAN, 100.0, 50.0, 150.0, 30.0];

        for &size_kb in &sizes {
            if size_kb.is_nan() {
                ie.max_kb = None;
            } else {
                ie.max_kb = Some(size_kb);
            }
            let mut loop_broken = false;
            for _ in 0..15 {
                let encoded = ie.encode(&frame);
                assert!(encoded.is_some());
                let encoded = encoded.unwrap();
                if ie.max_kb.is_none() {
                    if ie.quality_str() == "PNG" {
                        loop_broken = true;
                        break;
                    }
                } else {
                    let size_kb = ie.max_kb.unwrap();
                    if ((encoded.len() as f32 / 1024.0) - size_kb).abs() < size_kb / 10.0 {
                        loop_broken = true;
                        break;
                    }
                }
                println!("size_kb={:?}, quality={}", ie.max_kb, ie.quality_str());
            }
            assert!(
                loop_broken,
                "Loop did not break for size_kb={:?}",
                ie.max_kb
            );
        }
    }
}

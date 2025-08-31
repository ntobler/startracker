use std::path::Path;

use crate::cam;
use numpy::PyArrayMethods;
use pyo3::prelude::*;

use image::{ImageBuffer, Luma};

pub trait ImageSource {
    fn get(&mut self, exposure_us: u32, analogue_gain: u32, timestamp_ns: u64) -> cam::Frame<u16>;
}

pub struct FileImageSource {
    data_row_major: Vec<Vec<u16>>,
    width: usize,
    height: usize,
    index: usize,
}

impl FileImageSource {
    pub fn new<P>(path: P) -> Result<Self, String>
    where
        P: AsRef<Path>,
    {
        let (mut width, mut height) = (0, 0);

        // Get all png files in directory
        let mut files = match std::fs::read_dir(path) {
            Ok(dir) => dir
                .filter_map(|entry| {
                    let entry = entry.ok()?;
                    let path = entry.path();
                    if path.extension().map_or(false, |ext| ext == "png") {
                        Some(path)
                    } else {
                        None
                    }
                })
                .collect(),
            Err(_) => vec![],
        };

        let mut buffers = Vec::new();
        if files.len() > 0 {
            files.sort_by(|a, b| a.file_name().cmp(&b.file_name()));
            for file in files {
                let img = image::open(file).map_err(|e| e.to_string())?;
                let gray: ImageBuffer<Luma<u8>, Vec<u8>> = img.to_luma8();
                (width, height) = gray.dimensions();
                let buffer: Vec<u16> = gray.pixels().map(|&Luma([v])| v as u16).collect();
                buffers.push(buffer);
            }
        } else {
            println!("No files found, using blank images");
            height = 1080;
            width = 1920;
            buffers.push(vec![0u16; width as usize * height as usize]);
        }

        Ok(FileImageSource {
            data_row_major: buffers,
            width: width as usize,
            height: height as usize,
            index: 0,
        })
    }
}

impl ImageSource for FileImageSource {
    fn get(&mut self, exposure_us: u32, analogue_gain: u32, timestamp_ns: u64) -> cam::Frame<u16> {
        let _ = exposure_us; // Unused in mock, but kept for interface compatibility
        let _ = analogue_gain; // Unused in mock, but kept for interface compatibility

        self.index = if self.index >= self.data_row_major.len() - 1 {
            0
        } else {
            self.index + 1
        };

        cam::Frame {
            data_row_major: self.data_row_major[self.index].clone(),
            width: self.width,
            height: self.height,
            timestamp_ns,
        }
    }
}

// struct ArtificialStarCam {
//     camera_instance: Py<PyAny>,
// }

// impl ArtificialStarCam {
//     /// Create a new instance from a Python class
//     pub fn new(class_name: &'static str) -> Result<Self, String> {
//         pyo3::prepare_freethreaded_python();

//         let res: PyResult<ArtificialStarCam> = Python::with_gil(|py| {
//             // Add current directory to sys.path
//             let sys = py.import_bound("sys")?;
//             let path = sys.getattr("path")?;
//             let path = path.downcast()?;
//             path.insert(0, ".")?;

//             // Import module and class
//             let testing_utils_module = py.import_bound("startracker.testing_utils")?;
//             let camera_module = py.import_bound("startracker.camera")?;

//             // Call Python constructor
//             let cam_config = camera_module.getattr("CameraConfig")?.call0()?;
//             let camera_instance = testing_utils_module
//                 .getattr(class_name)?
//                 .call1((cam_config,))?;

//             Ok(Self {
//                 camera_instance: camera_instance.into_py(py),
//             })
//         });
//         match res {
//             Ok(instance) => Ok(instance),
//             Err(e) => Err(e.to_string()),
//         }
//     }

//     /// Call the get() method on the Python object
//     pub fn get(&self) -> Result<(Vec<u8>, usize, usize), String> {
//         Python::with_gil(|py| {
//             // Call capture function
//             let result_pyobject: Py<PyAny> = self
//                 .camera_instance
//                 .call_method_bound(py, "capture", (), None)
//                 .map_err(|e| e.to_string())?;

//             // Get numpy array
//             let image_numpy_array: &Bound<numpy::PyArrayDyn<u8>> = result_pyobject
//                 .downcast_bound(py)
//                 .map_err(|e| e.to_string())?;

//             Ok((
//                 image_numpy_array.to_vec().map_err(|e| e.to_string())?,
//                 image_numpy_array.dims()[0],
//                 image_numpy_array.dims()[1],
//             ))
//         })
//     }
// }

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn test_testing_utils() {
//         let instance = ArtificialStarCam::new("AxisAlignCalibrationTestCam").unwrap();

//         let (data, width, height) = instance.get().unwrap();

//         assert_eq!(data.len(), width * height);
//     }
// }

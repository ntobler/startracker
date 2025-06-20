#[cfg(feature = "cam")]
use libcamera::{
    camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    request::Request,
    stream::StreamRole,
};
use std::time::{Duration, Instant};

use crate::cam;

// drm-fourcc does not have MJPEG type yet, construct it from raw fourcc identifier
const PIXEL_FORMAT_SRGGB10: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'R', b'G', b'1', b'0']), 0);

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<(u32, u32)>,
    frame_tx: crossbeam_channel::Sender<cam::Frame<u16>>,
    mut exposure_us: u32,
    mut analogue_gain: u32,
) -> Result<(), String> {
    println!("Camera thread: starting");
    let mgr = CameraManager::new().map_err(|e| format!("CameraManager error: {e:?}"))?;

    let cameras = mgr.cameras();

    let cam = cameras.get(0).ok_or("No cameras found".to_string())?;

    let model = cam
        .properties()
        .get::<properties::Model>()
        .map_err(|_| "Failed to get camera model property".to_string())?;
    println!("Using camera: {}", *model);

    let mut active_cam = cam
        .acquire()
        .map_err(|_| "Unable to acquire camera".to_string())?;

    // This will generate default configuration for each specified role
    let mut cfgs = active_cam
        .generate_configuration(&[StreamRole::StillCapture])
        .ok_or("Failed to generate camera configuration".to_string())?;

    // Set stream format
    cfgs.get_mut(0)
        .ok_or("No configuration found".to_string())?
        .set_pixel_format(PIXEL_FORMAT_SRGGB10);
    match cfgs.validate() {
        CameraConfigurationStatus::Valid => println!("Camera configuration valid!"),
        CameraConfigurationStatus::Adjusted => {
            println!("Camera configuration was adjusted: {:#?}", cfgs)
        }
        CameraConfigurationStatus::Invalid => panic!("Error validating camera configuration"),
    }
    if cfgs.get(0).unwrap().get_pixel_format() != PIXEL_FORMAT_SRGGB10 {
        return Err("SRGGB10 is not supported by the camera".to_string());
    }

    let imsize: libcamera::geometry::Size = cfgs.get(0).unwrap().get_size();

    active_cam
        .configure(&mut cfgs)
        .expect("Unable to configure camera");

    let mut alloc = FrameBufferAllocator::new(&active_cam);

    // Allocate frame buffers for the stream
    let cfg = cfgs.get(0).unwrap();
    let stream = cfg.stream().unwrap();
    let buffers = alloc.alloc(&stream).unwrap();
    println!("Allocated {} buffers", buffers.len());

    // Convert FrameBuffer to MemoryMappedFrameBuffer, which allows reading &[u8]
    let buffers = buffers
        .into_iter()
        .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
        .collect::<Vec<_>>();

    // Create capture requests and attach buffers
    let reqs = buffers
        .into_iter()
        .map(|buf| {
            let mut req = active_cam.create_request(None).unwrap();
            req.add_buffer(&stream, buf).unwrap();
            req
        })
        .collect::<Vec<_>>();

    // Completed capture requests are returned as a callback
    let (req_tx, req_rx) = crossbeam_channel::bounded::<Request>(2); // frame response
    active_cam.on_request_completed(move |req| {
        req_tx.send(req).ok();
    });

    // Start camera
    active_cam.start(None).unwrap();

    // Queue all requests
    for mut req in reqs.into_iter() {
        set_controls(&mut req, exposure_us, analogue_gain).unwrap();
        active_cam.queue_request(req).unwrap();
    }

    println!("Camera thread: started");

    let poll_interval = Duration::from_millis(100);

    // Start acquisition loop
    loop {
        // Get pending requests. Use timeout to check for disconnected trigger
        let mut req = match req_rx.recv_timeout(poll_interval) {
            Ok(req) => req,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                continue;
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                return Err("Failure reading from request channel".to_string());
            }
        };

        //Check for trigger signal
        match trigger_rx.try_recv() {
            Ok((new_exposure_ns, new_analogue_gain)) => {
                // Get new parameters from trigger channel
                exposure_us = new_exposure_ns;
                analogue_gain = new_analogue_gain;

                // Trigger signal present -> get buffer and read data
                println!("Camera thread: trigger ok received");

                let metadata = req.metadata();
                let effective_exposure_us: libcamera::controls::ExposureTime =
                    metadata.get().map_err(|e| e.to_string())?;
                let effective_analogue_gain: libcamera::controls::AnalogueGain =
                    metadata.get().map_err(|e| e.to_string())?;
                println!(
                    "Camera thread: Exposure: {:?}, analogue gain {:?}",
                    effective_exposure_us, effective_analogue_gain
                );

                // Get framebuffer for our stream
                let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                    req.buffer(&stream).unwrap();

                // Get framebuffer metadata
                let frame_buffer_meta = framebuffer.metadata().unwrap();
                let bytes_used = frame_buffer_meta.planes().get(0).unwrap().bytes_used as usize;
                let timestamp_ns: u64 = frame_buffer_meta.timestamp();
                let sequence: u32 = frame_buffer_meta.sequence();
                println!(
                    "Camera thread: Timestamp {:?}, sequence {:?}, bytes used {:?}",
                    timestamp_ns, sequence, bytes_used
                );

                // Extract data
                let &buffer_data = framebuffer.data().get(0).unwrap();
                let start = Instant::now();
                // Copying from the buffer is very slow on the Raspberry Pi (30..45ms)
                // However it might be worth copying the data to an intermediate buffer before performing
                // random access operations on the data, as this is magnitudes slower on the buffer compared
                // to a freshly allocated Vec
                let frame_data: Vec<u16> = bytemuck::cast_slice(buffer_data).to_vec();
                let duration = start.elapsed();
                println!("Camera thread: Extracting frame took {:?}", duration);

                if frame_data.len() != (imsize.width * imsize.height) as usize {
                    return Err("frame_data size does not equate to width * height".to_string());
                }

                let frame = cam::Frame {
                    data_row_major: frame_data,
                    width: imsize.width as usize,
                    height: imsize.height as usize,
                    timestamp_ns,
                };
                frame_tx.send(frame).ok();
            }
            Err(crossbeam_channel::TryRecvError::Empty) => {
                // nothing available now, proceed
                println!("Camera thread: trigger not present, continuing");
            }
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                // Channel has been closed from the other side. We can shut down the thread
                println!("Camera thread: trigger error received, terminating gracefully");
                return Ok(());
            }
        }
        // Requeue request
        req.reuse(libcamera::request::ReuseFlag::REUSE_BUFFERS);
        set_controls(&mut req, exposure_us, analogue_gain).unwrap();
        active_cam.queue_request(req).unwrap();
    }
}

fn set_controls(
    request: &mut libcamera::request::Request,
    exposure_us: u32,
    analogue_gain: u32,
) -> Result<(), libcamera::control::ControlError> {
    let controls = request.controls_mut();
    controls.set(libcamera::controls::AeEnable(false))?;
    controls.set(libcamera::controls::FrameDuration(exposure_us as i64))?;
    controls.set(libcamera::controls::FrameDurationLimits([
        exposure_us as i64,
        exposure_us as i64,
    ]))?;
    controls.set(libcamera::controls::AnalogueGain(analogue_gain as f32))?;
    controls.set(libcamera::controls::ExposureTime(exposure_us as i32))?;
    controls.set(libcamera::controls::AnalogueGain(analogue_gain as f32))?;
    controls.set(libcamera::controls::DigitalGain(1.0f32))?;
    println!("Camera controls {:?} set!", controls);
    Ok(())
}

#[cfg(feature = "cam")]
use std::time::Duration;

#[cfg(feature = "cam")]
use libcamera::{
    camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    stream::StreamRole,
};

// drm-fourcc does not have MJPEG type yet, construct it from raw fourcc identifier
#[cfg(feature = "cam")]
const PIXEL_FORMAT_SRGGB10: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'R', b'G', b'1', b'0']), 0);

#[cfg(feature = "cam")]
pub fn get() -> Result<Vec<u8>, String> {
    let mgr = CameraManager::new().map_err(|e| format!("CameraManager error: {e:?}"))?;

    let cameras = mgr.cameras();

    let cam = cameras.get(0).ok_or("No cameras found".to_string())?;

    let model = cam
        .properties()
        .get::<properties::Model>()
        .map_err(|_| "Failed to get camera model property".to_string())?;
    println!("Using camera: {}", *model);

    let mut cam = cam
        .acquire()
        .map_err(|_| "Unable to acquire camera".to_string())?;

    // This will generate default configuration for each specified role
    let mut cfgs = cam
        .generate_configuration(&[StreamRole::ViewFinder])
        .map_err(|_| "Failed to generate camera configuration".to_string())?;

    // Use MJPEG format so we can write resulting frame directly into jpeg file
    cfgs.get_mut(0)
        .ok_or("No configuration found".to_string())?
        .set_pixel_format(PIXEL_FORMAT_SRGGB10);

    println!("Generated config: {:#?}", cfgs);

    match cfgs.validate() {
        CameraConfigurationStatus::Valid => println!("Camera configuration valid!"),
        CameraConfigurationStatus::Adjusted => {
            println!("Camera configuration was adjusted: {:#?}", cfgs)
        }
        CameraConfigurationStatus::Invalid => panic!("Error validating camera configuration"),
    }

    // Ensure that pixel format was unchanged
    assert_eq!(
        cfgs.get(0).unwrap().get_pixel_format(),
        PIXEL_FORMAT_SRGGB10,
        "SRGGB10 is not supported by the camera"
    );

    cam.configure(&mut cfgs)
        .expect("Unable to configure camera");

    let mut alloc = FrameBufferAllocator::new(&cam);

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
    let mut reqs = buffers
        .into_iter()
        .map(|buf| {
            let mut req = cam.create_request(None).unwrap();
            req.add_buffer(&stream, buf).unwrap();
            req
        })
        .collect::<Vec<_>>();

    // Completed capture requests are returned as a callback
    let (tx, rx) = std::sync::mpsc::channel();
    cam.on_request_completed(move |req| {
        tx.send(req).unwrap();
    });

    cam.start(None).unwrap();

    // Multiple requests can be queued at a time, but for this example we just want a single frame.
    cam.queue_request(reqs.pop().unwrap()).unwrap();

    println!("Waiting for camera request execution");
    let req = rx
        .recv_timeout(Duration::from_secs(2))
        .expect("Camera request failed");

    println!("Camera request {:?} completed!", req);
    println!("Metadata: {:#?}", req.metadata());

    // Get framebuffer for our stream
    let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
    println!("FrameBuffer metadata: {:#?}", framebuffer.metadata());

    // MJPEG format has only one data plane containing encoded jpeg data with all the headers
    let planes = framebuffer.data();
    let buffer_data = planes.get(0).unwrap();

    let data_len = framebuffer
        .metadata()
        .unwrap()
        .planes()
        .get(0)
        .unwrap()
        .bytes_used as usize;

    print!("data_len {:?}", data_len);
    let sum: u32 = buffer_data.iter().map(|&b| b as u32).sum();
    println!("data sum {:?}", sum);

    Ok(buffer_data.to_vec())
}

#[cfg(not(feature = "cam"))]
pub fn get() -> Result<Vec<u8>, String> {
    Ok(vec![0; 1920 * 1080 * 2])
}

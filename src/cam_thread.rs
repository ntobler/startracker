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

// drm-fourcc does not have MJPEG type yet, construct it from raw fourcc identifier
const PIXEL_FORMAT_SRGGB10: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'R', b'G', b'1', b'0']), 0);

pub fn camera_thread(
    trigger_rx: crossbeam_channel::Receiver<()>,
    frame_tx: crossbeam_channel::Sender<Vec<u16>>,
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
        .generate_configuration(&[StreamRole::ViewFinder])
        .ok_or("Failed to generate camera configuration".to_string())?;

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
    for req in reqs.into_iter() {
        active_cam.queue_request(req).unwrap();
    }

    println!("Camera thread: started");

    // Start acquisition loop
    loop {
        // Get pending request blocking
        let mut req = req_rx
            .recv()
            .map_err(|_| "Failure reading from request channel".to_string())?;

        println!("Camera request {:?} completed!", req);

        //Check for trigger signal
        match trigger_rx.try_recv() {
            Ok(_) => {
                // Trigger signal present -> get buffer and read data
                println!("Camera thread: trigger ok received");

                println!("Metadata: {:#?}", req.metadata());

                // Get framebuffer for our stream
                let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                    req.buffer(&stream).unwrap();
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

                let frame: Vec<u16> = buffer_data
                    .chunks_exact(2)
                    .map(|chunk| u16::from_le_bytes(chunk.try_into().unwrap()))
                    .collect();

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
        // Requeue request
        req.reuse(libcamera::request::ReuseFlag::REUSE_BUFFERS);
        active_cam.queue_request(req).unwrap();
    }
}

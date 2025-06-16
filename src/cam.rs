use std::thread;

#[cfg(feature = "cam")]
#[path = "cam_thread.rs"]
mod cam_thread;

#[cfg(not(feature = "cam"))]
#[path = "cam_thread_mock.rs"]
mod cam_thread;

pub struct Camera {
    thread_handle: Option<thread::JoinHandle<Result<(), String>>>,
    thread_error: Option<String>,
    trigger_tx: Option<crossbeam_channel::Sender<()>>,
    frame_rx: crossbeam_channel::Receiver<Vec<u16>>,
}

impl Camera {
    pub fn new() -> Result<Self, String> {
        let (trigger_tx, trigger_rx) = crossbeam_channel::bounded::<()>(0); // unbuffered: strictly 1:1 signal
        let (frame_tx, frame_rx) = crossbeam_channel::bounded::<Vec<u16>>(1); // frame response

        let thread_handle = thread::spawn(move || cam_thread::camera_thread(trigger_rx, frame_tx));

        Ok(Camera {
            thread_handle: Some(thread_handle),
            thread_error: None,
            trigger_tx: Some(trigger_tx),
            frame_rx,
        })
    }

    fn get_thread_error(&mut self) -> String {
        if let Some(err) = &self.thread_error {
            return err.clone();
        }

        let err = match self.thread_handle.take() {
            Some(handle) => match handle.join() {
                Ok(Ok(_)) => "Thread ended without errors.".to_string(),
                Ok(Err(e)) => e,
                Err(panic) => format!("Thread panicked: {:?}", panic),
            },
            None => "Thread already joined.".to_string(),
        };

        self.thread_error = Some(err.clone());
        err
    }

    pub fn capture(&mut self) -> Result<Vec<u16>, String> {
        if let Some(e) = &self.thread_error {
            return Err(e.clone());
        }
        println!("Waiting for camera request execution");
        match self
            .trigger_tx
            .as_ref()
            .ok_or("trigger has been dropped".to_string())?
            .send(())
        {
            Ok(_) => {}
            Err(_) => {
                return Err(self.get_thread_error());
            }
        };
        let frame = match self.frame_rx.recv() {
            Ok(f) => f,
            Err(_) => {
                return Err(self.get_thread_error());
            }
        };
        println!("Capture received frame");
        Ok(frame)
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        // Drop trigger_tx to close channel (happens automatically here)
        // But explicitly drop to show intent:
        drop(self.trigger_tx.take());

        if let Some(handle) = self.thread_handle.take() {
            println!("Dropping Camera: joining camera thread...");
            match handle.join() {
                Ok(Ok(())) => println!("Camera thread terminated gracefully."),
                Ok(Err(e)) => eprintln!("Camera thread returned error: {}", e),
                Err(e) => eprintln!("Camera thread panicked: {:?}", e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera() {
        let mut cam = Camera::new().unwrap();
        assert_eq!(cam.capture().unwrap().len(), 1920 * 1080);
    }
}

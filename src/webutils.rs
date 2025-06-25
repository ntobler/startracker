use std::error::Error;
use std::fmt;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

// Custom error for timeout
#[derive(Debug)]
pub struct TimeoutError;

impl fmt::Display for TimeoutError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Operation timed out")
    }
}

impl Error for TimeoutError {}

pub struct DataDispatcher<T> {
    data: Arc<Mutex<Option<T>>>,
    data_changed: Arc<Condvar>,
}

impl<T: Clone + Send + Sync + 'static> DataDispatcher<T> {
    pub fn new() -> Self {
        DataDispatcher {
            data: Arc::new(Mutex::new(None)),
            data_changed: Arc::new(Condvar::new()),
        }
    }

    pub fn put(&self, data: T) {
        let mut data_guard = self.data.lock().unwrap();
        *data_guard = Some(data);
        self.data_changed.notify_all();
    }

    pub fn get_blocking(&self) -> T {
        let mut data_guard = self.data.lock().unwrap();
        data_guard = self.data_changed.wait(data_guard).unwrap();
        data_guard.as_ref().unwrap().clone()
    }

    pub fn get_blocking_with_timeout(&self, timeout: Duration) -> Result<T, TimeoutError> {
        let mut data_guard = self.data.lock().unwrap();
        let (new_data_guard, wait_timeout_result) =
            self.data_changed.wait_timeout(data_guard, timeout).unwrap();
        data_guard = new_data_guard;

        if wait_timeout_result.timed_out() {
            return Err(TimeoutError);
        }

        Ok(data_guard.as_ref().unwrap().clone())
    }
}

impl<T> Clone for DataDispatcher<T> {
    fn clone(&self) -> Self {
        DataDispatcher {
            data: Arc::clone(&self.data),
            data_changed: Arc::clone(&self.data_changed),
        }
    }
}

// Example usage with Arc<Vec<u8>>
#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_image_dispatcher() {
        let dispatcher = DataDispatcher::<Arc<Vec<u8>>>::new();
        let dispatcher_clone_1 = dispatcher.clone();
        let dispatcher_clone_2 = dispatcher.clone();
        let dispatcher_clone_3 = dispatcher.clone();
        let dispatcher_clone_4 = dispatcher.clone();

        let handle1 = thread::spawn(move || {
            println!("Consumer 1 waiting for image...");
            let image_arc = dispatcher_clone_1.get_blocking();
            println!("Consumer 1 received image of size: {}", image_arc.len());
            assert_eq!(image_arc[0], 0xAA);
            assert_eq!(image_arc[image_arc.len() - 1], 0xFF);
        });

        let handle2 = thread::spawn(move || {
            println!("Consumer 2 waiting for image...");
            let image_arc = dispatcher_clone_2.get_blocking();
            println!("Consumer 2 received image of size: {}", image_arc.len());
            assert_eq!(image_arc[0], 0xAA);
            assert_eq!(image_arc[image_arc.len() - 1], 0xFF);
        });

        let handle3 = thread::spawn(move || {
            println!("Consumer 3 waiting for image...");
            let duration = Duration::from_millis(50);
            let image_arc = dispatcher_clone_3.get_blocking_with_timeout(duration);
            assert!(image_arc.is_err(), "Expected timeout error for Consumer 3");
        });

        let handle4 = thread::spawn(move || {
            println!("Consumer 4 waiting for image...");
            let duration = Duration::from_millis(150);
            let image_arc = match dispatcher_clone_4.get_blocking_with_timeout(duration) {
                Ok(image) => image,
                Err(e) => {
                    println!("Consumer 4 timed out: {}", e);
                    assert!(false, "Consumer 4 should not timeout");
                    return;
                }
            };
            println!("Consumer 4 received image of size: {}", image_arc.len());
            assert_eq!(image_arc[0], 0xAA);
            assert_eq!(image_arc[image_arc.len() - 1], 0xFF);
        });

        thread::sleep(Duration::from_millis(100)); // Give consumers time to start waiting

        let image_data: Vec<u8> = vec![0xAA, 0xBB, 0xCC, 0xFF]; // Small example image
        let image_arc = Arc::new(image_data);
        println!("Producer putting image...");
        dispatcher.put(image_arc);

        handle1.join().unwrap();
        handle2.join().unwrap();
        handle3.join().unwrap();
        handle4.join().unwrap();
    }
}

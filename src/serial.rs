use serialport::TTYPort;
use std::io::{self, Read, Write};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

fn calc_crc_ibm(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &x in data {
        crc ^= x as u16;
        for _ in 0..8 {
            crc = if (crc & 0x0001) != 0 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
        }
    }
    crc & 0xFFFF
}

pub struct Packet {
    pub cmd: u8,
    pub len: u8,
    pub payload: Vec<u8>,
}

struct PacketReader {
    buffer: Vec<u8>,
    callback: Arc<dyn Fn(Packet) + Send + Sync>,
}

impl PacketReader {
    fn new(callback: Arc<dyn Fn(Packet) + Send + Sync>) -> Self {
        PacketReader {
            buffer: vec![0; 255 + 4],
            callback,
        }
    }

    fn check_packet(&mut self, received_bytes: &[u8]) {
        // If there are no new bytes, flush
        if received_bytes.len() == 0 {
            self.buffer.clear();
            return;
        }

        for &data in received_bytes {
            self.buffer.push(data);

            if self.buffer.len() < 2 {
                continue;
            }
            let len = self.buffer[1] as usize;
            if self.buffer.len() < 2 + len + 2 {
                continue;
            }
            let crc = calc_crc_ibm(&self.buffer[0..(2 + len)]);
            if crc == ((self.buffer[2 + len] as u16) << 8 | (self.buffer[2 + len + 1] as u16)) {
                let packet = Packet {
                    cmd: self.buffer[0],
                    len: self.buffer[1],
                    payload: self.buffer[2..(2 + len)].to_vec(),
                };
                (self.callback)(packet);
            }
            self.buffer.clear();
        }
    }
}

fn serial_thread(
    port: String,
    baudrate: u32,
    terminate: Arc<AtomicBool>,
    callback: Arc<dyn Fn(Packet) + Send + Sync>,
) -> Result<(), String> {
    let mut reader = serialport::new(port, baudrate)
        .timeout(Duration::from_millis(10))
        .open_native()
        .map_err(|e| format!("Failed to open serial port: {}", e))?;

    let mut buffer: Vec<u8> = vec![0; 255 + 4];
    let mut packet_reader = PacketReader::new(callback);
    loop {
        match reader.read(buffer.as_mut_slice()) {
            Ok(t) => {
                if terminate.load(std::sync::atomic::Ordering::SeqCst) {
                    println!("Serial thread: terminating as requested.");
                    return Ok(());
                }
                packet_reader.check_packet(&buffer[..t]);
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                if terminate.load(std::sync::atomic::Ordering::SeqCst) {
                    println!("Serial thread: terminating as requested.");
                    return Ok(());
                }
            }
            Err(e) => {
                return Err(format!("Error reading from serial port: {}", e));
            }
        }
    }
}

pub struct Serial {
    thread_handle: Option<thread::JoinHandle<Result<(), String>>>,
    writer: TTYPort,
    terminate: Arc<AtomicBool>,
}

impl Serial {
    pub fn new(
        port: String,
        baudrate: u32,
        callback: Arc<dyn Fn(Packet) + Send + Sync>,
    ) -> Result<Self, String> {
        let terminate = Arc::new(AtomicBool::new(false));
        let writer = serialport::new(&port, baudrate)
            .timeout(Duration::from_millis(10))
            .open_native()
            .map_err(|e| format!("Failed to open serial port: {}", e))?;

        let terminate_clone = terminate.clone();
        let thread_handle =
            thread::spawn(move || serial_thread(port, baudrate, terminate_clone, callback));

        Ok(Serial {
            thread_handle: Some(thread_handle),
            writer,
            terminate: terminate,
        })
    }

    pub fn send(&mut self, packet: &Packet) -> Result<(), String> {
        let mut buffer = Vec::with_capacity(2 + packet.len as usize + 2);
        buffer.push(packet.cmd);
        buffer.push(packet.len);
        buffer.extend_from_slice(packet.payload.as_slice());
        let crc = calc_crc_ibm(&buffer);
        buffer.push((crc >> 8) as u8);
        buffer.push((crc & 0xFF) as u8);
        self.writer
            .write_all(buffer.as_slice())
            .map_err(|e| format!("Failed to write to serial port: {}", e))?;
        Ok(())
    }
}

impl Drop for Serial {
    fn drop(&mut self) {
        // Make the thread exit
        self.terminate
            .store(true, std::sync::atomic::Ordering::SeqCst);

        if let Some(handle) = self.thread_handle.take() {
            println!("Dropping Serial: joining serial thread...");
            match handle.join() {
                Ok(Ok(())) => println!("Serial thread terminated gracefully."),
                Ok(Err(e)) => eprintln!("Serial thread returned error: {}", e),
                Err(e) => eprintln!("Serial thread panicked: {:?}", e),
            }
        }
    }
}

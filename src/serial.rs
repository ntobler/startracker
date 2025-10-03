use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crate::utils;
use nix::libc::tcflush;
use nix::sys::termios::FlushArg;
use serialport::TTYPort;

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

pub struct RxPacket {
    pub cmd: u8,
    pub payload: Vec<u8>,
    pub rx_time_ns: u64,
}

pub struct TxPacket {
    pub cmd: u8,
    pub payload: Vec<u8>,
}

struct PacketReader {
    buffer: Vec<u8>,
    rx_time_ns: u64,
    callback: Arc<dyn Fn(RxPacket) + Send + Sync>,
    ns_per_byte: u64,
}

impl PacketReader {
    fn new(baudrate: u32, callback: Arc<dyn Fn(RxPacket) + Send + Sync>) -> Self {
        let ns_per_byte = (1_000_000_000 * 10) / baudrate as u64;
        PacketReader {
            buffer: Vec::with_capacity(255 + 4),
            rx_time_ns: 0,
            callback,
            ns_per_byte,
        }
    }

    fn flush(&mut self) {
        self.buffer.clear();
    }

    // Returns number of bytes needed to complete a packet
    fn check_packet(&mut self, received_bytes: &[u8], rx_time_ns: u64) -> usize {
        // If there are no new bytes, flush
        if received_bytes.len() == 0 {
            self.buffer.clear();
            return 4;
        }

        for (i, &data) in received_bytes.iter().enumerate() {
            self.buffer.push(data);

            // Record the time of the first byte
            if self.buffer.len() == 1 {
                let time_shift_ns = (received_bytes.len() - i) as u64 * self.ns_per_byte;
                self.rx_time_ns = rx_time_ns - time_shift_ns;
            }

            if self.buffer.len() < 2 {
                continue;
            }
            let len = self.buffer[1] as usize;
            if self.buffer.len() < 2 + len + 2 {
                continue;
            }
            let crc = calc_crc_ibm(&self.buffer[0..(2 + len)]);
            if crc == ((self.buffer[2 + len] as u16) << 8 | (self.buffer[2 + len + 1] as u16)) {
                let packet = RxPacket {
                    cmd: self.buffer[0],
                    payload: self.buffer[2..(2 + len)].to_vec(),
                    rx_time_ns: self.rx_time_ns,
                };
                (self.callback)(packet);
            }
            self.buffer.clear();
        }

        if self.buffer.len() < 2 {
            return 4 - self.buffer.len();
        } else {
            let len = self.buffer[1] as usize;
            return 2 + len + 2 - self.buffer.len();
        }
    }
}

fn flush_input(port: &TTYPort) {
    let fd = port.as_raw_fd();
    let _ = unsafe { tcflush(fd, FlushArg::TCIFLUSH as i32) };
}

fn serial_thread(
    port: String,
    baudrate: u32,
    tx_channel: crossbeam_channel::Receiver<Vec<u8>>,
    rx_callback: Arc<dyn Fn(RxPacket) + Send + Sync>,
) -> Result<(), String> {
    let mut reader = serialport::new(port, baudrate)
        .timeout(Duration::from_millis(10))
        .open_native()
        .map_err(|e| {
            eprintln!("Error opening serial port: {}", e);
            return format!("Failed to open serial port: {}", e);
        })?;

    flush_input(&reader);

    let mut buffer: Vec<u8> = vec![0; 255 + 4];
    let mut packet_reader = PacketReader::new(baudrate, rx_callback);
    let mut bytes_needed = 4;
    loop {
        bytes_needed = match reader.read_exact(buffer[..bytes_needed].as_mut()) {
            Ok(()) => {
                let rx_time_ns = utils::monotonic_time_ns();
                packet_reader.check_packet(&buffer[..bytes_needed], rx_time_ns)
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                packet_reader.flush();
                4
            }
            Err(e) => {
                return Err(format!("Error reading from serial port: {}", e));
            }
        };

        for _ in 0..10 {
            match tx_channel.try_recv() {
                Ok(data) => {
                    if let Err(e) = reader.write_all(&data) {
                        return Err(format!("Error writing to serial port: {}", e));
                    }
                }
                Err(crossbeam_channel::TryRecvError::Empty) => {
                    break; // No more data to send
                }
                Err(crossbeam_channel::TryRecvError::Disconnected) => {
                    println!("Serial thread: transmit channel disconnected, terminating.");
                    return Ok(());
                }
            }
        }
    }
}

pub struct Serial {
    thread_handle: Option<thread::JoinHandle<Result<(), String>>>,
    thread_error: Option<String>,
    tx_channel: Option<crossbeam_channel::Sender<Vec<u8>>>,
}

impl Serial {
    pub fn new(
        port: String,
        baudrate: u32,
        callback: Arc<dyn Fn(RxPacket) + Send + Sync>,
    ) -> Result<Self, String> {
        let (tx_channel_tx, tx_channel_rx) = crossbeam_channel::bounded::<Vec<u8>>(10); // unbuffered: strictly 1:1 signal

        let thread_handle =
            thread::spawn(move || serial_thread(port, baudrate, tx_channel_rx, callback));

        Ok(Serial {
            thread_handle: Some(thread_handle),
            thread_error: None,
            tx_channel: Some(tx_channel_tx),
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

    pub fn send(&mut self, packet: &TxPacket) -> Result<(), String> {
        let mut buffer = Vec::with_capacity(2 + packet.payload.len() + 2);
        buffer.push(packet.cmd);
        buffer.push(packet.payload.len() as u8);
        buffer.extend_from_slice(packet.payload.as_slice());
        let crc = calc_crc_ibm(&buffer);
        buffer.push((crc >> 8) as u8);
        buffer.push((crc & 0xFF) as u8);
        match self
            .tx_channel
            .as_ref()
            .ok_or("Tx channel has been dropped".to_string())?
            .send(buffer)
        {
            Ok(_) => Ok(()),
            Err(_) => Err(self.get_thread_error()),
        }
    }
}

impl Drop for Serial {
    fn drop(&mut self) {
        // Make the thread exit
        drop(self.tx_channel.take());

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

use crate::serial;

pub trait Command {
    const CMD: u8;
}

pub trait TxCommand {
    fn to_tx_packet(&self) -> serial::TxRawPacket;
}

pub trait RxCommand: Sized {
    fn from_rx_packet(rx: &serial::RxRawPacket) -> Option<Self>;
}

#[repr(C, packed)]
#[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod, Debug)]
pub struct Gyro {
    pub raw_gyro: [f32; 3],
    pub id: u16,
}

impl Command for Gyro {
    const CMD: u8 = 0x00;
}

impl RxCommand for Gyro {
    fn from_rx_packet(rx: &serial::RxRawPacket) -> Option<Self> {
        if (rx.cmd != Self::CMD) || (rx.payload.len() as usize != std::mem::size_of::<Self>()) {
            return None;
        }
        let payload: &Self = bytemuck::from_bytes(&rx.payload);
        Some(Self {
            raw_gyro: std::array::from_fn(|i| {
                f32::from_bits(u32::from_le(payload.raw_gyro[i].to_bits()))
            }),
            id: u16::from_le(payload.id),
        })
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod, Debug)]
pub struct StarQuat {
    pub quat: [f32; 4],
    pub scale: [f32; 3],
    pub bias: [f32; 3],
    pub id: u16,
}

impl StarQuat {
    pub fn new(quat: [f32; 4], scale: [f32; 3], bias: [f32; 3], id: u16) -> StarQuat {
        StarQuat {
            quat: std::array::from_fn(|i| f32::from_bits(quat[i].to_bits().to_le())),
            scale: std::array::from_fn(|i| f32::from_bits(scale[i].to_bits().to_le())),
            bias: std::array::from_fn(|i| f32::from_bits(bias[i].to_bits().to_le())),
            id: id.to_le(),
        }
    }

    pub fn empty() -> StarQuat {
        StarQuat {
            quat: [0.0; 4],
            scale: [0.0; 3],
            bias: [0.0; 3],
            id: 0,
        }
    }
}

impl Command for StarQuat {
    const CMD: u8 = 0x01;
}

impl TxCommand for StarQuat {
    fn to_tx_packet(&self) -> serial::TxRawPacket {
        serial::TxRawPacket {
            cmd: StarQuat::CMD,
            payload: bytemuck::bytes_of(self).to_vec(),
        }
    }
}

impl RxCommand for StarQuat {
    fn from_rx_packet(rx: &serial::RxRawPacket) -> Option<Self> {
        if (rx.cmd != Self::CMD) || (rx.payload.len() as usize != std::mem::size_of::<Self>()) {
            return None;
        }
        let payload: &Self = bytemuck::from_bytes(&rx.payload);
        Some(StarQuat {
            quat: std::array::from_fn(|i| f32::from_bits(u32::from_le(payload.quat[i].to_bits()))),
            scale: std::array::from_fn(|i| {
                f32::from_bits(u32::from_le(payload.scale[i].to_bits()))
            }),
            bias: std::array::from_fn(|i| f32::from_bits(u32::from_le(payload.bias[i].to_bits()))),
            id: u16::from_le(payload.id),
        })
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod, Debug)]
pub struct ShutdownRequest {
    pub code: u8,
}

impl ShutdownRequest {}

impl Command for ShutdownRequest {
    const CMD: u8 = 0x02;
}

impl RxCommand for ShutdownRequest {
    fn from_rx_packet(rx: &serial::RxRawPacket) -> Option<Self> {
        if (rx.cmd != Self::CMD) || (rx.payload.len() as usize != std::mem::size_of::<Self>()) {
            return None;
        }
        let payload: &Self = bytemuck::from_bytes(&rx.payload);
        Some(Self {
            code: u8::from_le(payload.code),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let payload = vec![
            0x6E, 0x88, 0xAE, 0xBB, 0x58, 0xA0, 0x8B, 0xB9, 0x6E, 0x88, 0xAE, 0x3B, 0xEF, 0x44,
        ];

        let rx = serial::RxRawPacket {
            cmd: 0x00,
            payload: payload,
            rx_time_ns: 0,
        };

        let packet = Gyro::from_rx_packet(&rx);

        assert!(packet.is_some());
    }
}

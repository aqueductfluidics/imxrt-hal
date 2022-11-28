use super::{Frame, Id, CAN};

use crate::iomuxc::consts::Unsigned;

/// A CAN ISO-TP builder that can build a CAN peripheral
pub struct IsoTPBuilder<M, F, G>
where
    M: Unsigned,
    F: Fn(u32) -> (),
    G: Fn(u32) -> (),
{
    can: CAN<M>,
    delay_ms: F,
    delay_us: G,
}

impl<M, F, G> IsoTPBuilder<M, F, G>
where
    M: Unsigned,
    F: Fn(u32) -> (),
    G: Fn(u32) -> (),
{
    pub fn new(can: CAN<M>, delay_ms: F, delay_us: G) -> Self {
        IsoTPBuilder {
            can,
            delay_ms,
            delay_us,
        }
    }

    pub fn build(self) -> IsoTP<M, F, G> {
        IsoTP::new(self.can, self.delay_ms, self.delay_us)
    }
}

pub enum FlowControlType {
    ClearToSend = 0,
    Wait = 1,
    Abort = 2,
}

pub struct IsoTPConfig {
    pub id: Id,
    pub use_padding: bool,
    pub separation_us: bool,
    pub len: u16,
    pub block_size: u16,
    pub flow_control_type: FlowControlType,
    pub separation_time: u16,
    pub padding_value: u8,
}

const RX_BUFFER_LENGTH: usize = 1024;

pub struct IsoTP<M, F, G>
where
    M: Unsigned,
    F: Fn(u32) -> (),
    G: Fn(u32) -> (),
{
    pub can: CAN<M>,
    delay_ms: F,
    delay_us: G,
    rx_buffer: [u8; RX_BUFFER_LENGTH],
}

impl<M, F, G> IsoTP<M, F, G>
where
    M: Unsigned,
    F: Fn(u32) -> (),
    G: Fn(u32) -> (),
{
    pub fn new(can: CAN<M>, delay_ms: F, delay_us: G) -> Self {
        Self {
            can,
            rx_buffer: [0x00; RX_BUFFER_LENGTH],
            delay_ms,
            delay_us,
        }
    }

    pub fn write(&mut self, config: &IsoTPConfig, buf: &[u8], size: u16) {
        let mut data: [u8; 8] = [config.padding_value; 8];
        data[0] = ((1 << 4) | size >> 8) as u8;
        data[1] = size as u8;
        data[2..8].copy_from_slice(&buf[0..6]);
        let mut msg: Frame = Frame::new_data(config.id, data);

        self.can.transmit(&msg);

        // let now =
    }
}

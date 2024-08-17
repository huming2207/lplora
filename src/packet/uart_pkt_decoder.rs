use crate::constants::CacheQueue;

use super::{slip_dequeue, UartPacketError, UartPacketType};

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub struct UartPacketDecoder {
    pkt_type: UartPacketType,
    curr_payload_len: u16,
    payload_buf: [u8; 300],
}

impl UartPacketDecoder {
    pub fn new(queue: &mut CacheQueue) -> Result<UartPacketDecoder, UartPacketError> {
        let mut buf: [u8; 300] = [0; 300];
        slip_dequeue(queue, &mut buf)?;

        let pkt_type = UartPacketType::try_from(buf[0])?;
        let payload_len_bytes: [u8; 2] = [buf[0], buf[1]];
        let curr_payload_len = u16::from_le_bytes(payload_len_bytes);
        if curr_payload_len > (buf.len() as u16) {
            return Err(UartPacketError::BufferFullError);
        }

        Ok(UartPacketDecoder {
            pkt_type,
            curr_payload_len,
            payload_buf: buf,
        })
    }

    pub fn get_type(&self) -> UartPacketType {
        self.pkt_type
    }

    pub fn get_payload_len(&self) -> u16 {
        self.curr_payload_len
    }
}
